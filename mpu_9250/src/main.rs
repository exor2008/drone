#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(slice_flatten)]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
// use embassy_rp::gpio::{Input, Level, Output};
use embassy_rp::i2c::{self, Async, Config as ConfigI2c, InterruptHandler as InterruptHandlerI2c};
use embassy_rp::peripherals::I2C0;
use embassy_rp::peripherals::USB;
use embassy_rp::usb::{Driver, Instance, InterruptHandler as InterruptHandlerUsb};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Ticker};
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::driver::EndpointError;
use embassy_usb::{Builder, Config as ConfigUsb, UsbDevice};
use mpu_9250::{Accelerometer, Gyro, ImuMeasurement, Magnetometer, Mpu9250};
use panic_probe as _;
use static_cell::StaticCell;

bind_interrupts!(struct IrqsI2c {
    I2C0_IRQ => InterruptHandlerI2c<I2C0>;
});

bind_interrupts!(struct IrqsUsb {
    USBCTRL_IRQ => InterruptHandlerUsb<USB>;
});

pub const ACC_ADDR: u8 = 0x68;
pub const MAG_ADDR: u8 = 0x0C;

static CHANNEL: Channel<ThreadModeRawMutex, ImuMeasurement, 1> = Channel::new();

static DEVICE_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
static CONFIG_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
static BOS_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();

static STATE: StaticCell<State<'_>> = StaticCell::new();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let driver = Driver::new(p.USB, IrqsUsb);

    let mut config = ConfigUsb::new(0xc0de, 0xcafe);
    config.manufacturer = Some("Embassy");
    config.product = Some("USB-serial example");
    config.serial_number = Some("12345678");
    config.max_power = 100;
    config.max_packet_size_0 = 64;

    // Required for windows compatibility.
    // https://developer.nordicsemi.com/nRF_Connect_SDK/doc/1.9.1/kconfig/CONFIG_CDC_ACM_IAD.html#help
    config.device_class = 0xEF;
    config.device_sub_class = 0x02;
    config.device_protocol = 0x01;
    config.composite_with_iads = true;

    // Create embassy-usb DeviceBuilder using the driver and config.
    // It needs some buffers for building the descriptors.
    let mut builder = Builder::new(
        driver,
        config,
        DEVICE_DESCRIPTOR.init([0; 256]),
        CONFIG_DESCRIPTOR.init([0; 256]),
        BOS_DESCRIPTOR.init([0; 256]),
        &mut [], // no msos descriptors
        CONTROL_BUF.init([0; 64]),
    );

    // Create classes on the builder.
    let class = CdcAcmClass::new(&mut builder, STATE.init(State::new()), 64);

    // Build the builder.
    let usb = builder.build();
    unwrap!(spawner.spawn(usb_task(usb)));
    unwrap!(spawner.spawn(send_measurements_usb_task(class)));

    let sda = p.PIN_16;
    let scl = p.PIN_17;
    let sensor: i2c::I2c<'_, I2C0, Async> =
        i2c::I2c::new_async(p.I2C0, scl, sda, IrqsI2c, ConfigI2c::default());

    let mut mpu_9250 = Mpu9250::new_async(sensor);

    // reset
    mpu_9250.reset().await.unwrap();

    // calibrate
    mpu_9250.calibrate().await.unwrap();

    // Initialize IMU
    mpu_9250.init().await.unwrap();

    // Check health
    if !mpu_9250.check().await.unwrap() {
        crate::panic!("IMU Sensor is not MPU 9250");
    }

    let mut ticket = Ticker::every(Duration::from_millis(1));
    loop {
        let acc = mpu_9250.acc().await.unwrap();
        let gyro = mpu_9250.gyro().await.unwrap();
        let mag = mpu_9250.mag().await.unwrap();
        // info!("acc {} {} {}", acc.x, acc.y, acc.z);
        // info!("gyro {} {} {}", gyro.x, gyro.y, gyro.z);

        let measurements = ImuMeasurement { acc, gyro, mag };
        let _ = CHANNEL.try_send(measurements);

        // if mpu_9250.is_mag_ready().await.unwrap() {
        //     let mag = mpu_9250.mag().await.unwrap();
        //     // info!("mag {} {} {}", mag.x, mag.y, mag.z);
        // }
        ticket.next().await;
    }
}

#[embassy_executor::task]
async fn usb_task(mut usb: UsbDevice<'static, Driver<'static, USB>>) -> ! {
    usb.run().await
}

#[embassy_executor::task]
async fn send_measurements_usb_task(mut class: CdcAcmClass<'static, Driver<'static, USB>>) -> ! {
    let mut buf = [0; 64];
    loop {
        class.wait_connection().await;
        info!("Connected");
        let _n = class.read_packet(&mut buf).await;
        info!("Sending...");
        let _ = send_measurements(&mut class).await;
        info!("Disconnected");
    }
}

struct Disconnected {}

impl From<EndpointError> for Disconnected {
    fn from(val: EndpointError) -> Self {
        match val {
            EndpointError::BufferOverflow => crate::panic!("Buffer overflow"),
            EndpointError::Disabled => Disconnected {},
        }
    }
}

async fn send_measurements<'d, T: Instance + 'd>(
    serial: &mut CdcAcmClass<'d, Driver<'d, T>>,
) -> Result<(), Disconnected> {
    let mut buf = [0; 64];
    loop {
        // Wait for request for measurements
        serial.read_packet(&mut buf).await?;

        // Wait for measurements from sensor
        let measurement = CHANNEL.receive().await;

        // Send measurements over serial port
        let data: [u8; 36] = measurement.into();
        serial.write_packet(&data).await?;
    }
}
