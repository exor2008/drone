#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(slice_flatten)]

use ahrs::Mahony;
use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::i2c::{self, Async, Config as ConfigI2c, InterruptHandler as InterruptHandlerI2c};
use embassy_rp::peripherals::I2C0;
use embassy_rp::peripherals::USB;
use embassy_rp::usb::{Driver, Instance, InterruptHandler as InterruptHandlerUsb};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Instant, Ticker, Timer};
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::driver::EndpointError;
use embassy_usb::{Builder, Config as ConfigUsb, UsbDevice};
use micromath::vector::F32x3;
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

const PI_180: f32 = core::f32::consts::PI / 180.0;

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
    Timer::after_millis(200).await;

    // calibrate gyro
    let gyro_offsets = mpu_9250.calibrate_gyro().await.unwrap();
    mpu_9250.set_gyro_offsets(gyro_offsets).await.unwrap();

    // 6 point acc calibration
    // mpu_9250.calibrate_acc_6_point().await.unwrap();

    // Set calibrated acc offsets values
    // Offsets added to factory offsets
    mpu_9250
        .set_acc_offsets(F32x3::from((0.35072445, 0.0, -0.1035959))) //y was 0.15323068
        .await
        .unwrap();

    mpu_9250.set_acc_g(F32x3::from((9.821422, 9.8252588, 9.990623)));

    let acc_offsets = mpu_9250.get_acc_offsets().await.unwrap();
    info!("Accelerometer offsets {}", acc_offsets.to_array());
    info!("Accelerometer G {}", mpu_9250.get_acc_g().to_array());

    // Set predefined offset data
    // mpu_9250
    //     .set_gyro_offsets(F32x3::from((-2.7188249, 2.5960972, -1.4283828)))
    //     .await
    //     .unwrap();

    // Set calibrated mag offsets
    mpu_9250.set_hard_iron(F32x3::from((25.08522, 28.437236, -58.497048)));
    mpu_9250.set_soft_iron(F32x3::from((0.9912555, 1.0275499, 0.982864)));

    // mpu_9250.set_hard_iron(F32x3::from((25.981121, 28.045206, -51.997375)));
    // mpu_9250.set_soft_iron(F32x3::from((1.0853868, 1.1138331, 0.8468341)));

    let soft_iron = mpu_9250.get_soft_iron();
    let hard_iron = mpu_9250.get_hard_iron();
    info!("Magnitometer hard iron offsets: {}", hard_iron.to_array());
    info!("Magnitometer soft iron offsets: {}", soft_iron.to_array());

    // Initialize IMU
    mpu_9250.init().await.unwrap();

    // Check health
    if !mpu_9250.check().await.unwrap() {
        crate::panic!("IMU Sensor is not MPU 9250");
    }

    // Creaate Mahony data fusion
    let mut mahony = Mahony::new(0.4, 0.01);

    let mut ticket = Ticker::every(Duration::from_millis(3));
    let mut timer = Instant::now();
    loop {
        let acc = mpu_9250.acc().await.unwrap();
        let gyro = mpu_9250.gyro().await.unwrap() * PI_180;

        if mpu_9250.is_mag_ready().await.unwrap() {
            let mag = mpu_9250.mag().await.unwrap();
            let dt = timer.elapsed().as_micros() as f32 / 1_000_000.0;
            mahony.update(gyro, acc, mag, dt);
            timer = Instant::now();
            let angle = mahony.quat.to_array();
            let measurements = ImuMeasurement {
                acc,
                gyro,
                mag,
                angle,
            };
            let _ = CHANNEL.try_send(measurements);
        } else {
            let dt = timer.elapsed().as_micros() as f32 / 1_000_000.0;
            mahony.update_imu(gyro, acc, dt);
            timer = Instant::now();
            let angle = mahony.quat.to_array();
            let measurements = ImuMeasurement {
                acc,
                gyro,
                mag: F32x3::default(),
                angle,
            };
            let _ = CHANNEL.try_send(measurements);
        }
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
        info!("Com port connected");
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
        let data: [u8; 52] = measurement.into();
        serial.write_packet(&data).await?;
    }
}
