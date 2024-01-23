#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(slice_flatten)]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
// use embassy_rp::gpio::{Input, Level, Output};
use embassy_rp::i2c::{self, Async, Config, InterruptHandler};
use embassy_rp::peripherals::I2C0;
use embassy_time::{Duration, Ticker};
use mpu_9250::{Accelerometer, Barometer, Gyro, Magnetometer, Mpu9250};
use panic_probe as _;

bind_interrupts!(struct Irqs {
    I2C0_IRQ => InterruptHandler<I2C0>;
});

pub const ACC_ADDR: u8 = 0x68;
pub const MAG_ADDR: u8 = 0x0C;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let sda = p.PIN_16;
    let scl = p.PIN_17;
    let sensor: i2c::I2c<'_, I2C0, Async> =
        i2c::I2c::new_async(p.I2C0, scl, sda, Irqs, Config::default());

    let mut mpu_9250 = Mpu9250::new_async(sensor);

    // Initialize device
    mpu_9250.init().await.unwrap();

    // Check health
    if !mpu_9250.check().await.unwrap() {
        crate::panic!("IMU Sensor is not MPU 9250");
    }

    mpu_9250.set_sample_rate(125).await.unwrap(); // Hz

    let mut ticket = Ticker::every(Duration::from_millis(1));
    loop {
        let acc = mpu_9250.acc().await.unwrap();
        let gyro = mpu_9250.gyro().await.unwrap();
        info!("acc {} {} {}", acc.x, acc.y, acc.z);
        info!("gyro {} {} {}", gyro.x, gyro.y, gyro.z);

        if mpu_9250.is_mag_ready().await.unwrap() {
            let mag = mpu_9250.mag().await.unwrap();
            info!("mag {} {} {}", mag.x, mag.y, mag.z);
        }
        ticket.next().await;
    }
}
