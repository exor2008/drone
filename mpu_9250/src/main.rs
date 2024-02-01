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
use mpu_9250::Mpu9250;
use panic_probe as _;

bind_interrupts!(struct IrqsI2c {
    I2C0_IRQ => InterruptHandlerI2c<I2C0>;
});

pub const ACC_ADDR: u8 = 0x68;
pub const MAG_ADDR: u8 = 0x0C;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let sda = p.PIN_16;
    let scl = p.PIN_17;
    let sensor: i2c::I2c<'_, I2C0, Async> =
        i2c::I2c::new_async(p.I2C0, scl, sda, IrqsI2c, ConfigI2c::default());

    let mut mpu_9250 = Mpu9250::new_async(sensor);

    // reset
    mpu_9250.reset().await.unwrap();

    // calibrate gyro
    mpu_9250.calibrate_gyro().await.unwrap();

    mpu_9250.calibrate_acc_6_point().await.unwrap();

    // Initialize IMU
    mpu_9250.init().await.unwrap();

    // Check health
    if !mpu_9250.check().await.unwrap() {
        crate::panic!("IMU Sensor is not MPU 9250");
    }
}
