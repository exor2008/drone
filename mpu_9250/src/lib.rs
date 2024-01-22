#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]
#![allow(async_fn_in_trait)]

use defmt::*;
use embassy_rp::i2c::{self, Error, Instance, Mode};
use embassy_time::Timer;
use embedded_hal_async::i2c::I2c;
use micromath::vector::I16x3;

pub const ACC_ADDR: u8 = 0x68;
pub const MAG_ADDR: u8 = 0x0C;

pub trait Accelerometer {
    async fn acc(&mut self) -> Result<I16x3, Error>;
}

pub trait Gyro {
    async fn gyro(&mut self) -> Result<I16x3, Error>;
}

pub trait Magnetometer {
    async fn mag(&mut self) -> Result<I16x3, Error>;
    async fn is_mag_ready(&mut self) -> Result<bool, Error>;
}
pub trait Barometer {
    async fn baro(&mut self) -> Result<f32, Error>;
}

pub struct Mpu9250<'d, T, M>
where
    T: Instance,
    M: Mode,
{
    i2c: i2c::I2c<'d, T, M>,
}

impl<'d, T> Mpu9250<'d, T, i2c::Async>
where
    T: Instance,
{
    pub fn new_async(i2c: i2c::I2c<'d, T, i2c::Async>) -> Self {
        Mpu9250 { i2c }
    }

    pub async fn init(&mut self) -> Result<(), Error> {
        // Sleep off
        self.write(ACC_ADDR, &[0x6B, 0x01]).await?;

        // Sample Rate Divider. Typical values:0x07(125Hz) 1KHz internal sample rate
        self.write(ACC_ADDR, &[0x19, 0x07]).await?;

        // Low pass filter 5 Hz
        self.write(ACC_ADDR, &[0x1A, 0x06]).await?;

        // Gyro Full Scale Select. Typical values:0x10(1000dps)
        self.write(ACC_ADDR, &[0x1B, 0x10]).await?;

        // Accel Full Scale Select. Typical values:0x01(2g)
        self.write(ACC_ADDR, &[0x1C, 0x00]).await?;

        // Bypass mode
        self.write(ACC_ADDR, &[0x37, 0x02]).await?;
        Timer::after_millis(10).await;

        // Single measurement mode
        self.write(MAG_ADDR, &[0x0A, 0x06]).await?;
        Timer::after_millis(10).await;

        Ok(())
    }

    pub async fn check(&mut self) -> Result<bool, Error> {
        let mut buffer = [0u8; 2];
        self.read(ACC_ADDR, &[0x75], &mut buffer[0..1]).await?;

        self.read(MAG_ADDR, &[0x0], &mut buffer[1..2]).await?;

        Ok(buffer[0] == 0x71 && buffer[1] == 0x48)
    }

    pub async fn read(&mut self, addr: u8, data: &[u8], buffer: &mut [u8]) -> Result<(), Error> {
        Ok(self.i2c.write_read(addr, data, buffer).await?)
    }

    pub async fn write(&mut self, addr: u8, data: &[u8]) -> Result<(), Error> {
        Ok(self.i2c.write(addr, data).await?)
    }
}

impl<'d, T> Accelerometer for Mpu9250<'d, T, i2c::Async>
where
    T: Instance,
{
    async fn acc(&mut self) -> Result<I16x3, Error> {
        let mut acc = [0u8; 6];

        // Acc X low
        self.read(ACC_ADDR, &[0x3C], &mut acc[0..1]).await?;

        // Acc X heigh
        self.read(ACC_ADDR, &[0x3B], &mut acc[1..2]).await?;

        // Acc Y low
        self.read(ACC_ADDR, &[0x3E], &mut acc[2..3]).await?;

        // Acc Y heigh
        self.read(ACC_ADDR, &[0x3D], &mut acc[3..4]).await?;

        // Acc Z low
        self.read(ACC_ADDR, &[0x40], &mut acc[4..5]).await?;

        // Acc Z heigh
        self.read(ACC_ADDR, &[0x3F], &mut acc[5..6]).await?;

        let x_acc = i16::from_le_bytes([acc[0], acc[1]]);
        let y_acc = i16::from_le_bytes([acc[2], acc[3]]);
        let z_acc = i16::from_le_bytes([acc[4], acc[5]]);
        Ok(I16x3::from((x_acc, y_acc, z_acc)))
    }
}

impl<'d, T> Gyro for Mpu9250<'d, T, i2c::Async>
where
    T: Instance,
{
    async fn gyro(&mut self) -> Result<I16x3, Error> {
        let mut gyro = [0u8; 6];

        // Gyro X low
        self.read(ACC_ADDR, &[0x44], &mut gyro[0..1]).await?;

        // Gyro X heigh
        self.read(ACC_ADDR, &[0x43], &mut gyro[1..2]).await?;

        // Gyro Y low
        self.read(ACC_ADDR, &[0x46], &mut gyro[2..3]).await?;

        // Gyro Y heigh
        self.read(ACC_ADDR, &[0x45], &mut gyro[3..4]).await?;

        // Gyro Z low
        self.read(ACC_ADDR, &[0x48], &mut gyro[4..5]).await?;

        // Gyro Z heigh
        self.read(ACC_ADDR, &[0x47], &mut gyro[5..6]).await?;

        let x_gyro = i16::from_le_bytes([gyro[0], gyro[1]]);
        let y_gyro = i16::from_le_bytes([gyro[2], gyro[3]]);
        let z_gyro = i16::from_le_bytes([gyro[4], gyro[5]]);
        Ok(I16x3::from((x_gyro, y_gyro, z_gyro)))
    }
}

impl<'d, T> Magnetometer for Mpu9250<'d, T, i2c::Async>
where
    T: Instance,
{
    async fn mag(&mut self) -> Result<I16x3, Error> {
        let mut mag = [0u8; 6];

        // Mag X low
        self.read(MAG_ADDR, &[0x03], &mut mag[0..1]).await?;

        // Mag X heigh
        self.read(MAG_ADDR, &[0x04], &mut mag[1..2]).await?;

        // Mag Y low
        self.read(MAG_ADDR, &[0x05], &mut mag[2..3]).await?;

        // Mag Y heigh
        self.read(MAG_ADDR, &[0x06], &mut mag[3..4]).await?;

        // Mag Z low
        self.read(MAG_ADDR, &[0x07], &mut mag[4..5]).await?;

        // Mag Z heigh
        self.read(MAG_ADDR, &[0x08], &mut mag[5..6]).await?;

        let x_mag = i16::from_le_bytes([mag[0], mag[1]]);
        let y_mag = i16::from_le_bytes([mag[2], mag[3]]);
        let z_mag = i16::from_le_bytes([mag[4], mag[5]]);

        // Reading is finished. Requiered to continue reading data.
        self.read(MAG_ADDR, &[0x09], &mut mag).await?;

        Ok(I16x3::from((x_mag, y_mag, z_mag)))
    }

    async fn is_mag_ready(&mut self) -> Result<bool, Error> {
        let mut buffer = [0u8];
        self.read(MAG_ADDR, &[0x02], &mut buffer).await?;
        Ok(buffer[0] & 0x1 == 0x1) // [0] - Data ready [1] - Data Overrun
    }
}
