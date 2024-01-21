#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

// use defmt::*;
use embassy_rp::i2c::{self, Error, Instance, Mode};
use embedded_hal_async::i2c::I2c;
use micromath::vector::{F32x3, I16x3};

pub const DEV_ADDR: u8 = 0x68;

#[allow(async_fn_in_trait)]
pub trait SensorAsync {
    type Measurements;
    async fn measure(&mut self) -> Result<Self::Measurements, Error>;
}
pub trait Accelerometer {
    fn acc(&self) -> I16x3;
}

pub trait Gyro {
    fn gyro(&self) -> F32x3;
}

pub trait Magnetometer {
    fn mag(&self) -> F32x3;
}
pub trait Barometer {
    fn baro(&self) -> f32;
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
        self.i2c.write(DEV_ADDR, &[0x6B, 0x01]).await?; // Sleep off
        self.i2c.write(DEV_ADDR, &[0x19, 0x07]).await?; // Sample Rate Divider. Typical values:0x07(125Hz) 1KHz internal sample rate
        self.i2c.write(DEV_ADDR, &[0x1A, 0x06]).await?; // Low pass filter 5 Hz
        self.i2c.write(DEV_ADDR, &[0x1B, 0x10]).await?; // Gyro Full Scale Select. Typical values:0x10(1000dps)
        self.i2c.write(DEV_ADDR, &[0x1C, 0x00]).await?; // Accel Full Scale Select. Typical values:0x01(2g)
        Ok(())
    }

    pub async fn check(&mut self) -> Result<bool, Error> {
        let mut buffer = [0u8];
        self.i2c
            .write_read(DEV_ADDR, &[0x75], &mut buffer)
            .await
            .unwrap();

        Ok(buffer[0] == 0x71)
    }
}

impl<'d, T> SensorAsync for Mpu9250<'d, T, i2c::Async>
where
    T: Instance,
{
    type Measurements = impl Accelerometer + Gyro + Magnetometer + Barometer;
    async fn measure(&mut self) -> Result<Self::Measurements, Error> {
        let mut acc = [0u8; 6];

        // X low
        self.i2c
            .write_read(DEV_ADDR, &[0x3C], &mut acc[0..1])
            .await?;

        // X heigh
        self.i2c
            .write_read(DEV_ADDR, &[0x3B], &mut acc[1..2])
            .await?;

        // Y low
        self.i2c
            .write_read(DEV_ADDR, &[0x3E], &mut acc[2..3])
            .await?;

        // Y heigh
        self.i2c
            .write_read(DEV_ADDR, &[0x3D], &mut acc[3..4])
            .await?;

        // Z low
        self.i2c
            .write_read(DEV_ADDR, &[0x40], &mut acc[4..5])
            .await?;

        // Z heigh
        self.i2c
            .write_read(DEV_ADDR, &[0x3F], &mut acc[5..6])
            .await?;

        Ok(Mpu9250Measurement {
            acc,
            gyro: F32x3::default(),
            mag: F32x3::default(),
            baro: 0.0,
        })
    }
}

#[derive(Default)]
struct Mpu9250Measurement {
    acc: [u8; 6],
    gyro: F32x3,
    mag: F32x3,
    baro: f32,
}

impl Accelerometer for Mpu9250Measurement {
    fn acc(&self) -> I16x3 {
        let x_acc = i16::from_le_bytes([self.acc[0], self.acc[1]]);
        let y_acc = i16::from_le_bytes([self.acc[2], self.acc[3]]);
        let z_acc = i16::from_le_bytes([self.acc[4], self.acc[5]]);
        I16x3::from((x_acc, y_acc, z_acc))
    }
}

impl Gyro for Mpu9250Measurement {
    fn gyro(&self) -> F32x3 {
        self.gyro
    }
}

impl Magnetometer for Mpu9250Measurement {
    fn mag(&self) -> F32x3 {
        self.mag
    }
}

impl Barometer for Mpu9250Measurement {
    fn baro(&self) -> f32 {
        self.baro
    }
}