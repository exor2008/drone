#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]
#![allow(async_fn_in_trait)]

use embassy_rp::i2c::{self, Error, Instance, Mode};
use embedded_hal_async::i2c::I2c;
use micromath::vector::{F32x3, I16x3};

mod register;
use register::*;

pub const ACC_ADDR: u8 = 0x68;
pub const MAG_ADDR: u8 = 0x0C;

pub trait Accelerometer {
    async fn acc(&mut self) -> Result<F32x3, Error>;
}

pub trait Gyro {
    async fn gyro(&mut self) -> Result<F32x3, Error>;
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
    gyro_range: GyroRange,
    acc_range: AccelRange,
}

impl<'d, T> Mpu9250<'d, T, i2c::Async>
where
    T: Instance,
{
    pub fn new_async(i2c: i2c::I2c<'d, T, i2c::Async>) -> Self {
        Mpu9250 {
            i2c,
            gyro_range: GyroRange::default(),
            acc_range: AccelRange::default(),
        }
    }

    pub async fn init(&mut self) -> Result<(), Error> {
        // Reset
        self.write(
            ACC_ADDR,
            &[Mpu9250Reg::PwrMgmt1.addr(), RegPwrMgmt1::Hreset.mask()],
        )
        .await?;

        // Low pass filter 5 Hz
        self.set_low_pass_filter(GyroDlpf::Hz5).await?;

        // Gyro Full Scale Select. Typical values:0x10(1000dps)
        self.set_gyro_range(self.gyro_range).await?;

        // Accel Full Scale Select. Typical values:0x02(4g)
        self.set_accel_range(self.acc_range).await?;

        // Bypass mode
        self.enable_bypass().await?;

        // Single measurement mode
        self.write(MAG_ADDR, &[0x0A, 0x06]).await?;

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

    pub async fn set_sample_rate(&mut self, rate: u16) -> Result<(), Error> {
        if rate > 1000 || rate < 4 {
            panic!("Error: Invalid sample rate.");
        }

        // Derived from: Sample Rate = Internal Sample Rate / (1 + SMPLRT_DIV)
        let smplrt_div = (1000 / rate) as u8 - 1;
        self.i2c
            .write(ACC_ADDR, &[Mpu9250Reg::SmplrtDiv.addr(), smplrt_div])
            .await?;
        Ok(())
    }

    pub async fn enable_bypass(&mut self) -> Result<(), Error> {
        //Reset gyro
        self.write(ACC_ADDR, &[Mpu9250Reg::UserCtrl.addr(), 0])
            .await?;

        // Enable bypass
        self.write(
            ACC_ADDR,
            &[
                Mpu9250Reg::IntPinCfg.addr(),
                RegIntPinCfg::Actl.mask() | RegIntPinCfg::BypassEn.mask(),
            ],
        )
        .await?;

        Ok(())
    }

    pub async fn set_low_pass_filter(&mut self, level: GyroDlpf) -> Result<(), Error> {
        self.write(ACC_ADDR, &[Mpu9250Reg::Config.addr(), level as u8])
            .await?;
        Ok(())
    }

    pub async fn set_accel_range(&mut self, acc_range: AccelRange) -> Result<(), Error> {
        let accel_config_byte: u8 = match acc_range {
            AccelRange::G2 => 0,
            AccelRange::G4 => 1,
            AccelRange::G8 => 2,
            AccelRange::G16 => 3,
        } << 3;
        self.write(
            ACC_ADDR,
            &[Mpu9250Reg::AccelConfig.addr(), accel_config_byte],
        )
        .await?;
        self.acc_range = acc_range;
        Ok(())
    }

    pub async fn set_gyro_range(&mut self, gyro_range: GyroRange) -> Result<(), Error> {
        let gyro_config_byte: u8 = match gyro_range {
            GyroRange::Dps250 => 0,
            GyroRange::Dps500 => 1,
            GyroRange::Dps1000 => 2,
            GyroRange::Dps2000 => 3,
        } << 3;
        self.write(ACC_ADDR, &[Mpu9250Reg::GyroConfig.addr(), gyro_config_byte])
            .await?;
        self.gyro_range = gyro_range;
        Ok(())
    }
}

impl<'d, T> Accelerometer for Mpu9250<'d, T, i2c::Async>
where
    T: Instance,
{
    async fn acc(&mut self) -> Result<F32x3, Error> {
        let mut acc = [0u8; 6];

        self.read(ACC_ADDR, &[Mpu9250Reg::AccelXoutH.addr()], &mut acc)
            .await?;

        let sensetivity = self.acc_range.get_sensitivity_mss();
        let x_acc = i16::from_be_bytes(acc[0..2].try_into().expect("Slice with incorrect length"));
        let y_acc = i16::from_be_bytes(acc[2..4].try_into().expect("Slice with incorrect length"));
        let z_acc = i16::from_be_bytes(acc[4..6].try_into().expect("Slice with incorrect length"));

        let x_acc = sensetivity * x_acc as f32;
        let y_acc = sensetivity * y_acc as f32;
        let z_acc = sensetivity * z_acc as f32;

        Ok(F32x3::from((x_acc, y_acc, z_acc)))
    }
}

impl<'d, T> Gyro for Mpu9250<'d, T, i2c::Async>
where
    T: Instance,
{
    async fn gyro(&mut self) -> Result<F32x3, Error> {
        let mut gyro = [0u8; 6];

        // Gyro X low
        self.read(ACC_ADDR, &[Mpu9250Reg::GyroConfigXoutH.addr()], &mut gyro)
            .await?;

        let dps = self.gyro_range.get_dps();
        let x_gyro =
            i16::from_be_bytes(gyro[0..2].try_into().expect("Slice with incorrect length"));
        let y_gyro =
            i16::from_be_bytes(gyro[2..4].try_into().expect("Slice with incorrect length"));
        let z_gyro =
            i16::from_be_bytes(gyro[4..6].try_into().expect("Slice with incorrect length"));

        let x_gyro = dps * x_gyro as f32;
        let y_gyro = dps * y_gyro as f32;
        let z_gyro = dps * z_gyro as f32;

        Ok(F32x3::from((x_gyro, y_gyro, z_gyro)))
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
