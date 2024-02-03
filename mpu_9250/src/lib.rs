#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]
#![allow(async_fn_in_trait)]

use core::mem::{size_of, transmute};
use defmt::info;
use embassy_rp::i2c::{self, Error, Instance, Mode};
use embassy_time::{Duration, Ticker, Timer};
use embedded_hal_async::i2c::I2c;
use micromath::vector::{F32x3, I16x3};

mod register;
use register::*;

const ACC_ADDR: u8 = 0x68;
const MAG_ADDR: u8 = 0x0C;
const CALLIBRATION_SAMPLES: u16 = 2000;
const CALLIBRATION_MAG_SAMPLES: u16 = 6000;

pub trait Accelerometer {
    async fn acc(&mut self) -> Result<F32x3, Error>;
}

pub trait Gyro {
    async fn gyro(&mut self) -> Result<F32x3, Error>;
}

pub trait Magnetometer {
    async fn mag(&mut self) -> Result<F32x3, Error>;
    async fn is_mag_ready(&mut self) -> Result<bool, Error>;
}
pub trait Barometer {
    async fn baro(&mut self) -> Result<f32, Error>;
}

#[derive(Debug, Clone, Copy)]
pub struct ImuMeasurement {
    pub acc: F32x3,
    pub gyro: F32x3,
    pub mag: F32x3,
    pub angle: [f32; 4],
}

type Float3 = [u8; 3 * size_of::<f32>()];
type Float4 = [u8; 4 * size_of::<f32>()];

impl Into<[u8; 52]> for ImuMeasurement {
    fn into(self) -> [u8; 52] {
        let mut data = [0u8; 52];
        let acc = self.acc.to_array();
        let gyro = self.gyro.to_array();
        let mag = self.mag.to_array();

        let mut buff3f: &Float3;
        let buff4f: &Float4;

        unsafe {
            buff3f = transmute::<&[f32; 3], &Float3>(&acc);
        };
        let target = &mut data[0..12];
        target.copy_from_slice(buff3f);

        unsafe {
            buff3f = transmute::<&[f32; 3], &Float3>(&gyro);
        };
        let target = &mut data[12..24];
        target.copy_from_slice(buff3f);

        unsafe {
            buff3f = transmute::<&[f32; 3], &Float3>(&mag);
        };
        let target = &mut data[24..36];
        target.copy_from_slice(buff3f);

        unsafe {
            buff4f = transmute::<&[f32; 4], &Float4>(&self.angle);
        };
        let target = &mut data[36..52];
        target.copy_from_slice(buff4f);

        data
    }
}

pub struct Mpu9250<'d, T, M>
where
    T: Instance,
    M: Mode,
{
    i2c: i2c::I2c<'d, T, M>,
    gyro_range: GyroRange,
    acc_range: AccelRange,
    mag_factory_adjust: F32x3,
    mag_sensitivity: Sensitivity,
    acc_g: F32x3,
    mag_hard_iron: F32x3,
    mag_soft_iron: F32x3,
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
            mag_factory_adjust: F32x3::default(),
            mag_sensitivity: Sensitivity::default(),
            acc_g: F32x3::from((9.8, 9.8, 9.8)),
            mag_hard_iron: F32x3::default(),
            mag_soft_iron: F32x3::from((1.0, 1.0, 1.0)),
        }
    }

    pub async fn init(&mut self) -> Result<(), Error> {
        // Low pass filter 5 Hz
        self.set_low_pass_filter(GyroDlpf::Hz5).await?;

        // Gyro Full Scale Select. Typical values:0x10(1000dps)
        self.set_gyro_range(self.gyro_range).await?;

        // Accel Full Scale Select. Typical values:0x02(4g)
        self.set_accel_range(self.acc_range).await?;

        // Sample rate 125 Hz
        self.set_sample_rate(333).await?;

        // Bypass mode
        self.enable_bypass().await?;

        // Get factory sensitivity adjustment values from Fuse ROM.
        self.mag_factory_adjust = self.read_sensitivity_adjustment().await?;

        // Set magnetometer sensitivity and 100 Hz sample rate
        self.config_mag(SampleRate::Hz100, Sensitivity::Bit16)
            .await?;

        info!("Sensor initialized");

        Ok(())
    }

    pub async fn calibrate_gyro(&mut self) -> Result<F32x3, Error> {
        // Prepare device for calibration
        // Low pass filter 184 Hz
        self.set_low_pass_filter(GyroDlpf::Hz184).await?;

        // Most sensetive mag mode
        self.set_gyro_range(GyroRange::Dps250).await?;

        // Max rate 1000 Hz
        self.set_sample_rate(1000).await?;

        let mut gyro = F32x3::default();

        info!("Calibrating gyro...");
        let mut timer = Ticker::every(Duration::from_millis(1));
        for _ in 0..CALLIBRATION_SAMPLES {
            gyro += self.gyro().await?;
            timer.next().await;
        }

        gyro = gyro * (1.0 / CALLIBRATION_SAMPLES as f32);
        info!("Calibrating done.");
        info!("Gyro offset (m/s) {}", gyro.to_array());

        Ok(gyro)
    }

    pub async fn calibrate_acc_6_point(&mut self) -> Result<(F32x3, F32x3), Error> {
        // Prepare device for calibration
        // Low pass filter 184 Hz
        self.set_low_pass_filter(GyroDlpf::Hz184).await?;

        // Most sensetive accel mode
        self.set_accel_range(AccelRange::G2).await?;

        // Max rate 1000 Hz
        self.set_sample_rate(1000).await?;

        info!("Calibrating accelerometer...");
        info!("Place Z up");
        Timer::after_secs(8).await;
        info!("Collecting...");
        let z_up = self.collect_acc().await?;

        info!("Place Z down");
        Timer::after_secs(8).await;
        info!("Collecting...");
        let z_down = self.collect_acc().await?;

        info!("Place X up");
        Timer::after_secs(8).await;
        info!("Collecting...");
        let x_up = self.collect_acc().await?;

        info!("Place X down");
        Timer::after_secs(8).await;
        info!("Collecting...");
        let x_down = self.collect_acc().await?;

        info!("Place Y up");
        Timer::after_secs(8).await;
        info!("Collecting...");
        let y_up = self.collect_acc().await?;

        info!("Place Y down");
        Timer::after_secs(8).await;
        info!("Collecting...");
        let y_down = self.collect_acc().await?;

        info!("Z up: {}", z_up.to_array());
        info!("Z down: {}", z_down.to_array());
        info!("X up: {}", x_up.to_array());
        info!("X down: {}", x_down.to_array());
        info!("Y up: {}", y_up.to_array());
        info!("Y down: {}", y_down.to_array());

        let x_offset = (y_up.x + y_down.x + z_up.x + z_down.x) * 0.25;
        let y_offset = (x_up.y + x_down.y + z_up.y + z_down.y) * 0.25;
        let z_offset = (x_up.z + x_down.z + y_up.z + y_down.z) * 0.25;

        let x_scale = (x_up.x - x_down.x) * 0.5;
        let y_scale = (y_up.y - y_down.y) * 0.5;
        let z_scale = (z_up.z - z_down.z) * 0.5;

        let acc_offset = F32x3::from((x_offset, y_offset, z_offset));
        let acc_g = F32x3::from((x_scale, y_scale, z_scale));

        info!(
            "6 point acc offset: {}, scale: {}",
            acc_offset.to_array(),
            acc_g.to_array()
        );

        // self.set_acc_offsets(acc).await?;

        Ok((acc_offset, acc_g))
    }

    pub async fn calibrate_mag(&mut self) -> Result<(F32x3, F32x3), Error> {
        info!("Calibrating magnitometer...");
        // Bypass mode
        self.enable_bypass().await?;

        // Set magnetometer sensitivity and 100 Hz sample rate
        self.config_mag(SampleRate::Hz100, Sensitivity::Bit16)
            .await?;

        info!("Rotate device over each degree for ~60 seconds.");
        info!("Collecting...");

        let mut timer = Ticker::every(Duration::from_millis(10));

        let mut max = F32x3::from((f32::MIN, f32::MIN, f32::MIN));
        let mut min = F32x3::from((f32::MAX, f32::MAX, f32::MAX));

        for _ in 0..CALLIBRATION_MAG_SAMPLES {
            let mag = self.mag().await?;

            max.x = mag.x.max(max.x);
            max.y = mag.y.max(max.y);
            max.z = mag.z.max(max.z);

            min.x = mag.x.min(min.x);
            min.y = mag.y.min(min.y);
            min.z = mag.z.min(min.z);

            timer.next().await;
        }

        let offset_x = (max.x + min.x) * 0.5;
        let offset_y = (max.y + min.y) * 0.5;
        let offset_z = (max.z + min.z) * 0.5;

        let offset = F32x3::from((offset_x, offset_y, offset_z));

        let avg_delta_x = (max.x - min.x) * 0.5;
        let avg_delta_y = (max.y - min.y) * 0.5;
        let avg_delta_z = (max.z - min.z) * 0.5;

        let avg_delta = (avg_delta_x + avg_delta_y + avg_delta_z) * 0.3333333333333333;

        let scale_x = avg_delta / avg_delta_x;
        let scale_y = avg_delta / avg_delta_y;
        let scale_z = avg_delta / avg_delta_z;

        let scale = F32x3::from((scale_x, scale_y, scale_z));

        info!("Calibration finished");
        info!("Hard iron offsets: {}", offset.to_array());
        info!("Soft iron scale: {}", scale.to_array());
        Ok((offset, scale))
    }

    async fn collect_acc(&mut self) -> Result<F32x3, Error> {
        let mut acc = F32x3::default();

        let mut timer = Ticker::every(Duration::from_millis(1));
        for _ in 0..CALLIBRATION_SAMPLES {
            acc += self.acc().await?;
            timer.next().await;
        }

        acc = acc * (1.0 / CALLIBRATION_SAMPLES as f32);
        Ok(acc)
    }
}

impl<'d, T> Mpu9250<'d, T, i2c::Async>
where
    T: Instance,
{
    pub async fn read(&mut self, addr: u8, data: &[u8], buffer: &mut [u8]) -> Result<(), Error> {
        Ok(self.i2c.write_read(addr, data, buffer).await?)
    }

    pub async fn write(&mut self, addr: u8, data: &[u8]) -> Result<(), Error> {
        Ok(self.i2c.write(addr, data).await?)
    }
}

impl<'d, T> Mpu9250<'d, T, i2c::Async>
where
    T: Instance,
{
    pub async fn reset(&mut self) -> Result<(), Error> {
        self.write(
            ACC_ADDR,
            &[Mpu9250Reg::PwrMgmt1.addr(), RegPwrMgmt1::Hreset.mask()],
        )
        .await?;
        Ok(())
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

    pub async fn read_sensitivity_adjustment(&mut self) -> Result<F32x3, Error> {
        // Power down mag before mode switch
        self.write(
            MAG_ADDR,
            &[Ak8963Reg::Cntl1.addr(), RegCntl1::PowerDn.mask()],
        )
        .await?;
        Timer::after_micros(100).await;

        // Enter FUSE ROM mode
        self.write(
            MAG_ADDR,
            &[Ak8963Reg::Cntl1.addr(), RegCntl1::FuseRom.mask()],
        )
        .await?;
        Timer::after_millis(1).await;

        // Read sensitivity values from ROM
        let mut buf: [u8; 3] = [0u8; 3];
        self.read(MAG_ADDR, &[Ak8963Reg::Asax.addr()], &mut buf)
            .await?;

        let factory_adjust = F32x3::from_iter(buf.map(|v| ((v - 128) as f32) / 256.0 + 1.0));
        info!(
            "Magnitometer factory adjustments: {}",
            factory_adjust.to_array()
        );

        Ok(factory_adjust)
    }

    pub async fn config_mag(
        &mut self,
        sample_rate: SampleRate,
        sensitivity: Sensitivity,
    ) -> Result<(), Error> {
        let mut cntl1_byte = 0u8;

        match sensitivity {
            Sensitivity::Bit14 => {}
            Sensitivity::Bit16 => cntl1_byte |= RegCntl1::Sensitivity16bit.mask(),
        }
        self.mag_sensitivity = sensitivity;

        match sample_rate {
            SampleRate::Hz8 => cntl1_byte |= RegCntl1::ContMeas1.mask(),
            SampleRate::Hz100 => cntl1_byte |= RegCntl1::ContMeas2.mask(),
        }

        // Power down mag before mode switch
        self.write(
            MAG_ADDR,
            &[Ak8963Reg::Cntl1.addr(), RegCntl1::PowerDn.mask()],
        )
        .await?;
        Timer::after_micros(100).await;

        self.write(MAG_ADDR, &[Ak8963Reg::Cntl1.addr(), cntl1_byte])
            .await?;
        Ok(())
    }

    pub async fn check(&mut self) -> Result<bool, Error> {
        let mut buffer = [0u8; 2];
        self.read(ACC_ADDR, &[0x75], &mut buffer[0..1]).await?;

        self.read(MAG_ADDR, &[0x0], &mut buffer[1..2]).await?;

        Ok(buffer[0] == 0x71 && buffer[1] == 0x48)
    }

    pub async fn set_gyro_offsets(&mut self, offsets: F32x3) -> Result<(), Error> {
        // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias
        // input format.
        // Biases are additive, so change sign on
        // calculated average gyro biases
        let dps = self.gyro_range.get_dps();

        let offsets = offsets * -0.25 * (1.0 / dps);

        let x_gyro: [u8; 2] = (offsets.x as i16).to_be_bytes();
        let y_gyro: [u8; 2] = (offsets.y as i16).to_be_bytes();
        let z_gyro: [u8; 2] = (offsets.z as i16).to_be_bytes();

        let data = [
            Mpu9250Reg::XgOffsetH.addr(),
            x_gyro[0],
            x_gyro[1],
            y_gyro[0],
            y_gyro[1],
            z_gyro[0],
            z_gyro[1],
        ];

        self.write(ACC_ADDR, &data).await?;
        Ok(())
    }

    pub async fn get_unscaled_gyro_offsets(&mut self) -> Result<I16x3, Error> {
        let mut gyro = [0u8; 6];

        self.read(ACC_ADDR, &[Mpu9250Reg::XgOffsetH.addr()], &mut gyro)
            .await?;

        let x_gyro =
            i16::from_be_bytes(gyro[0..2].try_into().expect("Slice with incorrect length"));
        let y_gyro =
            i16::from_be_bytes(gyro[2..4].try_into().expect("Slice with incorrect length"));
        let z_gyro =
            i16::from_be_bytes(gyro[4..6].try_into().expect("Slice with incorrect length"));

        Ok(I16x3::from((x_gyro, y_gyro, z_gyro)))
    }

    pub async fn get_unscaled_acc_offsets(&mut self) -> Result<I16x3, Error> {
        let mut acc = [0u8; 6];

        self.read(ACC_ADDR, &[Mpu9250Reg::XaOffsetH.addr()], &mut acc)
            .await?;

        let x_acc = i16::from_be_bytes(acc[0..2].try_into().expect("Slice with incorrect length"));
        let y_acc = i16::from_be_bytes(acc[2..4].try_into().expect("Slice with incorrect length"));
        let z_acc = i16::from_be_bytes(acc[4..6].try_into().expect("Slice with incorrect length"));

        Ok(I16x3::from((x_acc, y_acc, z_acc)))
    }

    pub async fn get_acc_offsets(&mut self) -> Result<F32x3, Error> {
        let offsets = F32x3::from(self.get_unscaled_acc_offsets().await?);
        let scale = self.acc_g * AccelRange::G16.get_sensitivity_g(); //self.acc_range.get_sensitivity_g();

        let x = offsets.x * scale.x;
        let y = offsets.y * scale.y;
        let z = offsets.z * scale.z;

        Ok(F32x3::from((x, y, z)))
    }

    pub async fn set_acc_offsets(&mut self, offsets: F32x3) -> Result<(), Error> {
        let factory = self.get_unscaled_acc_offsets().await?;
        let scale = self.acc_g * AccelRange::G16.get_sensitivity_g();

        let offset_x = (offsets.x / scale.x) as i16;
        let offset_y = (offsets.y / scale.y) as i16;
        let offset_z = (offsets.z / scale.z) as i16;

        let offsets = I16x3::from((offset_x, offset_y, offset_z));
        let offsets = factory - offsets;

        let x_acc: [u8; 2] = offsets.x.to_be_bytes();
        let y_acc: [u8; 2] = offsets.y.to_be_bytes();
        let z_acc: [u8; 2] = offsets.z.to_be_bytes();

        let data = [
            Mpu9250Reg::XaOffsetH.addr(),
            x_acc[0],
            x_acc[1],
            y_acc[0],
            y_acc[1],
            z_acc[0],
            z_acc[1],
        ];
        self.write(ACC_ADDR, &data).await?;

        Ok(())
    }

    pub fn set_acc_g(&mut self, g: F32x3) {
        self.acc_g = g;
    }

    pub fn get_acc_g(&mut self) -> F32x3 {
        self.acc_g
    }

    pub fn set_hard_iron(&mut self, hard_iron: F32x3) {
        self.mag_hard_iron = hard_iron
    }

    pub fn set_soft_iron(&mut self, soft_iron: F32x3) {
        self.mag_soft_iron = soft_iron
    }

    pub fn get_hard_iron(&mut self) -> F32x3 {
        self.mag_hard_iron
    }

    pub fn get_soft_iron(&mut self) -> F32x3 {
        self.mag_soft_iron
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

        let sensetivity = self.acc_g * self.acc_range.get_sensitivity_g();
        let x_acc = i16::from_be_bytes(acc[0..2].try_into().expect("Slice with incorrect length"));
        let y_acc = i16::from_be_bytes(acc[2..4].try_into().expect("Slice with incorrect length"));
        let z_acc = i16::from_be_bytes(acc[4..6].try_into().expect("Slice with incorrect length"));

        let acc = F32x3::from((
            sensetivity.x * x_acc as f32,
            sensetivity.y * y_acc as f32,
            sensetivity.z * z_acc as f32,
        ));

        Ok(acc)
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

        let scale = self.gyro_range.get_dps();
        let x_gyro =
            i16::from_be_bytes(gyro[0..2].try_into().expect("Slice with incorrect length"));
        let y_gyro =
            i16::from_be_bytes(gyro[2..4].try_into().expect("Slice with incorrect length"));
        let z_gyro =
            i16::from_be_bytes(gyro[4..6].try_into().expect("Slice with incorrect length"));

        let x_gyro = scale * x_gyro as f32;
        let y_gyro = scale * y_gyro as f32;
        let z_gyro = scale * z_gyro as f32;

        Ok(F32x3::from((x_gyro, y_gyro, z_gyro)))
    }
}

impl<'d, T> Magnetometer for Mpu9250<'d, T, i2c::Async>
where
    T: Instance,
{
    async fn mag(&mut self) -> Result<F32x3, Error> {
        let mut mag = [0u8; 6];

        // Mag X low
        self.read(MAG_ADDR, &[0x03], &mut mag).await?;

        let x_mag = i16::from_le_bytes(mag[0..2].try_into().expect("Slice with incorrect length"));
        let y_mag = i16::from_le_bytes(mag[2..4].try_into().expect("Slice with incorrect length"));
        let z_mag = i16::from_le_bytes(mag[4..6].try_into().expect("Slice with incorrect length"));

        // Reading is finished. Requiered to continue reading data.
        self.read(MAG_ADDR, &[Ak8963Reg::St2.addr()], &mut mag)
            .await?;

        let mag_sens = self.mag_sensitivity.get_sensetivity();

        let [x_adjust, y_adjust, z_adjust] = self.mag_factory_adjust.to_array();

        let x_mag = x_mag as f32 * x_adjust * mag_sens;
        let y_mag = y_mag as f32 * y_adjust * mag_sens;
        let z_mag = z_mag as f32 * z_adjust * mag_sens;

        // Apply calibration
        let x_mag = (x_mag - self.mag_hard_iron.x) * self.mag_soft_iron.x;
        let y_mag = (y_mag - self.mag_hard_iron.y) * self.mag_soft_iron.y;
        let z_mag = (z_mag - self.mag_hard_iron.z) * self.mag_soft_iron.z;

        Ok(F32x3::from((x_mag, y_mag, z_mag)))
    }

    async fn is_mag_ready(&mut self) -> Result<bool, Error> {
        // Check if new measurements are ready to read
        // [0] - Data ready [1] - Data Overrun
        let mut buffer = [0u8];
        self.read(MAG_ADDR, &[Ak8963Reg::St1.addr()], &mut buffer)
            .await?;
        Ok(buffer[0] & 0x1 == 0x1)
    }
}
