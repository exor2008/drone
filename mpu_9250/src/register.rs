#![allow(dead_code)]

pub const G: f32 = 9.80665;

#[derive(Clone, Copy)]
pub enum Mpu9250Reg {
    SmplrtDiv = 0x19,
    Config = 0x1A,
    GyroConfig = 0x1B,
    AccelConfig = 0x1C,
    AccelConfig2 = 0x1D,
    FifoEn = 0x23,
    I2cMstCtrl = 0x24,
    I2cSlv0Addr = 0x25,
    I2cSlv0Reg = 0x26,
    I2cSlv0Ctrl = 0x27,
    I2cSlv4Ctrl = 0x34,
    IntPinCfg = 0x37,
    IntEnable = 0x38,
    AccelXoutH = 0x3B,
    GyroConfigXoutH = 0x43,
    I2cMstDelayCtrl = 0x67,
    UserCtrl = 0x6A,
    PwrMgmt1 = 0x6B,
    FifoCountH = 0x72,
    FifoRw = 0x74,
    WhoAmI = 0x75,
}

impl Mpu9250Reg {
    pub fn addr(&self) -> u8 {
        *self as u8
    }
}

pub trait Register {
    fn mask(&self) -> u8;
}

pub enum RegPwrMgmt1 {
    Hreset,
    Sleep,
}

impl Register for RegPwrMgmt1 {
    fn mask(&self) -> u8 {
        match *self {
            RegPwrMgmt1::Sleep => 1 << 6,
            RegPwrMgmt1::Hreset => 1 << 7,
        }
    }
}

pub enum RegIntPinCfg {
    BypassEn,
    Actl, // NOTE: Active Low!
}

impl Register for RegIntPinCfg {
    fn mask(&self) -> u8 {
        match *self {
            RegIntPinCfg::BypassEn => 1 << 1,
            RegIntPinCfg::Actl => 1 << 7,
        }
    }
}

pub enum RegConfig {}

#[derive(Copy, Clone, Debug)]
pub enum GyroDlpf {
    Off = 7,
    Hz184 = 1,
    Hz92 = 2,
    Hz41 = 3,
    Hz20 = 4,
    Hz10 = 5,
    Hz5 = 6,
}

/// The full-scale range of the accelerometer.
/// 2G gives the highest sensitivity, but saturates at +-2G.
/// 16G gives the lowest sensitivity, but has the largest range.
#[derive(Copy, Clone, Debug)]
pub enum AccelRange {
    G2,
    G4,
    G8,
    G16,
}

impl Default for AccelRange {
    fn default() -> Self {
        AccelRange::G4
    }
}

impl AccelRange {
    /// Sensitivity is the measurement of the LSB.
    pub fn get_sensitivity_g(&self) -> f32 {
        return match *self {
            AccelRange::G2 => 2.0,
            AccelRange::G4 => 4.0,
            AccelRange::G8 => 8.0,
            AccelRange::G16 => 16.0,
        } / i16::MAX as f32;
    }

    // Returns sensitivity in m/s/s.
    pub fn get_sensitivity_mss(&self) -> f32 {
        // Repeat code from get_sensitivity_g to reduce floating point
        // precision loss from floating point math.
        return match *self {
            AccelRange::G2 => 2.0,
            AccelRange::G4 => 4.0,
            AccelRange::G8 => 8.0,
            AccelRange::G16 => 16.0,
        } * G
            / i16::MAX as f32;
    }
}

/// The full-scale range of the gyroscope.
/// 250 gives the highest resolution, but saturates at +-250 degrees per second.
/// 2000 gives the lowest resolution, but saturates at +-2000dps.
#[derive(Copy, Clone, Debug)]
pub enum GyroRange {
    Dps250,
    Dps500,
    Dps1000,
    Dps2000,
}

impl GyroRange {
    pub fn get_dps(&self) -> f32 {
        return match *self {
            GyroRange::Dps250 => 250.0,
            GyroRange::Dps500 => 500.0,
            GyroRange::Dps1000 => 1000.0,
            GyroRange::Dps2000 => 2000.0,
        } / i16::MAX as f32;
    }
}

impl Default for GyroRange {
    fn default() -> Self {
        GyroRange::Dps1000
    }
}
