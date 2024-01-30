// Taken from https://github.com/jmagnuson/ahrs-rs/tree/master
#![no_std]
#![no_main]

use micromath::{
    vector::{F32x2, F32x3, Vector, Vector2d},
    Quaternion,
};

#[derive(Debug)]
pub struct Mahony {
    /// Expected sampling period, in seconds.
    sample_period: f32,
    /// Proportional filter gain constant.
    kp: f32,
    /// Integral filter gain constant.
    ki: f32,
    /// Integral error vector.
    e_int: F32x3,
    /// Filter state quaternion.
    pub quat: Quaternion,
}

impl Default for Mahony {
    fn default() -> Mahony {
        Mahony {
            sample_period: (1.0) / (1000.0),
            kp: 0.5,
            ki: 0.0,
            e_int: F32x3::default(),
            quat: Quaternion::default(),
        }
    }
}

impl Mahony {
    pub fn new(sample_period: f32, kp: f32, ki: f32) -> Self {
        Mahony {
            sample_period,
            kp,
            ki,
            e_int: F32x3::default(),
            quat: Quaternion::default(),
        }
    }
}

impl Mahony {
    pub fn update(&mut self, gyroscope: F32x3, accelerometer: F32x3, magnetometer: F32x3) {
        let q = &self.quat;

        // Normalize accelerometer measurement
        let acc_mag = accelerometer.magnitude();
        let acc = if acc_mag == 0.0 {
            F32x3::default()
        } else {
            accelerometer * (1.0 / acc_mag)
        };

        // Normalize magnetometer measurement
        let mag_mag = magnetometer.magnitude();
        let mag = if mag_mag == 0.0 {
            F32x3::default()
        } else {
            magnetometer * (1.0 / mag_mag)
        };

        // Reference direction of Earth's magnetic field (Quaternion should still be conj of q)
        let h = *q * (Quaternion::new(0.0, mag.x, mag.y, mag.z) * q.conj());
        let b = Quaternion::new(0.0, Vector2d::from((h.w(), h.x())).magnitude(), 0.0, h.y());

        let v = F32x3::from((
            2.0 * (q.w() * q.y() - q.z() * q.x()),
            2.0 * (q.z() * q.w() + q.x() * q.y()),
            (q.z() * q.z() - q.w() * q.w() - q.x() * q.x() + q.y() * q.y()),
        ));

        #[rustfmt::skip]
        let w = F32x3::from((
            2.0 * b.x() * (0.5 - q.x() * q.x() - q.y() * q.y()) + 2.0 * b.z() * (q.w() * q.y() - q.z() * q.x()),
            2.0 * b.x() * (q.w() * q.x() - q.z() * q.y())       + 2.0 * b.z() * (q.z() * q.w() + q.x() * q.y()),
            2.0 * b.x() * (q.z() * q.x() + q.w() * q.y())       + 2.0 * b.z() * (0.5 - q.w() * q.w() - q.x() * q.x()),
        ));

        // cross(acc, v) + cross(mag, w)
        let e = acc * v + mag * w;

        // Error is sum of cross product between estimated direction and measured direction of fields
        if self.ki > 0.0 {
            self.e_int += e * self.sample_period;
        } else {
            self.e_int = F32x3::default()
        }

        // Apply feedback terms
        let gyro = gyroscope + e * self.kp + self.e_int * self.ki;

        // Compute rate of change of quaternion
        let q_dot = self.quat * Quaternion::new(0.0, gyro.x, gyro.y, gyro.z) * 0.5;

        // Integrate to yield quaternion
        self.quat = (self.quat + q_dot * self.sample_period).normalize();
    }

    pub fn update_imu(&mut self, gyroscope: F32x3, accelerometer: F32x3) {
        let q = &self.quat;

        // Normalize accelerometer measurement
        let acc_mag = accelerometer.magnitude();
        let acc = if acc_mag == 0.0 {
            F32x3::default()
        } else {
            accelerometer * (1.0 / acc_mag)
        };

        // #[rustfmt::skip]
        let v = F32x3::from((
            2.0 * (q.w() * q.y() - q.z() * q.x()),
            2.0 * (q.z() * q.w() + q.x() * q.y()),
            q.z() * q.z() - q.w() * q.w() - q.x() * q.x() + q.y() * q.y(),
        ));

        // cross(acc, v)
        let e = acc * v;

        // Error is sum of cross product between estimated direction and measured direction of fields
        if self.ki > 0.0 {
            self.e_int += e * self.sample_period;
        } else {
            self.e_int = F32x3::default();
        }

        // Apply feedback terms
        let gyro = gyroscope + e * self.kp + self.e_int * self.ki;

        // Compute rate of change of quaternion
        let q_dot = *q * Quaternion::new(0.0, gyro.x, gyro.y, gyro.z) * 0.5;

        // Integrate to yield quaternion
        self.quat = (self.quat + q_dot * self.sample_period).normalize();
    }
}
