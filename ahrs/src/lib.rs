// Taken from https://github.com/jmagnuson/ahrs-rs/tree/master
#![no_std]
#![no_main]

use micromath::{
    vector::{F32x3, Vector, Vector2d, Vector3d},
    Quaternion,
};

#[derive(Debug)]
pub struct Mahony {
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
            kp: 0.5,
            ki: 0.0,
            e_int: F32x3::default(),
            quat: Quaternion::default(),
        }
    }
}

impl Mahony {
    pub fn new(kp: f32, ki: f32) -> Self {
        Mahony {
            kp,
            ki,
            e_int: F32x3::default(),
            quat: Quaternion::default(),
        }
    }
}

impl Mahony {
    pub fn update(&mut self, gyroscope: F32x3, accelerometer: F32x3, magnetometer: F32x3, dt: f32) {
        let q = &self.quat;

        // Normalize accelerometer measurement
        let acc = accelerometer.normalize();

        // Normalize magnetometer measurement
        let mag = magnetometer.normalize();

        // Reference direction of Earth's magnetic field (Quaternion should still be conj of q)
        let h = *q * (Quaternion::new(0.0, mag.x, mag.y, mag.z) * q.conj());
        let b = Quaternion::new(0.0, Vector2d::from((h.x(), h.y())).magnitude(), 0.0, h.z());

        let v = F32x3::from((
            2.0 * (q.x() * q.z() - q.w() * q.y()),
            2.0 * (q.w() * q.x() + q.y() * q.z()),
            (q.w() * q.w() - q.x() * q.x() - q.y() * q.y() + q.z() * q.z()),
        ));

        #[rustfmt::skip]
        let w = F32x3::from((
            2.0 * b.x() * (0.5 - q.y() * q.y() - q.z() * q.z()) + 2.0 * b.z() * (q.x() * q.z() - q.w() * q.y()),
            2.0 * b.x() * (q.x() * q.y() - q.w() * q.z())       + 2.0 * b.z() * (q.w() * q.x() + q.y() * q.z()),
            2.0 * b.x() * (q.w() * q.y() + q.x() * q.z())       + 2.0 * b.z() * (0.5 - q.x() * q.x() - q.y() * q.y()),
        ));

        // cross(acc, v) + cross(mag, w)
        let e = acc * v + mag * w;

        // Error is sum of cross product between estimated direction and measured direction of fields
        if self.ki > 0.0 {
            self.e_int += e * dt;
        } else {
            self.e_int = F32x3::default()
        }

        // Apply feedback terms
        let gyro = gyroscope + e * self.kp + self.e_int * self.ki;

        // Compute rate of change of quaternion
        let q_dot = self.quat * Quaternion::new(0.0, gyro.x, gyro.y, gyro.z) * 0.5;

        // Integrate to yield quaternion
        let quat = self.quat + q_dot * dt;

        self.quat = quat.normalize();
    }

    pub fn update_imu(&mut self, gyroscope: F32x3, accelerometer: F32x3, dt: f32) {
        let q = &self.quat;

        // Normalize accelerometer measurement
        let acc = accelerometer.normalize();

        // #[rustfmt::skip]
        let v = F32x3::from((
            2.0 * (q.x() * q.z() - q.w() * q.y()),
            2.0 * (q.w() * q.x() + q.y() * q.z()),
            q.w() * q.w() - q.x() * q.x() - q.y() * q.y() + q.z() * q.z(),
        ));

        // cross(acc, v)
        let e = acc * v;

        // Error is sum of cross product between estimated direction and measured direction of fields
        if self.ki > 0.0 {
            self.e_int += e * dt;
        } else {
            self.e_int = F32x3::default();
        }

        // Apply feedback terms
        let gyro = gyroscope + e * self.kp + self.e_int * self.ki;

        // Compute rate of change of quaternion
        let q_dot = *q * Quaternion::new(0.0, gyro.x, gyro.y, gyro.z) * 0.5;

        // Integrate to yield quaternion
        self.quat = (self.quat + q_dot * dt).normalize();
    }
}

pub trait Normalize {
    fn normalize(&self) -> Vector3d<f32>;
}

impl Normalize for Vector3d<f32> {
    fn normalize(&self) -> Vector3d<f32> {
        let mag = self.magnitude();
        if mag == 0.0 {
            F32x3::default()
        } else {
            *self * (1.0 / mag)
        }
    }
}
