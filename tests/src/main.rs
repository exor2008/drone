#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(slice_flatten)]

use core::f32::consts;

use ahrs::{Ahrs, Mahony};
use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use nalgebra::{Quaternion, Vector3};
use panic_probe as _;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let _p = embassy_rp::init(Default::default());
    test_mahony_no_mag().await;
    test_mahony().await;
}

async fn test_mahony_no_mag() {
    let gyroscope = Vector3::from([60.1, 30.2, 20.3]) * (consts::PI / 180.0);
    let accelerometer = Vector3::from([0.1, 3.2, 9.8]);

    let mut mahony = Mahony::new(1e-1, 0.5, 0.01);

    mahony.update_imu(&gyroscope, &accelerometer).unwrap();
    let diff = (mahony.quat.quaternion()
        - Quaternion::new(0.9650766, 0.0578841, 0.02520694, 0.01709642))
    .magnitude();

    crate::assert!(diff < 1e-1);

    for _ in 0..100 {
        mahony.update_imu(&gyroscope, &accelerometer).unwrap();
    }
    let diff = (mahony.quat.quaternion()
        - Quaternion::new(
            0.5282846730738564,
            -0.042231151578480376,
            -0.6039791949152863,
            -0.59526545854861,
        ))
    .magnitude();
    crate::assert!(diff < 2e-1);

    info!("test_mahony_no_mag is OK");
}

async fn test_mahony() {
    let gyroscope = Vector3::from([60.1, 30.2, 20.3]) * (consts::PI / 180.0);
    let accelerometer = Vector3::from([0.1, 3.2, 9.8]);
    let magnetometer = Vector3::from([1.0, 0.2, 0.2]);

    let mut mahony = Mahony::new(1e-1, 0.5, 0.01);
    mahony
        .update(&gyroscope, &accelerometer, &magnetometer)
        .unwrap();
    let diff = (mahony.quat.quaternion()
        - Quaternion::new(
            0.9977106127395269,
            0.06101012434505351,
            0.026143375195607478,
            0.012954608743636662,
        ))
    .magnitude();
    crate::assert!(diff < 1e-1);

    for _ in 0..100 {
        mahony
            .update(&gyroscope, &accelerometer, &magnetometer)
            .unwrap();
    }
    let diff = (mahony.quat.quaternion()
        - Quaternion::new(
            0.5426803195378483,
            -0.23060351328044826,
            -0.43548639958944807,
            -0.6801997399452846,
        ))
    .magnitude();
    crate::assert!(diff < 1e-1);

    info!("test_mahony is OK");
}
