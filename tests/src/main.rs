#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(slice_flatten)]

use core::f32::consts;

use ahrs::{Mahony, Normalize};
use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use micromath::vector::F32x3;
use panic_probe as _;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let _p = embassy_rp::init(Default::default());
    test_f32x3_normalize().await;
    test_mahony().await;
}

async fn test_f32x3_normalize() {
    let a = F32x3::from((5.0, 5.0, 5.0));
    let actual = a.normalize();
    let expected = F32x3::from((0.5755396, 0.5755396, 0.5755396));

    crate::assert_eq!(actual.to_array(), expected.to_array());

    info!("test_f32x3_normalize is OK");
}

async fn test_mahony() {
    let gyroscope = F32x3::from((60.1, 30.2, 20.3)) * (consts::PI / 180.0);
    let accelerometer = F32x3::from((0.1, 0.2, 0.3));
    let magnetometer = F32x3::from((0.5, 0.6, 0.7));

    let mut mahony = Mahony::new(1.0 / 256.0, 0.5, 0.01);
    mahony.update(gyroscope, accelerometer, magnetometer);

    let (roll, pitch, yaw) = mahony.quat.to_euler();
    crate::assert_eq!(
        (roll, pitch, yaw),
        (0.0051228325, 0.0016469688, 0.00045724475)
    );

    for _ in 0..100 {
        mahony.update(gyroscope, accelerometer, magnetometer);
    }
    let (roll, pitch, yaw) = mahony.quat.to_euler();
    crate::assert_eq!((0.50525886, 0.10579579, 0.1318471), (roll, pitch, yaw));
    info!("test_mahony is OK");
}
