#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(slice_flatten)]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
// use embassy_rp::gpio::{Input, Level, Output};
use embassy_rp::i2c::{self, Async, Config, InterruptHandler};
use embassy_rp::peripherals::I2C0;
use embassy_time::{Duration, Ticker};
use embedded_hal_async::i2c::I2c;
use panic_probe as _;

bind_interrupts!(struct Irqs {
    I2C0_IRQ => InterruptHandler<I2C0>;
});

pub const DEV_ADDR: u8 = 0x68;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let sda = p.PIN_16;
    let scl = p.PIN_17;
    let mut sensor: i2c::I2c<'_, I2C0, Async> =
        i2c::I2c::new_async(p.I2C0, scl, sda, Irqs, Config::default());

    let mut buffer = [0u8];
    sensor
        .write_read(DEV_ADDR, &[0x75], &mut buffer)
        .await
        .unwrap();
    debug!("id: {:b}", buffer);

    sensor.write(DEV_ADDR, &[0x6B, 0x01]).await.unwrap(); // Sleep off
    sensor.write(DEV_ADDR, &[0x19, 0x07]).await.unwrap(); // Sample Rate Divider. Typical values:0x07(125Hz) 1KHz internal sample rate
    sensor.write(DEV_ADDR, &[0x1A, 0x06]).await.unwrap(); // Low pass filter 5 Hz
    sensor.write(DEV_ADDR, &[0x1B, 0x10]).await.unwrap(); // Gyro Full Scale Select. Typical values:0x10(1000dps)
    sensor.write(DEV_ADDR, &[0x1C, 0x00]).await.unwrap(); // Accel Full Scale Select. Typical values:0x01(2g)

    let mut ticket = Ticker::every(Duration::from_millis(50));
    loop {
        sensor
            .write_read(DEV_ADDR, &[0x3B], &mut buffer)
            .await
            .unwrap();
        let heigh = buffer.clone();

        sensor
            .write_read(DEV_ADDR, &[0x3C], &mut buffer)
            .await
            .unwrap();
        let low = buffer.clone();

        let x_acc = i16::from_le_bytes([low[0], heigh[0]]);
        info!("{}", x_acc);
        ticket.next().await;
    }
}
