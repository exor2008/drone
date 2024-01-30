#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::i2c;
use panic_probe as _;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {}
