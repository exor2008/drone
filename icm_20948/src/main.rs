#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(slice_flatten)]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::gpio::{Input, Level, Output};

use panic_probe as _;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    debug!("Hello, world");
}
