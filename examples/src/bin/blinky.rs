#![no_main]
#![no_std]

use defmt::info;
use embassy_executor::Spawner;
use embassy_npcx::gpio::{Level, Output, OutputOnly};
use embassy_npcx::Config;
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let (p, _mode) = embassy_npcx::init_lpc(Config::default());

    let mut led = Output::<'_, OutputOnly>::new(p.PJ07, Level::High);

    loop {
        info!("LED Toggle");
        led.toggle();
        Timer::after_secs(1).await;
    }
}
