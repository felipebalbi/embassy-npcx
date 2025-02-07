#![no_main]
#![no_std]

use embassy_executor::Spawner;
use embassy_npcx::gpio::{Level, Output, OutputOnly};
use embassy_npcx::timer::low_level::{self, ClockSource, MultiFunctionTimer, WakeUpEvent};
use embassy_npcx::{self as hal, bind_interrupts, Config};
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let (p, _) = embassy_npcx::init_espi(Config::default());

    let mut led = Output::<'_, OutputOnly>::new(p.PJ07, Level::High);
    loop {
        led.toggle();
        Timer::after_millis(500).await;
        defmt::info!("Ping");
    }
}
