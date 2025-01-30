#![no_main]
#![no_std]

use embassy_executor::Spawner;
use embassy_npcx::gpio::{Level, Output, OutputOnly};
use embassy_npcx::Config;
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut config = Config::default();
    {
        use embassy_npcx::cdcg::*;
        config.cdcg.lf_clock_source = LfClockSource::FreeRunningClock; // 0.032768 MHz
        config.cdcg.mult_m = 0xE4E; // 119.996416 MHz
        config.cdcg.vosc_mode = VoscClockMode::ExtendedFrequency;
        config.cdcg.core_clock_prescaler = MclkDivider::Div1;

        config.cdcg.fiu0_divider = None;
        config.cdcg.fiu1_divider = None;
        config.cdcg.ahb6_divider = None;

        config.cdcg.apb1_divider = MclkDivider::Div2;
        config.cdcg.apb2_divider = MclkDivider::Div2;
        config.cdcg.apb3_divider = MclkDivider::Div2;
        config.cdcg.apb4_divider = MclkDivider::Div2;
    }
    let p = embassy_npcx::init(config);

    let mut led = Output::<'_, OutputOnly>::new(p.PJ07, Level::High);

    loop {
        led.toggle();
        cortex_m::asm::delay(1_000_000);
    }
}
