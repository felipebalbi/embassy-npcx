#![no_main]
#![no_std]

use cortex_m::asm::delay;
use cortex_m_rt::entry;
use defmt::info;
use embassy_npcx::pac;
use pac::{interrupt, Interrupt};
use {defmt_rtt as _, panic_probe as _};

#[pac::interrupt]
fn ADC_IREF() {
    info!("Interrupt");
}

#[entry]
fn main() -> ! {
    info!("Hello world");

    unsafe { pac::NVIC::unmask(Interrupt::ADC_IREF) };

    loop {
        pac::NVIC::pend(Interrupt::ADC_IREF);
        delay(5_000_000);
    }
}
