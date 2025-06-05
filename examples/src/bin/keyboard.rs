#![no_main]
#![no_std]

use defmt::{error, info};
use embassy_executor::Spawner;
use embassy_npcx::keyboard::Keyboard;
use embassy_npcx::{Config, bind_interrupts, peripherals};
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(pub struct Irqs {
    KBS => embassy_npcx::keyboard::InterruptHandler<peripherals::KBS>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let (p, _mode) = embassy_npcx::init_lpc(Config::default());

    info!("Keyboard example");
    let mut keyboard = Keyboard::new(
        p.KBS,
        p.PA02,
        p.PA03,
        p.PA04,
        p.PB03,
        p.PB04,
        p.PC03,
        p.PC04,
        p.PC05,
        p.PB05,
        p.PB06,
        p.PB07,
        p.PB08,
        p.PC07,
        p.PC06,
        p.PC08,
        p.PB09,
        p.PC09,
        p.PC10,
        p.PB11,
        p.PB10,
        p.PC11,
        p.PD11,
        p.PD06,
        p.PD07,
        p.PD09,
        p.PD08,
        Irqs,
        Default::default(),
    );

    loop {
        let result = keyboard.scan().await;

        match result {
            Ok(buffer) => info!("BUF: {:02x}", buffer),
            Err(_) => error!("Timeout"),
        }

        Timer::after_millis(1000).await;
    }
}
