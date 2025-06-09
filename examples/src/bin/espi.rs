#![no_main]
#![no_std]

use defmt::info;
use embassy_executor::Spawner;
use embassy_npcx::espi::{Espi, InterruptHandler};
use embassy_npcx::{Config, bind_interrupts, peripherals};
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(
    struct Irqs {
        ESPI_SHI => InterruptHandler<peripherals::ESPI>;
    }
);

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let (p, _mode) = embassy_npcx::init_lpc(Config::default());

    info!("ESPI Example");

    let mut espi = Espi::new(p.ESPI, Irqs, Default::default());

    loop {
        let result = espi.listen().await;
        match result {
            Ok(event) => info!("Got new event: {}", event),
            Err(error) => info!("ERROR: {}", error),
        }
    }
}
