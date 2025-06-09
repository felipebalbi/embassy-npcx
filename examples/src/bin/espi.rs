#![no_main]
#![no_std]

use defmt::*;
use embassy_executor::Spawner;
use embassy_npcx::espi::{AlertMode, Config, Espi, Event, InterruptHandler, IoMode, State};
use embassy_npcx::{bind_interrupts, peripherals};
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(
    struct Irqs {
        ESPI_SHI => InterruptHandler<peripherals::ESPI>;
    }
);

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut config = embassy_npcx::Config::default();
    {
        use embassy_npcx::cdcg::*;
        config.cdcg.lf_clock_source = LfClockSource::FreeRunningClock; // 0.032768 MHz
        config.cdcg.mult_m = 3662; // 119.996416 MHz
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

    let (p, _mode) = embassy_npcx::init_espi(config);

    info!("ESPI Example");

    let mut config = Config::default();

    config.peripheral_channel_support = true;
    config.virtual_wire_support = true;
    config.oob_support = true;
    config.alert_mode = AlertMode::Pin;
    config.io_mode = IoMode::Quad;

    let mut espi = Espi::new(
        p.ESPI, p.PH01, p.PJ01, p.PK01, p.PL01, p.PL02, p.PK03, p.PM01, p.PL03, Irqs, config,
    );

    loop {
        let result = espi.listen().await;
        trace!("{:?}", result);

        match result {
            Ok(event) => {
                debug!("Got new event: {:#?}", event);

                match event {
                    Event::EspiReset(_) | Event::InBandResetCmdReceived => {
                        espi.disable_peripheral_channel();
                    }
                    Event::EspiConfigurationUpdated(cfg) => {
                        if cfg.host_flash_channel == State::Enabled {
                            espi.enable_flash_access_channel();
                        } else {
                            espi.disable_flash_access_channel();
                        }

                        if cfg.host_oob_channel == State::Enabled {
                            espi.enable_oob_channel();
                        } else {
                            espi.disable_oob_channel();
                        }

                        if cfg.host_vwire_channel == State::Enabled {
                            espi.enable_vwire_channel();
                        } else {
                            espi.disable_vwire_channel();
                        }

                        if cfg.host_peripheral_channel == State::Enabled {
                            espi.enable_peripheral_channel();
                        } else {
                            espi.disable_peripheral_channel();
                        }
                    }
                    _ => {}
                }
            }

            Err(err) => {
                error!("Failed while listening: {:#?}", err);
            }
        }
    }
}
