#![no_main]
#![no_std]

use defmt::*;
use embassy_espi::driver::{
    Driver, Event,
    oob::OobChannel,
    vwire::{self, VWireChannel},
};
use embassy_executor::Spawner;
use embassy_npcx::espi::{
    AlertMode, Config, Espi, InterruptHandler, IoMode, OobConfig, PayloadSize, PeripheralConfig, RequestSize,
    VWireConfig,
};
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

    config.peripheral_config = Some(PeripheralConfig {
        max_payload_size: PayloadSize::_256,
        max_request_size: RequestSize::_2048,
    });
    config.vwire_config = Some(VWireConfig {});
    config.oob_config = Some(OobConfig {
        max_payload_size: PayloadSize::_64,
    });
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
                    Event::Reset => {
                        debug!("Reset event (either In-band or eSPI_RST#)");
                        // reset everything from the application side.
                    }

                    Event::VWire => {
                        // VWires changed, for now let's just get and
                        // print all Readable types:

                        // info!("SLP_S3# {:?}", espi.read_vwire(vwire::SlpS3).unwrap());
                        // info!("SLP_S4# {:?}", espi.read_vwire(vwire::SlpS4).unwrap());
                        // info!("SLP_S5# {:?}", espi.read_vwire(vwire::SlpS5).unwrap());

                        // info!("SUS_STAT# {:?}", espi.read_vwire(vwire::SusStat).unwrap());
                        // info!("PLTRST# {:?}", espi.read_vwire(vwire::PltRst).unwrap());
                        // info!("OOB_RST_WARN# {:?}", espi.read_vwire(vwire::OobRstWarn).unwrap());

                        // info!("HOST_RST_WARN# {:?}", espi.read_vwire(vwire::HostRstWarn).unwrap());
                        // info!("SMIOUT# {:?}", espi.read_vwire(vwire::SmiOut).unwrap());
                        // info!("NMIOUT# {:?}", espi.read_vwire(vwire::NmiOut).unwrap());
                    }
                    Event::Oob => {
                        info!("OOB Received!");
                        let mut buf = [0; 256];

                        if let Ok(size) = espi.oob_receive(&mut buf).await {
                            info!("Received {}bytes: {:02x}", size, buf[0..size]);
                        } else {
                            error!("Failed receiving OOB");
                        }
                    }
                    _ => {}
                }
            }
            _ => {}
        }
    }
}
