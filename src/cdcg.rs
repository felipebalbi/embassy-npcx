//! Core Domain Clock Generator

use npcx490m_pac::lfcg::lfcgctl2::XtOscSlEn;
use npcx490m_pac::{Hfcg, Lfcg};

#[non_exhaustive]
#[derive(Debug, Clone)]
pub struct Config {
    pub lf_clock_source: LfClockSource,
    /// First multiplier of the LFCLK to form the VOSCCLK
    ///
    /// Range: 1..
    pub mult_m: u16,
    /// Second multiplier of the LFCLK to form the VOSCCLK
    ///
    /// Range: 1..64
    pub mult_n: u8,
    pub divider: VoscClockDivider,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            lf_clock_source: LfClockSource::FreeRunningClock,
            mult_m: 0x0ABA,
            mult_n: 0x02,
            divider: Default::default(),
        }
    }
}

#[derive(Debug, Clone, Copy, Default)]
pub enum VoscClockDivider {
    #[default]
    Div1,
    Div2,
    Div3,
}

impl VoscClockDivider {
    fn xf_range(&self) -> bool {
        todo!()
    }

    fn enable_40m(&self) -> bool {
        todo!()
    }
}

/// The main clock source of the microcontroller.
/// This clock source powers the LFCLK.
#[derive(Debug, Copy, Clone)]
pub enum LfClockSource {
    /// Use the ~930KHz FRCLK as the source of the low frequency clock (LFCLK)
    FreeRunningClock,
    /// Use the 32.768KHz XTCLK crystal as the source of the low frequency clock (LFCLK)
    ExternalOscillator,
}

impl From<LfClockSource> for XtOscSlEn {
    fn from(value: LfClockSource) -> Self {
        match value {
            LfClockSource::FreeRunningClock => XtOscSlEn::Lfcg,
            LfClockSource::ExternalOscillator => XtOscSlEn::Xtosc,
        }
    }
}

pub(crate) fn init_clocks(config: Config) {
    // Get the clock peripherals
    // Safety: These are not given to the user, and thus safe to steal
    let lfcg = unsafe { Lfcg::steal() };
    let hfcg = unsafe { Hfcg::steal() };

    // 4.32.2
    // Select the low frequency clock LFCLK source
    // The XTCLK may still be starting up, but the change happens automatically when that is stable
    lfcg.lfcgctl2()
        .modify(|_, w| w.xt_osc_sl_en().variant(config.lf_clock_source.into()));

    // We may only change the MCLK when the Host interface is reset.
    // Check that !LRESET and !eSPI_RST signals are asserted.
    assert!(unsafe { npcx490m_pac::Mswc::steal() }
        .mswctl1()
        .read()
        .lreset_pltrst_act()
        .bit_is_set());
    assert!(unsafe { npcx490m_pac::Espi::steal() }
        .espists()
        .read()
        .espirst()
        .bit_is_set());

    // Calculate the clock up to the MCLK
    assert!((0..64).contains(&config.mult_n));

    let voscclock = LFCLK * config.mult_m as u32 * config.mult_n as u32;

    let xf_range = false;

    hfcg.hfcgn().write(|w| {
        unsafe { w.hfcgn5_0().bits(config.mult_n) }
            ._40m_en()
            .variant(config.enable_40m)
            .xf_range()
            .variant(xf_range)
    });
}

const FRCLK: u32 = 930_000;
const LFCLK: u32 = 32_768;
