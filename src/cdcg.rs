//! Core Domain Clock Generator

use core::mem::MaybeUninit;

use npcx490m_pac::lfcg::lfcgctl2::XtOscSlEn;
use npcx490m_pac::{Hfcg, Lfcg};

#[non_exhaustive]
#[derive(Debug, Clone)]
pub struct Config {
    /// The source of the low frequency clock LFCLK
    pub lf_clock_source: LfClockSource,
    /// First multiplier of the LFCLK to form the VOSCCLK
    ///
    /// Range: 1..
    pub mult_m: u16,
    /// Second multiplier of the LFCLK to form the VOSCCLK
    ///
    /// Range: 1..64
    pub mult_n: u8,
    /// Decides how the VOSC clock is used after the multipliers to form the MCLK and FMCLK
    pub vosc_mode: VoscClockMode,
    /// The prescaler for the MCLK that turns it into the CLK
    #[doc(alias = "FPRED")]
    pub core_clock_prescaler: MclkDivider,

    /// The divider for the CLK that turns it into the AHB6_CLK
    pub ahb6_divider: Option<AhbDivider>,
    /// The divider for the CLK that turns it into the FIU0_CLK
    pub fiu0_divider: Option<AhbDivider>,
    /// The divider for the CLK that turns it into the FIU1_CLK
    pub fiu1_divider: Option<AhbDivider>,

    /// The divider for the MCLK that turns it into the APB4_CLK
    pub apb4_divider: MclkDivider,
    /// The divider for the MCLK that turns it into the APB3_CLK
    pub apb3_divider: MclkDivider,
    /// The divider for the MCLK that turns it into the APB2_CLK
    pub apb2_divider: MclkDivider,
    /// The divider for the MCLK that turns it into the APB1_CLK
    pub apb1_divider: MclkDivider,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            lf_clock_source: LfClockSource::FreeRunningClock,
            mult_m: 0x0ABA,
            mult_n: 0x02,
            vosc_mode: VoscClockMode::Normal,
            core_clock_prescaler: MclkDivider::Div2,

            ahb6_divider: Some(AhbDivider::Div1),
            fiu0_divider: Some(AhbDivider::Div1),
            fiu1_divider: Some(AhbDivider::Div3),

            apb4_divider: MclkDivider::Div4,
            apb3_divider: MclkDivider::Div2,
            apb2_divider: MclkDivider::Div4,
            apb1_divider: MclkDivider::Div8,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VoscClockMode {
    ExtendedFrequency,
    Normal,
    Mhz40,
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

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum MclkDivider {
    Div1 = 0,
    Div2 = 1,
    Div3 = 2,
    Div4 = 3,
    Div5 = 4,
    Div6 = 5,
    Div7 = 6,
    Div8 = 7,
    Div9 = 8,
    Div10 = 9,
}

impl MclkDivider {
    fn div_value(&self) -> u32 {
        *self as u32 + 1
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum AhbDivider {
    Div1 = 0,
    Div2 = 1,
    Div3 = 2,
}

impl AhbDivider {
    fn div_value(&self) -> u32 {
        *self as u32 + 1
    }
}

/// Frozen clock frequencies
static mut CLOCKS: MaybeUninit<Clocks> = MaybeUninit::uninit();

/// Set the frozen clock frequencies
///
/// Safety: May only be used before or in the [init_clocks] function.
unsafe fn set_clocks(clocks: Clocks) {
    #[cfg(feature = "defmt")]
    defmt::debug!("cdcg: {:?}", clocks);
    CLOCKS = MaybeUninit::new(clocks);
}

/// Safety: May only be used after the [init_clocks] function.
pub(crate) unsafe fn get_clocks() -> &'static Clocks {
    (*&raw mut CLOCKS).assume_init_ref()
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
    assert!(
        unsafe { npcx490m_pac::Mswc::steal() }
            .mswctl1()
            .read()
            .lreset_pltrst_act()
            .bit_is_set(),
        "LPC host interface must not be active"
    );
    assert!(
        unsafe { npcx490m_pac::Espi::steal() }
            .espists()
            .read()
            .espirst()
            .bit_is_set(),
        "ESPI host interface must not be active"
    );

    // Calculate the clock up to the MCLK
    assert!(
        (0..64).contains(&config.mult_n),
        "The n multiplier must be in range of 0..64"
    );

    let voscclock = LFCLK * config.mult_m as u32 * config.mult_n as u32;

    if voscclock > 60_000_000 {
        assert_ne!(
            config.vosc_mode,
            VoscClockMode::Normal,
            "Above 60MHz the clock, the normal mode is unavailable"
        );
    }

    // Select the best SIO clock
    // TODO: When voscclock is not exactly one of these options, it will lead to increased error in serial port baud rate.
    // TODO: This should probably be checked in the serial port constructor.
    let sio_clk_sel = match voscclock {
        ..93_000_000 => 0b11,            // 90 Mhz
        93_000_000..98_000_000 => 0b00,  // 96 Mhz
        98_000_000..110_000_000 => 0b01, // 100 Mhz
        110_000_000.. => 0b10,           // 120 Mhz
    };
    unsafe { npcx490m_pac::Sysconfig::steal() }
        .dev_ctl3()
        .modify(|_, w| unsafe { w.sio_clk_sel().bits(sio_clk_sel) });

    let mclk = match config.vosc_mode {
        VoscClockMode::ExtendedFrequency => voscclock,
        VoscClockMode::Normal => voscclock / 2,
        VoscClockMode::Mhz40 => voscclock / 3,
    };
    let fmclk = match config.vosc_mode {
        VoscClockMode::ExtendedFrequency => voscclock / 2,
        VoscClockMode::Normal => voscclock / 2,
        VoscClockMode::Mhz40 => voscclock / 3,
    };

    let clk = mclk / config.core_clock_prescaler.div_value();

    let ahb6_dividers = [config.ahb6_divider, config.fiu0_divider, config.fiu1_divider];
    let ahb6_clock_div2_used = ahb6_dividers.iter().flatten().any(|&div| div == AhbDivider::Div2);
    let ahb6_clock_div3_used = ahb6_dividers.iter().flatten().any(|&div| div == AhbDivider::Div3);

    assert!(
        ahb6_clock_div2_used != ahb6_clock_div3_used,
        "The ahb6_divider, fiu0_divider & fiu1_divider cannot use div 2 and div 3 divisions at the same time"
    );

    let ahb6_clk = config.ahb6_divider.map(|div| clk / div.div_value());
    let fiu0_clk = config.fiu0_divider.map(|div| clk / div.div_value());
    let fiu1_clk = config.fiu1_divider.map(|div| clk / div.div_value());

    let apb4_clk = mclk / config.apb4_divider.div_value();
    let apb3_clk = mclk / config.apb3_divider.div_value();
    let apb2_clk = mclk / config.apb2_divider.div_value();
    let apb1_clk = mclk / config.apb1_divider.div_value();

    // Set the core clock prescaler all bus dividers
    hfcg.hfcgp().write(|w| unsafe {
        w.fpred()
            .bits(config.core_clock_prescaler as u8)
            .ahb6div()
            .bits(config.ahb6_divider.unwrap_or(AhbDivider::Div1) as u8)
    });
    hfcg.hfcbcd().write(|w| unsafe {
        w.fiu0div()
            .bits(config.fiu0_divider.unwrap_or(AhbDivider::Div1) as u8)
            .fiu1div()
            .bits(config.fiu1_divider.unwrap_or(AhbDivider::Div1) as u8)
            .ahb6clk_blk()
            .bit(config.ahb6_divider.is_some() || config.fiu0_divider.is_some() || config.fiu1_divider.is_some())
    });
    hfcg.hfcbcd1().write(|w| unsafe {
        w.apb1div()
            .bits(config.apb1_divider as u8)
            .apb2div()
            .bits(config.apb2_divider as u8)
    });
    hfcg.hfcbcd2().write(|w| unsafe {
        w.apb3div()
            .bits(config.apb3_divider as u8)
            .apb4div()
            .bits(config.apb4_divider as u8)
    });

    // Set the high frequency values
    hfcg.hfcgn().write(|w| {
        unsafe { w.hfcgn5_0().bits(config.mult_n) }
            ._40m_en()
            .bit(matches!(config.vosc_mode, VoscClockMode::Mhz40))
            .xf_range()
            .bit(matches!(config.vosc_mode, VoscClockMode::ExtendedFrequency))
    });
    hfcg.hfcgml()
        .write(|w| unsafe { w.hfcgm70().bits(config.mult_m as u8) });
    hfcg.hfcgmh()
        .write(|w| unsafe { w.hfcgm158().bits((config.mult_m >> 8) as u8) });

    // Let the hardware load the new config
    hfcg.hfcgctrl().modify(|_, w| w.load().set_bit());
    while hfcg.hfcgctrl().read().clk_chng().bit_is_set() {}

    unsafe {
        set_clocks(Clocks {
            voscclock,
            mclk,
            fmclk,
            sio_clk: 24_000_000,
            clk,

            ahb6_clk,
            fiu0_clk,
            fiu1_clk,

            apb4_clk,
            apb3_clk,
            apb2_clk,
            apb1_clk,
        });
    }
}

const LFCLK: u32 = 32_768;

#[allow(unused)]
#[derive(Debug)]
#[cfg_attr(feature = "defmt", defmt::Format)]
pub(crate) struct Clocks {
    pub(crate) voscclock: u32,
    pub(crate) mclk: u32,
    pub(crate) fmclk: u32,
    pub(crate) sio_clk: u32,
    pub(crate) clk: u32,

    pub(crate) ahb6_clk: Option<u32>,
    pub(crate) fiu0_clk: Option<u32>,
    pub(crate) fiu1_clk: Option<u32>,

    pub(crate) apb4_clk: u32,
    pub(crate) apb3_clk: u32,
    pub(crate) apb2_clk: u32,
    pub(crate) apb1_clk: u32,
}
