//! Core Domain Clock Generator

use core::mem::MaybeUninit;

use npcx490m_pac::lfcg::lfcgctl2::XtOscSlEn;
use npcx490m_pac::{Hfcg, Lfcg, Shm};

const LFCLK: u32 = 32_768;

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

/// Clock config paramters
#[non_exhaustive]
#[derive(Debug, Clone)]
pub struct Config {
    /// The source of the low frequency clock LFCLK
    pub lf_clock_source: LfClockSource,
    /// First multiplier of the LFCLK to form the VOSCCLK
    ///
    /// Range: 1..
    pub mult_m: u16,
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

    /// The divider for the MCLK that turns it into the APB4_CLK.
    /// Must be a multiple of [Self::core_clock_prescaler].
    pub apb4_divider: MclkDivider,
    /// The divider for the MCLK that turns it into the APB3_CLK.
    /// Must be a multiple of [Self::core_clock_prescaler].
    pub apb3_divider: MclkDivider,
    /// The divider for the MCLK that turns it into the APB2_CLK.
    /// Must be a multiple of [Self::core_clock_prescaler].
    pub apb2_divider: MclkDivider,
    /// The divider for the MCLK that turns it into the APB1_CLK.
    /// Must be a multiple of [Self::core_clock_prescaler].
    pub apb1_divider: MclkDivider,

    /// The divider for the MCLK that turns it into the MCLKD
    pub mclkd_divider: MclkdDivider,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            lf_clock_source: LfClockSource::FreeRunningClock,
            mult_m: 0x0ABA,
            vosc_mode: VoscClockMode::Normal,
            core_clock_prescaler: MclkDivider::Div2,

            ahb6_divider: Some(AhbDivider::Div1),
            fiu0_divider: Some(AhbDivider::Div1),
            fiu1_divider: Some(AhbDivider::Div3),

            apb4_divider: MclkDivider::Div4,
            apb3_divider: MclkDivider::Div2,
            apb2_divider: MclkDivider::Div4,
            apb1_divider: MclkDivider::Div8,

            mclkd_divider: MclkdDivider::Div1,
        }
    }
}

pub(crate) fn init_clocks(config: Config) {
    // Get the clock peripherals
    // Safety: These are not given to the user, and thus safe to steal
    let lfcg = unsafe { Lfcg::steal() };
    let hfcg = unsafe { Hfcg::steal() };
    let shm = unsafe { Shm::steal() };

    // 4.32.2
    // Select the low frequency clock LFCLK source
    // The XTCLK may still be starting up, but the change happens automatically when that is stable
    lfcg.lfcgctl2()
        .modify(|_, w| w.xt_osc_sl_en().variant(config.lf_clock_source.into()));

    // Disable host access
    let host_access_stalled = shm.shm_ctl().read().stall_host().bit_is_set();
    if !host_access_stalled {
        shm.shm_ctl().modify(|_, w| w.stall_host().set_bit());
    }

    let voscclock = LFCLK * config.mult_m as u32;

    // Select the best SIO clock
    // TODO: When voscclock is not exactly one of these options, it will lead to increased error in serial port baud rate.
    // TODO: This should probably be checked in the serial port constructor.
    let sio_clk_sel = match voscclock {
        ..93_000_000 => 0b11,            // 90 MHz
        93_000_000..98_000_000 => 0b00,  // 96 MHz
        98_000_000..110_000_000 => 0b01, // 100 MHz
        110_000_000.. => 0b10,           // 120 MHz
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
        ahb6_dividers.iter().all(Option::is_none) || ahb6_clock_div2_used != ahb6_clock_div3_used,
        "The ahb6_divider, fiu0_divider & fiu1_divider cannot use div 2 and div 3 divisions at the same time"
    );

    let ahb6_clk = config.ahb6_divider.map(|div| clk / div.div_value());
    let fiu0_clk = config.fiu0_divider.map(|div| clk / div.div_value());
    let fiu1_clk = config.fiu1_divider.map(|div| clk / div.div_value());

    let apb4_clk = mclk / config.apb4_divider.div_value();
    let apb3_clk = mclk / config.apb3_divider.div_value();
    let apb2_clk = mclk / config.apb2_divider.div_value();
    let apb1_clk = mclk / config.apb1_divider.div_value();

    let mclkd = mclk / config.mclkd_divider.div_value();

    assert!(apb4_clk <= clk, "APB clock must not exceed the CLK");
    assert!(apb3_clk <= clk, "APB clock must not exceed the CLK");
    assert!(apb2_clk <= clk, "APB clock must not exceed the CLK");
    assert!(apb1_clk <= clk, "APB clock must not exceed the CLK");

    assert!(clk <= 120_000_000, "Max CLK speed is 120 MHz");
    assert!(clk >= 4_000_000, "Min CLK speed is 4 MHz");

    if clk > 60_000_000 {
        assert!(
            fiu0_clk.unwrap_or(0) <= clk / 2,
            "FIUm_CLK frequency must be slower than half CLK if CLK is > 60 MHz"
        );
        assert!(
            fiu1_clk.unwrap_or(0) <= clk / 2,
            "FIUm_CLK frequency must be slower than half CLK if CLK is > 60 MHz"
        );
    }

    let apb_dividers = [
        config.apb1_divider,
        config.apb2_divider,
        config.apb3_divider,
        config.apb4_divider,
    ];
    for apb_divider in apb_dividers {
        assert!(
            apb_divider.div_value() % config.core_clock_prescaler.div_value() == 0,
            "APB divider must be a multiple of the `core_clock_prescaler`"
        );
    }

    let max_apb_clocks = if mclk > 60_000_000 {
        [
            (mclk / 2).min(60_000_000),
            (mclk / 2).min(60_000_000),
            (mclk / 2).min(60_000_000),
            mclk.min(120_000_000),
        ]
    } else {
        [mclk; 4]
    };
    let min_apb_clocks = [4_000_000, 8_000_000, 12_500_000, 8_000_000];

    for (i, ((apb_clk, max), min)) in [apb1_clk, apb2_clk, apb3_clk, apb4_clk]
        .into_iter()
        .zip(max_apb_clocks)
        .zip(min_apb_clocks)
        .enumerate()
    {
        assert!(
            (min..=max).contains(&apb_clk),
            "APB{} clock is {apb_clk} hz, but must be in range: {min}..={max}",
            i + 1
        );
    }

    // When changing prescalers we need to make sure APB is never higher than CLK, so we first slow them down and then possibly speed them back up
    let current_apb1_divider = hfcg.hfcbcd1().read().apb1div().bits();
    let current_apb2_divider = hfcg.hfcbcd1().read().apb2div().bits();
    let current_apb3_divider = hfcg.hfcbcd2().read().apb3div().bits();
    let current_apb4_divider = hfcg.hfcbcd2().read().apb4div().bits();
    let current_core_clock_prescaler = hfcg.hfcgp().read().fpred().bits();
    let min_apb_divider = current_core_clock_prescaler.max(config.core_clock_prescaler as u8);

    if current_apb1_divider < min_apb_divider {
        hfcg.hfcbcd1()
            .modify(|_, w| unsafe { w.apb1div().bits(min_apb_divider) });
    }
    if current_apb2_divider < min_apb_divider {
        hfcg.hfcbcd1()
            .modify(|_, w| unsafe { w.apb2div().bits(min_apb_divider) });
    }
    if current_apb3_divider < min_apb_divider {
        hfcg.hfcbcd2()
            .modify(|_, w| unsafe { w.apb3div().bits(min_apb_divider) });
    }
    if current_apb4_divider < min_apb_divider {
        hfcg.hfcbcd2()
            .modify(|_, w| unsafe { w.apb4div().bits(min_apb_divider) });
    }

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
    hfcg.hfcbcd3()
        .write(|w| unsafe { w.mclkd_sl().bits(config.mclkd_divider as u8) });

    // Wait at least 16 clock cycles
    cortex_m::asm::delay(16);

    // Set the high frequency values
    hfcg.hfcgn().write(|w| {
        unsafe { w.hfcgn5_0().bits(2) }
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

            mclkd,
        });
    }

    if !host_access_stalled {
        shm.shm_ctl().modify(|_, w| w.stall_host().clear_bit());
    }
}

#[allow(unused)]
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
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

    pub(crate) mclkd: u32,
}

/// All possible ways the `VOSCCLK` is used for the dependend clocks
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VoscClockMode {
    /// Allow a higher `MCLK` frequency:
    /// - `FMCLK` = `VOSCCLK` / 2
    /// - `MCLK` = `VOSCCLK`
    ExtendedFrequency,
    /// The 'normal' mode:
    /// - `FMCLK` = `VOSCCLK` / 2
    /// - `MCLK` = `VOSCCLK` / 2
    Normal,
    /// The mode to create 40 MHz with when `VOSCCLK` is at its max of 120 MHz:
    /// - `FMCLK` = `VOSCCLK` / 3
    /// - `MCLK` = `VOSCCLK` / 3
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

/// A divider value for the `MCLK` clock
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum MclkDivider {
    /// No division
    Div1 = 0,
    /// Output is `MCLK` / 2
    Div2 = 1,
    /// Output is `MCLK` / 3
    Div3 = 2,
    /// Output is `MCLK` / 4
    Div4 = 3,
    /// Output is `MCLK` / 5
    Div5 = 4,
    /// Output is `MCLK` / 6
    Div6 = 5,
    /// Output is `MCLK` / 7
    Div7 = 6,
    /// Output is `MCLK` / 8
    Div8 = 7,
    /// Output is `MCLK` / 9
    Div9 = 8,
    /// Output is `MCLK` / 10
    Div10 = 9,
}

impl MclkDivider {
    fn div_value(&self) -> u32 {
        *self as u32 + 1
    }
}

/// A divider value for the `AHB` clock
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum AhbDivider {
    /// No division
    Div1 = 0,
    /// Output is `AHB` / 2
    Div2 = 1,
    /// Output is `AHB` / 3
    Div3 = 2,
}

impl AhbDivider {
    fn div_value(&self) -> u32 {
        *self as u32 + 1
    }
}

/// A divider value for the `MCLKD` clock
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum MclkdDivider {
    /// No division
    Div1 = 0,
    /// Output is `MCLKD` / 2
    Div2 = 1,
    /// Output is `MCLKD` / 3
    Div3 = 2,
}

impl MclkdDivider {
    fn div_value(&self) -> u32 {
        *self as u32 + 1
    }
}
