use embassy_hal_internal::{into_ref, Peripheral, PeripheralRef};

use crate::timer::MultiFunctionInstance;

#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ClockSource {
    /// Do not run this counter.
    NoClock = 0b000,
    /// Prescaled APB1 clock. (The counter is frozen in sleep mode)
    PrescaledAPB1Clock = 0b001,
    /// Trigger on external event on TBn.
    ExternalEvent = 0b010,
    /// Pulse Accumulate mode where the clock signal is gated on external input TBn.
    PulseAccumulate = 0b011,
    /// 32KHz clock.
    SlowSpeedClock = 0b100,
}

impl Default for ClockSource {
    fn default() -> Self {
        ClockSource::NoClock
    }
}

#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PulseAccumulateClockSelect {
    /// Prescaled APB1 clock. (The counter is frozen in sleep mode)
    PrescaledAPB1Clock = 0b0,
    /// LFCLK at 32KHz.
    LowFrequencyClock = 0b1,
}

impl Default for PulseAccumulateClockSelect {
    fn default() -> Self {
        PulseAccumulateClockSelect::PrescaledAPB1Clock
    }
}

#[derive(Default, Debug, Copy, Clone, Eq, PartialEq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ClockConfiguration {
    counter1_src: ClockSource,
    counter2_src: ClockSource,
    clkps: u8,
    pls_acc_clk: PulseAccumulateClockSelect,
    /// Enable timers in low power mode.
    low_pwr: bool,
}

#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Mode {
    /// PWM and Counter mode.
    Mode1 = 0b000,
    /// Dual-Input Capture mode.
    Mode2 = 0b001,
    /// Dual-Independent Timer mode.
    Mode3 = 0b010,
    /// Input Capture and Timer mode.
    Mode4 = 0b011,
    /// Dual-Independent Input Capture mode.
    Mode5 = 0b100,
    /// Duty Cycle Capture mode.
    Mode6 = 0b101,
}

impl Default for Mode {
    fn default() -> Self {
        Mode::Mode1
    }
}

#[derive(Default, Debug, Copy, Clone, Eq, PartialEq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Configuration {
    clk: ClockConfiguration,
    mode: Mode,
}

#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Counter {
    Counter1,
    Counter2,
}

struct Timer<'d, T: MultiFunctionInstance> {
    instance: PeripheralRef<'d, T>,
}

impl<'d, T: MultiFunctionInstance> Timer<'d, T> {
    pub fn new(instance: impl Peripheral<P = T> + 'd) -> Self {
        into_ref!(instance);
        Self { instance }
    }

    pub fn enable(&mut self, cfg: Configuration) {
        let r = T::regs();

        // Disable the clocksources before configuring.
        self.disable();

        r.tn_mctrl().write(|w| unsafe { w.mdsel().bits(cfg.mode as u8) });

        self.reset(Counter::Counter1);
        self.reset(Counter::Counter2);

        r.tn_prsc().write(|w| unsafe { w.bits(cfg.clk.clkps) });

        // Starts the clock.
        r.tn_ckc().write(|w| unsafe {
            w.low_pwr()
                .bit(cfg.clk.low_pwr)
                .pls_acc_clk()
                .bit(cfg.clk.pls_acc_clk as u8 == 0b1)
                .c1csel()
                .bits(cfg.clk.counter1_src as u8)
                .c2csel()
                .bits(cfg.clk.counter2_src as u8)
        });
    }

    pub fn disable(&mut self) {
        T::regs()
            .tn_ckc()
            .write(|w| unsafe { w.c1csel().bits(0b000).c2csel().bits(0b000) });
    }

    pub fn reset(&mut self, counter: Counter) {
        match counter {
            Counter::Counter1 => T::regs().tn_cnt1().write(|w| unsafe { w.bits(0x0000) }),
            Counter::Counter2 => T::regs().tn_cnt2().write(|w| unsafe { w.bits(0x0000) }),
        };
    }

    // pub fn start(&mut self) {
    //     T::regs().
    // }
}

impl<T: MultiFunctionInstance> Drop for Timer<'_, T> {
    fn drop(&mut self) {
        self.disable();
    }
}
