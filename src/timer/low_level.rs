//! Partial low-level implementation for the MFT16 timer.

use crate::{interrupt::typelevel::Interrupt, timer::MultiFunctionInstance};
use core::{future::poll_fn, marker::PhantomData, task::Poll};
use embassy_hal_internal::{into_ref, Peripheral, PeripheralRef};

#[derive(Default, Debug, Copy, Clone, Eq, PartialEq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ClockSource {
    /// Do not run this counter.
    #[default]
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

#[derive(Default, Debug, Copy, Clone, Eq, PartialEq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PulseAccumulateClockSelect {
    /// Prescaled APB1 clock. (The counter is frozen in sleep mode)
    #[default]
    PrescaledAPB1Clock = 0b0,
    /// LFCLK at 32KHz.
    LowFrequencyClock = 0b1,
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

#[derive(Default, Debug, Copy, Clone, Eq, PartialEq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Mode {
    /// PWM and Counter mode.
    Mode1 = 0b000,
    /// Dual-Input Capture mode.
    Mode2 = 0b001,
    /// Dual-Independent Timer mode.
    #[default]
    Mode3 = 0b010,
    /// Input Capture and Timer mode.
    Mode4 = 0b011,
    /// Dual-Independent Input Capture mode.
    Mode5 = 0b100,
    /// Duty Cycle Capture mode.
    Mode6 = 0b101,
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

#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum WakeUpEvent {
    A = 0,
    B,
    C,
    D,
    E,
    F,
}

impl WakeUpEvent {
    const fn mask(&self) -> u8 {
        1 << (*self as u8)
    }
}

#[derive(Default, Debug, Copy, Clone, Eq, PartialEq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct WakeUpEvents(u8);

impl WakeUpEvents {
    pub fn get(&self, event: WakeUpEvent) -> bool {
        self.0 & event.mask() != 0x0
    }

    pub fn set(&mut self, event: WakeUpEvent, value: bool) {
        if value {
            self.0 |= event.mask();
        } else {
            self.0 &= !event.mask();
        }
    }
}

/// Partial timer driver for one of three 16-bit MultiFunctionTimer(MFT16).
///
/// In practice this driver is mainly usable in Dual-Independent Timer mode.
/// Control of clock inputs and the various other modes has not yet been implemented.
pub struct MultiFunctionTimer<'d, T: MultiFunctionInstance> {
    _instance: PeripheralRef<'d, T>,
}

impl<'d, T: MultiFunctionInstance> MultiFunctionTimer<'d, T> {
    pub fn new(
        instance: impl Peripheral<P = T> + 'd,
        _irqs: impl crate::interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>>,
    ) -> Self {
        // Safety: _irqs ensures an interrupt handler is bound
        unsafe {
            T::Interrupt::enable();
        }

        into_ref!(instance);
        Self { _instance: instance }
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
        self.set_counter(counter, 0x0000);
    }

    pub fn counter(&self, counter: Counter) -> u16 {
        match counter {
            Counter::Counter1 => T::regs().tn_cnt1().read().bits(),
            Counter::Counter2 => T::regs().tn_cnt2().read().bits(),
        }
    }

    pub fn set_counter(&mut self, counter: Counter, value: u16) {
        match counter {
            Counter::Counter1 => T::regs().tn_cnt1().write(|w| unsafe { w.bits(value) }),
            Counter::Counter2 => T::regs().tn_cnt2().write(|w| unsafe { w.bits(value) }),
        };
    }

    pub fn set_compare(&mut self, counter: Counter, value: u16) {
        match counter {
            Counter::Counter1 => T::regs().tn_cpa().write(|w| unsafe { w.bits(value) }),
            Counter::Counter2 => T::regs().tn_cpb().write(|w| unsafe { w.bits(value) }),
        };
    }

    pub fn set_reload_capture(&mut self, counter: Counter, value: u16) {
        match counter {
            Counter::Counter1 => T::regs().tn_cra().write(|w| unsafe { w.bits(value) }),
            Counter::Counter2 => T::regs().tn_crb().write(|w| unsafe { w.bits(value) }),
        };
    }

    /// Await for at least a single wake up event to be pending.
    ///
    /// Will reset the pending event, deconfigure the interrupt, and will return all events that ended up triggering.
    pub async fn wait_for_single(&mut self, events: WakeUpEvents) -> WakeUpEvents {
        if events == WakeUpEvents::default() {
            return events; // Do not need to wait for any event.
        }

        // Reset enabled interrupt wake events to our set.
        // Note(cs): interrupt handler changes this register as well.
        critical_section::with(|_| {
            T::regs()
                .tn_wuen()
                .modify(|r, w| unsafe { w.bits((r.bits() & 0b1100_0000) | events.0) });
        });

        // Await until at least one of our events is pending.
        let pending = poll_fn(|cx| {
            T::waker().register(cx.waker());

            let pending = T::regs().tn_ectrl().read().bits();
            if pending & events.0 != 0x00 {
                // Note: interrupt was de-configured in interrupt handler.
                Poll::Ready(pending)
            } else {
                Poll::Pending
            }
        })
        .await;

        // Clear pending bits.
        T::regs().tn_eclr().write(|w| unsafe { w.bits(pending) });

        WakeUpEvents(pending)
    }
}

impl<T: MultiFunctionInstance> Drop for MultiFunctionTimer<'_, T> {
    fn drop(&mut self) {
        self.disable();
    }
}

pub struct InterruptHandler<T> {
    _phantom: PhantomData<T>,
}

impl<T: MultiFunctionInstance> crate::interrupt::typelevel::Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        T::waker().wake();
        // Deconfigure all wake-up events, but do not clear them.
        T::regs()
            .tn_wuen()
            .modify(|r, w| unsafe { w.bits(r.bits() & 0b1100_0000) });
    }
}
