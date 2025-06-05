//! Multi-Input Wake-Up (MIWU) control for exiting power states, and signal conditioning of external interrupt sources.
//!
//! ## Opinionated interrupt
//! The interrupts implemented here will unset the `enable` bit, but leave the `pending` bit intact. It is the future
//! that clears this `pending` bit when polled or dropped.
//!
//! This means that if the interrupt is run, all pending WakeUpInputs are disabled, and need to be re-enabled if used for
//! exiting a low power state.
//!
//! # Use cases
//! * View [AwaitableInput](crate::gpio_miwu::AwaitableInput) to configure an pin interrupt.
//! * These WakeUpInputs can be consumed by the HAL implementation for specific peripherals unrelated to GPIO pins.

use embassy_hal_internal::{impl_peripheral, Peri};
use embassy_sync::waitqueue::AtomicWaker;
use paste::paste;

/// Signal level used as signalling condition.
pub enum Level {
    /// A low signal
    Low,
    /// A high signal
    High,
}

/// Signal edge used as signalling condition.
pub enum Edge {
    /// Both falling and rising edges
    Any,
    /// The transition from low to high
    Falling,
    /// The transition from high to low
    Rising,
}

/// Signalling condition on which the [WakeUp] input is triggered.
pub enum Mode {
    /// Trigger when the signal is at this level
    Level(Level),
    /// Trigger when the signal changes
    Edge(Edge),
}

impl From<Level> for Mode {
    fn from(value: Level) -> Self {
        Mode::Level(value)
    }
}

impl From<Edge> for Mode {
    fn from(value: Edge) -> Self {
        Mode::Edge(value)
    }
}

mod sealed {
    use embassy_hal_internal::PeripheralType;
    use embassy_sync::waitqueue::AtomicWaker;

    pub(crate) trait SealedWakeUpInput: PeripheralType {
        fn waker() -> &'static AtomicWaker;

        fn port() -> &'static crate::pac::miwu0::RegisterBlock;
        fn group() -> u8;
        fn subgroup() -> u8;
    }
}

/// WakeUpInput (WUI) trait.
#[allow(private_bounds)]
pub trait WakeUpInput: sealed::SealedWakeUpInput {
    /// The interrupt used by this instance
    type Interrupt: crate::interrupt::typelevel::Interrupt;
}

/// WakeUpInput (WUI) driver.
pub struct WakeUp<'d> {
    wui: Peri<'d, AnyWakeUpInput>,
}

impl<'d> WakeUp<'d> {
    /// Construct the WakeUp driver without enabling the signalling condition.
    pub fn new<P: WakeUpInput + 'd>(
        wui: Peri<'d, P>,
        _irqs: impl crate::interrupt::typelevel::Binding<P::Interrupt, InterruptHandler<P>>,
    ) -> Self {
        // Safety: _irqs ensures an interrupt handler is bound
        unsafe {
            use crate::interrupt::typelevel::Interrupt;
            P::Interrupt::enable();
        }

        Self { wui: wui.into() }
    }

    /// Enable the [WakeUpInput] with a specific signalling condition [Mode], enabling triggering the WakeUp signal and/or interrupt.
    pub fn enable(&mut self, mode: impl Into<Mode>) {
        let wui = self.wui.reborrow();

        let port = wui.port;
        let group = wui.group as usize;

        use crate::pac::miwu0::*;
        let (wkmod, wkaedgn, wkedgn);
        match mode.into() {
            Mode::Level(level) => {
                wkmod = wkmodn::InputMode::Level;
                wkaedgn = None;
                wkedgn = Some(match level {
                    Level::Low => wkedgn::Edge::LowFalling,
                    Level::High => wkedgn::Edge::HighRising,
                });
            }
            Mode::Edge(edge) => {
                wkmod = wkmodn::InputMode::Edge;
                (wkaedgn, wkedgn) = match edge {
                    Edge::Any => (Some(wkaedgn::AnyEdge::Any), None),
                    Edge::Falling => (Some(wkaedgn::AnyEdge::Edge), Some(wkedgn::Edge::LowFalling)),
                    Edge::Rising => (Some(wkaedgn::AnyEdge::Edge), Some(wkedgn::Edge::HighRising)),
                };
            }
        }

        // Note(cs): WakeUpInputs can share MIWU and group, which use the same registers.
        critical_section::with(|_cs| {
            port.wkenn(group).modify(|_, w| w.input(wui.subgroup).disabled());
            port.wkmodn(group).modify(|_, w| w.input(wui.subgroup).variant(wkmod));

            if let Some(wkaedgn) = wkaedgn {
                port.wkaedgn(group)
                    .modify(|_, w| w.input(wui.subgroup).variant(wkaedgn));
            }

            if let Some(wkedgn) = wkedgn {
                port.wkedgn(group).modify(|_, w| w.input(wui.subgroup).variant(wkedgn));
            }

            port.wkinenn(group).modify(|_, w| w.input(wui.subgroup).enabled());
            port.wkpcln(group).write(|w| w.input(wui.subgroup).clear());
            port.wkenn(group).modify(|_, w| w.input(wui.subgroup).enabled());
        });
    }

    /// Disable the [WakeUpInput], forbidding the WakeUp signal and/or interrupt.
    pub fn disable(&mut self) {
        let wui = self.wui.reborrow();
        // Note(cs): WakeUpInputs can share MIWU and group, which use the same registers.
        critical_section::with(|_cs| {
            wui.port
                .wkenn(wui.group as usize)
                .modify(|_, w| w.input(wui.subgroup).disabled());
        });
    }

    /// Make the signal no longer pending due to a previous trigger
    pub fn clear_pending(&mut self) {
        let wui = self.wui.reborrow();
        // Note(no-cs): atomic write to clear a single bit, safe.
        wui.port
            .wkpcln(wui.group as usize)
            .write(|w| w.input(wui.subgroup).clear());
    }

    /// Indicates whether the input signal, regardless of signalling condition, is high or not.
    pub fn is_high(&self) -> bool {
        let wui = &self.wui;
        wui.port.wkstn(wui.group as usize).read().input(wui.subgroup).is_high()
    }

    /// Indicates whether the input signalling condition set in [Mode] (example: rising edge) has been triggered.
    pub fn is_pending(&self) -> bool {
        let wui = &self.wui;
        wui.port
            .wkpndn(wui.group as usize)
            .read()
            .input(wui.subgroup)
            .is_pending()
    }
}

/// Disables the [WakeUpInput] signalling condition when dropped.
impl Drop for WakeUp<'_> {
    fn drop(&mut self) {
        self.disable();
    }
}

struct AnyWakeUpInput {
    waker: &'static AtomicWaker,
    port: &'static crate::pac::miwu0::RegisterBlock,
    group: u8,
    subgroup: u8,
}

// Allow use of PeripheralRef to do lifetime management
impl_peripheral!(AnyWakeUpInput);

impl<T: WakeUpInput> From<T> for AnyWakeUpInput {
    fn from(_value: T) -> Self {
        Self {
            waker: T::waker(),
            port: T::port(),
            group: T::group(),
            subgroup: T::subgroup(),
        }
    }
}

macro_rules! impl_wake_up_input {
    ($peripheral:ty, $miwu_n:expr, $group:expr, $subgroup:expr, $interrupt:ident) => {
        impl sealed::SealedWakeUpInput for $peripheral {
            fn waker() -> &'static AtomicWaker {
                static WAKER: AtomicWaker = AtomicWaker::new();
                &WAKER
            }

            fn port() -> &'static crate::pac::miwu0::RegisterBlock {
                let ptr = paste! { crate::pac::[<Miwu $miwu_n>]::ptr() };

                // Safety:
                // the pac ptr functions return pointers to memory that is used for registers for the 'static lifetime
                // and the created reference is shared.
                unsafe { &*ptr }
            }

            fn group() -> u8 {
                $group
            }
            fn subgroup() -> u8 {
                $subgroup
            }
        }
        impl WakeUpInput for $peripheral {
            type Interrupt = crate::interrupt::typelevel::$interrupt;
        }
    };
}

use core::future::Future;
use core::marker::PhantomData;
use core::task::{Context, Poll};

impl<'d> WakeUp<'d> {
    /// Configures a specific signalling condition [Mode] and awaits for it to be signalled.
    pub async fn wait_for(&mut self, mode: impl Into<Mode>) {
        self.enable(mode);
        WakeUpInputFuture::<'_, 'd> { channel: self }.await
    }

    /// Configures the [Level::High] signalling condition and awaits for it to be signalled.
    pub async fn wait_for_high(&mut self) {
        self.wait_for(Level::High).await
    }

    /// Configures the [Level::Low] signalling condition and awaits for it to be signalled.
    pub async fn wait_for_low(&mut self) {
        self.wait_for(Level::Low).await
    }
}

struct WakeUpInputFuture<'a, 'd> {
    channel: &'a mut WakeUp<'d>,
}

impl Drop for WakeUpInputFuture<'_, '_> {
    fn drop(&mut self) {
        // Clean up, and do not assume that the interrupt has run.
        self.channel.disable();
        self.channel.clear_pending();
    }
}

impl Future for WakeUpInputFuture<'_, '_> {
    type Output = ();

    fn poll(self: core::pin::Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        self.channel.wui.waker.register(cx.waker());

        if self.channel.is_pending() {
            Poll::Ready(())
        } else {
            Poll::Pending
        }
    }
}

/// Interrupt handler for the [WakeUp] driver
pub struct InterruptHandler<T> {
    _phantom: PhantomData<T>,
}

impl<T: WakeUpInput> crate::interrupt::typelevel::Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        let port = T::port();
        let group = T::group() as usize;

        let pending = port.wkpndn(group).read();
        if pending.input(T::subgroup()).bit_is_set() {
            T::waker().wake();

            // Note(cs): other tasks can be modifying the same register.
            critical_section::with(|_cs| {
                port.wkenn(group).modify(|_, w| w.input(T::subgroup()).clear_bit());
            });
        }
    }
}

macro_rules! impl_wake_up_input_n {
    ($miwu_n:literal, $group:literal, $subgroup:literal, $interrupt:ident) => {
        paste! { impl_wake_up_input!(
                crate::peripherals::[<MIWU $miwu_n _ $group $subgroup>],
                $miwu_n,
                ($group - 1), // The groups are 1-indexed
                $subgroup,
                $interrupt
            );
        }
    };
}

macro_rules! impl_wake_up_input_nm {
    ($miwu_n:literal, $group:literal, $interrupt:ident) => {
        impl_wake_up_input_n!($miwu_n, $group, 0, $interrupt);
        impl_wake_up_input_n!($miwu_n, $group, 1, $interrupt);
        impl_wake_up_input_n!($miwu_n, $group, 2, $interrupt);
        impl_wake_up_input_n!($miwu_n, $group, 3, $interrupt);
        impl_wake_up_input_n!($miwu_n, $group, 4, $interrupt);
        impl_wake_up_input_n!($miwu_n, $group, 5, $interrupt);
        impl_wake_up_input_n!($miwu_n, $group, 6, $interrupt);
        impl_wake_up_input_n!($miwu_n, $group, 7, $interrupt);
    };
}

impl_wake_up_input_nm!(0, 1, WKINTA_0);
impl_wake_up_input_nm!(0, 2, WKINTB_0);
impl_wake_up_input_nm!(0, 3, WKINTC_0);
impl_wake_up_input_nm!(0, 4, WKINTD_0);
impl_wake_up_input_nm!(0, 5, WKINTE_0);
impl_wake_up_input_nm!(0, 6, WKINTF_0);
impl_wake_up_input_nm!(0, 7, WKINTG_0);
impl_wake_up_input_nm!(0, 8, WKINTH_0);
impl_wake_up_input_nm!(1, 1, WKINTA_1);
impl_wake_up_input_nm!(1, 2, WKINTB_1);
impl_wake_up_input_nm!(1, 3, WKINTC_1);
impl_wake_up_input_nm!(1, 4, WKINTD_1);
impl_wake_up_input_nm!(1, 5, WKINTE_1);
impl_wake_up_input_nm!(1, 6, WKINTF_1);
impl_wake_up_input_nm!(1, 7, WKINTG_1);
impl_wake_up_input_nm!(1, 8, WKINTH_1);
impl_wake_up_input_nm!(2, 1, WKINTA_2);
impl_wake_up_input_nm!(2, 2, WKINTB_2);
impl_wake_up_input_nm!(2, 3, WKINTC_2);
impl_wake_up_input_nm!(2, 4, WKINTD_2);
impl_wake_up_input_nm!(2, 5, WKINTE_2);
impl_wake_up_input_nm!(2, 6, WKINTF_2);
impl_wake_up_input_nm!(2, 7, WKINTG_2);
impl_wake_up_input_nm!(2, 8, WKINTH_2);
