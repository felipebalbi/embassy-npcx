const MIWU_COUNT: usize = 3;

const fn get_miwu(n: usize) -> &'static crate::pac::miwu0::RegisterBlock {
    const MIWU_N: [*const crate::pac::miwu0::RegisterBlock; MIWU_COUNT] = [
        crate::pac::Miwu0::ptr(),
        crate::pac::Miwu1::ptr(),
        crate::pac::Miwu2::ptr(),
    ];

    let ptr = MIWU_N[n];
    // Safety:
    // the pac ptr functions return pointers to memory that is used for registers for the 'static lifetime
    // and the created reference is shared.
    unsafe { &*ptr }
}

pub enum Level {
    Low,
    High,
}

pub enum Edge {
    Any,
    Falling,
    Rising,
}

pub enum Mode {
    Level(Level),
    Edge(Edge),
}

mod sealed {
    pub trait SealedWakeUpInput {
        #[must_use]
        fn group(&self) -> usize;

        #[must_use]
        fn subgroup(&self) -> usize;

        #[must_use]
        fn port_n(&self) -> usize;

        #[must_use]
        fn interrupt(&self) -> crate::pac::Interrupt;

        fn port(&self) -> &'static crate::pac::miwu0::RegisterBlock {
            super::get_miwu(self.port_n())
        }
    }
}

pub trait WakeUpInput: sealed::SealedWakeUpInput {
    fn enable(&mut self, mode: Mode) {
        let port = self.port();

        critical_section::with(|_cs| {
            port.wkenn(self.group())
                .modify(|_, w| w.input(self.subgroup() as u8).disabled());

            let (wkmod, wkaedgn, wkedgn);

            use crate::pac::miwu0::*;
            match mode {
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

            port.wkmodn(self.group())
                .modify(|_, w| w.input(self.subgroup() as u8).variant(wkmod));

            if let Some(wkaedgn) = wkaedgn {
                port.wkaedgn(self.group())
                    .modify(|_, w| w.input(self.subgroup() as u8).variant(wkaedgn));
            }

            if let Some(wkedgn) = wkedgn {
                port.wkedgn(self.group())
                    .modify(|_, w| w.input(self.subgroup() as u8).variant(wkedgn));
            }

            port.wkinenn(self.group())
                .modify(|_, w| w.input(self.subgroup() as u8).enabled());
            port.wkpcln(self.group())
                .write(|w| w.input(self.subgroup() as u8).clear());
            port.wkenn(self.group())
                .modify(|_, w| w.input(self.subgroup() as u8).enabled());
        });
    }

    fn disable(&mut self) {
        critical_section::with(|_cs| {
            self.port()
                .wkenn(self.group())
                .modify(|_, w| w.input(self.subgroup() as u8).disabled());
        });
    }

    fn clear_pending(&mut self) {
        self.port()
            .wkpcln(self.group())
            .write(|w| w.input(self.subgroup() as u8).clear());
    }

    fn is_high(&self) -> bool {
        self.port()
            .wkstn(self.group())
            .read()
            .input(self.subgroup() as u8)
            .is_high()
    }

    fn is_pending(&self) -> bool {
        self.port()
            .wkpndn(self.group())
            .read()
            .input(self.subgroup() as u8)
            .is_pending()
    }
}

macro_rules! impl_wake_up_input {
    ($peripheral:ty, $miwu_n:expr, $group:expr, $subgroup:expr, $interrupt:ident) => {
        impl sealed::SealedWakeUpInput for $peripheral {
            fn group(&self) -> usize {
                $group
            }
            fn subgroup(&self) -> usize {
                $subgroup
            }
            fn port_n(&self) -> usize {
                $miwu_n
            }
            fn interrupt(&self) -> crate::pac::Interrupt {
                crate::pac::Interrupt::$interrupt
            }
        }
        impl WakeUpInput for $peripheral {}

        #[cfg(feature = "rt")]
        impl rt::WakeUpInputWaitable for $peripheral {}
    };
}

#[cfg(feature = "rt")]
pub mod rt {
    use super::*;
    use crate::pac::interrupt;
    use core::{
        future::Future,
        task::{Context, Poll},
    };
    use embassy_sync::waitqueue::AtomicWaker;

    // Note: having 196 wakers costs quite a bit of RAM.
    // If desired, change to or add intrusive linked list waker to save RAM.
    const SUBGROUP_COUNT: usize = 8;
    const GROUP_COUNT: usize = 8;
    const WUI_COUNT: usize = MIWU_COUNT * GROUP_COUNT * SUBGROUP_COUNT;
    static MIWU_WAKERS: [AtomicWaker; WUI_COUNT] = [const { AtomicWaker::new() }; WUI_COUNT];

    const fn get_wui_i(miwu_n: usize, group: usize, subgroup: usize) -> usize {
        miwu_n * SUBGROUP_COUNT * GROUP_COUNT + group * SUBGROUP_COUNT + subgroup
    }

    fn get_wui(miwu_n: usize, group: usize, subgroup: usize) -> &'static AtomicWaker {
        &MIWU_WAKERS[get_wui_i(miwu_n, group, subgroup)]
    }

    pub trait WakeUpInputWaitable: WakeUpInput + Sized {
        fn wait_for(&mut self, mode: Mode) -> impl Future {
            self.enable(mode);
            WakeUpInputFuture::<Self> { channel: self }
        }

        fn wait_for_high(&mut self) -> impl Future {
            self.wait_for(Mode::Level(Level::High))
        }

        fn wait_for_low(&mut self) -> impl Future {
            self.wait_for(Mode::Level(Level::Low))
        }
    }

    struct WakeUpInputFuture<'a, T: WakeUpInputWaitable> {
        channel: &'a mut T,
    }

    impl<'a, T: WakeUpInputWaitable> WakeUpInputFuture<'a, T> {
        fn waker(&self) -> &'static AtomicWaker {
            get_wui(self.channel.port_n(), self.channel.group(), self.channel.subgroup())
        }
    }

    impl<'a, T: WakeUpInputWaitable> Drop for WakeUpInputFuture<'a, T> {
        fn drop(&mut self) {
            // Clean up, and do not assume that the interrupt has run.
            self.channel.disable();
            self.channel.clear_pending();
        }
    }

    impl<'a, T: WakeUpInputWaitable> Future for WakeUpInputFuture<'a, T> {
        type Output = ();

        fn poll(self: core::pin::Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
            self.waker().register(cx.waker());

            if self.channel.is_pending() {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        }
    }

    struct BitIter(u8);

    impl Iterator for BitIter {
        type Item = u8;

        fn next(&mut self) -> Option<Self::Item> {
            match self.0.trailing_zeros() {
                8 => None,
                b => {
                    self.0 &= !(1 << b);
                    Some(b as u8)
                }
            }
        }
    }

    fn on_irq(miwu_n: usize, group: usize) {
        let port = get_miwu(miwu_n);

        let pending = port.wkpndn(group).read();
        for subgroup in BitIter(pending.bits()) {
            let waker = get_wui(miwu_n, group, subgroup as usize);
            waker.wake();
        }

        critical_section::with(|_cs| {
            port.wkenn(group)
                .modify(|r, w| unsafe { w.bits(r.bits() & !pending.bits()) });
        });
    }

    macro_rules! impl_irq {
        ($interrupt:ident, $miwu_n:literal, $group:literal) => {
            #[allow(non_snake_case)]
            #[interrupt]
            unsafe fn $interrupt() {
                on_irq($miwu_n, $group - 1) // The groups are 1-indexed
            }
        };
    }

    impl_irq!(WKINTA_0, 0, 1);
    impl_irq!(WKINTB_0, 0, 2);
    impl_irq!(WKINTC_0, 0, 3);
    impl_irq!(WKINTD_0, 0, 4);
    impl_irq!(WKINTE_0, 0, 5);
    impl_irq!(WKINTF_0, 0, 6);
    impl_irq!(WKINTG_0, 0, 7);
    impl_irq!(WKINTH_0, 0, 8);
    impl_irq!(WKINTA_1, 1, 1);
    impl_irq!(WKINTB_1, 1, 2);
    impl_irq!(WKINTC_1, 1, 3);
    impl_irq!(WKINTD_1, 1, 4);
    impl_irq!(WKINTE_1, 1, 5);
    impl_irq!(WKINTF_1, 1, 6);
    impl_irq!(WKINTG_1, 1, 7);
    impl_irq!(WKINTH_1, 1, 8);
    impl_irq!(WKINTA_2, 2, 1);
    impl_irq!(WKINTB_2, 2, 2);
    impl_irq!(WKINTC_2, 2, 3);
    impl_irq!(WKINTD_2, 2, 4);
    impl_irq!(WKINTE_2, 2, 5);
    impl_irq!(WKINTF_2, 2, 6);
    impl_irq!(WKINTG_2, 2, 7);
    impl_irq!(WKINTH_2, 2, 8);
}

use paste::paste;

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
