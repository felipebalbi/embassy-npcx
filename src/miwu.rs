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
        fn subgroup(&self) -> u8;

        #[must_use]
        fn port(&self) -> &'static crate::pac::miwu0::RegisterBlock;

        #[must_use]
        fn interrupt(&self) -> crate::pac::Interrupt;
    }
}

pub trait WakeUpInput: sealed::SealedWakeUpInput {
    fn enable(&mut self, mode: Mode) {
        critical_section::with(|_| {
            let port = self.port();

            port.wkenn(self.group())
                .modify(|_, w| w.input(self.subgroup()).disabled());

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
                        Edge::Falling => (Some(wkaedgn::AnyEdge::Any), Some(wkedgn::Edge::LowFalling)),
                        Edge::Rising => (Some(wkaedgn::AnyEdge::Any), Some(wkedgn::Edge::HighRising)),
                    };
                }
            }

            port.wkmodn(self.group())
                .modify(|_, w| w.input(self.subgroup()).variant(wkmod));

            if let Some(wkaedgn) = wkaedgn {
                port.wkaedgn(self.group())
                    .modify(|_, w| w.input(self.subgroup()).variant(wkaedgn));
            }

            if let Some(wkedgn) = wkedgn {
                port.wkedgn(self.group())
                    .modify(|_, w| w.input(self.subgroup()).variant(wkedgn));
            }

            port.wkinenn(self.group())
                .modify(|_, w| w.input(self.subgroup()).enabled());
            port.wkpcln(self.group()).write(|w| w.input(self.subgroup()).clear());
            port.wkenn(self.group())
                .modify(|_, w| w.input(self.subgroup()).enabled());
        });
    }

    fn disable(&mut self) {
        let port = self.port();

        critical_section::with(|_| {
            port.wkenn(self.group())
                .modify(|_, w| w.input(self.subgroup()).disabled());
            port.wkinenn(self.group())
                .modify(|_, w| w.input(self.subgroup()).disabled());
        });
    }

    fn is_high(&self) -> bool {
        self.port().wkstn(self.group()).read().input(self.subgroup()).is_high()
    }

    fn is_pending(&self) -> bool {
        self.port()
            .wkpndn(self.group())
            .read()
            .input(self.subgroup())
            .is_pending()
    }
}

macro_rules! impl_wake_up_input {
    ($peripheral:ident, $port:expr, $group:expr, $subgroup:expr, $interrupt:ident) => {
        impl sealed::SealedWakeUpInput for crate::peripherals::$peripheral {
            fn group(&self) -> usize {
                $group
            }
            fn subgroup(&self) -> u8 {
                $subgroup
            }
            fn port(&self) -> &'static crate::pac::miwu0::RegisterBlock {
                let ptr = $port;

                // Safety:
                // the pac ptr functions return pointers to memory that is used for registers for the 'static lifetime
                // and the created reference is shared.
                unsafe { &*ptr }
            }
            fn interrupt(&self) -> crate::pac::Interrupt {
                crate::pac::Interrupt::$interrupt
            }
        }
        impl WakeUpInput for crate::peripherals::$peripheral {}
    };
}

// MIWU1 - WUI73 - WKINTG_1

impl_wake_up_input!(MIWU1_73, crate::pac::Miwu1::ptr(), 7, 3, WKINTG_1);
