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
    }

    fn disable(&mut self) {
        self.port()
            .wkenn(self.group())
            .modify(|_, w| w.input(self.subgroup()).disabled());
        self.port()
            .wkinenn(self.group())
            .modify(|_, w| w.input(self.subgroup()).disabled());
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

impl_wake_up_input!(MIWU1_73, crate::pac::Miwu1::ptr(), 6, 3, WKINTG_1);

#[cfg(feature = "rt")]
mod rt {
    use crate::pac::interrupt;
    use embassy_sync::waitqueue::AtomicWaker;

    const MIWU_COUNT: usize = 3;
    const WUI_COUNT: usize = 64 * MIWU_COUNT;
    static MIWU_WAKERS: [AtomicWaker; WUI_COUNT] = [const { AtomicWaker::new() }; WUI_COUNT];

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

    fn on_irq(miwu_n: usize, miwu_group: usize) {
        let port = get_miwu(miwu_n);

        let pending = port.wkpndn(miwu_group).read();
        for wui in BitIter(pending.bits()) {
            let wui_i = miwu_n * miwu_group * (wui as usize);
            MIWU_WAKERS[wui_i].wake();
        }

        port.wkpcln(miwu_group).write(|w| unsafe { w.bits(pending.bits()) });
    }

    macro_rules! impl_irq {
        ($interrupt:ident, $miwu_n:literal, $miwu_group:literal) => {
            #[allow(non_snake_case)]
            #[interrupt]
            unsafe fn $interrupt() {
                on_irq($miwu_n, $miwu_group)
            }
        };
    }

    impl_irq!(WKINTA_0, 0, 0);
    impl_irq!(WKINTB_0, 0, 1);
    impl_irq!(WKINTC_0, 0, 2);
    impl_irq!(WKINTD_0, 0, 3);
    impl_irq!(WKINTE_0, 0, 4);
    impl_irq!(WKINTF_0, 0, 5);
    impl_irq!(WKINTG_0, 0, 6);
    impl_irq!(WKINTH_0, 0, 7);
    impl_irq!(WKINTA_1, 1, 0);
    impl_irq!(WKINTB_1, 1, 1);
    impl_irq!(WKINTC_1, 1, 2);
    impl_irq!(WKINTD_1, 1, 3);
    impl_irq!(WKINTE_1, 1, 4);
    impl_irq!(WKINTF_1, 1, 5);
    impl_irq!(WKINTG_1, 1, 6);
    impl_irq!(WKINTH_1, 1, 7);
    impl_irq!(WKINTA_2, 2, 0);
    impl_irq!(WKINTB_2, 2, 1);
    impl_irq!(WKINTC_2, 2, 2);
    impl_irq!(WKINTD_2, 2, 3);
    impl_irq!(WKINTE_2, 2, 4);
    impl_irq!(WKINTF_2, 2, 5);
    impl_irq!(WKINTG_2, 2, 6);
    impl_irq!(WKINTH_2, 2, 7);
}
