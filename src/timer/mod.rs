//! Drivers for the timers in this device.

pub mod low_level;

use embassy_sync::waitqueue::AtomicWaker;

mod sealed {
    use embassy_sync::waitqueue::AtomicWaker;

    pub trait SealedMultiFunctionInstance {
        fn waker() -> &'static AtomicWaker;
        fn regs() -> &'static crate::pac::mft16_1::RegisterBlock;
    }
}

pub trait MultiFunctionInstance: sealed::SealedMultiFunctionInstance {
    type Interrupt: crate::interrupt::typelevel::Interrupt;
}

macro_rules! impl_instance {
    ($instance:ident, $pac:ident) => {
        impl sealed::SealedMultiFunctionInstance for crate::peripherals::$instance {
            fn waker() -> &'static AtomicWaker {
                static WAKER: AtomicWaker = AtomicWaker::new();
                &WAKER
            }

            fn regs() -> &'static crate::pac::mft16_1::RegisterBlock {
                // Safety: not owned, memory is always present
                unsafe { &*crate::pac::$pac::PTR }
            }
        }

        impl MultiFunctionInstance for crate::peripherals::$instance {
            type Interrupt = crate::interrupt::typelevel::$instance;
        }
    };
}

impl_instance!(MFT16_1, Mft16_1);
impl_instance!(MFT16_2, Mft16_2);

#[cfg(not(feature = "time-driver-mft16-3"))]
impl_instance!(MFT16_3, Mft16_3);
