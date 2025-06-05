//! Keyboard Scan.
//!
//! Implements support for the embedded 8 x 18 keyboard controller.

use crate::{gpio::Pin, interrupt::typelevel::Interrupt};

use core::{future::poll_fn, marker::PhantomData, task::Poll};

use embassy_hal_internal::Peri;
use embassy_sync::waitqueue::AtomicWaker;

/// Keyboard scan mode.
#[non_exhaustive]
#[derive(Clone, PartialEq, Eq)]
pub enum ScanMode {
    /// Manual scanning.
    Manual,

    /// Automatic scanning.
    Automatic,
}

impl From<ScanMode> for bool {
    fn from(value: ScanMode) -> bool {
        match value {
            ScanMode::Manual => false,
            ScanMode::Automatic => true,
        }
    }
}

/// Keyboard scan clock divisor.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum KbsClkDiv {
    /// Output is `APB1_CLK` / 1
    Div1 = 0,
    /// Output is `APB1_CLK` / 2
    Div2 = 1,
    /// Output is `APB1_CLK` / 3
    Div3 = 2,
    /// Output is `APB1_CLK` / 4
    Div4 = 3,
    /// Output is `APB1_CLK` / 5
    Div5 = 4,
    /// Output is `APB1_CLK` / 6
    Div6 = 5,
    /// Output is `APB1_CLK` / 7
    Div7 = 6,
    /// Output is `APB1_CLK` / 8
    Div8 = 7,
    /// Output is `APB1_CLK` / 9
    Div9 = 8,
    /// Output is `APB1_CLK` / 10
    Div10 = 9,
    /// Output is `APB1_CLK` / 11
    Div11 = 10,
    /// Output is `APB1_CLK` / 12
    Div12 = 11,
    /// Output is `APB1_CLK` / 13
    Div13 = 12,
    /// Output is `APB1_CLK` / 14
    Div14 = 13,
    /// Output is `APB1_CLK` / 15
    Div15 = 14,
    /// Output is `APB1_CLK` / 16
    Div16 = 15,
    /// Output is `APB1_CLK` / 17
    Div17 = 16,
    /// Output is `APB1_CLK` / 18
    Div18 = 17,
    /// Output is `APB1_CLK` / 19
    Div19 = 18,
    /// Output is `APB1_CLK` / 20
    Div20 = 19,
    /// Output is `APB1_CLK` / 21
    Div21 = 20,
    /// Output is `APB1_CLK` / 22
    Div22 = 21,
    /// Output is `APB1_CLK` / 23
    Div23 = 22,
    /// Output is `APB1_CLK` / 24
    Div24 = 23,
    /// Output is `APB1_CLK` / 25
    Div25 = 24,
    /// Output is `APB1_CLK` / 26
    Div26 = 25,
    /// Output is `APB1_CLK` / 27
    Div27 = 26,
    /// Output is `APB1_CLK` / 28
    Div28 = 27,
    /// Output is `APB1_CLK` / 29
    Div29 = 28,
    /// Output is `APB1_CLK` / 30
    Div30 = 29,
    /// Output is `APB1_CLK` / 31
    Div31 = 30,
    /// Output is `APB1_CLK` / 32
    Div32 = 31,
    /// Output is `APB1_CLK` / 33
    Div33 = 32,
    /// Output is `APB1_CLK` / 34
    Div34 = 33,
    /// Output is `APB1_CLK` / 35
    Div35 = 34,
    /// Output is `APB1_CLK` / 36
    Div36 = 35,
    /// Output is `APB1_CLK` / 37
    Div37 = 36,
    /// Output is `APB1_CLK` / 38
    Div38 = 37,
    /// Output is `APB1_CLK` / 39
    Div39 = 38,
    /// Output is `APB1_CLK` / 40
    Div40 = 39,
    /// Output is `APB1_CLK` / 41
    Div41 = 40,
    /// Output is `APB1_CLK` / 42
    Div42 = 41,
    /// Output is `APB1_CLK` / 43
    Div43 = 42,
    /// Output is `APB1_CLK` / 44
    Div44 = 43,
    /// Output is `APB1_CLK` / 45
    Div45 = 44,
    /// Output is `APB1_CLK` / 46
    Div46 = 45,
    /// Output is `APB1_CLK` / 47
    Div47 = 46,
    /// Output is `APB1_CLK` / 48
    Div48 = 47,
    /// Output is `APB1_CLK` / 49
    Div49 = 48,
    /// Output is `APB1_CLK` / 50
    Div50 = 49,
    /// Output is `APB1_CLK` / 51
    Div51 = 50,
    /// Output is `APB1_CLK` / 52
    Div52 = 51,
    /// Output is `APB1_CLK` / 53
    Div53 = 52,
    /// Output is `APB1_CLK` / 54
    Div54 = 53,
    /// Output is `APB1_CLK` / 55
    Div55 = 54,
    /// Output is `APB1_CLK` / 56
    Div56 = 55,
    /// Output is `APB1_CLK` / 57
    Div57 = 56,
    /// Output is `APB1_CLK` / 58
    Div58 = 57,
    /// Output is `APB1_CLK` / 59
    Div59 = 58,
    /// Output is `APB1_CLK` / 60
    Div60 = 59,
    /// Output is `APB1_CLK` / 61
    Div61 = 60,
    /// Output is `APB1_CLK` / 62
    Div62 = 61,
    /// Output is `APB1_CLK` / 63
    Div63 = 62,
    /// Output is `APB1_CLK` / 64
    Div64 = 63,
}

/// Keyboard configuration.
#[non_exhaustive]
#[derive(Clone)]
pub struct Config {
    /// Enable outputs high drive?
    pub outputs_high_drive: bool,

    /// Keyboard scan index auto-increment.
    ///
    /// When true, reading the keyboard scan buffer, automatically
    /// increments the index by 1.
    pub scan_index_inc: bool,

    /// Scanning mode for the keyboard matrix.
    pub scan_mode: ScanMode,

    /// Rise time delay in nanoseconds.
    pub rise_time_delay_ns: usize,

    /// Fall time delay in nanoseconds.
    pub fall_time_delay_ns: usize,

    /// Retry timeout in nanoseconds.
    ///
    /// Only used if keyboard is in automatic scanning
    /// mode. Additionally, must be strictly larger than
    /// `rise_time_delay_ns`.
    pub retry_timeout_ns: usize,

    /// Keyboard clock divisor
    pub clock_div: KbsClkDiv,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            outputs_high_drive: false,
            scan_index_inc: false,
            scan_mode: ScanMode::Manual,
            rise_time_delay_ns: 16 * 25,
            fall_time_delay_ns: 2 * 25,
            retry_timeout_ns: 255 * 25,
            clock_div: KbsClkDiv::Div25,
        }
    }
}

/// Keyboard errors
#[non_exhaustive]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// Timeout error.
    Timeout,
}

/// Keyboard scanner driver.
pub struct Keyboard<'a, T: Instance> {
    _peri: Peri<'a, T>,
}

impl<'a, T: Instance> Keyboard<'a, T> {
    /// Create an instance of the Keyboard driver.
    pub fn new(
        _peri: Peri<'a, T>,

        ksi0: Peri<'a, impl KbsIn>,
        ksi1: Peri<'a, impl KbsIn>,
        ksi2: Peri<'a, impl KbsIn>,
        ksi3: Peri<'a, impl KbsIn>,
        ksi4: Peri<'a, impl KbsIn>,
        ksi5: Peri<'a, impl KbsIn>,
        ksi6: Peri<'a, impl KbsIn>,
        ksi7: Peri<'a, impl KbsIn>,

        kso0: Peri<'a, impl KbsOut>,
        kso1: Peri<'a, impl KbsOut>,
        kso2: Peri<'a, impl KbsOut>,
        kso3: Peri<'a, impl KbsOut>,
        kso4: Peri<'a, impl KbsOut>,
        kso5: Peri<'a, impl KbsOut>,
        kso6: Peri<'a, impl KbsOut>,
        kso7: Peri<'a, impl KbsOut>,
        kso8: Peri<'a, impl KbsOut>,
        kso9: Peri<'a, impl KbsOut>,
        kso10: Peri<'a, impl KbsOut>,
        kso11: Peri<'a, impl KbsOut>,
        kso12: Peri<'a, impl KbsOut>,
        kso13: Peri<'a, impl KbsOut>,
        kso14: Peri<'a, impl KbsOut>,
        kso15: Peri<'a, impl KbsOut>,
        kso16: Peri<'a, impl KbsOut>,
        kso17: Peri<'a, impl KbsOut>,

        _irqs: impl crate::interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>>,
        config: Config,
    ) -> Self {
        let mut inst = Self { _peri };

        T::Interrupt::disable();
        T::Interrupt::unpend();

        inst.configure(config);

        critical_section::with(|cs| {
            // Safety: We have exclusive ownership over the peripherals.
            unsafe {
                ksi0.setup_pin(cs);
                ksi1.setup_pin(cs);
                ksi2.setup_pin(cs);
                ksi3.setup_pin(cs);
                ksi4.setup_pin(cs);
                ksi5.setup_pin(cs);
                ksi6.setup_pin(cs);
                ksi7.setup_pin(cs);

                kso0.setup_pin(cs);
                kso1.setup_pin(cs);
                kso2.setup_pin(cs);
                kso3.setup_pin(cs);
                kso4.setup_pin(cs);
                kso5.setup_pin(cs);
                kso6.setup_pin(cs);
                kso7.setup_pin(cs);
                kso8.setup_pin(cs);
                kso9.setup_pin(cs);
                kso10.setup_pin(cs);
                kso11.setup_pin(cs);
                kso12.setup_pin(cs);
                kso13.setup_pin(cs);
                kso14.setup_pin(cs);
                kso15.setup_pin(cs);
                kso16.setup_pin(cs);
                kso17.setup_pin(cs);
            };
        });

        // Safety: _irqs ensures an interrupt handler is bound
        unsafe {
            T::Interrupt::enable();
        }

        inst
    }

    fn configure(&mut self, _config: Config) {
        // Force it to automatic mode for now.
        // Clear potentially pending events
        T::regs().kbsevt().write(|w| w.kbsdone().set_bit().kbserr().set_bit());

        // Write CDIV
        T::regs().kbs_cfg_indx().write(|w| unsafe { w.bits(4) });
        T::regs().kbs_cfg_data().write(|w| unsafe { w.bits(5) });

        // Write CNUM
        T::regs().kbs_cfg_indx().write(|w| unsafe { w.bits(3) });
        T::regs().kbs_cfg_data().write(|w| unsafe { w.bits(18) });

        T::regs().kbsctl().modify(|_, w| w.kbsmode().set_bit());
    }

    /// Wait for automatic scanning and return buffer of all columns.
    pub async fn scan(&mut self) -> Result<[u8; 18], Error> {
        critical_section::with(|_| T::regs().kbsctl().modify(|_, w| w.kbsien().set_bit().start().set_bit()));

        // Read CDIV
        T::regs().kbs_cfg_indx().write(|w| unsafe { w.bits(4) });
        let cdiv = T::regs().kbs_cfg_data().read().bits();

        // Read CNUM
        T::regs().kbs_cfg_indx().write(|w| unsafe { w.bits(3) });
        let cnum = T::regs().kbs_cfg_data().read().bits();

        defmt::debug!("CTL {:x}", T::regs().kbsctl().read().bits());
        defmt::debug!("CNUM {}", cnum);
        defmt::debug!("CDIV {}", cdiv);

        poll_fn(move |cx| {
            T::waker().register(cx.waker());

            if T::regs().kbsevt().read().kbsdone().bit_is_set() {
                // Fix PAC to provide "clear_bit_by_one()"
                T::regs().kbsevt().write(|w| w.kbsdone().set_bit());

                if T::regs().kbsevt().read().kbserr().bit_is_set() {
                    // Fix PAC to provide "clear_bit_by_one()"
                    T::regs().kbsevt().write(|w| w.kbserr().set_bit());
                    Poll::Ready(Err(Error::Timeout))
                } else {
                    let mut bytes = [0; 18];

                    for (index, byte) in bytes.iter_mut().enumerate() {
                        // write index, read data
                        T::regs().kbs_buf_indx().write(|w| unsafe { w.bits(index as u8) });
                        *byte = T::regs().kbs_buf_data().read().bits();
                    }

                    Poll::Ready(Ok(bytes))
                }
            } else {
                Poll::Pending
            }
        })
        .await
    }
}

mod sealed {
    use embassy_hal_internal::PeripheralType;
    use embassy_sync::waitqueue::AtomicWaker;

    pub trait SealedInstance {
        fn regs() -> &'static crate::pac::kbs::RegisterBlock;
        fn waker() -> &'static AtomicWaker;
    }

    pub trait SealedKbs: PeripheralType {
        unsafe fn setup_pin(&self, cs: critical_section::CriticalSection);
    }
}

/// A marker trait implemented by all KbsIn pins
pub trait KbsIn: sealed::SealedKbs {}

/// A marker trait implemented by all KbsOut pins
pub trait KbsOut: sealed::SealedKbs {}

/// A marker trait implemented by the Keyboard Scanner peripheral
pub trait Instance: sealed::SealedInstance + embassy_hal_internal::PeripheralType {
    /// The interrupt used by this instance
    type Interrupt: crate::interrupt::typelevel::Interrupt;
}

/// The interrupt handler for the Keyboard Scanner driver.
pub struct InterruptHandler<T> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> crate::interrupt::typelevel::Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        T::regs().kbsctl().modify(|_, w| w.kbsien().clear_bit());
        T::waker().wake();
    }
}

macro_rules! impl_instance {
    ($instance:ident) => {
        impl sealed::SealedInstance for crate::peripherals::$instance {
            fn regs() -> &'static crate::pac::kbs::RegisterBlock {
                // Safety: not owned, memory is always present
                unsafe { &*crate::pac::Kbs::ptr() }
            }

            fn waker() -> &'static AtomicWaker {
                static WAKER: AtomicWaker = AtomicWaker::new();
                &WAKER
            }
        }

        impl Instance for crate::peripherals::$instance {
            type Interrupt = crate::interrupt::typelevel::KBS;
        }
    };
}

impl_instance!(KBS);

macro_rules! impl_kbsin {
    ($peri:ident, $pin_config:expr) => {
        impl sealed::SealedKbs for crate::peripherals::$peri {
            unsafe fn setup_pin(&self, _cs: critical_section::CriticalSection) {
                fn internal_set(f: impl FnOnce(crate::pac::Sysconfig)) {
                    f(unsafe { crate::pac::Sysconfig::steal() });
                }

                internal_set($pin_config);
            }
        }

        impl KbsIn for crate::peripherals::$peri {}
    };
}

macro_rules! impl_kbsout {
    ($peri:ident, $pin_config:expr) => {
        impl sealed::SealedKbs for crate::peripherals::$peri {
            unsafe fn setup_pin(&self, _cs: critical_section::CriticalSection) {
                fn internal_set(f: impl FnOnce(crate::pac::Sysconfig)) {
                    f(unsafe { crate::pac::Sysconfig::steal() });
                }

                internal_set($pin_config);
            }
        }

        impl KbsOut for crate::peripherals::$peri {}
    };
}

impl_kbsin!(PA02, |config| {
    config.devalt7().modify(|_, w| w.no_ksi0_sl().clear_bit());
    config.devaltl().modify(|_, w| w.ad15_sl().clear_bit());
    config.devalt5().modify(|_, w| w.trace_en().clear_bit());
});

impl_kbsin!(PA03, |config| {
    config.devalt7().modify(|_, w| w.no_ksi1_sl().clear_bit());
    config.devaltm().modify(|_, w| w.ad25_sl().clear_bit());
    config.devalt5().modify(|_, w| w.trace_en().clear_bit());
});

impl_kbsin!(PA04, |config| {
    config.devalt7().modify(|_, w| w.no_ksi2_sl().clear_bit());
    config.devalt5().modify(|_, w| w.trace_en().clear_bit());
    config.devaltl().modify(|_, w| w.ad14_sl().clear_bit());
});

impl_kbsin!(PB03, |config| {
    config.devalt7().modify(|_, w| w.no_ksi3_sl().clear_bit());
    config.devaltl().modify(|_, w| w.ad13_sl().clear_bit());
    config.devalt5().modify(|_, w| w.trace_en().clear_bit());
});

impl_kbsin!(PB04, |config| {
    config.devalt7().modify(|_, w| w.no_ksi4_sl().clear_bit());
    config.devaltm().modify(|_, w| w.ad24_sl().clear_bit());
    config.devalt5().modify(|_, w| w.trace_en().clear_bit());
});

impl_kbsin!(PC03, |config| {
    config.devalt7().modify(|_, w| w.no_ksi5_sl().clear_bit());
    config.devaltf().modify(|_, w| w.ad12_sl().clear_bit());
});

impl_kbsin!(PC04, |config| {
    config.devalt7().modify(|_, w| w.no_ksi6_sl().clear_bit());
    config.devaltm().modify(|_, w| w.ad21_sl().clear_bit());
    // Note: this messes with debug interfaces, maybe there is a better way?
    unsafe { crate::pac::Dev::steal() }
        .dbgctrl2()
        .modify(|_, w| w.ccdev_sel().bits(0));
});

impl_kbsin!(PC05, |config| {
    config.devalt7().modify(|_, w| w.no_ksi7_sl().clear_bit());
    config.devaltl().modify(|_, w| w.ad20_sl().clear_bit());
    // Note: this messes with debug interfaces, maybe there is a better way?
    unsafe { crate::pac::Dev::steal() }
        .dbgctrl2()
        .modify(|_, w| w.ccdev_sel().bits(0));
});

impl_kbsout!(PB05, |config| {
    config.devalt8().modify(|_, w| w.no_kso00_sl().clear_bit());
    // Note: this messes with debug interfaces, maybe there is a better way?
    config.jen_ctl1().modify(|_, w| w.jen_en().bits(6));
});

impl_kbsout!(PB06, |config| {
    config.devalt8().modify(|_, w| w.no_kso01_sl().clear_bit());
    // Note: this messes with debug interfaces, maybe there is a better way?
    config.jen_ctl1().modify(|_, w| w.jen_en().bits(6));
});

impl_kbsout!(PB07, |config| {
    config.devalt8().modify(|_, w| w.no_kso02_sl().clear_bit());
    // Note: this messes with debug interfaces, maybe there is a better way?
    config.jen_ctl1().modify(|_, w| w.jen_en().bits(6));
});

impl_kbsout!(PB08, |config| {
    config.devalt8().modify(|_, w| w.no_kso03_sl().clear_bit());
    // Note: this messes with debug interfaces, maybe there is a better way?
    config.jen_ctl1().modify(|_, w| w.jen_en().bits(6));
});

impl_kbsout!(PC07, |config| {
    config.devalt8().modify(|_, w| w.no_kso04_sl().set_bit());
    // No need to disconnect nTEST
});

impl_kbsout!(PC06, |config| {
    config.devalt8().modify(|_, w| w.no_kso05_sl().clear_bit());
});

impl_kbsout!(PC08, |config| {
    config.devalt8().modify(|_, w| w.no_kso06_sl().clear_bit());
});

impl_kbsout!(PB09, |config| {
    config.devalt8().modify(|_, w| w.no_kso07_sl().clear_bit());
});

impl_kbsout!(PC09, |config| {
    config.devalt9().modify(|_, w| w.no_kso08_sl().clear_bit());
    config.devaltj().modify(|_, w| w.cr_sout1_sl1().clear_bit());
});

impl_kbsout!(PC10, |config| {
    config.devalt9().modify(|_, w| w.no_kso09_sl().clear_bit());
    config.devaltj().modify(|_, w| w.cr_sin1_sl1().clear_bit());
});

impl_kbsout!(PB11, |config| {
    config.devalt9().modify(|_, w| w.no_kso10_sl().clear_bit());
});

impl_kbsout!(PB10, |config| {
    config.devalt9().modify(|_, w| w.no_kso11_sl().clear_bit());
});

impl_kbsout!(PC11, |config| {
    config.devalt9().modify(|_, w| w.no_kso12_sl().clear_bit());
});

impl_kbsout!(PD11, |config| {
    config.devalt9().modify(|_, w| w.no_kso13_sl().clear_bit());
});

impl_kbsout!(PD06, |config| {
    config.devalt9().modify(|_, w| w.no_kso14_sl().clear_bit());
});

impl_kbsout!(PD07, |config| {
    config.devalt9().modify(|_, w| w.no_kso15_sl().clear_bit());
});

impl_kbsout!(PD09, |config| {
    config.devalta().modify(|_, w| w.no_kso16_sl().clear_bit());
});

impl_kbsout!(PD08, |config| {
    config.devalta().modify(|_, w| w.no_kso17_sl().clear_bit());
    config.devalte().modify(|_, w| w.cr_sin4_sl().clear_bit());
});
