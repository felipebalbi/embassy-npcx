use core::marker::PhantomData;

use embassy_hal_internal::{into_ref, Peripheral, PeripheralRef};

#[derive(Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub enum Level {
    Low,
    High,
}

impl From<Level> for bool {
    fn from(value: Level) -> Self {
        match value {
            Level::Low => false,
            Level::High => true,
        }
    }
}

impl From<bool> for Level {
    fn from(value: bool) -> Self {
        match value {
            false => Level::Low,
            true => Level::High,
        }
    }
}

mod sealed {
    pub trait SealedPin {
        #[must_use]
        fn pin(&self) -> u8;

        #[must_use]
        fn port(&self) -> &'static crate::pac::gpio0::RegisterBlock;

        // Not ideal to mark this unsafe, but PeripheralRef is missing DerefMut...
        /// Safety assumptions:
        /// caller must ensure it has a mutable reference to the pin or
        /// has otherwise elimnated contention.
        unsafe fn set_pin_function(&self, cs: critical_section::CriticalSection);

        // Not ideal to mark this unsafe, but PeripheralRef is missing DerefMut...
        /// Change whether the pin is low voltage, may do nothing on pins without
        /// low voltage support.
        /// Safety assumptions:
        /// caller must ensure it has a mutable reference to the pin or
        /// has otherwise elimnated contention.
        /// caller must ensure this is not called to enable low voltage when the pin
        /// is in a state where it can be damaged by doing that.
        unsafe fn set_low_voltage(&self, cs: critical_section::CriticalSection, state: bool);
    }

    pub trait SealedInputPin {}

    pub trait SealedLowVoltagePin {}
}

struct AnyPin {
    pin: u8,
    port: &'static crate::pac::gpio0::RegisterBlock,
}

impl AnyPin {
    #[must_use]
    fn pin(&self) -> u8 {
        self.pin
    }

    #[must_use]
    fn port(&self) -> &'static crate::pac::gpio0::RegisterBlock {
        self.port
    }
}

impl<T: Pin> From<T> for AnyPin {
    fn from(pin: T) -> Self {
        AnyPin {
            pin: pin.pin(),
            port: pin.port(),
        }
    }
}

// Allow use of PeripheralRef to do lifetime management
impl Peripheral for AnyPin {
    type P = AnyPin;

    unsafe fn clone_unchecked(&self) -> Self::P {
        AnyPin {
            pin: self.pin,
            port: self.port,
        }
    }
}

pub struct OutputOnly {}

pub struct InputCapable {}

pub struct CanPullUp {}

pub struct PullDownOnly {}

pub struct Input<'d, T> {
    pin: PeripheralRef<'d, AnyPin>,
    _phantom: PhantomData<T>,
}

impl<'d> Input<'d, CanPullUp> {
    pub fn new(pin: impl Peripheral<P = impl InputPin + 'd> + 'd) -> Self {
        into_ref!(pin);

        critical_section::with(|cs| {
            // Safety:
            // We have a mutable reference to the pin through PeripheralRef
            unsafe { pin.set_low_voltage(cs, false) };
            unsafe { pin.set_pin_function(cs) };

            let regs = pin.port();

            regs.px_dir().modify(|_, w| w.pin(pin.pin()).input());
        });

        Input {
            pin: pin.map_into(),
            _phantom: PhantomData,
        }
    }

    pub fn enable_pullup(&mut self) {
        critical_section::with(|_| {
            let regs = self.pin.port();

            regs.px_pud().modify(|_, w| w.pin(self.pin.pin()).pull_up());
            regs.px_pull().modify(|_, w| w.pin(self.pin.pin()).enabled());
        });
    }

    #[must_use]
    pub fn degrade(self) -> Input<'d, PullDownOnly> {
        Input {
            pin: self.pin,
            _phantom: PhantomData,
        }
    }
}

impl<'d> Input<'d, PullDownOnly> {
    pub fn new_lowvoltage(pin: impl Peripheral<P = impl LowVoltagePin + 'd> + 'd) -> Self {
        into_ref!(pin);

        critical_section::with(|cs| {
            let regs = pin.port();

            regs.px_dir().modify(|_, w| w.pin(pin.pin()).input());
            regs.px_pud().modify(|_, w| w.pin(pin.pin()).pull_down());
            // Required for low voltage operations
            regs.px_otype().modify(|_, w| w.pin(pin.pin()).opendrain());

            // Safety:
            // We have a mutable reference to the pin through PeripheralRef
            // Everything is put in a safe state for low voltage above
            unsafe { pin.set_pin_function(cs) };
            unsafe { pin.set_low_voltage(cs, true) };
        });

        Self {
            pin: pin.map_into(),
            _phantom: PhantomData,
        }
    }
}

impl<T> Input<'_, T> {
    pub fn disable_pull(&mut self) {
        critical_section::with(|_| {
            let regs = self.pin.port();

            regs.px_pull().modify(|_, w| w.pin(self.pin.pin()).disabled());
        });
    }

    pub fn enable_pulldown(&mut self) {
        critical_section::with(|_| {
            let regs = self.pin.port();

            regs.px_pud().modify(|_, w| w.pin(self.pin.pin()).pull_down());
            regs.px_pull().modify(|_, w| w.pin(self.pin.pin()).enabled());
        });
    }

    #[must_use]
    pub fn is_low(&self) -> bool {
        let regs = self.pin.port();

        regs.px_din().read().pin(self.pin.pin()).is_low()
    }

    #[must_use]
    pub fn is_high(&self) -> bool {
        let regs = self.pin.port();

        regs.px_din().read().pin(self.pin.pin()).is_high()
    }
}

pub struct OutputOpenDrain<'d, T> {
    pin: PeripheralRef<'d, AnyPin>,
    _phantom: PhantomData<T>,
}

impl<'d> OutputOpenDrain<'d, OutputOnly> {
    pub fn new(pin: impl Peripheral<P = impl Pin + 'd> + 'd, level: Level) -> Self {
        into_ref!(pin);

        critical_section::with(|cs| {
            // Safety:
            // We have a mutable reference to the pin through PeripheralRef
            unsafe { pin.set_low_voltage(cs, false) };
            unsafe { pin.set_pin_function(cs) };

            let regs = pin.port();

            // Set data and output direction
            regs.px_dout().modify(|_, w| w.pin(pin.pin()).bit(level.into()));
            regs.px_otype().modify(|_, w| w.pin(pin.pin()).opendrain());
            regs.px_dir().modify(|_, w| w.pin(pin.pin()).output());
        });

        OutputOpenDrain {
            pin: pin.map_into(),
            _phantom: PhantomData,
        }
    }
}

impl<'d> OutputOpenDrain<'d, InputCapable> {
    pub fn new(pin: impl Peripheral<P = impl InputPin + 'd> + 'd, level: Level) -> Self {
        into_ref!(pin);

        critical_section::with(|cs| {
            // Safety:
            // We have a mutable reference to the pin through PeripheralRef
            unsafe { pin.set_low_voltage(cs, false) };
            unsafe { pin.set_pin_function(cs) };

            let regs = pin.port();

            // Set data and output direction
            regs.px_dout().modify(|_, w| w.pin(pin.pin()).bit(level.into()));
            regs.px_otype().modify(|_, w| w.pin(pin.pin()).opendrain());
            regs.px_dir().modify(|_, w| w.pin(pin.pin()).output());
        });

        OutputOpenDrain {
            pin: pin.map_into(),
            _phantom: PhantomData,
        }
    }

    pub fn degrade(self) -> OutputOpenDrain<'d, OutputOnly> {
        OutputOpenDrain {
            pin: self.pin,
            _phantom: PhantomData,
        }
    }
}

impl<T> OutputOpenDrain<'_, T> {
    pub fn set_low(&mut self) {
        critical_section::with(|_| {
            let regs = self.pin.port();

            // Set data
            regs.px_dout().modify(|_, w| w.pin(self.pin.pin()).clear_bit());
        });
    }

    pub fn set_high(&mut self) {
        critical_section::with(|_| {
            let regs = self.pin.port();

            // Set data
            regs.px_dout().modify(|_, w| w.pin(self.pin.pin()).set_bit());
        });
    }

    pub fn set_value(&mut self, value: Level) {
        critical_section::with(|_| {
            let regs = self.pin.port();

            // Set data
            regs.px_dout().modify(|_, w| w.pin(self.pin.pin()).bit(value.into()));
        });
    }

    #[must_use]
    pub fn is_set_low(&mut self) -> bool {
        let regs = self.pin.port();

        regs.px_dout().read().pin(self.pin.pin()).is_low()
    }

    #[must_use]
    pub fn is_set_high(&mut self) -> bool {
        let regs = self.pin.port();

        regs.px_dout().read().pin(self.pin.pin()).is_high()
    }

    pub fn toggle(&mut self) {
        critical_section::with(|_| {
            let regs = self.pin.port();

            regs.px_dout()
                .modify(|r, w| w.pin(self.pin.pin()).bit(r.pin(self.pin.pin()).is_low()));
        });
    }
}

impl OutputOpenDrain<'_, InputCapable> {
    #[must_use]
    pub fn is_low(&self) -> bool {
        let regs = self.pin.port();

        regs.px_din().read().pin(self.pin.pin()).is_low()
    }

    #[must_use]
    pub fn is_high(&self) -> bool {
        let regs = self.pin.port();

        regs.px_din().read().pin(self.pin.pin()).is_high()
    }

    pub fn drive_always(&self) {
        critical_section::with(|_| {
            let regs = self.pin.port();
            regs.px_envdd().modify(|_, w| w.pin(self.pin.pin()).disabled());
        })
    }

    pub fn drive_when_vdd_present(&self) {
        critical_section::with(|_| {
            let regs = self.pin.port();
            regs.px_envdd().modify(|_, w| w.pin(self.pin.pin()).enabled());
        })
    }
}

pub struct Output<'d, T> {
    pin: PeripheralRef<'d, AnyPin>,
    _phantom: PhantomData<T>,
}

impl<'d> Output<'d, OutputOnly> {
    pub fn new(pin: impl Peripheral<P = impl Pin + 'd> + 'd, level: Level) -> Self {
        into_ref!(pin);

        critical_section::with(|cs| {
            // Safety:
            // We have a mutable reference to the pin through PeripheralRef
            unsafe { pin.set_low_voltage(cs, false) };
            unsafe { pin.set_pin_function(cs) };

            let regs = pin.port();

            // Set data and output direction
            regs.px_dout().modify(|_, w| w.pin(pin.pin()).bit(level.into()));
            regs.px_otype().modify(|_, w| w.pin(pin.pin()).pushpull());
            regs.px_dir().modify(|_, w| w.pin(pin.pin()).output());
        });

        Output {
            pin: pin.map_into(),
            _phantom: PhantomData,
        }
    }
}

impl<'d> Output<'d, InputCapable> {
    pub fn new(pin: impl Peripheral<P = impl InputPin + 'd> + 'd, level: Level) -> Self {
        into_ref!(pin);

        critical_section::with(|cs| {
            // Safety:
            // We have a mutable reference to the pin through PeripheralRef
            unsafe { pin.set_low_voltage(cs, false) };
            unsafe { pin.set_pin_function(cs) };

            let regs = pin.port();

            // Set data and output direction
            regs.px_dout().modify(|_, w| w.pin(pin.pin()).bit(level.into()));
            regs.px_otype().modify(|_, w| w.pin(pin.pin()).pushpull());
            regs.px_dir().modify(|_, w| w.pin(pin.pin()).output());
        });

        Output {
            pin: pin.map_into(),
            _phantom: PhantomData,
        }
    }

    pub fn new_lowvoltage(pin: impl Peripheral<P = impl LowVoltagePin + 'd> + 'd, level: Level) -> Self {
        into_ref!(pin);

        critical_section::with(|cs| {
            let regs = pin.port();

            // Required for low voltage operation
            regs.px_pud().modify(|_, w| w.pin(pin.pin()).pull_down());
            regs.px_otype().modify(|_, w| w.pin(pin.pin()).opendrain());
            regs.px_dir().modify(|_, w| w.pin(pin.pin()).output());
            regs.px_dout().modify(|_, w| w.pin(pin.pin()).bit(level.into()));

            // Safety:
            // We have a mutable reference to the pin through PeripheralRef
            // Everything is put in a safe state for low voltage above
            unsafe { pin.set_pin_function(cs) };
            unsafe { pin.set_low_voltage(cs, true) };
        });

        Self {
            pin: pin.map_into(),
            _phantom: PhantomData,
        }
    }

    #[must_use]
    pub fn degrade(self) -> Output<'d, OutputOnly> {
        Output {
            pin: self.pin,
            _phantom: PhantomData,
        }
    }
}

impl<T> Output<'_, T> {
    pub fn set_low(&mut self) {
        critical_section::with(|_| {
            let regs = self.pin.port();

            // Set data
            regs.px_dout().modify(|_, w| w.pin(self.pin.pin()).clear_bit());
        });
    }

    pub fn set_high(&mut self) {
        critical_section::with(|_| {
            let regs = self.pin.port();

            // Set data
            regs.px_dout().modify(|_, w| w.pin(self.pin.pin()).set_bit());
        });
    }

    pub fn set_value(&mut self, value: Level) {
        critical_section::with(|_| {
            let regs = self.pin.port();

            // Set data
            regs.px_dout().modify(|_, w| w.pin(self.pin.pin()).bit(value.into()));
        });
    }

    #[must_use]
    pub fn is_set_low(&mut self) -> bool {
        let regs = self.pin.port();

        regs.px_dout().read().pin(self.pin.pin()).is_low()
    }

    #[must_use]
    pub fn is_set_high(&mut self) -> bool {
        let regs = self.pin.port();

        regs.px_dout().read().pin(self.pin.pin()).is_high()
    }

    pub fn toggle(&mut self) {
        critical_section::with(|_| {
            let regs = self.pin.port();

            regs.px_dout()
                .modify(|r, w| w.pin(self.pin.pin()).bit(r.pin(self.pin.pin()).is_low()));
        });
    }
}

impl Output<'_, InputCapable> {
    pub fn drive_always(&self) {
        critical_section::with(|_| {
            let regs = self.pin.port();
            regs.px_envdd().modify(|_, w| w.pin(self.pin.pin()).disabled());
        })
    }

    pub fn drive_when_vdd_present(&self) {
        critical_section::with(|_| {
            let regs = self.pin.port();
            regs.px_envdd().modify(|_, w| w.pin(self.pin.pin()).enabled());
        })
    }
}

pub trait LowVoltagePin: InputPin + sealed::SealedLowVoltagePin {}

pub trait InputPin: Pin + sealed::SealedInputPin {}

pub trait Pin: sealed::SealedPin {}

macro_rules! impl_pin {
    ($peripheral:ident, $port:expr, $pin:expr, $pin_function:expr) => {
        impl_pin!($peripheral, $port, $pin, $pin_function, |_, _| {});
    };
    ($peripheral:ident, $port:expr, $pin:expr, $pin_function:expr, $low_voltage_function:expr) => {
        impl sealed::SealedPin for crate::peripherals::$peripheral {
            fn pin(&self) -> u8 {
                $pin
            }

            fn port(&self) -> &'static crate::pac::gpio0::RegisterBlock {
                let ptr = $port;

                // Safety:
                // the pac ptr functions return pointers to memory that is used for registers for the 'static lifetime
                // and the created reference is shared.
                unsafe { &*ptr }
            }

            unsafe fn set_pin_function(&self, _cs: critical_section::CriticalSection) {
                // Safety:
                // We have the unique reference to the pin and are within a critical section, so no contention
                fn internal_set(f: impl FnOnce(crate::pac::Sysconfig)) {
                    f(unsafe { crate::pac::Sysconfig::steal() });
                }
                internal_set($pin_function);
            }

            unsafe fn set_low_voltage(&self, _cs: critical_section::CriticalSection, state: bool) {
                // Safety:
                // We have the unique reference to the pin and are within a critical section, so no contention
                fn internal_set(f: impl FnOnce(crate::pac::Sysconfig, bool), state: bool) {
                    f(unsafe { crate::pac::Sysconfig::steal() }, state);
                }
                internal_set($low_voltage_function, state);
            }
        }
        impl Pin for crate::peripherals::$peripheral {}
    };
}

macro_rules! impl_input_pin {
    ($peripheral:ident, $port:expr, $pin:expr, $pin_function:expr) => {
        impl_input_pin!($peripheral, $port, $pin, $pin_function, |_, _| {});
    };
    ($peripheral:ident, $port:expr, $pin:expr, $pin_function:expr, $low_voltage_function:expr) => {
        impl_pin!($peripheral, $port, $pin, $pin_function, $low_voltage_function);
        impl sealed::SealedInputPin for crate::peripherals::$peripheral {}
        impl InputPin for crate::peripherals::$peripheral {}
    };
}

macro_rules! impl_lowvoltage_pin {
    ($peripheral:ident, $port:expr, $pin:expr, $pin_function:expr, $low_voltage_function:expr) => {
        impl_input_pin!($peripheral, $port, $pin, $pin_function, $low_voltage_function);
        impl sealed::SealedLowVoltagePin for crate::peripherals::$peripheral {}
        impl LowVoltagePin for crate::peripherals::$peripheral {}
    };
}

// GPIO0
impl_lowvoltage_pin!(
    PE07,
    crate::pac::Gpio0::ptr(),
    0,
    |regs| {
        regs.devaltd().modify(|_, w| w.n_psl_in2_sl().set_bit());
        // No need to disconnect EXT_PURST
    },
    |regs, state| {
        regs.lv_gpio_ctl0().modify(|_, w| w.g00_lv().bit(state));
    }
);
impl_lowvoltage_pin!(
    PE06,
    crate::pac::Gpio0::ptr(),
    1,
    |regs| {
        regs.devaltd().modify(|_, w| w.psl_in3_sl().clear_bit());
    },
    |regs, state| {
        regs.lv_gpio_ctl5().modify(|_, w| w.g01_lv().bit(state));
    }
);
impl_lowvoltage_pin!(
    PF07,
    crate::pac::Gpio0::ptr(),
    2,
    |regs| {
        regs.devaltd().modify(|_, w| w.psl_in4_sl().clear_bit());
    },
    |regs, state| {
        regs.lv_gpio_ctl5().modify(|_, w| w.g02_lv().bit(state));
    }
);
impl_lowvoltage_pin!(
    PD09,
    crate::pac::Gpio0::ptr(),
    3,
    |regs| {
        regs.devalta().modify(|_, w| w.no_kso16_sl().set_bit());
    },
    |regs, state| {
        regs.lv_gpio_ctl6().modify(|_, w| w.g03_lv().bit(state));
    }
);
impl_lowvoltage_pin!(
    PD11,
    crate::pac::Gpio0::ptr(),
    4,
    |regs| {
        regs.devalt9().modify(|_, w| w.no_kso13_sl().set_bit());
    },
    |regs, state| {
        regs.lv_gpio_ctl6().modify(|_, w| w.g04_lv().bit(state));
    }
);
impl_lowvoltage_pin!(
    PC11,
    crate::pac::Gpio0::ptr(),
    5,
    |regs| {
        regs.devalt9().modify(|_, w| w.no_kso12_sl().set_bit());
    },
    |regs, state| {
        regs.lv_gpio_ctl6().modify(|_, w| w.g05_lv().bit(state));
    }
);
impl_lowvoltage_pin!(
    PB10,
    crate::pac::Gpio0::ptr(),
    6,
    |regs| {
        regs.devalt9().modify(|_, w| w.no_kso11_sl().set_bit());
    },
    |regs, state| {
        regs.lv_gpio_ctl6().modify(|_, w| w.g06_lv().bit(state));
    }
);
impl_lowvoltage_pin!(
    PB11,
    crate::pac::Gpio0::ptr(),
    7,
    |regs| {
        regs.devalt9().modify(|_, w| w.no_kso10_sl().set_bit());
    },
    |regs, state| {
        regs.lv_gpio_ctl6().modify(|_, w| w.g07_lv().bit(state));
    }
);

// GPIO1
impl_lowvoltage_pin!(
    PC10,
    crate::pac::Gpio1::ptr(),
    0,
    |regs| {
        regs.devalt9().modify(|_, w| w.no_kso09_sl().set_bit());
        regs.devaltj().modify(|_, w| w.cr_sin1_sl1().clear_bit());
    },
    |regs, state| {
        regs.lv_gpio_ctl6().modify(|_, w| w.g10_lv().bit(state));
    }
);
impl_lowvoltage_pin!(
    PC09,
    crate::pac::Gpio1::ptr(),
    1,
    |regs| {
        regs.devalt9().modify(|_, w| w.no_kso08_sl().set_bit());
        regs.devaltj().modify(|_, w| w.cr_sout1_sl1().clear_bit());
    },
    |regs, state| {
        regs.lv_gpio_ctl6().modify(|_, w| w.g11_lv().bit(state));
    }
);
impl_input_pin!(PB09, crate::pac::Gpio1::ptr(), 2, |regs| {
    regs.devalt8().modify(|_, w| w.no_kso07_sl().set_bit());
});
impl_lowvoltage_pin!(
    PC08,
    crate::pac::Gpio1::ptr(),
    3,
    |regs| {
        regs.devalt8().modify(|_, w| w.no_kso06_sl().set_bit());
    },
    |regs, state| {
        regs.lv_gpio_ctl6().modify(|_, w| w.g13_lv().bit(state));
    }
);
impl_lowvoltage_pin!(
    PC06,
    crate::pac::Gpio1::ptr(),
    4,
    |regs| {
        regs.devalt8().modify(|_, w| w.no_kso05_sl().set_bit());
    },
    |regs, state| {
        regs.lv_gpio_ctl7().modify(|_, w| w.g14_lv().bit(state));
    }
);
impl_lowvoltage_pin!(
    PC07,
    crate::pac::Gpio1::ptr(),
    5,
    |regs| {
        regs.devalt8().modify(|_, w| w.no_kso04_sl().set_bit());
        // No need to disconnect nTEST
    },
    |regs, state| {
        regs.lv_gpio_ctl7().modify(|_, w| w.g15_lv().bit(state));
    }
);
impl_lowvoltage_pin!(
    PB08,
    crate::pac::Gpio1::ptr(),
    6,
    |regs| {
        regs.devalt8().modify(|_, w| w.no_kso03_sl().set_bit());
        // Note: this messes with debug interfaces, maybe there is a better way?
        regs.jen_ctl1().modify(|_, w| w.jen_en().bits(6));
    },
    |regs, state| {
        regs.lv_gpio_ctl7().modify(|_, w| w.g16_lv().bit(state));
    }
);
impl_lowvoltage_pin!(
    PB07,
    crate::pac::Gpio1::ptr(),
    7,
    |regs| {
        regs.devalt8().modify(|_, w| w.no_kso02_sl().set_bit());
        // Note: this messes with debug interfaces, maybe there is a better way?
        regs.jen_ctl1().modify(|_, w| w.jen_en().bits(6));
    },
    |regs, state| {
        regs.lv_gpio_ctl7().modify(|_, w| w.g17_lv().bit(state));
    }
);

// GPIO2
impl_lowvoltage_pin!(
    PB06,
    crate::pac::Gpio2::ptr(),
    0,
    |regs| {
        regs.devalt8().modify(|_, w| w.no_kso01_sl().set_bit());
        // Note: this messes with debug interfaces, maybe there is a better way?
        regs.jen_ctl1().modify(|_, w| w.jen_en().bits(6));
    },
    |regs, state| {
        regs.lv_gpio_ctl7().modify(|_, w| w.g20_lv().bit(state));
    }
);
impl_lowvoltage_pin!(
    PB05,
    crate::pac::Gpio2::ptr(),
    1,
    |regs| {
        regs.devalt8().modify(|_, w| w.no_kso00_sl().set_bit());
        // Note: this messes with debug interfaces, maybe there is a better way?
        regs.jen_ctl1().modify(|_, w| w.jen_en().bits(6));
    },
    |regs, state| {
        regs.lv_gpio_ctl7().modify(|_, w| w.g21_lv().bit(state));
    }
);
impl_lowvoltage_pin!(
    PC05,
    crate::pac::Gpio2::ptr(),
    2,
    |regs| {
        regs.devalt7().modify(|_, w| w.no_ksi7_sl().set_bit());
        regs.devaltl().modify(|_, w| w.ad20_sl().clear_bit());
        // Note: this messes with debug interfaces, maybe there is a better way?
        unsafe { crate::pac::Dev::steal() }
            .dbgctrl2()
            .modify(|_, w| w.ccdev_sel().bits(0));
    },
    |regs, state| {
        regs.lv_gpio_ctl7().modify(|_, w| w.g22_lv().bit(state));
    }
);
impl_lowvoltage_pin!(
    PC04,
    crate::pac::Gpio2::ptr(),
    3,
    |regs| {
        regs.devalt7().modify(|_, w| w.no_ksi6_sl().set_bit());
        regs.devaltm().modify(|_, w| w.ad21_sl().clear_bit());
        // Note: this messes with debug interfaces, maybe there is a better way?
        unsafe { crate::pac::Dev::steal() }
            .dbgctrl2()
            .modify(|_, w| w.ccdev_sel().bits(0));
    },
    |regs, state| {
        regs.lv_gpio_ctl7().modify(|_, w| w.g23_lv().bit(state));
    }
);
impl_input_pin!(PC03, crate::pac::Gpio2::ptr(), 4, |regs| {
    regs.devalt7().modify(|_, w| w.no_ksi5_sl().set_bit());
    regs.devaltf().modify(|_, w| w.ad12_sl().clear_bit());
});
impl_input_pin!(PB04, crate::pac::Gpio2::ptr(), 5, |regs| {
    regs.devalt7().modify(|_, w| w.no_ksi4_sl().set_bit());
    regs.devaltm().modify(|_, w| w.ad24_sl().clear_bit());
    regs.devalt5().modify(|_, w| w.trace_en().clear_bit());
});
impl_input_pin!(PB03, crate::pac::Gpio2::ptr(), 6, |regs| {
    regs.devalt7().modify(|_, w| w.no_ksi3_sl().set_bit());
    regs.devaltl().modify(|_, w| w.ad13_sl().clear_bit());
    regs.devalt5().modify(|_, w| w.trace_en().clear_bit());
});
impl_input_pin!(PA04, crate::pac::Gpio2::ptr(), 7, |regs| {
    regs.devalt7().modify(|_, w| w.no_ksi2_sl().set_bit());
    regs.devalt5().modify(|_, w| w.trace_en().clear_bit());
    regs.devaltl().modify(|_, w| w.ad14_sl().clear_bit());
});

// GPIO3
impl_input_pin!(PA03, crate::pac::Gpio3::ptr(), 0, |regs| {
    regs.devalt7().modify(|_, w| w.no_ksi1_sl().set_bit());
    regs.devaltm().modify(|_, w| w.ad25_sl().clear_bit());
    regs.devalt5().modify(|_, w| w.trace_en().clear_bit());
});
impl_input_pin!(PA02, crate::pac::Gpio3::ptr(), 1, |regs| {
    regs.devalt7().modify(|_, w| w.no_ksi0_sl().set_bit());
    regs.devaltl().modify(|_, w| w.ad15_sl().clear_bit());
    regs.devalt5().modify(|_, w| w.trace_en().clear_bit());
});
impl_pin!(PE04, crate::pac::Gpio3::ptr(), 2, |_| {
    // No need to disconnect nTRIS strap
});
impl_lowvoltage_pin!(
    PD05,
    crate::pac::Gpio3::ptr(),
    3,
    |regs| {
        regs.devalt2().modify(|_, w| w.i2c5_0_sl().clear_bit());
        regs.devaltb().modify(|_, w| w.cts_sl().clear_bit());
    },
    |regs, state| {
        regs.lv_gpio_ctl0().modify(|_, w| w.g33_lv().bit(state));
    }
);
impl_lowvoltage_pin!(
    PB02,
    crate::pac::Gpio3::ptr(),
    4,
    |regs| {
        regs.devalt3().modify(|_, w| w.ps2_2_sl().clear_bit());
        regs.devaltf().modify(|_, w| w.ad6_sl().clear_bit());
    },
    |regs, state| {
        regs.lv_gpio_ctl2().modify(|_, w| w.g34_lv().bit(state));
    }
);
impl_pin!(PK02, crate::pac::Gpio3::ptr(), 5, |regs| {
    regs.devalte().modify(|_, w| w.cr_sout4_sl().clear_bit());
    // No need to disconnect nTEST strap
});
impl_lowvoltage_pin!(
    PD04,
    crate::pac::Gpio3::ptr(),
    6,
    |regs| {
        regs.devalt2().modify(|_, w| w.i2c5_0_sl().clear_bit());
        regs.devaltb().modify(|_, w| w.rts_sl().clear_bit());
    },
    |regs, state| {
        regs.lv_gpio_ctl1().modify(|_, w| w.g36_lv().bit(state));
    }
);
impl_lowvoltage_pin!(
    PC01,
    crate::pac::Gpio3::ptr(),
    7,
    |regs| {
        regs.devalt3().modify(|_, w| w.ps2_2_sl().clear_bit());
        regs.devaltf().modify(|_, w| w.ad5_sl().clear_bit());
    },
    |regs, state| {
        regs.lv_gpio_ctl3().modify(|_, w| w.g37_lv().bit(state));
    }
);

// GPIO4
impl_lowvoltage_pin!(
    PE05,
    crate::pac::Gpio4::ptr(),
    0,
    |regs| {
        regs.devalt3().modify(|_, w| w.ta1_sl1().clear_bit());
    },
    |regs, state| {
        regs.lv_gpio_ctl3().modify(|_, w| w.g40_lv().bit(state));
    }
);
impl_lowvoltage_pin!(
    PC02,
    crate::pac::Gpio4::ptr(),
    1,
    |regs| {
        regs.devalt6().modify(|_, w| w.ad4_sl().clear_bit());
    },
    |regs, state| {
        regs.lv_gpio_ctl8().modify(|_, w| w.g41_lv().bit(state));
    }
);
impl_lowvoltage_pin!(
    PD03,
    crate::pac::Gpio4::ptr(),
    2,
    |regs| {
        regs.devalt6().modify(|_, w| w.ad3_sl().clear_bit());
        regs.devaltb().modify(|_, w| w.ri_sl().clear_bit());
    },
    |regs, state| {
        regs.lv_gpio_ctl8().modify(|_, w| w.g42_lv().bit(state));
    }
);
impl_lowvoltage_pin!(
    PE02,
    crate::pac::Gpio4::ptr(),
    3,
    |regs| {
        regs.devalt6().modify(|_, w| w.ad2_sl().clear_bit());
    },
    |regs, state| {
        regs.lv_gpio_ctl8().modify(|_, w| w.g43_lv().bit(state));
    }
);
impl_lowvoltage_pin!(
    PE03,
    crate::pac::Gpio4::ptr(),
    4,
    |regs| {
        regs.devalt6().modify(|_, w| w.ad1_sl().clear_bit());
    },
    |regs, state| {
        regs.lv_gpio_ctl8().modify(|_, w| w.g44_lv().bit(state));
    }
);
impl_lowvoltage_pin!(
    PF02,
    crate::pac::Gpio4::ptr(),
    5,
    |regs| {
        regs.devalt6().modify(|_, w| w.ad0_sl().clear_bit());
    },
    |regs, state| {
        regs.lv_gpio_ctl8().modify(|_, w| w.g45_lv().bit(state));
    }
);
impl_input_pin!(PH01, crate::pac::Gpio4::ptr(), 6, |regs| {
    regs.devalt1().modify(|_, w| w.no_lpc_espi().set_bit());
    regs.devaltc().modify(|_, w| w.shi_sl().clear_bit());
});
impl_input_pin!(PJ01, crate::pac::Gpio4::ptr(), 7, |regs| {
    regs.devalt1().modify(|_, w| w.no_lpc_espi().set_bit());
    regs.devaltc().modify(|_, w| w.shi_sl().clear_bit());
});

// GPIO5
impl_lowvoltage_pin!(
    PG10,
    crate::pac::Gpio5::ptr(),
    0,
    |regs| {
        regs.devaltn().modify(|_, w| w.i3c2_sl().clear_bit());
    },
    |regs, state| {
        regs.lv_gpio_ctl5().modify(|_, w| w.g50_lv().bit(state));
    }
);
impl_input_pin!(PK01, crate::pac::Gpio5::ptr(), 1, |regs| {
    regs.devalt1().modify(|_, w| w.no_lpc_espi().set_bit());
    regs.devaltc().modify(|_, w| w.shi_sl().clear_bit());
});
impl_input_pin!(PL01, crate::pac::Gpio5::ptr(), 2, |regs| {
    regs.devalt1().modify(|_, w| w.no_lpc_espi().set_bit());
    regs.devaltc().modify(|_, w| w.shi_sl().clear_bit());
});
impl_input_pin!(PL02, crate::pac::Gpio5::ptr(), 3, |regs| {
    regs.devalt1().modify(|_, w| w.no_lpc_espi().set_bit());
    regs.devaltc().modify(|_, w| w.shi_sl().clear_bit());
});
impl_input_pin!(PK03, crate::pac::Gpio5::ptr(), 4, |regs| {
    regs.devalt1().modify(|_, w| w.no_lpc_espi().set_bit());
    regs.devaltc().modify(|_, w| w.shi_sl().clear_bit());
});
impl_input_pin!(PM01, crate::pac::Gpio5::ptr(), 5, |regs| {
    regs.devalt1().modify(|_, w| w.no_lpc_espi().set_bit());
    regs.devaltc().modify(|_, w| w.shi_sl().clear_bit());
});
impl_input_pin!(PM02, crate::pac::Gpio5::ptr(), 6, |regs| {
    regs.devalt1().modify(|_, w| w.clkrn_sl().clear_bit());
    regs.devaltn().modify(|_, w| w.i3c2_sl().clear_bit());
});
impl_input_pin!(PL03, crate::pac::Gpio5::ptr(), 7, |regs| {
    regs.devalt1().modify(|_, w| w.no_lpc_espi().clear_bit());
});

// GPIO6
impl_lowvoltage_pin!(
    PG06,
    crate::pac::Gpio6::ptr(),
    0,
    |regs| {
        regs.devalt4().modify(|_, w| w.pwm7_sl().clear_bit());
    },
    |regs, state| {
        regs.lv_gpio_ctlb().modify(|_, w| w.g60_lv().bit(state));
    }
);
impl_lowvoltage_pin!(
    PK04,
    crate::pac::Gpio6::ptr(),
    1,
    |regs| {
        regs.devalt1().modify(|_, w| w.pwroff_sl().clear_bit());
    },
    |regs, state| {
        regs.lv_gpio_ctl9().modify(|_, w| w.g61_lv().bit(state));
    }
);
impl_lowvoltage_pin!(
    PH02,
    crate::pac::Gpio6::ptr(),
    2,
    |regs| {
        regs.devalt3().modify(|_, w| w.ps2_1_sl().clear_bit());
        regs.devaltl().modify(|_, w| w.ad16_sl().clear_bit());
    },
    |regs, state| {
        regs.lv_gpio_ctl9().modify(|_, w| w.g62_lv().bit(state));
    }
);
impl_lowvoltage_pin!(
    PJ02,
    crate::pac::Gpio6::ptr(),
    3,
    |regs| {
        regs.devalt3().modify(|_, w| w.ps2_1_sl().clear_bit());
        regs.devaltl().modify(|_, w| w.ad17_sl().clear_bit());
        regs.devalt5().modify(|_, w| w.intrud2_sl().clear_bit());
    },
    |regs, state| {
        regs.lv_gpio_ctl9().modify(|_, w| w.g63_lv().bit(state));
    }
);
impl_lowvoltage_pin!(
    PG04,
    crate::pac::Gpio6::ptr(),
    4,
    |regs| {
        regs.devaltj().modify(|_, w| w.cr_sin1_sl2().clear_bit());
    },
    |regs, state| {
        regs.lv_gpio_ctl1().modify(|_, w| w.g64_lv().bit(state));
    }
);
impl_pin!(PH04, crate::pac::Gpio6::ptr(), 5, |regs| {
    regs.devaltj().modify(|_, w| w.cr_sout1_sl2().clear_bit());
    // No need to disable strap
});
impl_pin!(
    PG02,
    crate::pac::Gpio6::ptr(),
    6,
    |_| {
        // No need to disable strap
    },
    |regs, state| {
        // Despite the datasheet stating this is a output only port
        // it still documents a low-voltage input register for it. We
        // implement it here for safety
        regs.lv_gpio_ctl1().modify(|_, w| w.g66_lv().bit(state));
    }
);
impl_lowvoltage_pin!(
    PJ03,
    crate::pac::Gpio6::ptr(),
    7,
    |regs| {
        regs.devalt3().modify(|_, w| w.ps2_0_sl().clear_bit());
        regs.devaltl().modify(|_, w| w.ad18_sl().clear_bit());
    },
    |regs, state| {
        regs.lv_gpio_ctl9().modify(|_, w| w.g67_lv().bit(state));
    }
);

// GPIO7
impl_lowvoltage_pin!(
    PJ04,
    crate::pac::Gpio7::ptr(),
    0,
    |regs| {
        regs.devalt3().modify(|_, w| w.ps2_0_sl().clear_bit());
        regs.devaltl().modify(|_, w| w.ad19_sl().clear_bit());
    },
    |regs, state| {
        regs.lv_gpio_ctl9().modify(|_, w| w.g70_lv().bit(state));
    }
);
impl_lowvoltage_pin!(
    PM04,
    crate::pac::Gpio7::ptr(),
    2,
    |regs| {
        regs.devalt1().modify(|_, w| w.no_pwrgd().set_bit());
    },
    |regs, state| {
        regs.lv_gpio_ctl5().modify(|_, w| w.g72_lv().bit(state));
    }
);
impl_lowvoltage_pin!(
    PG05,
    crate::pac::Gpio7::ptr(),
    3,
    |regs| {
        regs.devalt3().modify(|_, w| w.ta2_sl1().clear_bit());
    },
    |regs, state| {
        regs.lv_gpio_ctl2().modify(|_, w| w.g73_lv().bit(state));
    }
);
impl_lowvoltage_pin!(
    PH05,
    crate::pac::Gpio7::ptr(),
    4,
    |regs| {
        regs.devaltm().modify(|_, w| w.ad23_sl().clear_bit());
    },
    |regs, state| {
        regs.lv_gpio_ctl2().modify(|_, w| w.g74_lv().bit(state));
    }
);
impl_lowvoltage_pin!(
    PJ06,
    crate::pac::Gpio7::ptr(),
    5,
    |regs| {
        regs.devalta().modify(|_, w| w._32k_out_sl().clear_bit());
        regs.devalt2().modify(|_, w| w.i2c4_0_sl().clear_bit());
        regs.devaltb().modify(|_, w| w.rxd_sl().clear_bit());
        regs.devaltj().modify(|_, w| w.cr_sin2_sl().clear_bit());
    },
    |regs, state| {
        regs.lv_gpio_ctl3().modify(|_, w| w.g75_lv().bit(state));
    }
);
impl_lowvoltage_pin!(
    PJ05,
    crate::pac::Gpio7::ptr(),
    6,
    |regs| {
        regs.devalt1().modify(|_, w| w.ec_sci_sl().clear_bit());
    },
    |regs, state| {
        regs.lv_gpio_ctl9().modify(|_, w| w.g76_lv().bit(state));
    }
);
impl_pin!(PK06, crate::pac::Gpio7::ptr(), 7, |regs| {
    regs.devalta().modify(|_, w| w.no_vcc1_rst().set_bit());
});

// GPIO8
impl_lowvoltage_pin!(
    PK05,
    crate::pac::Gpio8::ptr(),
    0,
    |regs| {
        regs.devalt4().modify(|_, w| w.pwm3_sl().clear_bit());
    },
    |regs, state| {
        regs.lv_gpio_ctl3().modify(|_, w| w.g80_lv().bit(state));
    }
);
impl_input_pin!(PM07, crate::pac::Gpio8::ptr(), 1, |regs| {
    regs.devalta().modify(|_, w| w.no_peci_en().set_bit());
});
impl_lowvoltage_pin!(
    PD06,
    crate::pac::Gpio8::ptr(),
    2,
    |regs| {
        regs.devalt9().modify(|_, w| w.no_kso14_sl().set_bit());
    },
    |regs, state| {
        regs.lv_gpio_ctl3().modify(|_, w| w.g82_lv().bit(state));
    }
);
impl_lowvoltage_pin!(
    PD07,
    crate::pac::Gpio8::ptr(),
    3,
    |regs| {
        regs.devalt9().modify(|_, w| w.no_kso15_sl().set_bit());
    },
    |regs, state| {
        regs.lv_gpio_ctl9().modify(|_, w| w.g83_lv().bit(state));
    }
);
impl_pin!(PJ08, crate::pac::Gpio8::ptr(), 5, |regs| {
    regs.devaltg().modify(|_, w| w.psl_out_sl().clear_bit());
});
impl_pin!(PJ09, crate::pac::Gpio8::ptr(), 6, |regs| {
    regs.devalt2().modify(|_, w| w.i2c4_0_sl().clear_bit());
    regs.devaltb().modify(|_, w| w.txd_sl().clear_bit());
    regs.devaltj().modify(|_, w| w.cr_sout2_sl().clear_bit());
    // No need to disconnect FLPRG2 strap
});
impl_lowvoltage_pin!(
    PK07,
    crate::pac::Gpio8::ptr(),
    7,
    |regs| {
        regs.devalt2().modify(|_, w| w.i2c1_0_sl().clear_bit());
    },
    |regs, state| {
        regs.lv_gpio_ctl0().modify(|_, w| w.g87_lv().bit(state));
    }
);

// GPIO9
impl_lowvoltage_pin!(
    PK08,
    crate::pac::Gpio9::ptr(),
    0,
    |regs| {
        regs.devalt2().modify(|_, w| w.i2c1_0_sl().clear_bit());
    },
    |regs, state| {
        regs.lv_gpio_ctl0().modify(|_, w| w.g90_lv().bit(state));
    }
);
impl_lowvoltage_pin!(
    PK09,
    crate::pac::Gpio9::ptr(),
    1,
    |regs| {
        regs.devalt2().modify(|_, w| w.i2c2_0_sl().clear_bit());
    },
    |regs, state| {
        regs.lv_gpio_ctl1().modify(|_, w| w.g91_lv().bit(state));
    }
);
impl_lowvoltage_pin!(
    PL08,
    crate::pac::Gpio9::ptr(),
    2,
    |regs| {
        regs.devalt2().modify(|_, w| w.i2c2_0_sl().clear_bit());
    },
    |regs, state| {
        regs.lv_gpio_ctl1().modify(|_, w| w.g92_lv().bit(state));
    }
);
impl_input_pin!(PE11, crate::pac::Gpio9::ptr(), 3, |regs| {
    regs.devaltc().modify(|_, w| w.ta1_sl2().clear_bit());
    regs.devalt0().modify(|_, w| w.f_spi_quad().clear_bit());
    regs.devalth().modify(|_, w| w.flm_sl().clear_bit());
});
impl_input_pin!(PM11, crate::pac::Gpio9::ptr(), 4, |_| {});
impl_input_pin!(PM12, crate::pac::Gpio9::ptr(), 5, |regs| {
    regs.devalt0().modify(|_, w| w.spip_sl().clear_bit());
    regs.devalt0().modify(|_, w| w.gpio_no_spip().set_bit());
});
impl_input_pin!(PG12, crate::pac::Gpio9::ptr(), 6, |regs| {
    regs.devalth().modify(|_, w| w.flm_sl().clear_bit());
    regs.devalt0().modify(|_, w| w.no_f_spi().set_bit());
});
impl_input_pin!(PL10, crate::pac::Gpio9::ptr(), 7, |regs| {
    regs.devalt0().modify(|_, w| w.gpio_no_spip().set_bit());
});

// GPIOA
impl_input_pin!(PG11, crate::pac::Gpioa::ptr(), 0, |regs| {
    regs.devalth().modify(|_, w| w.flm_sl().clear_bit());
    regs.devalt0().modify(|_, w| w.no_f_spi().set_bit());
});
impl_input_pin!(PL12, crate::pac::Gpioa::ptr(), 1, |regs| {
    regs.devalt0().modify(|_, w| w.spip_sl().clear_bit());
    regs.devalt0().modify(|_, w| w.gpio_no_spip().set_bit());
});
impl_input_pin!(PF12, crate::pac::Gpioa::ptr(), 2, |regs| {
    regs.devalth().modify(|_, w| w.flm_sl().clear_bit());
    regs.devalt0().modify(|_, w| w.no_f_spi().set_bit());
});
impl_input_pin!(PK12, crate::pac::Gpioa::ptr(), 3, |regs| {
    regs.devalt0().modify(|_, w| w.spip_sl().clear_bit());
    regs.devalt0().modify(|_, w| w.gpio_no_spip().set_bit());
});
impl_input_pin!(PH11, crate::pac::Gpioa::ptr(), 4, |regs| {
    regs.devalth().modify(|_, w| w.flm_sl().clear_bit());
    regs.devalt0().modify(|_, w| w.no_f_spi().set_bit());
    regs.devalt3().modify(|_, w| w.tb1_sl1().clear_bit());
});
impl_input_pin!(PK11, crate::pac::Gpioa::ptr(), 5, |_| {});
impl_input_pin!(PF11, crate::pac::Gpioa::ptr(), 6, |regs| {
    regs.devaltc().modify(|_, w| w.ps2_3_sl2().clear_bit());
    regs.devaltc().modify(|_, w| w.ta2_sl2().clear_bit());
    regs.devalt0().modify(|_, w| w.f_spi_cs1().clear_bit());
});
impl_input_pin!(PJ11, crate::pac::Gpioa::ptr(), 7, |regs| {
    regs.devaltc().modify(|_, w| w.ps2_3_sl2().clear_bit());
    regs.devaltc().modify(|_, w| w.tb2_sl2().clear_bit());
    regs.devalt0().modify(|_, w| w.f_spi_quad().clear_bit());
    regs.devalth().modify(|_, w| w.flm_sl().clear_bit());
});

// GPIOB
impl_input_pin!(PL11, crate::pac::Gpiob::ptr(), 0, |_| {});
impl_lowvoltage_pin!(
    PD08,
    crate::pac::Gpiob::ptr(),
    1,
    |regs| {
        regs.devalta().modify(|_, w| w.no_kso17_sl().set_bit());
        regs.devalte().modify(|_, w| w.cr_sin4_sl().clear_bit());
    },
    |regs, state| {
        regs.lv_gpio_ctla().modify(|_, w| w.gb1_lv().bit(state));
    }
);
impl_lowvoltage_pin!(
    PK10,
    crate::pac::Gpiob::ptr(),
    2,
    |regs| {
        regs.devalt2().modify(|_, w| w.i2c7_0_sl().clear_bit());
        regs.devaltb().modify(|_, w| w.dsr_sl().clear_bit());
    },
    |regs, state| {
        regs.lv_gpio_ctl0().modify(|_, w| w.gb2_lv().bit(state));
    }
);
impl_lowvoltage_pin!(
    PJ10,
    crate::pac::Gpiob::ptr(),
    3,
    |regs| {
        regs.devalt2().modify(|_, w| w.i2c7_0_sl().clear_bit());
        regs.devaltb().modify(|_, w| w.dcd_sl().clear_bit());
    },
    |regs, state| {
        regs.lv_gpio_ctl0().modify(|_, w| w.gb3_lv().bit(state));
    }
);
impl_lowvoltage_pin!(
    PB12,
    crate::pac::Gpiob::ptr(),
    4,
    |regs| {
        regs.devalt2().modify(|_, w| w.i2c0_0_sl().clear_bit());
    },
    |regs, state| {
        regs.lv_gpio_ctl0().modify(|_, w| w.gb4_lv().bit(state));
    }
);
impl_lowvoltage_pin!(
    PC12,
    crate::pac::Gpiob::ptr(),
    5,
    |regs| {
        regs.devalt2().modify(|_, w| w.i2c0_0_sl().clear_bit());
    },
    |regs, state| {
        regs.lv_gpio_ctl0().modify(|_, w| w.gb5_lv().bit(state));
    }
);
impl_pin!(
    PL09,
    crate::pac::Gpiob::ptr(),
    6,
    |regs| {
        regs.devalt4().modify(|_, w| w.pwm4_sl().clear_bit());
        // no need to disconnect strap
    },
    |regs, state| {
        // Despite the datasheet stating this is a output only port
        // it still documents a low-voltage input register for it. We
        // implement it here for safety
        regs.lv_gpio_ctla().modify(|_, w| w.gb6_lv().bit(state));
    }
);
impl_lowvoltage_pin!(
    PJ07,
    crate::pac::Gpiob::ptr(),
    7,
    |regs| {
        regs.devalt4().modify(|_, w| w.pwm5_sl().clear_bit());
        regs.devaltk().modify(|_, w| w.i2c7_1_sl().clear_bit());
    },
    |regs, state| {
        regs.lv_gpio_ctla().modify(|_, w| w.gb7_lv().bit(state));
    }
);

// GPIOC
impl_lowvoltage_pin!(
    PH08,
    crate::pac::Gpioc::ptr(),
    0,
    |regs| {
        regs.devalt4().modify(|_, w| w.pwm6_sl().clear_bit());
        regs.devaltk().modify(|_, w| w.i2c7_1_sl().clear_bit());
        // Note: this messes with debug interfaces, maybe there is a better way?
        unsafe { crate::pac::Dev::steal() }
            .dbgctrl2()
            .modify(|_, w| w.ccdev_sel().bits(6));
    },
    |regs, state| {
        regs.lv_gpio_ctla().modify(|_, w| w.gc0_lv().bit(state));
    }
);
impl_lowvoltage_pin!(
    PH09,
    crate::pac::Gpioc::ptr(),
    1,
    |regs| {
        regs.devalt2().modify(|_, w| w.i2c6_0_sl().clear_bit());
        regs.devaltm().modify(|_, w| w.ad22_sl().clear_bit());
    },
    |regs, state| {
        regs.lv_gpio_ctl2().modify(|_, w| w.gc1_lv().bit(state));
    }
);
impl_lowvoltage_pin!(
    PH10,
    crate::pac::Gpioc::ptr(),
    2,
    |regs| {
        regs.devalt4().modify(|_, w| w.pwm1_sl().clear_bit());
        regs.devalt2().modify(|_, w| w.i2c6_0_sl().clear_bit());
    },
    |regs, state| {
        regs.lv_gpio_ctl4().modify(|_, w| w.gc2_lv().bit(state));
    }
);
impl_lowvoltage_pin!(
    PG09,
    crate::pac::Gpioc::ptr(),
    3,
    |regs| {
        regs.devalt4().modify(|_, w| w.pwm0_sl().clear_bit());
    },
    |regs, state| {
        regs.lv_gpio_ctla().modify(|_, w| w.gc3_lv().bit(state));
    }
);
impl_lowvoltage_pin!(
    PG08,
    crate::pac::Gpioc::ptr(),
    4,
    |regs| {
        regs.devalt4().modify(|_, w| w.pwm2_sl().clear_bit());
    },
    |regs, state| {
        regs.lv_gpio_ctla().modify(|_, w| w.gc4_lv().bit(state));
    }
);
impl_lowvoltage_pin!(
    PH07,
    crate::pac::Gpioc::ptr(),
    5,
    |regs| {
        regs.devalt1().modify(|_, w| w.kbrst_sl().clear_bit());
    },
    |regs, state| {
        regs.lv_gpio_ctl3().modify(|_, w| w.gc5_lv().bit(state));
    }
);
impl_lowvoltage_pin!(
    PD10,
    crate::pac::Gpioc::ptr(),
    6,
    |regs| {
        regs.devalt1().modify(|_, w| w.smi_sl().clear_bit());
    },
    |regs, state| {
        regs.lv_gpio_ctl3().modify(|_, w| w.gc6_lv().bit(state));
    }
);
impl_lowvoltage_pin!(
    PF10,
    crate::pac::Gpioc::ptr(),
    7,
    |regs| {
        regs.devaltb().modify(|_, w| w.dtr_bout_sl().clear_bit());
        regs.devaltf().modify(|_, w| w.ad11_sl().clear_bit());
    },
    |regs, state| {
        regs.lv_gpio_ctl2().modify(|_, w| w.gc7_lv().bit(state));
    }
);

// GPIOD
impl_lowvoltage_pin!(
    PF09,
    crate::pac::Gpiod::ptr(),
    0,
    |regs| {
        regs.devalt2().modify(|_, w| w.i2c3_0_sl().clear_bit());
        // Note, manual says something about devaltm.ad28_sl here, which doesnt exist.
        // ignoring for now
    },
    |regs, state| {
        regs.lv_gpio_ctl1().modify(|_, w| w.gd0_lv().bit(state));
    }
);
impl_lowvoltage_pin!(
    PF08,
    crate::pac::Gpiod::ptr(),
    1,
    |regs| {
        regs.devalt2().modify(|_, w| w.i2c3_0_sl().clear_bit());
    },
    |regs, state| {
        regs.lv_gpio_ctl1().modify(|_, w| w.gd1_lv().bit(state));
    }
);
impl_lowvoltage_pin!(
    PG07,
    crate::pac::Gpiod::ptr(),
    2,
    |regs| {
        regs.devaltd().modify(|_, w| w.n_psl_in1_sl().set_bit());
    },
    |regs, state| {
        regs.lv_gpio_ctla().modify(|_, w| w.gd2_lv().bit(state));
    }
);
impl_lowvoltage_pin!(
    PE10,
    crate::pac::Gpiod::ptr(),
    3,
    |regs| {
        regs.devaltc().modify(|_, w| w.tb1_sl2().clear_bit());
    },
    |regs, state| {
        regs.lv_gpio_ctla().modify(|_, w| w.gd3_lv().bit(state));
    }
);
impl_lowvoltage_pin!(
    PA09,
    crate::pac::Gpiod::ptr(),
    4,
    |regs| {
        regs.devaltj().modify(|_, w| w.cr_sin3_sl().clear_bit());
        // Note: This messes with debug interfaces, maybe there is a better way?
        regs.jen_ctl1().modify(|_, w| w.jen_en().bits(6));
    },
    |regs, state| {
        regs.lv_gpio_ctlb().modify(|_, w| w.gd4_lv().bit(state));
    }
);
impl_lowvoltage_pin!(
    PA10,
    crate::pac::Gpiod::ptr(),
    5,
    |regs| {
        regs.devalt5().modify(|_, w| w.intrud1_sl().clear_bit());
        // Note: This messes with debug interfaces, maybe there is a better way?
        regs.jen_ctl1().modify(|_, w| w.jen_en().bits(6));
    },
    |regs, state| {
        regs.lv_gpio_ctlb().modify(|_, w| w.gd5_lv().bit(state));
    }
);
impl_lowvoltage_pin!(
    PH03,
    crate::pac::Gpiod::ptr(),
    6,
    |regs| {
        regs.devaltj().modify(|_, w| w.cr_sout3_sl().clear_bit());
        regs.devalth().modify(|_, w| w.flm_sl().clear_bit());
    },
    |regs, state| {
        regs.lv_gpio_ctl5().modify(|_, w| w.gd6_lv().bit(state));
    }
);
impl_pin!(PH06, crate::pac::Gpiod::ptr(), 7, |regs| {
    regs.devaltg().modify(|_, w| w.psl_gpo_sl().clear_bit());
});

// GPIOE
impl_lowvoltage_pin!(
    PF04,
    crate::pac::Gpioe::ptr(),
    0,
    |regs| {
        regs.devaltf().modify(|_, w| w.ad10_sl().clear_bit());
    },
    |regs, state| {
        regs.lv_gpio_ctl8().modify(|_, w| w.ge0_lv().bit(state));
    }
);
impl_lowvoltage_pin!(
    PF03,
    crate::pac::Gpioe::ptr(),
    1,
    |regs| {
        regs.devaltf().modify(|_, w| w.ad7_sl().clear_bit());
    },
    |regs, state| {
        regs.lv_gpio_ctl8().modify(|_, w| w.ge1_lv().bit(state));
    }
);
impl_lowvoltage_pin!(
    PA11,
    crate::pac::Gpioe::ptr(),
    2,
    |regs| {
        // Note: This messes with debug interfaces, maybe there is a better way?
        regs.jen_ctl1().modify(|_, w| w.jen_en().bits(6));
    },
    |regs, state| {
        regs.lv_gpio_ctl5().modify(|_, w| w.ge2_lv().bit(state));
    }
);
impl_lowvoltage_pin!(
    PL07,
    crate::pac::Gpioe::ptr(),
    3,
    |regs| {
        regs.devalt6().modify(|_, w| w.i2c6_1_sl().clear_bit());
        regs.devaltn().modify(|_, w| w.i3c1_sl().clear_bit());
    },
    |regs, state| {
        regs.lv_gpio_ctl4().modify(|_, w| w.ge3_lv().bit(state));
    }
);
impl_lowvoltage_pin!(
    PL06,
    crate::pac::Gpioe::ptr(),
    4,
    |regs| {
        regs.devalt6().modify(|_, w| w.i2c6_1_sl().clear_bit());
        regs.devaltn().modify(|_, w| w.i3c1_sl().clear_bit());
    },
    |regs, state| {
        regs.lv_gpio_ctl4().modify(|_, w| w.ge4_lv().bit(state));
    }
);
impl_lowvoltage_pin!(
    PA12,
    crate::pac::Gpioe::ptr(),
    5,
    |regs| {
        // Note: This messes with debug interfaces, maybe there is a better way?
        regs.jen_ctl1().modify(|_, w| w.jen_en().bits(6));
    },
    |regs, state| {
        regs.lv_gpio_ctlb().modify(|_, w| w.ge5_lv().bit(state));
    }
);
impl_input_pin!(
    PL05,
    crate::pac::Gpioe::ptr(),
    7,
    |regs| {
        regs.devalta().modify(|_, w| w._32kclkin_sl().clear_bit());
        unsafe { crate::pac::Lfcg::steal() }
            .lfcgctl2()
            .modify(|_, w| w.xt_osc().clear_bit());
    },
    |regs, state| {
        // Manual doesn't state this pin is low voltage capable, but does
        // expose a low voltage control register bit for it. Provide set
        // functionality so that we can disable it for safety.
        regs.lv_gpio_ctl2().modify(|_, w| w.ge7_lv().bit(state));
    }
);

// GPIOF
impl_lowvoltage_pin!(
    PD02,
    crate::pac::Gpiof::ptr(),
    0,
    |regs| {
        regs.devaltf().modify(|_, w| w.ad9_sl().clear_bit());
    },
    |regs, state| {
        regs.lv_gpio_ctl8().modify(|_, w| w.gf0_lv().bit(state));
    }
);
impl_lowvoltage_pin!(
    PG03,
    crate::pac::Gpiof::ptr(),
    1,
    |regs| {
        regs.devaltf().modify(|_, w| w.ad8_sl().clear_bit());
    },
    |regs, state| {
        regs.lv_gpio_ctl9().modify(|_, w| w.gf1_lv().bit(state));
    }
);
impl_lowvoltage_pin!(
    PF06,
    crate::pac::Gpiof::ptr(),
    2,
    |regs| {
        regs.devalt6().modify(|_, w| w.i2c4_1_sl().clear_bit());
    },
    |regs, state| {
        regs.lv_gpio_ctl4().modify(|_, w| w.gf2_lv().bit(state));
    }
);
impl_lowvoltage_pin!(
    PF05,
    crate::pac::Gpiof::ptr(),
    3,
    |regs| {
        regs.devalt6().modify(|_, w| w.i2c4_1_sl().clear_bit());
    },
    |regs, state| {
        regs.lv_gpio_ctl4().modify(|_, w| w.gf3_lv().bit(state));
    }
);
impl_lowvoltage_pin!(
    PE09,
    crate::pac::Gpiof::ptr(),
    4,
    |regs| {
        regs.devalt6().modify(|_, w| w.i2c5_1_sl().clear_bit());
        regs.devaltn().modify(|_, w| w.i3c3_sl().clear_bit());
    },
    |regs, state| {
        regs.lv_gpio_ctl4().modify(|_, w| w.gf4_lv().bit(state));
    }
);
impl_lowvoltage_pin!(
    PE08,
    crate::pac::Gpiof::ptr(),
    5,
    |regs| {
        regs.devalt6().modify(|_, w| w.i2c5_1_sl().clear_bit());
        regs.devaltn().modify(|_, w| w.i3c3_sl().clear_bit());
    },
    |regs, state| {
        regs.lv_gpio_ctl4().modify(|_, w| w.gf5_lv().bit(state));
    }
);
