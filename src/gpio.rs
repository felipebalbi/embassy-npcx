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
        fn pin(&self) -> u8;

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
    fn pin(&self) -> u8 {
        self.pin
    }

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
        critical_section::with(|_| {
            let regs = self.pin.port();

            regs.px_din().read().pin(self.pin.pin()).is_low()
        })
    }

    #[must_use]
    pub fn is_high(&self) -> bool {
        critical_section::with(|_| {
            let regs = self.pin.port();

            regs.px_din().read().pin(self.pin.pin()).is_high()
        })
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
        critical_section::with(|_| {
            let regs = self.pin.port();

            regs.px_dout().read().pin(self.pin.pin()).is_low()
        })
    }

    #[must_use]
    pub fn is_set_high(&mut self) -> bool {
        critical_section::with(|_| {
            let regs = self.pin.port();

            regs.px_dout().read().pin(self.pin.pin()).is_high()
        })
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
        critical_section::with(|_| {
            let regs = self.pin.port();

            regs.px_din().read().pin(self.pin.pin()).is_low()
        })
    }

    #[must_use]
    pub fn is_high(&self) -> bool {
        critical_section::with(|_| {
            let regs = self.pin.port();

            regs.px_din().read().pin(self.pin.pin()).is_high()
        })
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
        critical_section::with(|_| {
            let regs = self.pin.port();

            regs.px_dout().read().pin(self.pin.pin()).is_low()
        })
    }

    #[must_use]
    pub fn is_set_high(&mut self) -> bool {
        critical_section::with(|_| {
            let regs = self.pin.port();

            regs.px_dout().read().pin(self.pin.pin()).is_high()
        })
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
    PJ09,
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
        regs.devalt4().modify(|_, w| w.pwm4_led0_sl().clear_bit());
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
        regs.devalt4().modify(|_, w| w.pwm4_led0_sl().clear_bit());
        regs.devaltk().modify(|_, w| w.i2c7_1_sl().clear_bit());
    },
    |regs, state| {
        regs.lv_gpio_ctla().modify(|_, w| w.gb7_lv().bit(state));
    }
);
