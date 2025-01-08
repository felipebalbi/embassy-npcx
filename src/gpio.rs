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

fn gpio_block(port: u8) -> &'static crate::pac::gpio0::RegisterBlock {
    let ptr = match port {
        0 => crate::pac::Gpio0::ptr(),
        1 => crate::pac::Gpio1::ptr(),
        2 => crate::pac::Gpio2::ptr(),
        3 => crate::pac::Gpio3::ptr(),
        4 => crate::pac::Gpio4::ptr(),
        5 => crate::pac::Gpio5::ptr(),
        6 => crate::pac::Gpio6::ptr(),
        7 => crate::pac::Gpio7::ptr(),
        8 => crate::pac::Gpio8::ptr(),
        9 => crate::pac::Gpio9::ptr(),
        10 => crate::pac::Gpioa::ptr(),
        11 => crate::pac::Gpiob::ptr(),
        12 => crate::pac::Gpioc::ptr(),
        13 => crate::pac::Gpiod::ptr(),
        14 => crate::pac::Gpioe::ptr(),
        15 => crate::pac::Gpiof::ptr(),
        _ => unreachable!(),
    };

    // Safety:
    // the pac ptr functions return pointers to memory that is used for registers for the 'static lifetime
    // and the created reference is shared.
    unsafe { &*ptr }
}

mod sealed {
    pub trait SealedPin: Sized {
        fn pin_port(&self) -> u8;

        fn port(&self) -> u8 {
            self.pin_port() / 8
        }

        fn pin(&self) -> u8 {
            self.pin_port() % 8
        }

        // Not ideal to mark this unsafe, but PeripheralRef is missing DerefMut...
        /// Safety assumptions:
        /// caller must ensure it has a mutable reference to the pin or
        /// has otherwise elimnated contention.
        unsafe fn set_pin_function(&self, cs: critical_section::CriticalSection);
    }

    pub trait SealedInputPin {}

    pub trait SealedLowVoltagePin {}
}

struct AnyPin {
    pin_port: u8,
}

impl AnyPin {
    fn port(&self) -> u8 {
        self.pin_port / 8
    }

    fn pin(&self) -> u8 {
        self.pin_port % 8
    }
}

impl<T: Pin> From<T> for AnyPin {
    fn from(pin: T) -> Self {
        AnyPin {
            pin_port: pin.pin_port(),
        }
    }
}

// Allow use of PeripheralRef to do lifetime management
impl Peripheral for AnyPin {
    type P = AnyPin;

    unsafe fn clone_unchecked(&self) -> Self::P {
        AnyPin {
            pin_port: self.pin_port,
        }
    }
}

pub struct OutputOnly {}

pub struct InputCapable {}

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
            unsafe { pin.set_pin_function(cs) };

            let regs = gpio_block(pin.port());

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
            unsafe { pin.set_pin_function(cs) };

            let regs = gpio_block(pin.port());

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
            let regs = gpio_block(self.pin.port());

            // Set data
            regs.px_dout().modify(|_, w| w.pin(self.pin.pin()).clear_bit());
        });
    }

    pub fn set_high(&mut self) {
        critical_section::with(|_| {
            let regs = gpio_block(self.pin.port());

            // Set data
            regs.px_dout().modify(|_, w| w.pin(self.pin.pin()).set_bit());
        });
    }

    pub fn set_value(&mut self, value: Level) {
        critical_section::with(|_| {
            let regs = gpio_block(self.pin.port());

            // Set data
            regs.px_dout().modify(|_, w| w.pin(self.pin.pin()).bit(value.into()));
        });
    }

    #[must_use]
    pub fn is_set_low(&mut self) -> bool {
        critical_section::with(|_| {
            let regs = gpio_block(self.pin.port());

            regs.px_dout().read().pin(self.pin.pin()).is_low()
        })
    }

    #[must_use]
    pub fn is_set_high(&mut self) -> bool {
        critical_section::with(|_| {
            let regs = gpio_block(self.pin.port());

            regs.px_dout().read().pin(self.pin.pin()).is_high()
        })
    }

    pub fn toggle(&mut self) {
        critical_section::with(|_| {
            let regs = gpio_block(self.pin.port());

            regs.px_dout()
                .modify(|r, w| w.pin(self.pin.pin()).bit(r.pin(self.pin.pin()).is_low()));
        });
    }
}

impl OutputOpenDrain<'_, InputCapable> {
    pub fn disable_pull(&mut self) {
        critical_section::with(|_| {
            let regs = gpio_block(self.pin.port());

            regs.px_pull().modify(|_, w| w.pin(self.pin.pin()).disabled());
        });
    }

    pub fn enable_pullup(&mut self) {
        critical_section::with(|_| {
            let regs = gpio_block(self.pin.port());

            regs.px_pud().modify(|_, w| w.pin(self.pin.pin()).pull_up());
            regs.px_pull().modify(|_, w| w.pin(self.pin.pin()).enabled());
        });
    }

    pub fn enable_pulldown(&mut self) {
        critical_section::with(|_| {
            let regs = gpio_block(self.pin.port());

            regs.px_pud().modify(|_, w| w.pin(self.pin.pin()).pull_down());
            regs.px_pull().modify(|_, w| w.pin(self.pin.pin()).enabled());
        });
    }

    pub fn is_low(&self) -> bool {
        critical_section::with(|_| {
            let regs = gpio_block(self.pin.port());

            regs.px_din().read().pin(self.pin.pin()).is_low()
        })
    }

    pub fn is_high(&self) -> bool {
        critical_section::with(|_| {
            let regs = gpio_block(self.pin.port());

            regs.px_din().read().pin(self.pin.pin()).is_high()
        })
    }
}

pub struct Output<'d> {
    pin: PeripheralRef<'d, AnyPin>,
}

impl<'d> Output<'d> {
    pub fn new(pin: impl Peripheral<P = impl Pin + 'd> + 'd, level: Level) -> Self {
        into_ref!(pin);

        critical_section::with(|cs| {
            // Safety:
            // We have a mutable reference to the pin through PeripheralRef
            unsafe { pin.set_pin_function(cs) };

            let regs = gpio_block(pin.port());

            // Set data and output direction
            regs.px_dout().modify(|_, w| w.pin(pin.pin()).bit(level.into()));
            regs.px_otype().modify(|_, w| w.pin(pin.pin()).pushpull());
            regs.px_dir().modify(|_, w| w.pin(pin.pin()).output());
        });

        Output { pin: pin.map_into() }
    }

    pub fn set_low(&mut self) {
        critical_section::with(|_| {
            let regs = gpio_block(self.pin.port());

            // Set data
            regs.px_dout().modify(|_, w| w.pin(self.pin.pin()).clear_bit());
        });
    }

    pub fn set_high(&mut self) {
        critical_section::with(|_| {
            let regs = gpio_block(self.pin.port());

            // Set data
            regs.px_dout().modify(|_, w| w.pin(self.pin.pin()).set_bit());
        });
    }

    pub fn set_value(&mut self, value: Level) {
        critical_section::with(|_| {
            let regs = gpio_block(self.pin.port());

            // Set data
            regs.px_dout().modify(|_, w| w.pin(self.pin.pin()).bit(value.into()));
        });
    }

    #[must_use]
    pub fn is_set_low(&mut self) -> bool {
        critical_section::with(|_| {
            let regs = gpio_block(self.pin.port());

            regs.px_dout().read().pin(self.pin.pin()).is_low()
        })
    }

    #[must_use]
    pub fn is_set_high(&mut self) -> bool {
        critical_section::with(|_| {
            let regs = gpio_block(self.pin.port());

            regs.px_dout().read().pin(self.pin.pin()).is_high()
        })
    }

    pub fn toggle(&mut self) {
        critical_section::with(|_| {
            let regs = gpio_block(self.pin.port());

            regs.px_dout()
                .modify(|r, w| w.pin(self.pin.pin()).bit(r.pin(self.pin.pin()).is_low()));
        });
    }
}

pub trait LowVoltagePin: InputPin + sealed::SealedLowVoltagePin {}

pub trait InputPin: Pin + sealed::SealedInputPin {}

pub trait Pin: sealed::SealedPin {}

macro_rules! impl_pin {
    ($peripheral:ident, $port:expr, $pin:expr, $pin_function:expr) => {
        impl sealed::SealedPin for crate::peripherals::$peripheral {
            fn pin_port(&self) -> u8 {
                $port * 8 + $pin
            }

            unsafe fn set_pin_function(&self, _cs: critical_section::CriticalSection) {
                // Safety:
                // We have the unique reference to the pin and are within a critical section, so no contention
                // Note: allow unused as not every pin is dual function
                fn internal_set(f: impl FnOnce(crate::pac::Sysconfig)) {
                    f(unsafe { crate::pac::Sysconfig::steal() });
                }
                internal_set($pin_function);
            }
        }
        impl Pin for crate::peripherals::$peripheral {}
    };
}

macro_rules! impl_input_pin {
    ($peripheral:ident, $port:expr, $pin:expr, $pin_function:expr) => {
        impl_pin!($peripheral, $port, $pin, $pin_function);
        impl sealed::SealedInputPin for crate::peripherals::$peripheral {}
        impl InputPin for crate::peripherals::$peripheral {}
    };
}

macro_rules! impl_lowvoltage_pin {
    ($peripheral:ident, $port:expr, $pin:expr, $pin_function:expr) => {
        impl_input_pin!($peripheral, $port, $pin, $pin_function);
        impl sealed::SealedLowVoltagePin for crate::peripherals::$peripheral {}
        impl LowVoltagePin for crate::peripherals::$peripheral {}
    };
}

// GPIOB
impl_input_pin!(PL11, 11, 0, |_| {});
impl_lowvoltage_pin!(PD08, 11, 1, |regs| {
    regs.devalta().modify(|_, w| w.no_kso17_sl().set_bit());
    regs.devalte().modify(|_, w| w.cr_sin4_sl().clear_bit());
});
impl_lowvoltage_pin!(PK10, 11, 2, |regs| {
    regs.devalt2().modify(|_, w| w.i2c7_0_sl().clear_bit());
    regs.devaltb().modify(|_, w| w.dsr_sl().clear_bit());
});
impl_lowvoltage_pin!(PJ09, 11, 3, |regs| {
    regs.devalt2().modify(|_, w| w.i2c7_0_sl().clear_bit());
    regs.devaltb().modify(|_, w| w.dcd_sl().clear_bit());
});
impl_lowvoltage_pin!(PB12, 11, 4, |regs| {
    regs.devalt2().modify(|_, w| w.i2c0_0_sl().clear_bit());
});
impl_lowvoltage_pin!(PC12, 11, 5, |regs| {
    regs.devalt2().modify(|_, w| w.i2c0_0_sl().clear_bit());
});
impl_pin!(PL09, 11, 6, |regs| {
    regs.devalt4().modify(|_, w| w.pwm4_led0_sl().clear_bit());
});
impl_lowvoltage_pin!(PJ07, 11, 7, |regs| {
    regs.devalt4().modify(|_, w| w.pwm4_led0_sl().clear_bit());
    regs.devaltk().modify(|_, w| w.i2c7_1_sl().clear_bit());
});
