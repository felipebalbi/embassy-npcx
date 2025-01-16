use core::convert::Infallible;
use core::ops::{Deref, DerefMut};

use embassy_hal_internal::Peripheral;

use crate::gpio::{CanPullUp, Input, InputPin, LowVoltagePin, PullDownOnly};
use crate::miwu::{WakeUp, WakeUpInput};

pub trait AwaitableInputPin {}

pub struct AwaitableInput<'d, T> {
    pin: Input<'d, T>,
    wui: WakeUp<'d>,
}

impl<'d, T> AwaitableInput<'d, T> {
    pub fn new<PIN, WUI>(
        pin: impl Peripheral<P = PIN> + 'd,
        wui: impl Peripheral<P = WUI> + 'd,
    ) -> AwaitableInput<'d, CanPullUp>
    where
        PIN: InputPin + 'd,
        WUI: WakeUpInput + 'd,
        (PIN, WUI): AwaitableInputPin,
    {
        AwaitableInput {
            pin: Input::new(pin),
            wui: WakeUp::new(wui),
        }
    }

    pub fn new_lowvoltage<PIN, WUI>(
        pin: impl Peripheral<P = PIN> + 'd,
        wui: impl Peripheral<P = WUI> + 'd,
    ) -> AwaitableInput<'d, PullDownOnly>
    where
        PIN: LowVoltagePin + 'd,
        WUI: WakeUpInput + 'd,
        (PIN, WUI): AwaitableInputPin,
    {
        AwaitableInput {
            pin: Input::new_lowvoltage(pin),
            wui: WakeUp::new(wui),
        }
    }
}

impl<'d, T> Deref for AwaitableInput<'d, T> {
    type Target = Input<'d, T>;

    fn deref(&self) -> &Self::Target {
        &self.pin
    }
}

impl<'d, T> DerefMut for AwaitableInput<'d, T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.pin
    }
}

impl<'d, T> embedded_hal::digital::ErrorType for AwaitableInput<'d, T> {
    type Error = Infallible;
}

impl<'d, T> embedded_hal_async::digital::Wait for AwaitableInput<'d, T> {
    async fn wait_for_high(&mut self) -> Result<(), Self::Error> {
        Ok(self.wui.wait_for_high().await)
    }

    async fn wait_for_low(&mut self) -> Result<(), Self::Error> {
        Ok(self.wui.wait_for_low().await)
    }

    async fn wait_for_rising_edge(&mut self) -> Result<(), Self::Error> {
        Ok(self
            .wui
            .wait_for(crate::miwu::Mode::Edge(crate::miwu::Edge::Rising))
            .await)
    }

    async fn wait_for_falling_edge(&mut self) -> Result<(), Self::Error> {
        Ok(self
            .wui
            .wait_for(crate::miwu::Mode::Edge(crate::miwu::Edge::Falling))
            .await)
    }

    async fn wait_for_any_edge(&mut self) -> Result<(), Self::Error> {
        Ok(self.wui.wait_for(crate::miwu::Mode::Edge(crate::miwu::Edge::Any)).await)
    }
}

macro_rules! impl_pin_channel {
    ($pin:ident, $channel:ident) => {
        impl AwaitableInputPin for (crate::gpio::$pin, crate::peripherals::$channel) {}
    };
}

impl_pin_channel!(Gpio63, MIWU1_73);
impl_pin_channel!(Gpio90, MIWU0_20);
