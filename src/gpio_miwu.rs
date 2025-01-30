//! Awaitable GPIO input pins.

use core::convert::Infallible;
use core::ops::{Deref, DerefMut};

use embassy_hal_internal::Peripheral;

use crate::gpio::{CanPullUp, Input, InputPin, LowVoltagePin, PullDownOnly};
use crate::miwu::{Edge, InterruptHandler, Level, WakeUp, WakeUpInput};

mod sealed {
    pub trait SealedAwaitableInputPin {}
}

/// GPIO pins that have an WakeUpInput channel associated with them.
pub trait AwaitableInputPin: sealed::SealedAwaitableInputPin {}

/// Driver for GPIO input pins and their WakeUpInput channel.
///
/// Dereferences to the underlying [Input] driver instance,
/// but also implements [embedded_hal_async::digital::Wait].
pub struct AwaitableInput<'d, T> {
    pin: Input<'d, T>,
    wui: WakeUp<'d>,
}

impl<'d> AwaitableInput<'d, CanPullUp> {
    pub fn new<PIN, WUI>(
        pin: impl Peripheral<P = PIN> + 'd,
        wui: impl Peripheral<P = WUI> + 'd,
        irqs: impl crate::interrupt::typelevel::Binding<WUI::Interrupt, InterruptHandler<WUI>>,
    ) -> Self
    where
        PIN: InputPin + 'd,
        WUI: WakeUpInput + 'd,
        (PIN, WUI): AwaitableInputPin,
    {
        AwaitableInput {
            pin: Input::new(pin),
            wui: WakeUp::new(wui, irqs),
        }
    }
}

impl<'d> AwaitableInput<'d, PullDownOnly> {
    pub fn new_lowvoltage<PIN, WUI>(
        pin: impl Peripheral<P = PIN> + 'd,
        wui: impl Peripheral<P = WUI> + 'd,
        irqs: impl crate::interrupt::typelevel::Binding<WUI::Interrupt, InterruptHandler<WUI>>,
    ) -> Self
    where
        PIN: LowVoltagePin + 'd,
        WUI: WakeUpInput + 'd,
        (PIN, WUI): AwaitableInputPin,
    {
        AwaitableInput {
            pin: Input::new_lowvoltage(pin),
            wui: WakeUp::new(wui, irqs),
        }
    }
}

impl<'d, T> Deref for AwaitableInput<'d, T> {
    type Target = Input<'d, T>;

    fn deref(&self) -> &Self::Target {
        &self.pin
    }
}

impl<T> DerefMut for AwaitableInput<'_, T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.pin
    }
}

impl<T> embedded_hal::digital::ErrorType for AwaitableInput<'_, T> {
    type Error = Infallible;
}

impl<T> embedded_hal_async::digital::Wait for AwaitableInput<'_, T> {
    async fn wait_for_high(&mut self) -> Result<(), Self::Error> {
        if self.is_high() {
            return Ok(());
        }
        self.wui.wait_for(Level::High).await;
        Ok(())
    }

    async fn wait_for_low(&mut self) -> Result<(), Self::Error> {
        if self.is_low() {
            return Ok(());
        }
        self.wui.wait_for(Level::Low).await;
        Ok(())
    }

    async fn wait_for_rising_edge(&mut self) -> Result<(), Self::Error> {
        self.wui.wait_for(Edge::Rising).await;
        Ok(())
    }

    async fn wait_for_falling_edge(&mut self) -> Result<(), Self::Error> {
        self.wui.wait_for(Edge::Falling).await;
        Ok(())
    }

    async fn wait_for_any_edge(&mut self) -> Result<(), Self::Error> {
        self.wui.wait_for(Edge::Any).await;
        Ok(())
    }
}

macro_rules! impl_pin_channel {
    ($pin:ident, $channel:ident) => {
        impl sealed::SealedAwaitableInputPin for (crate::gpio::$pin, crate::peripherals::$channel) {}
        impl AwaitableInputPin for (crate::gpio::$pin, crate::peripherals::$channel) {}
    };
}

impl_pin_channel!(Gpio80, MIWU0_10);
impl_pin_channel!(Gpio81, MIWU0_11);
impl_pin_channel!(Gpio82, MIWU0_12);
impl_pin_channel!(Gpio83, MIWU0_13);
impl_pin_channel!(Gpio87, MIWU0_17);

impl_pin_channel!(Gpio90, MIWU0_20);
impl_pin_channel!(Gpio91, MIWU0_21);
impl_pin_channel!(Gpio92, MIWU0_22);
impl_pin_channel!(Gpio93, MIWU0_23);
impl_pin_channel!(Gpio94, MIWU0_24);
impl_pin_channel!(Gpio95, MIWU0_25);

impl_pin_channel!(Gpio96, MIWU0_30);
impl_pin_channel!(Gpio97, MIWU0_31);
impl_pin_channel!(Gpioa0, MIWU0_32);
impl_pin_channel!(Gpioa1, MIWU0_33);
impl_pin_channel!(Gpioa2, MIWU0_34);
impl_pin_channel!(Gpioa3, MIWU0_35);
impl_pin_channel!(Gpioa4, MIWU0_36);
impl_pin_channel!(Gpioa5, MIWU0_37);

impl_pin_channel!(Gpioa6, MIWU0_40);
impl_pin_channel!(Gpioa7, MIWU0_41);
impl_pin_channel!(Gpiob0, MIWU0_42);
impl_pin_channel!(Gpiob1, MIWU0_45);
impl_pin_channel!(Gpiob2, MIWU0_46);

impl_pin_channel!(Gpiob3, MIWU0_50);
impl_pin_channel!(Gpiob4, MIWU0_51);
impl_pin_channel!(Gpiob5, MIWU0_52);
impl_pin_channel!(Gpiob7, MIWU0_54);

impl_pin_channel!(Gpioc0, MIWU0_60);
impl_pin_channel!(Gpioc1, MIWU0_61);
impl_pin_channel!(Gpioc2, MIWU0_62);
impl_pin_channel!(Gpioc3, MIWU0_63);
impl_pin_channel!(Gpioc4, MIWU0_64);
impl_pin_channel!(Gpioc5, MIWU0_65);
impl_pin_channel!(Gpioc6, MIWU0_66);
impl_pin_channel!(Gpioc7, MIWU0_67);

impl_pin_channel!(Gpiod0, MIWU0_70);
impl_pin_channel!(Gpiod1, MIWU0_71);
impl_pin_channel!(Gpiod2, MIWU0_72);
impl_pin_channel!(Gpiod3, MIWU0_73);
impl_pin_channel!(Gpiod4, MIWU0_74);
impl_pin_channel!(Gpiod5, MIWU0_75);
impl_pin_channel!(Gpioe0, MIWU0_77);

impl_pin_channel!(Gpioe1, MIWU0_80);
impl_pin_channel!(Gpioe2, MIWU0_81);
impl_pin_channel!(Gpioe3, MIWU0_82);
impl_pin_channel!(Gpioe4, MIWU0_83);
impl_pin_channel!(Gpioe5, MIWU0_84);
impl_pin_channel!(Gpiof0, MIWU0_85);
impl_pin_channel!(Gpiof3, MIWU0_86);
impl_pin_channel!(Gpioe7, MIWU0_87);

impl_pin_channel!(Gpio00, MIWU1_10);
impl_pin_channel!(Gpio01, MIWU1_11);
impl_pin_channel!(Gpio02, MIWU1_12);
impl_pin_channel!(Gpio03, MIWU1_13);
impl_pin_channel!(Gpio04, MIWU1_14);
impl_pin_channel!(Gpio05, MIWU1_15);
impl_pin_channel!(Gpio06, MIWU1_16);
impl_pin_channel!(Gpio07, MIWU1_17);

impl_pin_channel!(Gpio10, MIWU1_20);
impl_pin_channel!(Gpio11, MIWU1_21);
impl_pin_channel!(Gpiof4, MIWU1_22);
impl_pin_channel!(Gpio13, MIWU1_23);
impl_pin_channel!(Gpio14, MIWU1_24);
impl_pin_channel!(Gpio15, MIWU1_25);
impl_pin_channel!(Gpio16, MIWU1_26);
impl_pin_channel!(Gpio17, MIWU1_27);

impl_pin_channel!(Gpio31, MIWU1_30);
impl_pin_channel!(Gpio30, MIWU1_31);
impl_pin_channel!(Gpio27, MIWU1_32);
impl_pin_channel!(Gpio26, MIWU1_33);
impl_pin_channel!(Gpio25, MIWU1_34);
impl_pin_channel!(Gpio24, MIWU1_35);
impl_pin_channel!(Gpio23, MIWU1_36);
impl_pin_channel!(Gpio22, MIWU1_37);

impl_pin_channel!(Gpio20, MIWU1_40);
impl_pin_channel!(Gpio21, MIWU1_41);
impl_pin_channel!(Gpiof5, MIWU1_42);
impl_pin_channel!(Gpio33, MIWU1_43);
impl_pin_channel!(Gpio34, MIWU1_44);
impl_pin_channel!(Gpio36, MIWU1_46);
impl_pin_channel!(Gpio37, MIWU1_47);

impl_pin_channel!(Gpio40, MIWU1_50);
impl_pin_channel!(Gpio41, MIWU1_51);
impl_pin_channel!(Gpio42, MIWU1_52);
impl_pin_channel!(Gpio43, MIWU1_53);
impl_pin_channel!(Gpio44, MIWU1_54);
impl_pin_channel!(Gpio45, MIWU1_55);
impl_pin_channel!(Gpio46, MIWU1_56);
impl_pin_channel!(Gpio47, MIWU1_57);

impl_pin_channel!(Gpio50, MIWU1_60);
impl_pin_channel!(Gpio51, MIWU1_61);
impl_pin_channel!(Gpio52, MIWU1_62);
impl_pin_channel!(Gpio53, MIWU1_63);
impl_pin_channel!(Gpio54, MIWU1_64);
impl_pin_channel!(Gpio55, MIWU1_65);
impl_pin_channel!(Gpio56, MIWU1_66);
impl_pin_channel!(Gpio57, MIWU1_67);

impl_pin_channel!(Gpio60, MIWU1_70);
impl_pin_channel!(Gpio61, MIWU1_71);
impl_pin_channel!(Gpio62, MIWU1_72);
impl_pin_channel!(Gpio63, MIWU1_73);
impl_pin_channel!(Gpio64, MIWU1_74);

impl_pin_channel!(Gpio70, MIWU1_80);
impl_pin_channel!(Gpio67, MIWU1_81);
impl_pin_channel!(Gpio72, MIWU1_82);
impl_pin_channel!(Gpio73, MIWU1_83);
impl_pin_channel!(Gpio74, MIWU1_84);
impl_pin_channel!(Gpio75, MIWU1_85);
impl_pin_channel!(Gpio76, MIWU1_86);

impl_pin_channel!(Gpio12, MIWU2_60);
impl_pin_channel!(Gpiof1, MIWU2_61);
impl_pin_channel!(Gpiof2, MIWU2_62);
impl_pin_channel!(Gpiod6, MIWU2_65);
