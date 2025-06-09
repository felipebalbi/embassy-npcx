//! eSPI driver.

use core::future::poll_fn;
use core::marker::PhantomData;
use core::task::Poll;

use embassy_hal_internal::{Peri, PeripheralType};
use embassy_sync::waitqueue::AtomicWaker;

/// nALERT Mode.
#[non_exhaustive]
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum AlertMode {
    /// nALERT is multiplexed with eSPI_IO1 (default).
    IO1,

    /// nALERT is generated via neSPI_ALERT pin.
    Pin,
}

impl From<AlertMode> for bool {
    fn from(value: AlertMode) -> bool {
        match value {
            AlertMode::IO1 => false,
            AlertMode::Pin => true,
        }
    }
}

/// I/O Mode.
#[non_exhaustive]
#[derive(Clone, Copy)]
pub enum IoMode {
    /// Single I/O (default).
    Single,

    /// Dual I/O.
    Dual,

    /// Quad I/O.
    Quad,
}

impl From<IoMode> for u8 {
    fn from(value: IoMode) -> u8 {
        match value {
            IoMode::Single => 0,
            IoMode::Dual => 1,
            IoMode::Quad => 2,
        }
    }
}

/// Operating frequency.
#[non_exhaustive]
#[derive(Clone, Copy)]
pub enum Frequency {
    /// 20MHz (default).
    _20MHz,

    /// 25MHz
    _25MHz,

    /// 33MHz.
    _33MHz,

    /// 50MHz.
    _50MHz,

    /// 66MHz.
    _66MHz,
}

impl From<Frequency> for u8 {
    fn from(value: Frequency) -> u8 {
        match value {
            Frequency::_20MHz => 0,
            Frequency::_25MHz => 1,
            Frequency::_33MHz => 2,
            Frequency::_50MHz => 3,
            Frequency::_66MHz => 4,
        }
    }
}

/// Flash access channel mode
#[non_exhaustive]
#[derive(Clone, Copy)]
pub enum FlashAccessMode {
    /// Controller Attached Flash
    CAF,

    /// Target Attached Flash
    TAF,
}

impl From<FlashAccessMode> for bool {
    fn from(value: FlashAccessMode) -> bool {
        match value {
            FlashAccessMode::CAF => false,
            FlashAccessMode::TAF => true,
        }
    }
}

/// eSPI configuration.
#[non_exhaustive]
#[derive(Clone, Copy)]
pub struct Config {
    /// Enable CRC check.
    pub crc_enable: bool,

    /// Alert pin configuration
    pub alert_mode: AlertMode,

    /// I/O width.
    pub io_mode: IoMode,

    /// Operating frequency.
    pub frequency: Frequency,

    /// Maximum wait state.
    ///
    /// Valid values are from 1 to 16.
    pub wait_state: u8,

    /// Enable Peripheral Channel support.
    pub peripheral_channel_support: bool,

    /// Enable Virtual Wire Channel support.
    pub virtual_wire_support: bool,

    /// Enable OOB Channel support.
    pub oob_support: bool,

    /// Flash Access Channel support.
    pub flash_access_support: bool,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            crc_enable: false,
            alert_mode: AlertMode::IO1,
            io_mode: IoMode::Single,
            frequency: Frequency::_20MHz,
            wait_state: 1,
            peripheral_channel_support: false,
            virtual_wire_support: false,
            oob_support: false,
            flash_access_support: false,
        }
    }
}

/// eSPI errors.
#[non_exhaustive]
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// Bus Master Burst Mode Write Transfer Error.
    BusMasterBurstModeWriteTransferError,

    /// Bus Master Burst Mode Read Transrfer Error.
    BusMasterBurstModeReadTransferError,

    /// Flash Protection Error.
    FlashProtectionError,

    /// Automatic Mode Transfer Error.
    AutomaticModeTransferError,

    /// eSPI Bus Error.
    EspiBusError,
}

/// eSPI events.
#[non_exhaustive]
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Event {
    /// Bus Master Burst Mode Write Transfer Done.
    BusMasterBurstModeWriteTransferDone,

    /// Bus Master Burst Mode Read Transfer Done.
    BusMasterBurstModeReadTransferDone,

    /// Automatic Read Disable Status.
    AutomaticReadDisableStatus,

    /// Flash Automatic Read Queue Empty.
    FlashAutomaticReadQueueEmpty,

    /// Flash Automatic Read Request Pending.
    FlashAutomaticReadRequestPending,

    /// Flash Automatic Read Request Start.
    FlashAutomaticReadRequestStart,

    /// Peripheral Message Data Received.
    PeripheralMessageDataReceived,

    /// Peripheral Bus Master Data Received.
    PeripheralBusMasterDataReceived,

    /// Peripheral Bus Master Data Transmitted.
    PeripheralBusMasterDataTransmitted,

    /// Flash Non-posted Request Sent.
    FlashNonPostedRequestSent,

    /// Virtual Wire Updated Wake-up.
    VirtualWireUpdatedWakeUp,

    /// Automatic Mode Transfer Done.
    AutomaticModeTransferDone,

    /// Platform Reset.
    PlatformReset,

    /// eSPI Reset
    EspiReset,

    /// Virtual Wire Updated.
    VirtualWireUpdated,

    /// Peripheral Channel Transaction Deferred.
    PeripheralChannelTransactionDeferred,

    /// Peripheral Channel Access Detected.
    PeripheralChannelAccessDetected,

    /// Flash Non-automatic Completion Sent.
    FlashNonAutomaticCompletionSent,

    /// Flash Data Received.
    FlashDataReceived,

    /// OOB Data Received.
    OobDataReceived,

    /// eSPI Configuration Updated.
    EspiConfigurationUpdated,

    /// In-band Reset Command Received.
    InBandResetCmdReceived,
}

/// An instance of the eSPI driver
pub struct Espi<'p, T: Instance> {
    _peri: Peri<'p, T>,
    regs: &'static crate::pac::espi::RegisterBlock,
    waker: &'static AtomicWaker,
}

impl<'p, T: Instance> Espi<'p, T> {
    /// Create a new instance of eSPI.
    pub fn new(
        _peri: Peri<'p, T>,
        _irqs: impl crate::interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>>,
        config: Config,
    ) -> Self {
        T::regs().espicfg().write(|w| unsafe {
            w.flashchn_supp()
                .variant(config.flash_access_support)
                .oobchn_supp()
                .variant(config.oob_support)
                .vwchn_supp()
                .variant(config.virtual_wire_support)
                .pcchn_supp()
                .variant(config.peripheral_channel_support)
                .maxfreq()
                .bits(config.frequency.into())
                .iomode()
                .bits(config.io_mode.into())
        });

        Self {
            _peri,
            regs: T::regs(),
            waker: T::waker(),
        }
    }

    /// Listen for a new event.
    pub async fn listen(&mut self) -> Result<Event, Error> {
        self.wait_for(
            |_| {
                let status = T::regs().espists().read();

                if status.ibrst().bit_is_set() {
                    T::regs().espists().write(|w| w.ibrst().set_bit());
                    Poll::Ready(Ok(Event::InBandResetCmdReceived))
                } else if status.cfgupd().bit_is_set() {
                    T::regs().espists().write(|w| w.cfgupd().set_bit());
                    Poll::Ready(Ok(Event::EspiConfigurationUpdated))
                } else if status.berr().bit_is_set() {
                    T::regs().espists().write(|w| w.berr().set_bit());
                    Poll::Ready(Err(Error::EspiBusError))
                } else if status.oobrx().bit_is_set() {
                    T::regs().espists().write(|w| w.oobrx().set_bit());
                    Poll::Ready(Ok(Event::OobDataReceived))
                } else if status.flashrx().bit_is_set() {
                    T::regs().espists().write(|w| w.flashrx().set_bit());
                    Poll::Ready(Ok(Event::FlashDataReceived))
                } else if status.flnacs().bit_is_set() {
                    T::regs().espists().write(|w| w.flnacs().set_bit());
                    Poll::Ready(Ok(Event::FlashNonAutomaticCompletionSent))
                } else if status.peracc().bit_is_set() {
                    T::regs().espists().write(|w| w.peracc().set_bit());
                    Poll::Ready(Ok(Event::PeripheralChannelAccessDetected))
                } else if status.dfrd().bit_is_set() {
                    T::regs().espists().write(|w| w.dfrd().set_bit());
                    Poll::Ready(Ok(Event::PeripheralChannelTransactionDeferred))
                } else if status.vwupd().bit_is_set() {
                    T::regs().espists().write(|w| w.vwupd().set_bit());
                    Poll::Ready(Ok(Event::VirtualWireUpdated))
                } else if status.espirst().bit_is_set() {
                    T::regs().espists().write(|w| w.espirst().set_bit());
                    Poll::Ready(Ok(Event::EspiReset))
                } else if status.pltrst().bit_is_set() {
                    T::regs().espists().write(|w| w.pltrst().set_bit());
                    Poll::Ready(Ok(Event::PlatformReset))
                } else if status.amerr().bit_is_set() {
                    T::regs().espists().write(|w| w.amerr().set_bit());
                    Poll::Ready(Err(Error::AutomaticModeTransferError))
                } else if status.amdone().bit_is_set() {
                    T::regs().espists().write(|w| w.amdone().set_bit());
                    Poll::Ready(Ok(Event::AutomaticModeTransferDone))
                } else if status.vwupdw().bit_is_set() {
                    T::regs().espists().write(|w| w.vwupdw().set_bit());
                    Poll::Ready(Ok(Event::VirtualWireUpdatedWakeUp))
                } else if status.flnprqs().bit_is_set() {
                    T::regs().espists().write(|w| w.flnprqs().set_bit());
                    Poll::Ready(Ok(Event::FlashNonPostedRequestSent))
                } else if status.bmtxdone().bit_is_set() {
                    T::regs().espists().write(|w| w.bmtxdone().set_bit());
                    Poll::Ready(Ok(Event::PeripheralBusMasterDataTransmitted))
                } else if status.pbmrx().bit_is_set() {
                    T::regs().espists().write(|w| w.pbmrx().set_bit());
                    Poll::Ready(Ok(Event::PeripheralBusMasterDataReceived))
                } else if status.pmsgrx().bit_is_set() {
                    T::regs().espists().write(|w| w.pmsgrx().set_bit());
                    Poll::Ready(Ok(Event::PeripheralMessageDataReceived))
                } else if status.bmbursterr().bit_is_set() {
                    T::regs().espists().write(|w| w.bmbursterr().set_bit());
                    Poll::Ready(Err(Error::BusMasterBurstModeReadTransferError))
                } else if status.bmburstdone().bit_is_set() {
                    T::regs().espists().write(|w| w.bmburstdone().set_bit());
                    Poll::Ready(Ok(Event::BusMasterBurstModeReadTransferDone))
                } else if status.flprterr().bit_is_set() {
                    T::regs().espists().write(|w| w.flprterr().set_bit());
                    Poll::Ready(Err(Error::FlashProtectionError))
                } else if status.flautordstr().bit_is_set() {
                    T::regs().espists().write(|w| w.flautordstr().set_bit());
                    Poll::Ready(Ok(Event::FlashAutomaticReadRequestStart))
                } else if status.flautordpnd().bit_is_set() {
                    T::regs().espists().write(|w| w.flautordpnd().set_bit());
                    Poll::Ready(Ok(Event::FlashAutomaticReadRequestPending))
                } else if status.flautordqemp().bit_is_set() {
                    T::regs().espists().write(|w| w.flautordqemp().set_bit());
                    Poll::Ready(Ok(Event::FlashAutomaticReadQueueEmpty))
                } else if status.auto_rd_dis_sts().bit_is_set() {
                    T::regs().espists().write(|w| w.auto_rd_dis_sts().set_bit());
                    Poll::Ready(Ok(Event::AutomaticReadDisableStatus))
                } else if status.bmwbursterr().bit_is_set() {
                    T::regs().espists().write(|w| w.bmwbursterr().set_bit());
                    Poll::Ready(Err(Error::BusMasterBurstModeWriteTransferError))
                } else if status.bmwburstdone().bit_is_set() {
                    T::regs().espists().write(|w| w.bmwburstdone().set_bit());
                    Poll::Ready(Ok(Event::BusMasterBurstModeWriteTransferDone))
                } else {
                    Poll::Pending
                }
            },
            |_| {
                // Enable all interrupts
                T::regs().espiie().write(|w| {
                    w.bmwburstdoneie()
                        .set_bit()
                        .bmwbursterrie()
                        .set_bit()
                        .flautorddisie()
                        .set_bit()
                        .flautordqempie()
                        .set_bit()
                        .flautordpndie()
                        .set_bit()
                        .flautordstrie()
                        .set_bit()
                        .flprterrie()
                        .set_bit()
                        .bmburstdoneie()
                        .set_bit()
                        .bmbursterrie()
                        .set_bit()
                        .pmsgrxie()
                        .set_bit()
                        .pbmrxie()
                        .set_bit()
                        .bmtxdoneie()
                        .set_bit()
                        .flnprqsie()
                        .set_bit()
                        .amdoneie()
                        .set_bit()
                        .amerrie()
                        .set_bit()
                        .pltrstie()
                        .set_bit()
                        .espirstie()
                        .set_bit()
                        .vwupdie()
                        .set_bit()
                        .dfrdie()
                        .set_bit()
                        .peraccie()
                        .set_bit()
                        .flnacsie()
                        .set_bit()
                        .flashrxie()
                        .set_bit()
                        .oobrxie()
                        .set_bit()
                        .berrie()
                        .set_bit()
                        .cfgupdie()
                        .set_bit()
                        .ibrstie()
                        .set_bit()
                });
            },
        )
        .await
    }

    /// Calls `f` to check if we are ready or not.
    /// If not, `g` is called once the waker is set (to eg enable the required interrupts).
    async fn wait_for<F, U, G>(&mut self, mut f: F, mut g: G) -> U
    where
        F: FnMut(&mut Self) -> Poll<U>,
        G: FnMut(&mut Self),
    {
        poll_fn(|cx| {
            // Register waker before checking condition, to ensure that wakes/interrupts
            // aren't lost between f() and g()
            T::waker().register(cx.waker());
            let r = f(self);

            if r.is_pending() {
                g(self);
            }

            r
        })
        .await
    }
}

/// The interrupt handler for the [Espi] controller.
pub struct InterruptHandler<T> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> crate::interrupt::typelevel::Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        if T::regs().espists().read().bits() != 0 {
            // Disable all interrupts
            T::regs().espiie().write(|w| {
                w.bmwburstdoneie()
                    .clear_bit()
                    .bmwbursterrie()
                    .clear_bit()
                    .flautorddisie()
                    .clear_bit()
                    .flautordqempie()
                    .clear_bit()
                    .flautordpndie()
                    .clear_bit()
                    .flautordstrie()
                    .clear_bit()
                    .flprterrie()
                    .clear_bit()
                    .bmburstdoneie()
                    .clear_bit()
                    .bmbursterrie()
                    .clear_bit()
                    .pmsgrxie()
                    .clear_bit()
                    .pbmrxie()
                    .clear_bit()
                    .bmtxdoneie()
                    .clear_bit()
                    .flnprqsie()
                    .clear_bit()
                    .amdoneie()
                    .clear_bit()
                    .amerrie()
                    .clear_bit()
                    .pltrstie()
                    .clear_bit()
                    .espirstie()
                    .clear_bit()
                    .vwupdie()
                    .clear_bit()
                    .dfrdie()
                    .clear_bit()
                    .peraccie()
                    .clear_bit()
                    .flnacsie()
                    .clear_bit()
                    .flashrxie()
                    .clear_bit()
                    .oobrxie()
                    .clear_bit()
                    .berrie()
                    .clear_bit()
                    .cfgupdie()
                    .clear_bit()
                    .ibrstie()
                    .clear_bit()
            });

            // Wake the waker
            T::waker().wake()
        }
    }
}

mod sealed {
    use embassy_sync::waitqueue::AtomicWaker;

    pub trait SealedInstance {
        fn waker() -> &'static AtomicWaker;
        fn regs() -> &'static crate::pac::espi::RegisterBlock;
    }
}

/// A marker trait implemented by all eSPI instances
pub trait Instance: sealed::SealedInstance + PeripheralType + 'static + Send {
    /// The interrupt used by this instance
    type Interrupt: crate::interrupt::typelevel::Interrupt;
}

macro_rules! impl_instance {
    ($peri:ident, $irq:ident) => {
        impl sealed::SealedInstance for crate::peripherals::$peri {
            fn waker() -> &'static AtomicWaker {
                static WAKER: AtomicWaker = AtomicWaker::new();
                &WAKER
            }

            fn regs() -> &'static crate::pac::espi::RegisterBlock {
                // Safety: not owned, memory is always present
                unsafe { &*crate::pac::Espi::ptr() }
            }
        }

        impl Instance for crate::peripherals::$peri {
            type Interrupt = crate::interrupt::typelevel::$irq;
        }
    };
}

impl_instance!(ESPI, ESPI_SHI);
