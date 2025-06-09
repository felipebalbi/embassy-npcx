//! eSPI driver.

use core::future::poll_fn;
use core::marker::PhantomData;
use core::task::Poll;

use embassy_hal_internal::{Peri, PeripheralType};
use embassy_sync::waitqueue::AtomicWaker;

use crate::interrupt::typelevel::Interrupt;
use crate::pac::espi::espierr::EspierrSpec;
use crate::pac::espi::espists::EspistsSpec;
use crate::pac::generic::Writable;

/// Pin that can be used as ESPI_IO0.
pub type Io0Pin = crate::peripherals::PH01;
/// Pin that can be used as ESPI_IO1.
pub type Io1Pin = crate::peripherals::PJ01;
/// Pin that can be used as ESPI_IO2.
pub type Io2Pin = crate::peripherals::PK01;
/// Pin that can be used as ESPI_IO3.
pub type Io3Pin = crate::peripherals::PL01;
/// Pin that can be used as ESPI_CS.
pub type CsPin = crate::peripherals::PL02;
/// Pin that can be used as ESPI_RST
pub type RstPin = crate::peripherals::PK03;
/// Pin that can be used as ESPI_CLK.
pub type ClkPin = crate::peripherals::PM01;
/// Pin that can be used as ESPI_ALERT.
pub type AlertPin = crate::peripherals::PL03;

/// nALERT Mode.
#[non_exhaustive]
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum AlertMode {
    /// nALERT is multiplexed with eSPI_IO1 (default).
    IO1,

    /// nALERT is generated via neSPI_ALERT pin.
    Pin,
}

impl From<bool> for AlertMode {
    fn from(value: bool) -> AlertMode {
        match value {
            false => AlertMode::IO1,
            true => AlertMode::Pin,
        }
    }
}

/// Enabled state
#[non_exhaustive]
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum State {
    /// Disabled,
    Disabled,

    /// Enabled,
    Enabled,
}

impl From<bool> for State {
    fn from(value: bool) -> State {
        match value {
            false => State::Disabled,
            true => State::Enabled,
        }
    }
}

/// I/O Mode.
#[non_exhaustive]
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum IoMode {
    /// Single I/O (default).
    Single,

    /// Dual I/O.
    Dual,

    /// Quad I/O.
    Quad,
}

impl TryFrom<u8> for IoMode {
    type Error = Error;

    fn try_from(value: u8) -> Result<IoMode, Error> {
        match value {
            0 => Ok(IoMode::Single),
            1 => Ok(IoMode::Dual),
            2 => Ok(IoMode::Quad),
            _ => Err(Error::Other),
        }
    }
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
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
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

impl TryFrom<u8> for Frequency {
    type Error = Error;

    fn try_from(value: u8) -> Result<Frequency, Error> {
        match value {
            0 => Ok(Frequency::_20MHz),
            1 => Ok(Frequency::_25MHz),
            2 => Ok(Frequency::_33MHz),
            3 => Ok(Frequency::_50MHz),
            4 => Ok(Frequency::_66MHz),
            _ => Err(Error::Other),
        }
    }
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
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FlashAccessMode {
    /// Controller Attached Flash
    CAF,

    /// Target Attached Flash
    TAF,
}

impl From<bool> for FlashAccessMode {
    fn from(value: bool) -> FlashAccessMode {
        match value {
            false => FlashAccessMode::CAF,
            true => FlashAccessMode::TAF,
        }
    }
}

/// eSPI configuration.
#[non_exhaustive]
#[derive(Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Config {
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

/// eSPI bus errors.
#[non_exhaustive]
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum BusError {
    /// Unsuccessful flash completion.
    UnsuccessfulFlashCompletion,

    /// Unsuccessful bus master completion.
    UnsuccessfulBusMasterCompletion,

    /// Virtual Wire channel access error.
    VWChannelAccessError,

    /// Extra eSPI clock cycles.
    ExtraCycles,

    /// Unsupported command or cycle type.
    UnsupportedCmd,

    /// Posted peripheral channel bad address alignment.
    PostedBadAlignment,

    /// Non-posted peripheral channel bad address alignment.
    NonPostedBadAlignment,

    /// Bad size.
    BadSize,

    /// Protocol error.
    ProtocolError,

    /// Abnormal completion.
    AbnormalCompletion,

    /// CRC error.
    CRCError,

    /// Invalid cycle type.
    InvalidCycleType,

    /// Invalid command type.
    InvalidCommandType,
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
    EspiBusError(BusError),

    /// Other eSPI error.
    Other,
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
    EspiReset(bool),

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
    EspiConfigurationUpdated(Configuration),

    /// In-band Reset Command Received.
    InBandResetCmdReceived,
}

/// Host side eSPI configuration data
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Configuration {
    /// CRC check state
    pub crc_check: State,

    /// Selected alert mode
    pub alert_mode: AlertMode,

    /// Selected I/O mode
    pub io_mode: IoMode,

    /// Selected operating frequency
    pub frequency: Frequency,

    /// Selected flash channel access mode
    pub flash_access_mode: FlashAccessMode,

    /// Host-side flash channel state
    pub host_flash_channel: State,

    /// Host-side OOB channel state
    pub host_oob_channel: State,

    /// Host-side VWire channel state
    pub host_vwire_channel: State,

    /// Host-side peripheral channel state
    pub host_peripheral_channel: State,
}

/// An instance of the eSPI driver
pub struct Espi<'p, T: Instance> {
    _peri: Peri<'p, T>,
    pltrst_received: bool,
}

impl<'p, T: Instance> Espi<'p, T> {
    /// Create a new instance of eSPI.
    pub fn new(
        _peri: Peri<'p, T>,
        _io0: Peri<'p, Io0Pin>,
        _io1: Peri<'p, Io1Pin>,
        _io2: Peri<'p, Io2Pin>,
        _io3: Peri<'p, Io3Pin>,
        _cs: Peri<'p, CsPin>,
        _rst: Peri<'p, RstPin>,
        _clk: Peri<'p, ClkPin>,
        _alert: Peri<'p, AlertPin>,
        _irqs: impl crate::interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>>,
        config: Config,
    ) -> Self
    where
        (T, Io0Pin, Io1Pin, Io2Pin, Io3Pin, CsPin, RstPin, ClkPin, AlertPin): ValidEspiConfig,
    {
        critical_section::with(|cs| {
            // Safety: we have exclusive ownership over the peripherals.
            unsafe {
                <(T, Io0Pin, Io1Pin, Io2Pin, Io3Pin, CsPin, RstPin, ClkPin, AlertPin) as sealed::SealedValidEspiConfig>::setup_pins(cs);
                <(T, Io0Pin, Io1Pin, Io2Pin, Io3Pin, CsPin, RstPin, ClkPin, AlertPin) as sealed::SealedValidEspiConfig>::setup_pullup(cs);
            }

            T::regs().espicfg().modify(|_, w| unsafe {
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

            if config.virtual_wire_support {
                // configure inputs
                for reg in T::regs().vwevms_iter() {
                    reg.modify(|_, w| w.ie().set_bit().we().set_bit().index_en().set_bit());
                }

                // configure outputs
                for reg in T::regs().vwevsm_iter() {
                    reg.modify(|_, w| unsafe { w.hw_wire3_0().bits(0) }.index_en().set_bit());
                }

                // configure gpio outputs
                for reg in T::regs().vwgpsm_iter() {
                    reg.modify(|_, w| w.index_en().set_bit());
                }
            }
        });

        // Clear any pending status bits
        T::regs()
            .espists()
            .write(|w| unsafe { w.bits(EspistsSpec::ONE_TO_MODIFY_FIELDS_BITMAP) });

        T::Interrupt::unpend();

        // Safety: _Irqs ensures an interrupt handler is bound
        unsafe {
            T::Interrupt::enable();
        }

        Self {
            _peri,
            pltrst_received: false,
        }
    }

    /// Listen for a new event.
    pub async fn listen(&mut self) -> Result<Event, Error> {
        self.wait_for(
            |me| {
                let status = T::regs().espists().read();

                // Clear all events
                T::regs()
                    .espists()
                    .write(|w| unsafe { w.bits(EspistsSpec::ONE_TO_MODIFY_FIELDS_BITMAP) });

                if status.ibrst().bit_is_set() {
                    me.pltrst_received = false;
                    Poll::Ready(Ok(Event::InBandResetCmdReceived))
                } else if status.cfgupd().bit_is_set() {
                    let cfg = T::regs().espicfg().read();

                    let configuration = Configuration {
                        crc_check: cfg.crc_chk_en().bit().into(),
                        alert_mode: cfg.alertmode().bit().into(),
                        io_mode: cfg.iomodesel().bits().try_into()?,
                        frequency: cfg.opfreq().bits().try_into()?,
                        flash_access_mode: cfg.flchanmode().bit().into(),
                        host_flash_channel: cfg.hflashchanen().bit().into(),
                        host_oob_channel: cfg.hoobchanen().bit().into(),
                        host_vwire_channel: cfg.hvwchanen().bit().into(),
                        host_peripheral_channel: cfg.hpchanen().bit().into(),
                    };

                    Poll::Ready(Ok(Event::EspiConfigurationUpdated(configuration)))
                } else if status.berr().bit_is_set() {
                    // Clear all errors
                    let err = T::regs().espierr().read();
                    T::regs()
                        .espierr()
                        .write(|w| unsafe { w.bits(EspierrSpec::ONE_TO_MODIFY_FIELDS_BITMAP) });

                    if err.unflash().bit_is_set() {
                        Poll::Ready(Err(Error::EspiBusError(BusError::UnsuccessfulFlashCompletion)))
                    } else if err.unpbm().bit_is_set() {
                        Poll::Ready(Err(Error::EspiBusError(BusError::UnsuccessfulBusMasterCompletion)))
                    } else if err.vwerr().bit_is_set() {
                        Poll::Ready(Err(Error::EspiBusError(BusError::VWChannelAccessError)))
                    } else if err.extracyc().bit_is_set() {
                        Poll::Ready(Err(Error::EspiBusError(BusError::ExtraCycles)))
                    } else if err.uncmd().bit_is_set() {
                        Poll::Ready(Err(Error::EspiBusError(BusError::UnsupportedCmd)))
                    } else if err.pcbadaln().bit_is_set() {
                        Poll::Ready(Err(Error::EspiBusError(BusError::PostedBadAlignment)))
                    } else if err.npbadaln().bit_is_set() {
                        Poll::Ready(Err(Error::EspiBusError(BusError::NonPostedBadAlignment)))
                    } else if err.badsize().bit_is_set() {
                        Poll::Ready(Err(Error::EspiBusError(BusError::BadSize)))
                    } else if err.proterr().bit_is_set() {
                        Poll::Ready(Err(Error::EspiBusError(BusError::ProtocolError)))
                    } else if err.abcomp().bit_is_set() {
                        Poll::Ready(Err(Error::EspiBusError(BusError::AbnormalCompletion)))
                    } else if err.crcerr().bit_is_set() {
                        Poll::Ready(Err(Error::EspiBusError(BusError::CRCError)))
                    } else if err.invcyc().bit_is_set() {
                        Poll::Ready(Err(Error::EspiBusError(BusError::InvalidCycleType)))
                    } else if err.invcmd().bit_is_set() {
                        Poll::Ready(Err(Error::EspiBusError(BusError::InvalidCommandType)))
                    } else {
                        Poll::Ready(Err(Error::Other))
                    }
                } else if status.oobrx().bit_is_set() {
                    Poll::Ready(Ok(Event::OobDataReceived))
                } else if status.flashrx().bit_is_set() {
                    Poll::Ready(Ok(Event::FlashDataReceived))
                } else if status.flnacs().bit_is_set() {
                    Poll::Ready(Ok(Event::FlashNonAutomaticCompletionSent))
                } else if status.peracc().bit_is_set() {
                    Poll::Ready(Ok(Event::PeripheralChannelAccessDetected))
                } else if status.dfrd().bit_is_set() {
                    Poll::Ready(Ok(Event::PeripheralChannelTransactionDeferred))
                } else if status.pltrst().bit_is_set() {
                    me.pltrst_received = true;
                    Poll::Ready(Ok(Event::PlatformReset))
                } else if status.vwupd().bit_is_set() {
                    let supported = T::regs().espicfg().read().pcchn_supp().bit();
                    let index3 = T::regs().vwevms(1).read().bits();
                    let pltrst = index3 & (1 << 1) == 0;
                    let pltrst_valid = index3 & (1 << 5) != 0;

                    // Peripheral channel is somewhat quirky. We can only enable
                    // it after PLTRST# asserted.
                    if me.pltrst_received && pltrst && pltrst_valid && supported {
                        critical_section::with(|_| T::regs().espicfg().modify(|_, w| w.pchanen().set_bit()));
                    }

                    Poll::Ready(Ok(Event::VirtualWireUpdated))
                } else if status.espirst().bit_is_set() {
                    let level = T::regs().espists().read().espirst_lvl().bit();
                    me.pltrst_received = false;
                    Poll::Ready(Ok(Event::EspiReset(level)))
                } else if status.amerr().bit_is_set() {
                    Poll::Ready(Err(Error::AutomaticModeTransferError))
                } else if status.amdone().bit_is_set() {
                    Poll::Ready(Ok(Event::AutomaticModeTransferDone))
                } else if status.flnprqs().bit_is_set() {
                    Poll::Ready(Ok(Event::FlashNonPostedRequestSent))
                } else if status.bmtxdone().bit_is_set() {
                    Poll::Ready(Ok(Event::PeripheralBusMasterDataTransmitted))
                } else if status.pbmrx().bit_is_set() {
                    Poll::Ready(Ok(Event::PeripheralBusMasterDataReceived))
                } else if status.pmsgrx().bit_is_set() {
                    Poll::Ready(Ok(Event::PeripheralMessageDataReceived))
                } else if status.bmbursterr().bit_is_set() {
                    Poll::Ready(Err(Error::BusMasterBurstModeReadTransferError))
                } else if status.bmburstdone().bit_is_set() {
                    Poll::Ready(Ok(Event::BusMasterBurstModeReadTransferDone))
                } else if status.flprterr().bit_is_set() {
                    Poll::Ready(Err(Error::FlashProtectionError))
                } else if status.flautordstr().bit_is_set() {
                    Poll::Ready(Ok(Event::FlashAutomaticReadRequestStart))
                } else if status.flautordpnd().bit_is_set() {
                    Poll::Ready(Ok(Event::FlashAutomaticReadRequestPending))
                } else if status.flautordqemp().bit_is_set() {
                    Poll::Ready(Ok(Event::FlashAutomaticReadQueueEmpty))
                } else if status.auto_rd_dis_sts().bit_is_set() {
                    Poll::Ready(Ok(Event::AutomaticReadDisableStatus))
                } else if status.bmwbursterr().bit_is_set() {
                    Poll::Ready(Err(Error::BusMasterBurstModeWriteTransferError))
                } else if status.bmwburstdone().bit_is_set() {
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

    /// Enable flash access channel
    pub fn enable_flash_access_channel(&mut self) {
        critical_section::with(|_| T::regs().espicfg().modify(|_, w| w.flashchanen().set_bit()));
    }

    /// Disable flash access channel
    pub fn disable_flash_access_channel(&mut self) {
        critical_section::with(|_| T::regs().espicfg().modify(|_, w| w.flashchanen().clear_bit()));
    }

    /// Enable OOB channel
    pub fn enable_oob_channel(&mut self) {
        critical_section::with(|_| T::regs().espicfg().modify(|_, w| w.oobchanen().set_bit()));
    }

    /// Disable OOB channel
    pub fn disable_oob_channel(&mut self) {
        critical_section::with(|_| T::regs().espicfg().modify(|_, w| w.oobchanen().clear_bit()));
    }

    /// Enable VWire channel
    pub fn enable_vwire_channel(&mut self) {
        critical_section::with(|_| T::regs().espicfg().modify(|_, w| w.vwchanen().set_bit()));
    }

    /// Disable VWire channel
    pub fn disable_vwire_channel(&mut self) {
        critical_section::with(|_| T::regs().espicfg().modify(|_, w| w.vwchanen().clear_bit()));
    }

    /// Enable peripheral channel
    pub fn enable_peripheral_channel(&mut self) {
        // critical_section::with(|_| T::regs().espicfg().modify(|_, w| w.pchanen().set_bit()));
    }

    /// Disable peripheral channel
    pub fn disable_peripheral_channel(&mut self) {
        // critical_section::with(|_| T::regs().espicfg().modify(|_, w| w.pchanen().clear_bit()));
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

    pub trait SealedValidEspiConfig {
        unsafe fn setup_pins(cs: critical_section::CriticalSection);
        unsafe fn setup_pullup(cs: critical_section::CriticalSection);
    }
}

/// A marker trait implemented for valid eSPI configs
pub trait ValidEspiConfig: sealed::SealedValidEspiConfig {}

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

macro_rules! impl_config {
    ($peri:ident, $io0:ident, $io1:ident, $io2:ident, $io3:ident, $cs:ident, $rst:ident, $clk:ident, $alert:ident, $pin_config:expr, $pullup_config:expr) => {
        impl sealed::SealedValidEspiConfig
            for (
                crate::peripherals::$peri,
                crate::peripherals::$io0,
                crate::peripherals::$io1,
                crate::peripherals::$io2,
                crate::peripherals::$io3,
                crate::peripherals::$cs,
                crate::peripherals::$rst,
                crate::peripherals::$clk,
                crate::peripherals::$alert,
            )
        {
            unsafe fn setup_pins(_cs: critical_section::CriticalSection) {
                fn internal_set(f: impl FnOnce(&'static crate::pac::sysconfig::RegisterBlock)) {
                    f(unsafe { &*crate::pac::Sysconfig::ptr() });
                }

                internal_set($pin_config);
            }

            unsafe fn setup_pullup(_cs: critical_section::CriticalSection) {
                fn internal_set(f: impl FnOnce(&'static crate::pac::gpio0::RegisterBlock, &'static crate::pac::gpio0::RegisterBlock)) {
                    f(unsafe { &*crate::pac::Gpio4::ptr() }, unsafe { &* crate::pac::Gpio5::ptr()});
                }

                internal_set($pullup_config);
            }
        }

        impl ValidEspiConfig
            for (
                crate::peripherals::$peri,
                crate::peripherals::$io0,
                crate::peripherals::$io1,
                crate::peripherals::$io2,
                crate::peripherals::$io3,
                crate::peripherals::$cs,
                crate::peripherals::$rst,
                crate::peripherals::$clk,
                crate::peripherals::$alert,
            )
        {
        }
    };
}

impl_instance!(ESPI, ESPI_SHI);

impl_config!(
    ESPI,
    PH01,
    PJ01,
    PK01,
    PL01,
    PL02,
    PK03,
    PM01,
    PL03,
    |config| {
        config.devalt1().modify(|_, w| w.no_lpc_espi().clear_bit());
        config.devaltc().modify(|_, w| w.shi_sl().clear_bit());
    },
    |gpio4, gpio5| {
        gpio4.px_pud().modify(|_, w| w.pin6().pull_up());
        gpio4.px_pull().modify(|_, w| w.pin6().enabled());

        gpio5
            .px_dir()
            .modify(|_, w| w.pin3().input().pin4().input().pin5().input().pin7().output());
        gpio5
            .px_pud()
            .modify(|_, w| w.pin3().pull_up().pin4().pull_up().pin5().pull_down().pin7().pull_up());
        gpio5
            .px_pull()
            .modify(|_, w| w.pin3().enabled().pin4().enabled().pin5().enabled().pin7().enabled());
    }
);
