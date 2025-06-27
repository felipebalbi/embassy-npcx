//! eSPI driver.

use core::future::poll_fn;
use core::marker::PhantomData;
use core::task::Poll;

use embassy_espi_driver::{Cycle, Driver, oob, vwire};
use embassy_hal_internal::{Peri, PeripheralType};
use embassy_sync::waitqueue::AtomicWaker;

use crate::interrupt::typelevel::Interrupt;
use crate::pac::espi::espiie::EspiieSpec;
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

    /// Peripheral channel configuration
    pub peripheral_config: Option<PeripheralConfig>,

    /// OOB channel configuration
    pub oob_config: Option<OobConfig>,

    /// VWire channel configuration
    pub vwire_config: Option<VWireConfig>,

    /// Flash access channel configuration
    pub flash_config: Option<FlashConfig>,
}

/// Peripheral channel configuration.
#[derive(Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PeripheralConfig {}

/// OOB channel configuration.
#[derive(Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct OobConfig {
    /// Maximum OOB payload size.
    pub max_payload_size: OobPayload,
}

/// OOB payload size
#[derive(Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum OobPayload {
    /// 64 bytes,
    _64,
    /// 128 bytes,
    _128,
    /// 256 bytes,
    _256,
}

impl TryFrom<u8> for OobPayload {
    type Error = Error;

    fn try_from(value: u8) -> Result<OobPayload, Error> {
        match value {
            0 => Ok(OobPayload::_64),
            1 => Ok(OobPayload::_128),
            3 => Ok(OobPayload::_256),
            _ => Err(Error::Other),
        }
    }
}

impl From<OobPayload> for u8 {
    fn from(value: OobPayload) -> u8 {
        match value {
            OobPayload::_64 => 0,
            OobPayload::_128 => 1,
            OobPayload::_256 => 3,
        }
    }
}

/// VWire channel configuration.
#[derive(Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct VWireConfig {}

/// Flash access channel configuration.
#[derive(Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FlashConfig {}

impl Default for Config {
    fn default() -> Self {
        Self {
            alert_mode: AlertMode::IO1,
            io_mode: IoMode::Single,
            frequency: Frequency::_20MHz,
            wait_state: 1,
            peripheral_config: None,
            oob_config: None,
            vwire_config: None,
            flash_config: None,
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
                    .variant(config.flash_config.is_some())
                    .oobchn_supp()
                    .variant(config.oob_config.is_some())
                    .vwchn_supp()
                    .variant(config.vwire_config.is_some())
                    .pcchn_supp()
                    .variant(config.peripheral_config.is_some())
                    .maxfreq()
                    .bits(config.frequency.into())
                    .iomode()
                    .bits(config.io_mode.into())
            });

            T::regs().oobctl().modify(|_, w| unsafe {
                w.oobplsize().bits(
                    config
                        .oob_config
                        .unwrap_or(OobConfig {
                            max_payload_size: OobPayload::_64,
                        })
                        .max_payload_size
                        .into(),
                )
            });

            // Let user tell us which vwire indices to enable.
            if config.vwire_config.is_some() {
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

    /// Enable flash access channel
    fn enable_flash_access_channel(&mut self) {
        critical_section::with(|_| T::regs().espicfg().modify(|_, w| w.flashchanen().set_bit()));
    }

    /// Disable flash access channel
    fn disable_flash_access_channel(&mut self) {
        critical_section::with(|_| T::regs().espicfg().modify(|_, w| w.flashchanen().clear_bit()));
    }

    /// Enable OOB channel
    fn enable_oob_channel(&mut self) {
        critical_section::with(|_| T::regs().espicfg().modify(|_, w| w.oobchanen().set_bit()));
    }

    /// Disable OOB channel
    fn disable_oob_channel(&mut self) {
        critical_section::with(|_| T::regs().espicfg().modify(|_, w| w.oobchanen().clear_bit()));
    }

    /// Enable VWire channel
    fn enable_vwire_channel(&mut self) {
        critical_section::with(|_| T::regs().espicfg().modify(|_, w| w.vwchanen().set_bit()));
    }

    /// Disable VWire channel
    fn disable_vwire_channel(&mut self) {
        critical_section::with(|_| T::regs().espicfg().modify(|_, w| w.vwchanen().clear_bit()));
    }

    /// Enable peripheral channel
    fn enable_peripheral_channel(&mut self) {
        critical_section::with(|_| T::regs().espicfg().modify(|_, w| w.pchanen().set_bit()));
    }

    /// Disable peripheral channel
    fn disable_peripheral_channel(&mut self) {
        critical_section::with(|_| T::regs().espicfg().modify(|_, w| w.pchanen().clear_bit()));
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

impl<'p, T: Instance> Driver<'p> for Espi<'p, T> {
    async fn listen(&mut self) -> Result<embassy_espi_driver::Event, embassy_espi_driver::EspiError> {
        self.wait_for(
            |me| {
                let status = T::regs().espists().read();

                // Clear all events
                critical_section::with(|_| {
                    T::regs()
                        .espists()
                        .modify(|_, w| unsafe { w.bits(EspistsSpec::ONE_TO_MODIFY_FIELDS_BITMAP) })
                });

                if status.ibrst().bit_is_set() {
                    me.pltrst_received = false;
                    Poll::Ready(Ok(embassy_espi_driver::Event::Reset))
                } else if status.cfgupd().bit_is_set() {
                    let cfg = T::regs().espicfg().read();

                    if cfg.hflashchanen().bit_is_set() {
                        me.enable_flash_access_channel();
                    } else {
                        me.disable_flash_access_channel();
                    }

                    if cfg.hoobchanen().bit_is_set() {
                        me.enable_oob_channel();
                    } else {
                        me.disable_oob_channel();
                    }

                    if cfg.hvwchanen().bit_is_set() {
                        me.enable_vwire_channel();
                    } else {
                        me.disable_vwire_channel();
                    }

                    Poll::Pending
                } else if status.vwupd().bit_is_set() {
                    let supported = T::regs().espicfg().read().pcchn_supp().bit();
                    let index3 = T::regs().vwevms(1).read().bits();
                    let pltrst = index3 & (1 << 1) == 0;
                    let pltrst_valid = index3 & (1 << 5) != 0;

                    // Peripheral channel is somewhat quirky. We can only enable
                    // it after PLTRST# asserted.
                    if me.pltrst_received && pltrst && pltrst_valid && supported {
                        me.enable_peripheral_channel();
                    } else {
                        me.disable_peripheral_channel();
                    }

                    Poll::Ready(Ok(embassy_espi_driver::Event::VWire))
                } else if status.espirst().bit_is_set() {
                    me.pltrst_received = false;
                    Poll::Ready(Ok(embassy_espi_driver::Event::Reset))
                } else if status.oobrx().bit_is_set() {
                    Poll::Ready(Ok(embassy_espi_driver::Event::Oob))
                } else {
                    Poll::Pending
                }
            },
            |_| {
                // Enable all interrupts
                T::regs().espiie().write(|w| {
                    w.pltrstie()
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
}

impl<'p, T: Instance> vwire::VWireChannel for Espi<'p, T> {
    fn read_vwire<VWIRE: vwire::Readable>(
        &mut self,
        vwire: VWIRE,
    ) -> Result<(bool, bool), embassy_espi_driver::EspiError> {
        let index = match vwire.index() {
            2 => 0,
            3 => 1,
            7 => 2,
            _ => return Err(embassy_espi_driver::EspiError::UnsupportedVWire),
        };

        let bit = T::regs().vwevms(index).read().wire3_0().bits() & (1 << vwire.bit()) != 0;
        let valid = T::regs().vwevms(index).read().wire3_0valid().bits() & (1 << vwire.bit()) != 0;

        Ok((bit, valid))
    }

    fn write_vwire<VWIRE: vwire::Writeable>(
        &mut self,
        vwire: VWIRE,
        value: bool,
    ) -> Result<(), embassy_espi_driver::EspiError> {
        let bit = vwire.bit();
        let index = match vwire.index() {
            4 => 0,
            5 => 1,
            6 => 2,
            _ => return Err(embassy_espi_driver::EspiError::UnsupportedVWire),
        };

        T::regs().vwevsm(index).modify(|r, w| {
            let mut wires = r.wire3_0().bits();
            let mut valid = r.wire3_0valid().bits();

            if value {
                wires |= 1 << bit;
            } else {
                wires &= !(1 << bit);
            }

            valid |= 1 << bit;

            unsafe { w.wire3_0().bits(wires).wire3_0valid().bits(valid) }
        });

        Ok(())
    }
}

impl<'p, T: Instance> oob::OobChannel for Espi<'p, T> {
    async fn oob_receive(&mut self, buf: &mut [u8]) -> Result<usize, embassy_espi_driver::EspiError> {
        // Get the header
        let header = T::regs().oobrxbuf(0).read().bits();
        // Extract transfer size.
        //
        // REVISIT: should define a structure for this. Probably as part of embassy_espi_driver.
        let size = (header & 0xff00_0000) >> 24 | (header & 0x000f_0000) >> 8;

        let max_payload_size = match T::regs().oobctl().read().oobplsize().bits().try_into().unwrap() {
            OobPayload::_64 => 64,
            OobPayload::_128 => 128,
            OobPayload::_256 => 256,
        };

        if buf.len() < size as usize || buf.len() > max_payload_size {
            Err(embassy_espi_driver::EspiError::DataSize(size))
        } else {
            let aligned = (size / 4) as usize;
            let remaining = (size % 4) as usize;

            for (i, oobrx) in T::regs().oobrxbuf_iter().skip(1).take(aligned).enumerate() {
                let data = oobrx.read().bits().to_ne_bytes();
                buf[(i * 4)..(i * 4 + 4)].copy_from_slice(&data);
            }

            if remaining > 0 {
                let data = T::regs().oobrxbuf(aligned + 1).read().bits().to_ne_bytes();

                for i in 0..remaining {
                    buf[aligned * 4 + i] = data[i];
                }
            }

            critical_section::with(|_| T::regs().oobctl().modify(|_, w| w.oob_free().set_bit()));

            Ok(size as usize)
        }
    }

    async fn oob_send(&mut self, buf: &[u8]) -> Result<usize, embassy_espi_driver::EspiError> {
        let max_payload_size = match T::regs().oobctl().read().oobplsize().bits().try_into().unwrap() {
            OobPayload::_64 => 64,
            OobPayload::_128 => 128,
            OobPayload::_256 => 256,
        };

        if T::regs().oobctl().read().oob_avail().bit_is_set() {
            Err(embassy_espi_driver::EspiError::BufferFull)
        } else if buf.len() > max_payload_size {
            Err(embassy_espi_driver::EspiError::DataSize(buf.len() as u32))
        } else {
            // # Safety: slice is valid and we can reinterpret &[u8] as &[u32].
            let (_, aligned, suffix) = unsafe { buf.align_to::<u32>() };

            for (i, reg) in T::regs().oobtxbuf_iter().skip(1).take(aligned.len()).enumerate() {
                reg.write(|w| unsafe { w.bits(aligned[i]) });
            }

            let mut remaining = 0_u32;

            for byte in suffix {
                remaining |= u32::from(*byte);
                remaining <<= 8;
            }

            T::regs()
                .oobtxbuf(aligned.len() + 1)
                .write(|w| unsafe { w.bits(remaining) });

            // REVISIT: add a struct for the header.
            let pktlen = buf.len() as u32 + 3;
            let cycle = u32::from(Cycle::OOB.into_byte());
            let tag = 0;
            let header = pktlen | cycle << 8 | tag << 16 | (buf.len() as u32) << 24;
            T::regs().oobtxbuf(0).write(|w| unsafe { w.bits(header) });

            critical_section::with(|_| T::regs().oobctl().modify(|_, w| w.oob_avail().set_bit()));

            Ok(buf.len())
        }
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
            //
            // # Safety: There is an assumption that the PAC properly
            // marks W1C bits. As long as that invariant is held, the
            // following is safe.
            T::regs()
                .espiie()
                .write(|w| unsafe { w.bits(EspiieSpec::ONE_TO_MODIFY_FIELDS_BITMAP) });

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
