//! Implementation of the I2C peripheral

use core::future::Future;
use core::marker::PhantomData;
use core::sync::atomic::{AtomicU32, Ordering};

use embassy_hal_internal::{impl_peripheral, Peri, PeripheralType};
use embassy_sync::waitqueue::AtomicWaker;
pub use embedded_hal_async::i2c::{I2c, Operation};
use embedded_hal_i2c::{
    AnyAddress, AsyncI2cTarget, AsyncReadTransaction, AsyncWriteTransaction, ReadResult, Transaction, WriteResult,
};

use crate::cdcg::get_clocks;
use crate::gpio::Pin;
use crate::interrupt::typelevel::Interrupt;

// Size of the peripherals fifo
const FIFO_SIZE: u8 = 32;
// Number of bytes between refilling/emptying of the fifo
const GROUP_SIZE: u8 = 24;

/// SMBUS Broadcast address
pub const GENERALCALL_ADDRESS: u8 = 0;
/// SMBUS ARP address
pub const ARP_ADDRESS: u8 = 0b1100001;

// Interrupt remediations
const REMEDIATION_NONE: u32 = 0;
const REMEDIATION_SLAVE_READ: u32 = 1;
const REMEDIATION_SLAVE_WRITE: u32 = 2;

/// The interrupt handler for the [I2CController]
pub struct InterruptHandler<T> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> crate::interrupt::typelevel::Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        T::waker().wake();
        critical_section::with(|_| match T::remediation().load(Ordering::Acquire) {
            REMEDIATION_SLAVE_READ => {
                let st = T::regs().smbn_st().read();
                if st.ber().bit_is_set() || st.negack().bit_is_set() {
                    T::regs().smbn_fif_cts().write(|w| w.clr_fifo().set_bit());
                    T::regs().smbn_st().write(|w| w.negack().set_bit().ber().set_bit());
                    T::regs().smbn_ctl1().modify(|_, w| w.inten().clear_bit());
                    T::remediation().store(REMEDIATION_NONE, Ordering::Release);
                } else {
                    for _ in 0..GROUP_SIZE {
                        T::regs().smbn_sda().write(|w| unsafe { w.bits(0xFF) });
                    }
                    T::regs().smbn_txf_sts().write(|w| w.tx_thst().set_bit());
                }
            }
            REMEDIATION_SLAVE_WRITE => {
                let st = T::regs().smbn_st().read();
                let fif_cts = T::regs().smbn_fif_cts().read();
                if st.ber().bit_is_set() || st.slvstp().bit_is_set() || fif_cts.slvrstr().bit_is_set() {
                    T::regs().smbn_fif_cts().write(|w| w.clr_fifo().set_bit());
                    T::regs().smbn_st().write(|w| w.ber().set_bit().slvstp().set_bit());
                    T::regs().smbn_ctl1().modify(|_, w| w.inten().clear_bit());
                    T::remediation().store(REMEDIATION_NONE, Ordering::Release);
                } else {
                    for _ in 0..GROUP_SIZE {
                        let _ = T::regs().smbn_sda().read();
                    }
                    T::regs().smbn_rxf_sts().write(|w| w.rx_thst().set_bit());
                }
            }
            _ => {
                T::regs().smbn_ctl1().modify(|_, w| w.inten().clear_bit());
            }
        });
    }
}

struct WaitForCondition<F> {
    regs: &'static crate::pac::smb0::RegisterBlock,
    waker: &'static AtomicWaker,
    f: F,
}

impl<O, F: Fn() -> Option<O>> Future for WaitForCondition<F> {
    type Output = O;

    fn poll(self: core::pin::Pin<&mut Self>, cx: &mut core::task::Context<'_>) -> core::task::Poll<Self::Output> {
        if let Some(out) = (self.f)() {
            core::task::Poll::Ready(out)
        } else {
            self.waker.register(cx.waker());
            critical_section::with(|_| {
                self.regs.smbn_ctl1().modify(|_, w| w.inten().set_bit());
            });
            core::task::Poll::Pending
        }
    }
}

struct ReadCompletion<'a> {
    regs: &'static crate::pac::smb0::RegisterBlock,
    group: &'a mut [u8],
}

impl ReadCompletion<'_> {
    fn complete(self) {
        if self.group.is_empty() {
            self.regs.smbn_st().write(|w| w.stastr().set_bit());
            self.regs.smbn_ctl1().modify(|_, w| w.stastre().clear_bit());
        } else {
            for b in self.group {
                *b = self.regs.smbn_sda().read().bits();
            }
        }
    }
}

/// The speed of the I2C bus
#[derive(Default, Debug, Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Speed {
    /// 100 kbit/s
    #[default]
    Standard,
    /// 400 kbit/s
    Fast,
    /// 1 Mbit/s
    FastPlus,
}

/// Configuration for the I2C bus
#[derive(Default, Debug, Copy, Clone, Eq, PartialEq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Config {
    /// The target clock speed of the bus
    pub speed: Speed,
    /// If true, the internal pullups are enabled
    pub pullup: bool,
}

/// Error type for the I2C operations
#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// A transaction couldn't be done because the bus was already busy and we had to give up
    LostArbitration,
    /// Something went wrong on the bus
    BusError,
    /// The address phase of the transaction nacked
    AddressNack,
    /// The data phase of the transaction nacked
    DataNack,
}

impl embedded_hal_async::i2c::Error for Error {
    fn kind(&self) -> embedded_hal::i2c::ErrorKind {
        match self {
            Error::LostArbitration => embedded_hal::i2c::ErrorKind::ArbitrationLoss,
            Error::BusError => embedded_hal::i2c::ErrorKind::Bus,
            Error::DataNack => {
                embedded_hal::i2c::ErrorKind::NoAcknowledge(embedded_hal::i2c::NoAcknowledgeSource::Data)
            }
            Error::AddressNack => {
                embedded_hal::i2c::ErrorKind::NoAcknowledge(embedded_hal::i2c::NoAcknowledgeSource::Address)
            }
        }
    }
}

/// Error type for [I2CController::listen]
#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ListenError {
    /// The given list of addresses has something wrong with it
    InvalidAddressList,
    /// Something went wrong on the bus.
    BusError,
}

/// Commands the user has to handle in the listen code
#[derive(Debug, Eq, PartialEq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ListenCommand<'a> {
    /// Part of a master's write has been received
    PartialWrite(&'a [u8]),
    /// The master's write has been completed.
    WriteFinished,
    /// Prepare a buffer for a read operation. Note, not
    /// all bytes in the buffer may be read. Actions
    /// that should happen once bytes have been read should
    /// be done in ReadFinished.
    PrepareRead(&'a mut [u8]),
    /// A read has been finished. Reports how many bytes were
    /// read in total as part of the read.
    ReadFinished(usize),
    /// A bus error occured. No action is needed, but gives
    /// the handler a chance to report this. Always occurs with
    /// address 0. The corresponding finished command for the
    /// transaction will still be provided after the bus error.
    BusError,
}

struct AnySMB {}

impl<T: Instance> From<T> for AnySMB {
    fn from(_smb: T) -> Self {
        AnySMB {}
    }
}

// Allow use of Peri to do lifetime management
impl_peripheral!(AnySMB);

/// An instance of the I2C driver
///
/// Note, although this has target support, it doesn't support sending negative acknowledgements as a target.
pub struct I2CController<'a> {
    _dev: Peri<'a, AnySMB>,
    regs: &'static crate::pac::smb0::RegisterBlock,
    waker: &'static AtomicWaker,
    remediation: &'static AtomicU32,
}

trait IteratorExt: ExactSizeIterator + Sized {
    fn mark_last(mut self) -> impl Iterator<Item = (Self::Item, bool)> {
        core::iter::from_fn(move || self.next().map(|v| (v, self.len() == 0)))
    }
}

impl<T: ExactSizeIterator + Sized> IteratorExt for T {}

impl<'p> I2CController<'p> {
    fn wait_for<O, F: Fn() -> Option<O>>(&mut self, f: F) -> impl Future<Output = O> {
        WaitForCondition {
            f,
            regs: self.regs,
            waker: self.waker,
        }
    }

    async fn wait_read(&mut self) -> Result<(), Error> {
        let r = self
            .wait_for(|| {
                let r = self.regs.smbn_st().read();
                let f = self.regs.smbn_rxf_sts().read();
                if r.ber().bit_is_set() || r.sdast().bit_is_set() || f.rx_thst().bit_is_set() {
                    Some(r)
                } else {
                    None
                }
            })
            .await;
        if r.ber().bit_is_set() {
            return self.handle_ber();
        }
        Ok(())
    }

    // Select register bank
    fn bank_sel(&mut self, bnk: bool) {
        self.regs
            .smbn_ctl3()
            .modify(|_, w| w.scl_lvl().set_bit().sda_lvl().set_bit().bnk_sel().bit(bnk));
    }

    /// Initialize speed settings
    ///
    /// Should be called only with bank 0 selected and peripheral disabled
    fn speed_init(&mut self, speed: Speed, clk: u32) {
        struct StandardMode {
            sclfrq: u16,
            hldt: u8,
        }

        struct FastMode {
            scllt: u8,
            sclht: u8,
            hldt: u8,
        }

        const STANDARDMODE: [(u32, StandardMode); 7] = [
            (15_000_000, StandardMode { sclfrq: 38, hldt: 15 }),
            (25_000_000, StandardMode { sclfrq: 63, hldt: 15 }),
            (30_000_000, StandardMode { sclfrq: 77, hldt: 17 }),
            (40_000_000, StandardMode { sclfrq: 101, hldt: 17 }),
            (48_000_000, StandardMode { sclfrq: 121, hldt: 17 }),
            (50_000_000, StandardMode { sclfrq: 126, hldt: 17 }),
            (60_000_000, StandardMode { sclfrq: 152, hldt: 17 }),
        ];

        const FASTMODE: [(u32, FastMode); 8] = [
            (
                15_000_000,
                FastMode {
                    scllt: 12,
                    sclht: 9,
                    hldt: 7,
                },
            ),
            (
                20_000_000,
                FastMode {
                    scllt: 16,
                    sclht: 11,
                    hldt: 7,
                },
            ),
            (
                24_000_000,
                FastMode {
                    scllt: 20,
                    sclht: 13,
                    hldt: 8,
                },
            ),
            (
                30_000_000,
                FastMode {
                    scllt: 24,
                    sclht: 16,
                    hldt: 10,
                },
            ),
            (
                40_000_000,
                FastMode {
                    scllt: 32,
                    sclht: 21,
                    hldt: 13,
                },
            ),
            (
                48_000_000,
                FastMode {
                    scllt: 39,
                    sclht: 25,
                    hldt: 16,
                },
            ),
            (
                50_000_000,
                FastMode {
                    scllt: 40,
                    sclht: 26,
                    hldt: 17,
                },
            ),
            (
                60_000_000,
                FastMode {
                    scllt: 48,
                    sclht: 31,
                    hldt: 20,
                },
            ),
        ];

        const FASTMODEPLUS: [(u32, FastMode); 7] = [
            (
                15_000_000,
                FastMode {
                    scllt: 7,
                    sclht: 5,
                    hldt: 7,
                },
            ),
            (
                24_000_000,
                FastMode {
                    scllt: 8,
                    sclht: 5,
                    hldt: 7,
                },
            ),
            (
                30_000_000,
                FastMode {
                    scllt: 10,
                    sclht: 7,
                    hldt: 7,
                },
            ),
            (
                40_000_000,
                FastMode {
                    scllt: 13,
                    sclht: 10,
                    hldt: 7,
                },
            ),
            (
                48_000_000,
                FastMode {
                    scllt: 15,
                    sclht: 11,
                    hldt: 7,
                },
            ),
            (
                50_000_000,
                FastMode {
                    scllt: 16,
                    sclht: 11,
                    hldt: 8,
                },
            ),
            (
                60_000_000,
                FastMode {
                    scllt: 19,
                    sclht: 13,
                    hldt: 9,
                },
            ),
        ];

        match speed {
            Speed::Standard => {
                let mut i = 0;
                while STANDARDMODE[i].0 < clk {
                    i += 1;
                }

                self.regs.smbn_ctl3().modify(|_, w| unsafe {
                    w._400k_mode()
                        .clear_bit()
                        .sclfrq8_7()
                        .bits(((STANDARDMODE[i].1.sclfrq & 0x180) >> 7) as u8)
                });
                self.regs
                    .smbn_ctl4()
                    .modify(|_, w| unsafe { w.hldt().bits(STANDARDMODE[i].1.hldt) });
                self.regs
                    .smbn_ctl2()
                    .modify(|_, w| unsafe { w.sclfrq6_0().bits((STANDARDMODE[i].1.sclfrq & 0x7F) as u8) });
            }
            Speed::Fast => {
                let mut i = 0;
                while FASTMODE[i].0 < clk {
                    i += 1;
                }

                self.regs
                    .smbn_ctl3()
                    .modify(|_, w| unsafe { w._400k_mode().set_bit().sclfrq8_7().bits(0) });
                self.regs
                    .smbn_ctl4()
                    .modify(|_, w| unsafe { w.hldt().bits(FASTMODE[i].1.hldt) });
                self.regs.smbn_ctl2().modify(|_, w| unsafe { w.sclfrq6_0().bits(0) });
                self.regs.smbn_scllt().write(|w| unsafe { w.bits(FASTMODE[i].1.scllt) });
                self.regs.smbn_sclht().write(|w| unsafe { w.bits(FASTMODE[i].1.sclht) });
            }
            Speed::FastPlus => {
                let mut i = 0;
                while FASTMODEPLUS[i].0 < clk {
                    i += 1;
                }

                self.regs
                    .smbn_ctl3()
                    .modify(|_, w| unsafe { w._400k_mode().set_bit().sclfrq8_7().bits(0) });
                self.regs
                    .smbn_ctl4()
                    .modify(|_, w| unsafe { w.hldt().bits(FASTMODEPLUS[i].1.hldt) });
                self.regs.smbn_ctl2().modify(|_, w| unsafe { w.sclfrq6_0().bits(0) });
                self.regs
                    .smbn_scllt()
                    .write(|w| unsafe { w.bits(FASTMODEPLUS[i].1.scllt) });
                self.regs
                    .smbn_sclht()
                    .write(|w| unsafe { w.bits(FASTMODEPLUS[i].1.sclht) });
            }
        }
    }

    /// Create a new instance of I2C
    pub fn new<T: Instance + 'p, Scl: Pin, Sda: Pin, Mode>(
        peri: Peri<'p, T>,
        scl: Peri<'p, Scl>,
        sda: Peri<'p, Sda>,
        _irqs: impl crate::interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>>,
        _mode: Mode,
        config: Config,
    ) -> Self
    where
        (T, Scl, Sda, Mode): ValidI2CConfig,
    {
        // Safety: _irqs ensures an interrupt handler is bound
        unsafe {
            T::Interrupt::enable();
        }

        critical_section::with(|cs| {
            // Safety: We are disabling low voltage mode and exclusively own the peripherals
            unsafe {
                scl.set_low_voltage(cs, false);
            }
            unsafe {
                sda.set_low_voltage(cs, false);
            }
            // Safety: We have exclusive ownership over the peripherals.
            unsafe { <(T, Scl, Sda, Mode) as sealed::SealedValidI2CConfig>::setup_pins(cs) };
            unsafe { <(T, Scl, Sda, Mode) as sealed::SealedValidI2CConfig>::setup_pullup(cs, config.pullup) };
        });

        let mut dev = Self {
            _dev: peri.into(),
            regs: T::regs(),
            waker: T::waker(),
            remediation: T::remediation(),
        };

        dev.remediation.store(REMEDIATION_NONE, Ordering::Release);

        dev.regs.smbn_ctl3().write(|w| {
            w.scl_lvl()
                .set_bit()
                .sda_lvl()
                .set_bit()
                .bnk_sel()
                .clear_bit()
                .slp_start()
                .clear_bit()
                .arpmen()
                .clear_bit()
        });
        dev.regs.smbn_ctl4().modify(|_, w| w.lvl_we().clear_bit());
        // Safety: We have the peripheral, so init was called.
        dev.speed_init(config.speed, unsafe { T::clockfreq() });
        dev.regs.smbn_fif_ctl().modify(|_, w| w.fifo_en().set_bit());
        dev.regs.smbn_ctl2().modify(|_, w| w.enable().set_bit());
        dev.bank_sel(true);

        dev
    }

    fn handle_ber<T>(&mut self) -> Result<T, Error> {
        // This should be enough for arbitration errors. However, the documentation is somewhat unclear on more
        // serious problems.
        self.regs.smbn_fif_cts().write(|w| w.clr_fifo().set_bit());
        self.regs.smbn_st().write(|w| w.ber().set_bit());

        Err(Error::BusError)
    }

    fn handle_negack(&mut self) {
        self.regs.smbn_fif_cts().write(|w| w.clr_fifo().set_bit());
        self.regs.smbn_ctl1().modify(|_, w| w.stop().set_bit());
        self.regs.smbn_st().write(|w| w.negack().set_bit());
    }

    async fn bulk_write(&mut self, data: &[u8]) -> Result<(), Error> {
        // Set up transmission of first 32 bytes
        let mut data = data.iter().copied();
        for b in (&mut data).take((FIFO_SIZE).into()) {
            self.regs.smbn_sda().write(|w| unsafe { w.bits(b) });
        }
        self.regs.smbn_txf_sts().modify(|_, w| w.tx_thst().set_bit());

        while data.len() > 0 {
            // Wait for 8 bytes remaining in fifo
            self.regs
                .smbn_txf_ctl()
                .modify(|_, w| unsafe { w.thr_txie().set_bit().tx_thr().bits(FIFO_SIZE - GROUP_SIZE) });
            let r = self
                .wait_for(|| {
                    let r = self.regs.smbn_st().read();
                    let f = self.regs.smbn_txf_sts().read();
                    if r.ber().bit_is_set()
                        || r.negack().bit_is_set()
                        || r.sdast().bit_is_set()
                        || f.tx_thst().bit_is_set()
                    {
                        Some(r)
                    } else {
                        None
                    }
                })
                .await;
            if r.ber().bit_is_set() {
                return self.handle_ber();
            }
            if r.negack().bit_is_set() {
                self.handle_negack();
                return Err(Error::DataNack);
            }

            // Add new data
            for b in (&mut data).take(GROUP_SIZE.into()) {
                self.regs.smbn_sda().write(|w| unsafe { w.bits(b) });
            }

            // Reset threshold trigger
            self.regs.smbn_txf_sts().write(|w| w.tx_thst().set_bit());
        }

        // Wait for completion
        self.regs
            .smbn_txf_ctl()
            .modify(|_, w| unsafe { w.thr_txie().clear_bit().tx_thr().bits(0) });
        let r = self
            .wait_for(|| {
                let r = self.regs.smbn_st().read();
                if r.ber().bit_is_set() || r.negack().bit_is_set() || r.sdast().bit_is_set() {
                    Some(r)
                } else {
                    None
                }
            })
            .await;
        if r.ber().bit_is_set() {
            return self.handle_ber();
        }
        if r.negack().bit_is_set() {
            self.handle_negack();
            return Err(Error::DataNack);
        }

        Ok(())
    }

    // Should not be called with zero-length data
    async fn bulk_read<'a>(
        &mut self,
        mut data: &'a mut [u8],
        negack_last: bool,
        start: impl '_ + FnOnce(),
    ) -> Result<ReadCompletion<'a>, Error> {
        debug_assert_ne!(data.len(), 0);
        let mut group;

        // Setup transfer
        if data.len() <= FIFO_SIZE.into() {
            group = data;
            data = &mut [];
            // Add nack at end if required
            self.regs.smbn_rxf_ctl().write(|w| unsafe {
                w.last_pec()
                    .bit(negack_last)
                    .thr_rxie()
                    .clear_bit()
                    .rx_thr()
                    .bits(group.len() as u8)
            });
        } else if data.len() <= (2 * FIFO_SIZE).into() {
            (group, data) = data.split_at_mut(FIFO_SIZE as _);
            // Ensure we stall before the final group
            self.regs
                .smbn_rxf_ctl()
                .write(|w| unsafe { w.last_pec().clear_bit().thr_rxie().clear_bit().rx_thr().bits(FIFO_SIZE) });
        } else {
            (group, data) = data.split_at_mut(GROUP_SIZE as _);
            // Can do bulk transfer without stalling
            self.regs
                .smbn_rxf_ctl()
                .write(|w| unsafe { w.last_pec().clear_bit().thr_rxie().set_bit().rx_thr().bits(GROUP_SIZE) });
        }

        start();

        // Do bulk
        while !data.is_empty() {
            self.wait_read().await?;

            let next_group;

            if data.len() <= FIFO_SIZE.into() {
                next_group = data;
                data = &mut [];

                // The last group is somewhat tricky, as we want to avoid
                // the nack being sent at the wrong time. The approach in the manual
                // isn't really working for us, so we do some different trickery here.

                // We implement two cases here, based on the length of the next group

                if next_group.len() == 1 {
                    // No real risk, just setup the transfer
                    self.regs.smbn_rxf_ctl().write(|w| unsafe {
                        w.last_pec()
                            .bit(negack_last)
                            .thr_rxie()
                            .clear_bit()
                            .rx_thr()
                            .bits(next_group.len() as u8)
                    });
                } else {
                    // Transfer just 1 byte from the next group at first, with no nack yet
                    self.regs
                        .smbn_rxf_ctl()
                        .write(|w| unsafe { w.last_pec().clear_bit().thr_rxie().clear_bit().rx_thr().bits(1) });
                }
            } else if data.len() <= (2 * FIFO_SIZE).into() {
                (next_group, data) = data.split_at_mut(FIFO_SIZE as _);
                // Ensure we stall before the final group
                self.regs
                    .smbn_rxf_ctl()
                    .write(|w| unsafe { w.last_pec().clear_bit().thr_rxie().clear_bit().rx_thr().bits(FIFO_SIZE) });
            } else {
                (next_group, data) = data.split_at_mut(GROUP_SIZE as _);
                // Continue bulk transfer
            }

            // Next group is setup, so we can safely read data
            for b in group {
                *b = self.regs.smbn_sda().read().bits();
            }

            if data.is_empty() {
                // setup the actual last group
                self.regs.smbn_rxf_ctl().write(|w| unsafe {
                    w.last_pec()
                        .bit(negack_last)
                        .thr_rxie()
                        .clear_bit()
                        .rx_thr()
                        .bits(next_group.len() as u8)
                });
            }

            // And clear rx threshold status
            self.regs.smbn_rxf_sts().write(|w| w.rx_thst().set_bit());

            group = next_group;
        }

        // Finish last block
        self.wait_read().await?;

        // Return completion
        Ok(ReadCompletion { regs: self.regs, group })
    }

    /// Do a transaction. This uses the [embedded_hal_async::i2c::I2c::transaction] model.
    pub async fn transaction(&mut self, address: u8, operations: &mut [Operation<'_>]) -> Result<(), Error> {
        enum PrevOpType<'a> {
            None,
            Read(ReadCompletion<'a>),
            Write,
        }

        let mut prevop = PrevOpType::None;

        for (op, last) in operations.iter_mut().mark_last() {
            match op {
                Operation::Write(data) => {
                    if !matches!(prevop, PrevOpType::Write) {
                        // Handle (repeated) start and address bytes
                        self.regs.smbn_ctl1().modify(|_, w| w.start().set_bit());
                        if let PrevOpType::Read(completion) = prevop {
                            completion.complete();
                        }

                        // Wait for completion
                        let r = self
                            .wait_for(|| {
                                let r = self.regs.smbn_st().read();
                                if r.ber().bit_is_set() || r.sdast().bit_is_set() {
                                    Some(r)
                                } else {
                                    None
                                }
                            })
                            .await;
                        if r.ber().bit_is_set() {
                            let _ = self.handle_ber::<()>();
                            return Err(Error::LostArbitration);
                        }

                        self.regs.smbn_sda().write(|w| unsafe { w.bits(address << 1) });

                        // Wait for completion
                        let r = self
                            .wait_for(|| {
                                let r = self.regs.smbn_st().read();
                                if r.ber().bit_is_set() || r.negack().bit_is_set() || r.sdast().bit_is_set() {
                                    Some(r)
                                } else {
                                    None
                                }
                            })
                            .await;
                        if r.ber().bit_is_set() {
                            let _ = self.handle_ber::<()>();
                            return Err(Error::LostArbitration);
                        }
                        if r.negack().bit_is_set() {
                            self.handle_negack();
                            return Err(Error::AddressNack);
                        }
                    }

                    self.bulk_write(data).await?;

                    prevop = PrevOpType::Write;
                }
                Operation::Read(data) => {
                    if data.is_empty() && matches!(prevop, PrevOpType::Read(_)) {
                        // Ignore chained zero sized reads
                    } else if let PrevOpType::Read(completion) = prevop {
                        prevop = PrevOpType::Read(self.bulk_read(data, last, || completion.complete()).await?);
                    } else {
                        // Send start and address
                        self.regs
                            .smbn_ctl1()
                            .modify(|_, w| w.stastre().set_bit().start().set_bit());
                        // Wait for completion
                        let r = self
                            .wait_for(|| {
                                let r = self.regs.smbn_st().read();
                                if r.ber().bit_is_set() || r.sdast().bit_is_set() {
                                    Some(r)
                                } else {
                                    None
                                }
                            })
                            .await;
                        if r.ber().bit_is_set() {
                            let _ = self.handle_ber::<()>();
                            return Err(Error::LostArbitration);
                        }

                        self.regs.smbn_sda().write(|w| unsafe { w.bits((address << 1) | 1) });

                        // Wait for completion
                        let r = self
                            .wait_for(|| {
                                let r = self.regs.smbn_st().read();
                                if r.ber().bit_is_set()
                                    || r.negack().bit_is_set()
                                    || r.sdast().bit_is_set()
                                    || r.stastr().bit_is_set()
                                {
                                    Some(r)
                                } else {
                                    None
                                }
                            })
                            .await;
                        if r.ber().bit_is_set() {
                            let _ = self.handle_ber::<()>();
                            return Err(Error::LostArbitration);
                        }
                        if r.negack().bit_is_set() {
                            self.handle_negack();
                            return Err(Error::AddressNack);
                        }

                        if data.is_empty() {
                            // This causes proper completion of the zero sized read on the next operation.
                            // See also the implementation of readcompletion.
                            prevop = PrevOpType::Read(ReadCompletion {
                                regs: self.regs,
                                group: &mut [],
                            });
                        } else {
                            let regs = self.regs;
                            prevop = PrevOpType::Read(
                                self.bulk_read(data, last, || {
                                    regs.smbn_st().write(|w| w.stastr().set_bit());
                                    regs.smbn_ctl1().modify(|_, w| w.stastre().clear_bit());
                                })
                                .await?,
                            );
                        }
                    }
                }
            }
        }

        // Generate stop bit and complete final read if any
        self.regs.smbn_ctl1().modify(|_, w| w.stop().set_bit());

        if let PrevOpType::Read(completion) = prevop {
            completion.complete();
        }

        // And reset fifos
        self.regs.smbn_fif_cts().write(|w| w.clr_fifo().set_bit());

        Ok(())
    }

    // Functions below ensure you don't need the embedded_hal_async trait

    /// Helper function. Mirror of [embedded_hal_async::i2c::I2c::read]
    pub async fn read(&mut self, address: u8, read: &mut [u8]) -> Result<(), Error> {
        self.transaction(address, &mut [Operation::Read(read)]).await
    }

    /// Helper function. Mirror of [embedded_hal_async::i2c::I2c::write]
    pub async fn write(&mut self, address: u8, write: &[u8]) -> Result<(), Error> {
        self.transaction(address, &mut [Operation::Write(write)]).await
    }

    /// Helper function. Mirror of [embedded_hal_async::i2c::I2c::write_read]
    pub async fn write_read(&mut self, address: u8, write: &[u8], read: &mut [u8]) -> Result<(), Error> {
        self.transaction(address, &mut [Operation::Write(write), Operation::Read(read)])
            .await
    }

    /// Set the addresses on which to listen.
    pub async fn configure_addresses(&mut self, addresses: &[u8]) -> Result<(), ListenError> {
        let mut cnt = 0;
        for (i, addr) in addresses.iter().copied().enumerate() {
            if addr & 0x80 != 0 {
                // Not 7 bit
                return Err(ListenError::InvalidAddressList);
            }
            if addresses[..i].contains(&addr) {
                // Disallow duplicate addresses
                return Err(ListenError::InvalidAddressList);
            }
            // Don't count the addresses with special matching hardware
            if addr != GENERALCALL_ADDRESS && addr != ARP_ADDRESS {
                cnt += 1;
            }
        }
        // Check we don't exceed capacity
        if cnt > 8 {
            return Err(ListenError::InvalidAddressList);
        }

        // Disable previous addresses.
        self.disable_addresses();

        // Handle any still pending match
        while self.regs.smbn_st().read().nmatch().bit_is_set() {
            if self.decode_addr().is_some() {
                // Drop it immediately to handle the transaction in the default way.
                let _ = self.listen().await;
            } else {
                self.regs.smbn_st().write(|w| w.nmatch().set_bit());
            }
        }

        // Store the new addresses in the address registers
        let mut addr_reg_index = 0;
        self.bank_sel(false);
        for addr in addresses.iter().copied() {
            if addr == ARP_ADDRESS {
                self.regs.smbn_ctl3().modify(|_, w| w.arpmen().set_bit());
            } else if addr == GENERALCALL_ADDRESS {
                self.regs.smbn_ctl1().modify(|_, w| w.gcmen().set_bit());
            } else {
                // Unfortunately, the address registers are irregularly spaced in memory, so this match is needed
                match addr_reg_index {
                    0 => self
                        .regs
                        .smbn_addr1()
                        .write(|w| unsafe { w.addr().bits(addr).saen().set_bit() }),
                    1 => self
                        .regs
                        .smbn_addr2()
                        .write(|w| unsafe { w.addr().bits(addr).saen().set_bit() }),
                    2 => self
                        .regs
                        .smbn_addr3()
                        .write(|w| unsafe { w.addr().bits(addr).saen().set_bit() }),
                    3 => self
                        .regs
                        .smbn_addr4()
                        .write(|w| unsafe { w.addr().bits(addr).saen().set_bit() }),
                    4 => self
                        .regs
                        .smbn_addr5()
                        .write(|w| unsafe { w.addr().bits(addr).saen().set_bit() }),
                    5 => self
                        .regs
                        .smbn_addr6()
                        .write(|w| unsafe { w.addr().bits(addr).saen().set_bit() }),
                    6 => self
                        .regs
                        .smbn_addr7()
                        .write(|w| unsafe { w.addr().bits(addr).saen().set_bit() }),
                    7 => self
                        .regs
                        .smbn_addr8()
                        .write(|w| unsafe { w.addr().bits(addr).saen().set_bit() }),
                    _ => unreachable!("Incorrectly checked address list"),
                };
                addr_reg_index += 1;
            }
        }
        self.bank_sel(true);

        // And enable interrupts for listen.
        self.regs.smbn_ctl1().modify(|_, w| w.nminte().set_bit());

        Ok(())
    }

    fn disable_addresses(&mut self) {
        self.bank_sel(false);
        self.regs.smbn_ctl1().modify(|_, w| w.gcmen().clear_bit());
        self.regs.smbn_ctl3().modify(|_, w| w.arpmen().clear_bit());
        self.regs.smbn_addr1().write(|w| unsafe { w.bits(0) });
        self.regs.smbn_addr2().write(|w| unsafe { w.bits(0) });
        self.regs.smbn_addr3().write(|w| unsafe { w.bits(0) });
        self.regs.smbn_addr4().write(|w| unsafe { w.bits(0) });
        self.regs.smbn_addr5().write(|w| unsafe { w.bits(0) });
        self.regs.smbn_addr6().write(|w| unsafe { w.bits(0) });
        self.regs.smbn_addr7().write(|w| unsafe { w.bits(0) });
        self.regs.smbn_addr8().write(|w| unsafe { w.bits(0) });
        self.bank_sel(true);
    }

    fn decode_addr(&mut self) -> Option<u8> {
        let r = self.regs.smbn_cst().read();
        if r.gcmatch().bit_is_set() {
            Some(GENERALCALL_ADDRESS)
        } else if r.arpmatch().bit_is_set() {
            Some(ARP_ADDRESS)
        } else {
            let r = self.regs.smbn_cst2().read();
            if r.matcha1f().bit_is_set() {
                Some(self.regs.smbn_addr1().read().addr().bits())
            } else if r.matcha2f().bit_is_set() {
                Some(self.regs.smbn_addr2().read().addr().bits())
            } else if r.matcha3f().bit_is_set() {
                self.bank_sel(false);
                let a = self.regs.smbn_addr3().read();
                self.bank_sel(true);
                Some(a.addr().bits())
            } else if r.matcha4f().bit_is_set() {
                self.bank_sel(false);
                let a = self.regs.smbn_addr4().read();
                self.bank_sel(true);
                Some(a.addr().bits())
            } else if r.matcha5f().bit_is_set() {
                self.bank_sel(false);
                let a = self.regs.smbn_addr5().read();
                self.bank_sel(true);
                Some(a.addr().bits())
            } else if r.matcha6f().bit_is_set() {
                self.bank_sel(false);
                let a = self.regs.smbn_addr6().read();
                self.bank_sel(true);
                Some(a.addr().bits())
            } else if r.matcha7f().bit_is_set() {
                self.bank_sel(false);
                let a = self.regs.smbn_addr7().read();
                self.bank_sel(true);
                Some(a.addr().bits())
            } else if self.regs.smbn_cst3().read().matcha8f().bit_is_set() {
                self.bank_sel(false);
                let a = self.regs.smbn_addr8().read();
                self.bank_sel(true);
                Some(a.addr().bits())
            } else {
                None
            }
        }
    }
}

impl AsyncI2cTarget for I2CController<'_> {
    type Error = ListenError;

    type Read<'a>
        = ReadHandler<'a>
    where
        Self: 'a;

    type Write<'a>
        = WriteHandler<'a>
    where
        Self: 'a;

    async fn listen(&mut self) -> Result<Transaction<Self::Read<'_>, Self::Write<'_>>, Self::Error> {
        let addr = loop {
            let st = self
                .wait_for(|| {
                    let st = self.regs.smbn_st().read();
                    if st.ber().bit_is_set() || st.slvstp().bit_is_set() || st.nmatch().bit_is_set() {
                        Some(st)
                    } else {
                        None
                    }
                })
                .await;

            if st.ber().bit_is_set() {
                self.regs.smbn_fif_cts().write(|w| w.clr_fifo().set_bit());
                self.regs.smbn_st().write(|w| w.ber().set_bit());
                return Err(ListenError::BusError);
            }

            if st.slvstp().bit_is_set() {
                self.regs.smbn_st().write(|w| w.slvstp().set_bit());
                return Ok(Transaction::Deselect);
            }

            let Some(addr) = self.decode_addr() else {
                self.regs.smbn_st().write(|w| w.nmatch().set_bit());
                continue;
            };

            break addr;
        };

        if self.regs.smbn_st().read().xmit().bit_is_set() {
            self.regs.smbn_st().write(|w| w.nmatch().set_bit());
            Ok(Transaction::Read {
                address: AnyAddress::Seven(addr),
                handler: ReadHandler {
                    _lifetime: PhantomData,
                    regs: self.regs,
                    waker: self.waker,
                    remediation: self.remediation,
                },
            })
        } else {
            // Wait for one more than group size to catch all restarts!
            self.regs.smbn_rxf_ctl().write(|w| unsafe {
                w.last_pec()
                    .clear_bit()
                    .thr_rxie()
                    .set_bit()
                    .rx_thr()
                    .bits(GROUP_SIZE + 1)
            });
            self.regs.smbn_st().write(|w| w.nmatch().set_bit());
            Ok(Transaction::Write {
                address: AnyAddress::Seven(addr),
                handler: WriteHandler {
                    _lifetime: PhantomData,
                    regs: self.regs,
                    waker: self.waker,
                    remediation: self.remediation,
                },
            })
        }
    }
}

/// Handler for i2c target reads.
pub struct ReadHandler<'a> {
    _lifetime: PhantomData<&'a mut ()>,
    regs: &'static crate::pac::smb0::RegisterBlock,
    waker: &'static AtomicWaker,
    remediation: &'static AtomicU32,
}

impl ReadHandler<'_> {
    fn wait_for<O, F: Fn() -> Option<O>>(&mut self, f: F) -> impl Future<Output = O> {
        WaitForCondition {
            f,
            regs: self.regs,
            waker: self.waker,
        }
    }
}

impl AsyncReadTransaction for ReadHandler<'_> {
    type Error = ListenError;

    async fn handle_part(mut self, buffer: &[u8]) -> Result<embedded_hal_i2c::ReadResult<Self>, Self::Error> {
        macro_rules! status_check {
            ($self:ident, $st:ident, $read:ident) => {
                if $st.ber().bit_is_set() {
                    return Err(ListenError::BusError);
                }
                if $st.negack().bit_is_set() {
                    $read -= self.regs.smbn_txf_sts().read().tx_bytes().bits() as usize;
                    $self.regs.smbn_fif_cts().write(|w| w.clr_fifo().set_bit());
                    $self.regs.smbn_txf_ctl().write(|w| w.thr_txie().clear_bit());
                    $self.regs.smbn_st().write(|w| w.negack().set_bit());
                    // Stop remediation from happening
                    core::mem::forget($self);
                    return Ok(ReadResult::Complete($read));
                }
            };
        }

        let mut chunks = buffer.chunks(GROUP_SIZE.into());
        let Some(first) = chunks.next() else {
            return Ok(ReadResult::Partial(self));
        };
        self.regs
            .smbn_txf_ctl()
            .modify(|_, w| unsafe { w.tx_thr().bits(FIFO_SIZE - GROUP_SIZE).thr_txie().set_bit() });
        let mut read = first.len();
        for b in first {
            self.regs.smbn_sda().write(|w| unsafe { w.bits(*b) });
        }
        self.regs.smbn_txf_sts().write(|w| w.tx_thst().set_bit());
        for chunk in chunks {
            let st = self
                .wait_for(|| {
                    let st = self.regs.smbn_st().read();
                    let txf_sts = self.regs.smbn_txf_sts().read();
                    if txf_sts.tx_thst().bit_is_set()
                        || st.ber().bit_is_set()
                        || st.negack().bit_is_set()
                        || st.sdast().bit_is_set()
                    {
                        Some(st)
                    } else {
                        None
                    }
                })
                .await;

            status_check!(self, st, read);

            read += chunk.len();
            for b in chunk {
                self.regs.smbn_sda().write(|w| unsafe { w.bits(*b) });
            }
            self.regs.smbn_txf_sts().write(|w| w.tx_thst().set_bit());
        }

        // Stall bus on threshold, as we can't modify the threshold without being stalled.
        self.regs.smbn_txf_ctl().modify(|_, w| w.thr_txie().clear_bit());

        let st = self
            .wait_for(|| {
                let st = self.regs.smbn_st().read();
                if st.ber().bit_is_set() || st.negack().bit_is_set() || st.sdast().bit_is_set() {
                    Some(st)
                } else {
                    None
                }
            })
            .await;

        status_check!(self, st, read);

        // Set threshold to 0 to send last bytes
        self.regs
            .smbn_txf_ctl()
            .write(|w| unsafe { w.thr_txie().clear_bit().tx_thr().bits(0) });

        let st = self
            .wait_for(|| {
                let st = self.regs.smbn_st().read();
                if st.ber().bit_is_set() || st.negack().bit_is_set() || st.sdast().bit_is_set() {
                    Some(st)
                } else {
                    None
                }
            })
            .await;

        status_check!(self, st, read);

        Ok(ReadResult::Partial(self))
    }
}

impl Drop for ReadHandler<'_> {
    fn drop(&mut self) {
        self.regs
            .smbn_txf_ctl()
            .modify(|_, w| unsafe { w.tx_thr().bits(FIFO_SIZE - GROUP_SIZE).thr_txie().set_bit() });
        self.remediation.store(REMEDIATION_SLAVE_READ, Ordering::Release);
        self.regs.smbn_ctl1().modify(|_, w| w.inten().set_bit());
    }
}

/// Handler for i2c target writes.
pub struct WriteHandler<'a> {
    _lifetime: PhantomData<&'a mut ()>,
    regs: &'static crate::pac::smb0::RegisterBlock,
    waker: &'static AtomicWaker,
    remediation: &'static AtomicU32,
}

impl WriteHandler<'_> {
    fn wait_for<O, F: Fn() -> Option<O>>(&mut self, f: F) -> impl Future<Output = O> {
        WaitForCondition {
            f,
            regs: self.regs,
            waker: self.waker,
        }
    }
}

impl AsyncWriteTransaction for WriteHandler<'_> {
    type Error = ListenError;

    async fn handle_part(mut self, buffer: &mut [u8]) -> Result<embedded_hal_i2c::WriteResult<Self>, Self::Error> {
        let mut total = 0;
        for chunk in buffer.chunks_mut(GROUP_SIZE.into()) {
            let (st, fif_cts) = self
                .wait_for(|| {
                    let st = self.regs.smbn_st().read();
                    let rxf_sts = self.regs.smbn_rxf_sts().read();
                    let fif_cts = self.regs.smbn_fif_cts().read();
                    if st.ber().bit_is_set()
                        || st.slvstp().bit_is_set()
                        || st.sdast().bit_is_set()
                        || rxf_sts.rx_thst().bit_is_set()
                        || fif_cts.slvrstr().bit_is_set()
                    {
                        Some((st, fif_cts))
                    } else {
                        None
                    }
                })
                .await;

            if st.ber().bit_is_set() {
                self.regs.smbn_fif_cts().modify(|_, w| w.clr_fifo().set_bit());
                return Err(ListenError::BusError);
            }
            if st.slvstp().bit_is_set() || fif_cts.slvrstr().bit_is_set() {
                // End of write, empty fifo
                let avail: usize = self.regs.smbn_rxf_sts().read().rx_bytes().bits().into();
                if avail < chunk.len() {
                    // we are done after this chunk, so need special handling
                    total += avail;
                    for b in chunk.iter_mut().take(avail) {
                        *b = self.regs.smbn_sda().read().bits();
                    }
                    // Reset fifo and slvrstsr bits
                    self.regs
                        .smbn_fif_cts()
                        .modify(|_, w| w.clr_fifo().set_bit().slvrstr().set_bit());
                    core::mem::forget(self);
                    return Ok(WriteResult::Complete(total));
                }
            }

            // There are enough bytes for at least the current chunk
            total += chunk.len();
            for b in chunk {
                *b = self.regs.smbn_sda().read().bits();
            }
            self.regs.smbn_rxf_sts().write(|w| w.rx_thst().set_bit());
        }

        Ok(WriteResult::Partial(self))
    }
}

impl Drop for WriteHandler<'_> {
    fn drop(&mut self) {
        self.remediation.store(REMEDIATION_SLAVE_WRITE, Ordering::Release);
        self.regs.smbn_ctl1().modify(|_, w| w.inten().set_bit());
    }
}

impl embedded_hal_async::i2c::ErrorType for I2CController<'_> {
    type Error = Error;
}

impl I2c for I2CController<'_> {
    async fn transaction(&mut self, address: u8, operations: &mut [Operation<'_>]) -> Result<(), Self::Error> {
        (*self).transaction(address, operations).await
    }
}

mod sealed {
    use core::sync::atomic::AtomicU32;

    use embassy_sync::waitqueue::AtomicWaker;

    pub trait SealedValidI2CConfig {
        unsafe fn setup_pins(cs: critical_section::CriticalSection);
        unsafe fn setup_pullup(_cs: critical_section::CriticalSection, enable: bool);
    }

    pub trait SealedInstance {
        fn waker() -> &'static AtomicWaker;
        fn regs() -> &'static crate::pac::smb0::RegisterBlock;
        fn remediation() -> &'static AtomicU32;
        /// Safety: should only be called after clock init
        unsafe fn clockfreq() -> u32;
    }
}

/// A marker trait implemented for all valid configs
pub trait ValidI2CConfig: sealed::SealedValidI2CConfig {}

/// A marker trait implemented for all peripherals that can do I2C
pub trait Instance: sealed::SealedInstance + PeripheralType + 'static + Send {
    /// The interrupt use by this instance
    type Interrupt: crate::interrupt::typelevel::Interrupt;
}

macro_rules! impl_instance {
    ($instance:ident, $pac:ident, $clock:ident) => {
        impl sealed::SealedInstance for crate::peripherals::$instance {
            fn waker() -> &'static AtomicWaker {
                static WAKER: AtomicWaker = AtomicWaker::new();
                &WAKER
            }

            fn regs() -> &'static crate::pac::smb0::RegisterBlock {
                // Safety: not owned, memory is always present
                unsafe { &*crate::pac::$pac::PTR }
            }

            fn remediation() -> &'static AtomicU32 {
                static REMEDIATION: AtomicU32 = AtomicU32::new(REMEDIATION_NONE);
                &REMEDIATION
            }

            unsafe fn clockfreq() -> u32 {
                // Safety: We require clock init to be called before this is called
                unsafe { get_clocks() }.$clock
            }
        }

        impl Instance for crate::peripherals::$instance {
            type Interrupt = crate::interrupt::typelevel::$instance;
        }
    };
}

macro_rules! impl_config {
    ($instance:ident, $scl:ident, $sda:ident, $pin_config:expr, $pullup_config:expr) => {
        impl<T> sealed::SealedValidI2CConfig
            for (
                crate::peripherals::$instance,
                crate::peripherals::$scl,
                crate::peripherals::$sda,
                T,
            )
        {
            unsafe fn setup_pins(_cs: critical_section::CriticalSection) {
                fn internal_set(f: impl FnOnce(crate::pac::Sysconfig, crate::pac::Sysglue)) {
                    f(unsafe { crate::pac::Sysconfig::steal() }, unsafe {
                        crate::pac::Sysglue::steal()
                    });
                }
                internal_set($pin_config);
            }

            unsafe fn setup_pullup(_cs: critical_section::CriticalSection, enable: bool) {
                fn internal_set(f: impl FnOnce(crate::pac::Sysconfig, bool), enable: bool) {
                    f(unsafe { crate::pac::Sysconfig::steal() }, enable);
                }
                internal_set($pullup_config, enable);
            }
        }

        impl<T> ValidI2CConfig
            for (
                crate::peripherals::$instance,
                crate::peripherals::$scl,
                crate::peripherals::$sda,
                T,
            )
        {
        }
    };
}

macro_rules! impl_config_lpc {
    ($instance:ident, $scl:ident, $sda:ident, $pin_config:expr, $pullup_config:expr) => {
        impl sealed::SealedValidI2CConfig
            for (
                crate::peripherals::$instance,
                crate::peripherals::$scl,
                crate::peripherals::$sda,
                crate::Lpc,
            )
        {
            unsafe fn setup_pins(_cs: critical_section::CriticalSection) {
                fn internal_set(f: impl FnOnce(crate::pac::Sysconfig, crate::pac::Sysglue)) {
                    f(unsafe { crate::pac::Sysconfig::steal() }, unsafe {
                        crate::pac::Sysglue::steal()
                    });
                }
                internal_set($pin_config);
            }

            unsafe fn setup_pullup(_cs: critical_section::CriticalSection, enable: bool) {
                fn internal_set(f: impl FnOnce(crate::pac::Sysconfig, bool), enable: bool) {
                    f(unsafe { crate::pac::Sysconfig::steal() }, enable);
                }
                internal_set($pullup_config, enable);
            }
        }

        impl ValidI2CConfig
            for (
                crate::peripherals::$instance,
                crate::peripherals::$scl,
                crate::peripherals::$sda,
                crate::Lpc,
            )
        {
        }
    };
}

impl_instance!(SMB0, Smb0, apb3_clk);
impl_instance!(SMB1, Smb1, apb3_clk);
impl_instance!(SMB2, Smb2, apb2_clk);
impl_instance!(SMB3, Smb3, apb2_clk);
impl_instance!(SMB4, Smb4, apb3_clk);
impl_instance!(SMB5, Smb5, apb3_clk);
impl_instance!(SMB6, Smb6, apb3_clk);
impl_instance!(SMB7, Smb7, apb3_clk);

impl_config!(
    SMB0,
    PC12,
    PB12,
    |config, _| {
        config.devalt2().modify(|_, w| w.i2c0_0_sl().set_bit());
    },
    |config, enable| {
        config.pupd_en0().modify(|_, w| w.i2c0_0_pue().bit(enable));
    }
);

impl_config!(
    SMB1,
    PK08,
    PK07,
    |config, _| {
        config.devalt2().modify(|_, w| w.i2c1_0_sl().set_bit());
    },
    |config, enable| {
        config.pupd_en0().modify(|_, w| w.i2c1_0_pue().bit(enable));
    }
);

impl_config!(
    SMB2,
    PL08,
    PK09,
    |config, _| {
        config.devalt2().modify(|_, w| w.i2c2_0_sl().set_bit());
    },
    |config, enable| {
        config.pupd_en0().modify(|_, w| w.i2c2_0_pue().bit(enable));
    }
);

impl_config!(
    SMB3,
    PF08,
    PF09,
    |config, _| {
        config.devalt2().modify(|_, w| w.i2c3_0_sl().set_bit());
        config.devaltm().modify(|_, w| w.ad22_sl().clear_bit());
    },
    |config, enable| {
        config.pupd_en0().modify(|_, w| w.i2c3_0_pue().bit(enable));
    }
);

impl_config!(
    SMB4,
    PJ06,
    PJ09,
    |config, glue| {
        glue.smb_sel().modify(|_, w| w.smb4_sl().clear_bit());
        // safety: we are in a critical section
        unsafe { crate::pac::Miwu0::steal() }
            .wkpcln5()
            .write(|w| w.input3().set_bit());
        config.devalta().modify(|_, w| w._32k_out_sl().clear_bit());
        config.devalt2().modify(|_, w| w.i2c4_0_sl().set_bit());
        config
            .devaltb()
            .modify(|_, w| w.rxd_sl().clear_bit().txd_sl().clear_bit());
        config
            .devaltj()
            .modify(|_, w| w.cr_sin2_sl().clear_bit().cr_sout2_sl().clear_bit());
    },
    |config, enable| {
        config.pupd_en0().modify(|_, w| w.i2c4_0_pue().bit(enable));
    }
);

impl_config!(
    SMB4,
    PF05,
    PF06,
    |config, glue| {
        glue.smb_sel().modify(|_, w| w.smb4_sl().set_bit());
        // safety: we are in a critical section
        unsafe { crate::pac::Miwu0::steal() }
            .wkpcln5()
            .write(|w| w.input3().set_bit());
        config.devalt6().modify(|_, w| w.i2c4_1_sl().set_bit());
    },
    |config, enable| {
        config.pupd_en1().modify(|_, w| w.i2c4_1_pue().bit(enable));
    }
);

impl_config!(
    SMB5,
    PD05,
    PD04,
    |config, glue| {
        glue.smb_sel().modify(|_, w| w.smb5_sl().clear_bit());
        // safety: we are in a critical section
        unsafe { crate::pac::Miwu2::steal() }
            .wkpcln7()
            .write(|w| w.input0().set_bit());
        config.devalt2().modify(|_, w| w.i2c5_0_sl().set_bit());
        config
            .devaltb()
            .modify(|_, w| w.cts_sl().clear_bit().rts_sl().clear_bit());
    },
    |config, enable| {
        config.pupd_en0().modify(|_, w| w.i2c5_0_pue().bit(enable));
    }
);

impl_config_lpc!(
    SMB5,
    PE08,
    PE09,
    |config, glue| {
        glue.smb_sel().modify(|_, w| w.smb5_sl().set_bit());
        // safety: we are in a critical section
        unsafe { crate::pac::Miwu2::steal() }
            .wkpcln7()
            .write(|w| w.input0().set_bit());
        config.devalt6().modify(|_, w| w.i2c5_1_sl().set_bit());
        config.devaltn().modify(|_, w| w.i3c1_sl().clear_bit());
    },
    |config, enable| {
        config.pupd_en1().modify(|_, w| w.i2c5_1_pue().bit(enable));
    }
);

impl_config!(
    SMB6,
    PH10,
    PH09,
    |config, glue| {
        glue.smb_sel().modify(|_, w| w.smb6_sl().clear_bit());
        // safety: we are in a critical section
        unsafe { crate::pac::Miwu2::steal() }
            .wkpcln7()
            .write(|w| w.input1().set_bit());
        config.devalt4().modify(|_, w| w.pwm1_sl().clear_bit());
        config.devalt2().modify(|_, w| w.i2c6_0_sl().set_bit());
        config.devaltm().modify(|_, w| w.ad22_sl().clear_bit());
    },
    |config, enable| {
        config.pupd_en0().modify(|_, w| w.i2c6_0_pue().bit(enable));
    }
);

impl_config!(
    SMB6,
    PL06,
    PL07,
    |config, glue| {
        glue.smb_sel().modify(|_, w| w.smb6_sl().set_bit());
        // safety: we are in a critical section
        unsafe { crate::pac::Miwu2::steal() }
            .wkpcln7()
            .write(|w| w.input1().set_bit());
        config.devalt6().modify(|_, w| w.i2c6_1_sl().set_bit());
        config.devaltn().modify(|_, w| w.i3c1_sl().clear_bit());
    },
    |config, enable| {
        config.pupd_en1().modify(|_, w| w.i2c6_1_pue().bit(enable));
    }
);

impl_config!(
    SMB7,
    PJ10,
    PK10,
    |config, glue| {
        glue.smb_sel().modify(|_, w| w.smb7_sl().clear_bit());
        // safety: we are in a critical section
        unsafe { crate::pac::Miwu2::steal() }
            .wkpcln7()
            .write(|w| w.input2().set_bit());
        config.devalt2().modify(|_, w| w.i2c7_0_sl().set_bit());
        config
            .devaltb()
            .modify(|_, w| w.dsr_sl().clear_bit().dcd_sl().clear_bit());
    },
    |config, enable| {
        config.pupd_en0().modify(|_, w| w.i2c7_0_pue().bit(enable));
    }
);

impl_config!(
    SMB7,
    PH08,
    PJ07,
    |config, glue| {
        glue.smb_sel().modify(|_, w| w.smb7_sl().set_bit());
        // safety: we are in a critical section
        unsafe { crate::pac::Miwu2::steal() }
            .wkpcln7()
            .write(|w| w.input2().set_bit());
        config
            .devalt4()
            .modify(|_, w| w.pwm5_sl().clear_bit().pwm6_sl().clear_bit());
        config.devaltk().modify(|_, w| w.i2c7_1_sl().set_bit());
        // Note: this messes with debug interfaces, maybe there is a better way?
        unsafe { crate::pac::Dev::steal() }
            .dbgctrl2()
            .modify(|_, w| w.ccdev_sel().bits(6));
    },
    |config, enable| {
        config.pupd_en1().modify(|_, w| w.i2c7_1_pue().bit(enable));
    }
);
