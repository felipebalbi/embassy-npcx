use core::future::Future;
use core::marker::PhantomData;

use embassy_hal_internal::{into_ref, Peripheral, PeripheralRef};
use embassy_sync::waitqueue::AtomicWaker;
pub use embedded_hal_async::i2c::{I2c, Operation};

use crate::interrupt::typelevel::Interrupt;

// Size of the peripherals fifo
const FIFO_SIZE: u8 = 32;
// Number of bytes between refilling/emptying of the fifo
const GROUP_SIZE: u8 = 24;

pub struct InterruptHandler<T> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> crate::interrupt::typelevel::Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        T::waker().wake();
        critical_section::with(|_| {
            T::regs().smbn_ctl1().modify(|_, w| w.inten().clear_bit());
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

pub enum Speed {
    Slow,
    Fast,
    FastPlus,
}

#[derive(Debug)]
pub enum Error {
    LostArbitration,
    BusError,
    NegativeAckAddress,
    NegativeAckData,
}

impl embedded_hal_async::i2c::Error for Error {
    fn kind(&self) -> embedded_hal::i2c::ErrorKind {
        match self {
            Error::LostArbitration => embedded_hal::i2c::ErrorKind::ArbitrationLoss,
            Error::BusError => embedded_hal::i2c::ErrorKind::Bus,
            Error::NegativeAckData => {
                embedded_hal::i2c::ErrorKind::NoAcknowledge(embedded_hal::i2c::NoAcknowledgeSource::Data)
            }
            Error::NegativeAckAddress => {
                embedded_hal::i2c::ErrorKind::NoAcknowledge(embedded_hal::i2c::NoAcknowledgeSource::Address)
            }
        }
    }
}

struct AnySMB {}

impl<T: Instance> From<T> for AnySMB {
    fn from(_smb: T) -> Self {
        AnySMB {}
    }
}

// Allow use of PeripheralRef to do lifetime management
impl Peripheral for AnySMB {
    type P = AnySMB;

    unsafe fn clone_unchecked(&self) -> Self::P {
        AnySMB {}
    }
}

pub struct I2CController<'a> {
    _dev: PeripheralRef<'a, AnySMB>,
    regs: &'static crate::pac::smb0::RegisterBlock,
    waker: &'static AtomicWaker,
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
    fn speed_init(&mut self, speed: Speed) {
        match speed {
            Speed::Slow => {
                self.regs
                    .smbn_ctl3()
                    .modify(|_, w| unsafe { w._400k_mode().clear_bit().sclfrq8_7().bits(0) });
                self.regs.smbn_ctl4().modify(|_, w| unsafe { w.hldt().bits(15) });
                self.regs.smbn_ctl2().modify(|_, w| unsafe { w.sclfrq6_0().bits(63) });
            }
            Speed::Fast => todo!(),
            Speed::FastPlus => todo!(),
        }
    }

    pub fn new<T: Instance + 'p, Scl, Sda, Mode>(
        peri: impl Peripheral<P = T> + 'p,
        _scl: impl Peripheral<P = Scl> + 'p,
        _sda: impl Peripheral<P = Sda> + 'p,
        _irqs: impl crate::interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>>,
        _mode: Mode,
    ) -> Self
    where
        (T, Scl, Sda, Mode): ValidI2CConfig,
    {
        // Safety: _irqs ensures an interrupt handler is bound
        unsafe {
            T::Interrupt::enable();
        }

        into_ref!(peri);

        critical_section::with(|cs| {
            // Safety: We have exclusive ownership over the peripherals.
            unsafe { <(T, Scl, Sda, Mode) as sealed::SealedValidI2CConfig>::setup_pins(cs) };
        });

        let mut dev = Self {
            _dev: peri.map_into(),
            regs: T::regs(),
            waker: T::waker(),
        };

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
        dev.speed_init(Speed::Slow);
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
                return Err(Error::NegativeAckData);
            }

            // Add new data
            for b in (&mut data).take(GROUP_SIZE.into()) {
                self.regs.smbn_sda().write(|w| unsafe { w.bits(b) });
            }

            // Reset threshold trigger
            self.regs.smbn_txf_sts().write(|w| w.tx_thst().set_bit());
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
            (data, group) = data.split_at_mut(FIFO_SIZE as _);
            // Ensure we stall before the final group
            self.regs
                .smbn_rxf_ctl()
                .write(|w| unsafe { w.last_pec().clear_bit().thr_rxie().clear_bit().rx_thr().bits(FIFO_SIZE) });
        } else {
            (data, group) = data.split_at_mut(GROUP_SIZE as _);
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
                // Add nack at end
                self.regs.smbn_rxf_ctl().write(|w| unsafe {
                    w.last_pec()
                        .bit(negack_last)
                        .thr_rxie()
                        .clear_bit()
                        .rx_thr()
                        .bits(group.len() as u8)
                });
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

            // And clear rx threshold status
            self.regs.smbn_rxf_sts().write(|w| w.rx_thst().set_bit());

            group = next_group;
        }

        // Finish last block
        self.wait_read().await?;

        // Return completion
        Ok(ReadCompletion { regs: self.regs, group })
    }

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
                            return Err(Error::NegativeAckAddress);
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
                            return Err(Error::NegativeAckAddress);
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

    pub async fn read(&mut self, address: u8, read: &mut [u8]) -> Result<(), Error> {
        self.transaction(address, &mut [Operation::Read(read)]).await
    }

    pub async fn write(&mut self, address: u8, write: &[u8]) -> Result<(), Error> {
        self.transaction(address, &mut [Operation::Write(write)]).await
    }

    pub async fn write_read(&mut self, address: u8, write: &[u8], read: &mut [u8]) -> Result<(), Error> {
        self.transaction(address, &mut [Operation::Write(write), Operation::Read(read)])
            .await
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
    use embassy_sync::waitqueue::AtomicWaker;

    pub trait SealedValidI2CConfig {
        unsafe fn setup_pins(cs: critical_section::CriticalSection);
    }

    pub trait SealedInstance {
        fn waker() -> &'static AtomicWaker;
        fn regs() -> &'static crate::pac::smb0::RegisterBlock;
    }
}

pub trait ValidI2CConfig: sealed::SealedValidI2CConfig {}

pub trait Instance: sealed::SealedInstance + embassy_hal_internal::Peripheral<P = Self> {
    type Interrupt: crate::interrupt::typelevel::Interrupt;
}

macro_rules! impl_instance {
    ($instance:ident, $pac:ident) => {
        impl sealed::SealedInstance for crate::peripherals::$instance {
            fn waker() -> &'static AtomicWaker {
                static WAKER: AtomicWaker = AtomicWaker::new();
                &WAKER
            }

            fn regs() -> &'static crate::pac::smb0::RegisterBlock {
                // Safety: not owned, memory is always present
                unsafe { &*crate::pac::$pac::PTR }
            }
        }

        impl Instance for crate::peripherals::$instance {
            type Interrupt = crate::interrupt::typelevel::$instance;
        }
    };
}

macro_rules! impl_config {
    ($instance:ident, $scl:ident, $sda:ident, $pin_config:expr) => {
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
    ($instance:ident, $scl:ident, $sda:ident, $pin_config:expr) => {
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

impl_instance!(SMB0, Smb0);
impl_instance!(SMB1, Smb1);
impl_instance!(SMB2, Smb2);
impl_instance!(SMB3, Smb3);
impl_instance!(SMB4, Smb4);
impl_instance!(SMB5, Smb5);
impl_instance!(SMB6, Smb6);
impl_instance!(SMB7, Smb7);

impl_config!(SMB0, PC12, PB12, |config, _| {
    config.devalt2().modify(|_, w| w.i2c0_0_sl().set_bit());
});

impl_config!(SMB1, PK08, PK07, |config, _| {
    config.devalt2().modify(|_, w| w.i2c1_0_sl().set_bit());
});

impl_config!(SMB2, PL08, PK09, |config, _| {
    config.devalt2().modify(|_, w| w.i2c2_0_sl().set_bit());
});

impl_config!(SMB3, PF08, PF09, |config, _| {
    config.devalt2().modify(|_, w| w.i2c3_0_sl().set_bit());
    config.devaltm().modify(|_, w| w.ad22_sl().clear_bit());
});

impl_config!(SMB4, PJ06, PJ09, |config, glue| {
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
});

impl_config!(SMB4, PF05, PF06, |config, glue| {
    glue.smb_sel().modify(|_, w| w.smb4_sl().set_bit());
    // safety: we are in a critical section
    unsafe { crate::pac::Miwu0::steal() }
        .wkpcln5()
        .write(|w| w.input3().set_bit());
    config.devalt6().modify(|_, w| w.i2c4_1_sl().set_bit());
});

impl_config!(SMB5, PD05, PD04, |config, glue| {
    glue.smb_sel().modify(|_, w| w.smb5_sl().clear_bit());
    // safety: we are in a critical section
    unsafe { crate::pac::Miwu2::steal() }
        .wkpcln7()
        .write(|w| w.input0().set_bit());
    config.devalt2().modify(|_, w| w.i2c5_0_sl().set_bit());
    config
        .devaltb()
        .modify(|_, w| w.cts_sl().clear_bit().rts_sl().clear_bit());
});

impl_config_lpc!(SMB5, PE08, PE09, |config, glue| {
    glue.smb_sel().modify(|_, w| w.smb5_sl().set_bit());
    // safety: we are in a critical section
    unsafe { crate::pac::Miwu2::steal() }
        .wkpcln7()
        .write(|w| w.input0().set_bit());
    config.devalt6().modify(|_, w| w.i2c5_1_sl().set_bit());
    config.devaltn().modify(|_, w| w.i3c1_sl().clear_bit());
});

impl_config!(SMB6, PH10, PH09, |config, glue| {
    glue.smb_sel().modify(|_, w| w.smb6_sl().clear_bit());
    // safety: we are in a critical section
    unsafe { crate::pac::Miwu2::steal() }
        .wkpcln7()
        .write(|w| w.input1().set_bit());
    config.devalt4().modify(|_, w| w.pwm1_sl().clear_bit());
    config.devalt2().modify(|_, w| w.i2c6_0_sl().set_bit());
    config.devaltm().modify(|_, w| w.ad22_sl().clear_bit());
});

impl_config!(SMB6, PL06, PL07, |config, glue| {
    glue.smb_sel().modify(|_, w| w.smb6_sl().set_bit());
    // safety: we are in a critical section
    unsafe { crate::pac::Miwu2::steal() }
        .wkpcln7()
        .write(|w| w.input1().set_bit());
    config.devalt6().modify(|_, w| w.i2c6_1_sl().set_bit());
    config.devaltn().modify(|_, w| w.i3c1_sl().clear_bit());
});

impl_config!(SMB7, PJ10, PK10, |config, glue| {
    glue.smb_sel().modify(|_, w| w.smb7_sl().clear_bit());
    // safety: we are in a critical section
    unsafe { crate::pac::Miwu2::steal() }
        .wkpcln7()
        .write(|w| w.input2().set_bit());
    config.devalt2().modify(|_, w| w.i2c7_0_sl().set_bit());
    config
        .devaltb()
        .modify(|_, w| w.dsr_sl().clear_bit().dcd_sl().clear_bit());
});

impl_config!(SMB7, PH08, PJ07, |config, glue| {
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
});
