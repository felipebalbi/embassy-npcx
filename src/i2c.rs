use core::future::Future;

use defmt::info;
use embassy_sync::waitqueue::AtomicWaker;
pub use embedded_hal_async::i2c::{Operation, I2c};

use crate::pac::{self, interrupt, Interrupt, NVIC};

// Size of the peripherals fifo
const FIFO_SIZE: u8 = 32;
// Number of bytes between refilling/emptying of the fifo
const GROUP_SIZE: u8 = 24;

static WAKER: AtomicWaker = AtomicWaker::new();

#[crate::pac::interrupt]
fn SMB5() {
    WAKER.wake();
    critical_section::with(|_| {
        unsafe { pac::Smb5::steal() }
            .smbn_ctl1()
            .modify(|_, w| w.inten().clear_bit());
    })
}

struct WaitForCondition<F> {
    regs: &'static pac::smb0::RegisterBlock,
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
    regs: &'static pac::smb0::RegisterBlock,
    group: &'a mut [u8],
}

impl ReadCompletion<'_> {
    fn complete(self) {
        for b in self.group {
            *b = self.regs.smbn_sda().read().bits();
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
    BusError,
    NegativeAck,
}

impl embedded_hal_async::i2c::Error for Error {
    fn kind(&self) -> embedded_hal::i2c::ErrorKind {
        match self {
            Error::BusError => embedded_hal::i2c::ErrorKind::Bus,
            Error::NegativeAck => embedded_hal::i2c::ErrorKind::NoAcknowledge(embedded_hal::i2c::NoAcknowledgeSource::Unknown),
        }
    }
}

pub struct I2CController {
    regs: &'static pac::smb0::RegisterBlock,
    waker: &'static AtomicWaker,
}

impl I2CController {
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

    pub fn new(regs: &'static crate::pac::smb0::RegisterBlock, waker: &'static AtomicWaker) -> Self {
        let mut dev = Self { regs, waker };

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

    pub fn new_smb5() -> Self {
        unsafe {crate::pac::Sysconfig::steal() }.devcnt().modify(|_, w| unsafe {w.hif_typ_sel().bits(1)});
        unsafe {crate::pac::Sysglue::steal() }.smb_sel().modify(|_, w| w.smb5_sl().set_bit());
        unsafe {crate::pac::Miwu2::steal() }.wkpcln7().write(|w| w.input0().set_bit());
        unsafe {crate::pac::Sysconfig::steal() }.devalt6().modify(|_, w| w.i2c5_1_sl().set_bit());

        let res = Self::new(unsafe { &*crate::pac::Smb5::PTR }, &WAKER);

        unsafe { NVIC::unmask(Interrupt::SMB5) };

        res
    }

    fn handle_ber<T>(&mut self) -> Result<T, Error> {
        // TODO: Implement more robust recovery
        self.regs.smbn_fif_cts().write(|w| w.clr_fifo().set_bit());
        self.regs.smbn_st().write(|w| w.ber().set_bit());

        Err(Error::BusError)
    }

    fn handle_negack(&mut self) -> Result<(), Error> {
        self.regs.smbn_fif_cts().write(|w| w.clr_fifo().set_bit());
        self.regs.smbn_ctl1().modify(|_, w| w.stop().set_bit());
        self.regs.smbn_st().write(|w| w.negack().set_bit());

        Err(Error::NegativeAck)
    }

    async fn do_write_with_addr(&mut self, address: u8, data: &[u8], complete_prev: impl '_ + FnOnce() -> ()) -> Result<(), Error> {
        info!("Writing {} bytes to {:02x}", data.len(), address);
        
        // Send (repeated) start
        self.regs.smbn_ctl1().modify(|_, w| w.start().set_bit());

        // Complete previous
        complete_prev();

        //
        self.regs.smbn_sda().write(|w| unsafe { w.bits(address << 1) });

        // Do bulk write
        self.do_write_no_addr(data).await
    }

    async fn do_write_no_addr(&mut self, data: &[u8]) -> Result<(), Error> {
        // Set up transmission of first 31 bytes
        let mut data = data.iter().copied();
        for b in (&mut data).take((FIFO_SIZE - 1).into()) {
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
                return self.handle_negack();
            }

            // Add new data
            for b in (&mut data).take(GROUP_SIZE.into()) {
                self.regs.smbn_sda().write(|w| unsafe { w.bits(b) });
            }
        }

        Ok(())
    }

    async fn do_read_with_addr<'a>(&mut self, address: u8, data: &'a mut [u8], complete_prev: impl '_ + FnOnce() -> ()) -> Result<ReadCompletion<'a>, Error> {
        info!("Reading {} bytes from {:02x}", data.len(), address);
        // Send (repeated) start
        self.regs.smbn_ctl1().modify(|_, w| w.start().set_bit());

        // Complete previous
        complete_prev();

        // Do bulk transfer
        let regs = self.regs;
        self.bulk_read(data, move || {
            regs.smbn_sda().write(|w| unsafe { w.bits((address << 1) | 1) });
        }).await
    }

    async fn do_read_no_addr<'a>(&mut self, data: &'a mut [u8], complete_prev: impl '_ + FnOnce() -> ()) -> Result<ReadCompletion<'a>, Error> {
        // Do bulk transfer
        self.bulk_read(data, move || {
            complete_prev();
        }).await
    }

    async fn bulk_read<'a>(&mut self, mut data: &'a mut [u8], start: impl '_ + FnOnce() -> ()) -> Result<ReadCompletion<'a>, Error> {
        let mut group;

        // Setup transfer
        if data.len() <= FIFO_SIZE.into() {
            group = data;
            data = &mut [];
            // Add nack at end
            self.regs.smbn_rxf_ctl().write(|w| unsafe {
                w.last_pec()
                    .set_bit()
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
        while data.len() > 0 {
            self.wait_read().await?;

            let next_group;

            if data.len() <= FIFO_SIZE.into() {
                next_group = data;
                data = &mut [];
                // Add nack at end
                self.regs.smbn_rxf_ctl().write(|w| unsafe {
                    w.last_pec()
                        .set_bit()
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
        Ok(ReadCompletion {
            regs: self.regs,
            group,
        })
    }
}

impl embedded_hal_async::i2c::ErrorType  for I2CController {
    type Error = Error;
}

fn option_as_fnonce(o: Option<ReadCompletion<'_>>) -> impl '_ + FnOnce() -> () {
    move || {
        match o {
            Some(f) => f.complete(),
            None => {}
        }
    }
}

enum PrevOpType {
    None,
    Read,
    Write,
}

impl I2c for I2CController {
    async fn transaction(
        &mut self,
        address: u8,
        operations: &mut [embedded_hal::i2c::Operation<'_>],
    ) -> Result<(), Self::Error> {
        let mut completion = None;
        let mut prev_op = PrevOpType::None;

        for op in operations {
            match (op, &prev_op) {
                (embedded_hal::i2c::Operation::Read(data), PrevOpType::Read) => {
                    completion = Some(self.do_read_no_addr(*data, option_as_fnonce(completion)).await?);
                    prev_op = PrevOpType::Read;
                },
                (embedded_hal::i2c::Operation::Read(data), _) => {
                    completion = Some(self.do_read_with_addr(address, *data, option_as_fnonce(completion)).await?);
                    prev_op = PrevOpType::Read;
                },
                (embedded_hal::i2c::Operation::Write(data), PrevOpType::Write) => {
                    self.do_write_no_addr(*data).await?;
                    completion = None;
                    prev_op = PrevOpType::Write;
                },
                (embedded_hal::i2c::Operation::Write(data), _) => {
                    self.do_write_with_addr(address, *data, option_as_fnonce(completion)).await?;
                    completion = None;
                    prev_op = PrevOpType::Write;
                },
            }
        }
        self.regs.smbn_ctl1().modify(|_, w| w.stop().set_bit());
        option_as_fnonce(completion)();
        Ok(())
    }
}
