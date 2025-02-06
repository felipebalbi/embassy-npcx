use core::sync::atomic::AtomicBool;

use maitake_sync::WaitQueue;

/// A token to cancel an operation with
pub struct CancellationToken {
    cancelled: AtomicBool,
    waiters: WaitQueue,
}

impl CancellationToken {
    /// Create a new token that has not yet been cancelled
    pub const fn new() -> Self {
        Self {
            cancelled: AtomicBool::new(false),
            waiters: WaitQueue::new(),
        }
    }

    /// Cancel any operation depending on this token
    pub fn cancel(&self) {
        self.cancelled.store(true, core::sync::atomic::Ordering::Release);
        self.waiters.wake_all();
    }

    /// Returns true if the [Self::cancel] has been called in the past for this token
    pub fn is_cancelled(&self) -> bool {
        self.cancelled.load(core::sync::atomic::Ordering::Acquire)
    }

    /// Wait for cancel being called
    pub(crate) async fn wait_for_cancel(&self) {
        let _ = self.waiters.wait_for_value(|| self.is_cancelled().then_some(())).await;
    }
}

impl Default for CancellationToken {
    fn default() -> Self {
        Self::new()
    }
}
