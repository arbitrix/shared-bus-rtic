#![no_std]

extern crate atsamd_hal as hal;

use core::sync::atomic::{AtomicBool, Ordering};
use embedded_hal::{blocking, spi};
use hal::sercom::v2::spi::{Spi, ValidConfig};

/// A convenience type to use for declaring the underlying bus type.
pub type SharedBus<T> = &'static CommonBus<T>;

// pub struct Test {
//     test: sercom::v2::spi::Spi<impl spi::ValidConfig>,
// }

//type Bla<C> = hal::hal::sercom::v2::spi::Spi<C>;

type ValidSpi<C: ValidConfig> = Spi<C>;

pub struct CommonBus<BUS> {
    bus: core::cell::UnsafeCell<BUS>,
    busy: AtomicBool,
}

pub struct BusProxy<'a, BUS> {
    manager: &'a CommonBus<BUS>,
    mode: spi::Mode,
}

impl<BUS> CommonBus<BUS> {
    pub fn new(bus: BUS) -> Self {
        CommonBus {
            bus: core::cell::UnsafeCell::new(bus),
            busy: AtomicBool::from(false),
        }
    }

    fn lock<R, F: FnOnce(&mut BUS) -> R>(&self, f: F) -> R {
        let compare =
            atomic::compare_exchange(&self.busy, false, true, Ordering::SeqCst, Ordering::SeqCst)
                .is_err();
        if compare {
            panic!("Bus conflict");
        }

        let result = f(unsafe { &mut *self.bus.get() });

        self.busy.store(false, Ordering::SeqCst);

        result
    }

    pub fn acquire(&self, mode: spi::Mode) -> BusProxy<BUS> {
        BusProxy {
            manager: self,
            mode,
        }
    }
}

unsafe impl<BUS> Sync for CommonBus<BUS> {}

macro_rules! spi {
    ($($T:ty),*) => {
        $(
        impl<BUS: blocking::spi::Write<$T>> blocking::spi::Write<$T> for BusProxy<'_, BUS> {
            type Error = BUS::Error;

            fn write(&mut self, words: &[$T]) -> Result<(), Self::Error> {
                self.manager.lock(|bus| bus.write(words))
            }
        }

        impl<BUS: blocking::spi::Transfer<$T>> blocking::spi::Transfer<$T> for BusProxy<'_, BUS> {
            type Error = BUS::Error;

            fn transfer<'w>(&mut self, words: &'w mut [$T]) -> Result<&'w [$T], Self::Error> {
                self.manager.lock(move |bus| bus.transfer(words))
            }
        }

        impl<BUS: spi::FullDuplex<$T>> spi::FullDuplex<$T> for BusProxy<'_, BUS> {
            type Error = BUS::Error;

            fn read(&mut self) -> nb::Result<$T, Self::Error> {
                self.manager.lock(|bus| bus.read())
            }

            fn send(&mut self, word: $T) -> nb::Result<(), Self::Error> {
                self.manager.lock(|bus| bus.send(word))
            }
        }

        impl<BUS: blocking::spi::Write<$T>> blocking::spi::Write<$T> for &CommonBus<BUS> {
            type Error = BUS::Error;

            fn write(&mut self, words: &[$T]) -> Result<(), Self::Error> {
                self.lock(|bus| bus.write(words))
            }
        }

        impl<BUS: blocking::spi::Transfer<$T>> blocking::spi::Transfer<$T> for &CommonBus<BUS> {
            type Error = BUS::Error;

            fn transfer<'w>(&mut self, words: &'w mut [$T]) -> Result<&'w [$T], Self::Error> {
                self.lock(move |bus| bus.transfer(words))
            }
        }

        impl<BUS: spi::FullDuplex<$T>> spi::FullDuplex<$T> for &CommonBus<BUS> {
            type Error = BUS::Error;

            fn read(&mut self) -> nb::Result<$T, Self::Error> {
                self.lock(|bus| bus.read())
            }

            fn send(&mut self, word: $T) -> nb::Result<(), Self::Error> {
                self.lock(|bus| bus.send(word))
            }
        }
        )*
    }
}

spi!(u8, u16, u32, u64);

#[cfg(feature = "thumbv6")]
mod atomic {
    use core::sync::atomic::{AtomicBool, Ordering};

    #[inline(always)]
    pub fn compare_exchange(
        atomic: &AtomicBool,
        current: bool,
        new: bool,
        _success: Ordering,
        _failure: Ordering,
    ) -> Result<bool, bool> {
        cortex_m::interrupt::free(|_cs| {
            let prev = atomic.load(Ordering::Acquire);
            if prev == current {
                atomic.store(new, Ordering::Release);
                Ok(prev)
            } else {
                Err(false)
            }
        })
    }
}

#[cfg(not(feature = "thumbv6"))]
mod atomic {
    use core::sync::atomic::{AtomicBool, Ordering};

    #[inline(always)]
    pub fn compare_exchange(
        atomic: &AtomicBool,
        current: bool,
        new: bool,
        success: Ordering,
        failure: Ordering,
    ) -> Result<bool, bool> {
        atomic.compare_exchange(current, new, success, failure)
    }
}

/// Provides a method of generating a shared bus.
///
/// ## Args:
/// * `bus` - The actual bus that should be shared
/// * `T` - The full type of the bus that is being shared.
///
/// ## Example:
/// ```rust
/// let bus: I2C = ();
/// let manager = shared_bus_rtic::new!(bus, I2C);
///
/// let device = Device::new(manager.acquire());
/// ```
#[macro_export]
macro_rules! new {
    ($bus:ident, $T:ty) => {
        unsafe {
            static mut _MANAGER: core::mem::MaybeUninit<shared_bus_rtic::CommonBus<$T>> =
                core::mem::MaybeUninit::uninit();
            _MANAGER = core::mem::MaybeUninit::new(shared_bus_rtic::CommonBus::new($bus));
            &*_MANAGER.as_ptr()
        };
    };
}
