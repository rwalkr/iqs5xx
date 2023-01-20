//! IQS5xx capacitive touchpad controller device driver
//!
//! This crate provides a device driver for the Azoteq IQS5xx capacitive
//! touchpad controller.
//!
//! The IQS5xx touchpad connects to the target via I2C and two GPIO pins. The
//! [`embedded_hal`](https://docs.rs/embedded-hal) `blocking::i2c` and
//! `digital::v2` interfaces are used, so should work with any target that
//! provides these.
//!
//! # Examples
//!
//! Examples using the Raspberry PI Pico to report touchpad events via
//! `defmt_rtt` and to act as a basic USB mouse are located in the
//! [`examples/rp-pico`](examples/rp-pico) directory.
//!
//! An IQS5xx device is created with:
//!
//! ```rust
//!     let sda_pin = pins.gpio0.into_mode::<gpio::FunctionI2C>();
//!     let scl_pin = pins.gpio1.into_mode::<gpio::FunctionI2C>();
//!     let i2c = bsp::hal::I2C::i2c0(
//!         pac.I2C0,
//!         sda_pin,
//!         scl_pin,
//!         400.kHz(),
//!         &mut pac.RESETS,
//!         &clocks.system_clock,
//!     );
//!     let rdy_pin = pins.gpio3.into_floating_input();
//!     let rst_pin = pins.gpio2.into_push_pull_output();
//!     let mut iqs = iqs5xx::IQS5xx::new(i2c, iqs5xx::DEFAULT_I2C_ADDR, rdy_pin, rst_pin);
//! ```
//!
//! After initialization, accesses to the IQS5xx device should be wrapped in a
//! call to `iqs.transact()`.  This ensures the IQS5xx device is ready
//! (indicated by the RDY pin) and signals the end of the transaction to the
//! device:
//!
//! ```rust
//!     let res = iqs.transact(&mut delay, |iqs| { iqs.get_report() });
//!     match res {
//!         Ok(report) => {
//!             info!(
//!                 "{:02x}:{:02x}:{:02x}:{:02x}, {}: {}, {}",
//!                 report.events0, report.events1, report.sys_info0, report.sys_info1,
//!                 report.num_fingers, report.rel_x, report.rel_y
//!             );
//!             for i in 0..report.num_fingers as usize {
//!                 let t = &report.touches[i];
//!                 info!("{},{} : {} : {}", t.abs_x, t.abs_y, t.strength, t.size);
//!             }
//!
//!             let event = iqs5xx::Event::from(&report);
//!             if event != iqs5xx::Event::None {
//!                 info!("Event: {}", event);
//!             }
//!         }
//!         Err(_) => {
//!             warn!("Error");
//!         }
//!     }
//! ```
//!
//!
//! The caller can use interrupts on the ready pin to avoid busy waits by
//! polling for events.  The `transact()` function should be called from
//! the interrupt handler or a task scheduled from the interrupt handler.
//! Interrupts should be cleared with `clear_irq()`.

#![no_std]
#![no_main]

use defmt::*;
use embedded_hal::blocking::delay::DelayMs;
use paste;

pub mod registers;

/// Errors produced by the IQS5xx device
#[derive(Debug, Format)]
pub enum Error {
    /// An error accessing the GPIO pins
    GPIOError,
    /// An error accessing the I2C interface
    I2CError,
    /// Timeout waiting for the device to be ready
    Timeout,
}

pub type Result<T> = core::result::Result<T, Error>;

/// IQS5xx driver
pub struct IQS5xx<I2C, RDY, RST> {
    i2c: I2C,
    rdy_pin: RDY,
    rst_pin: RST,
    addr: u8,
}

/// Default I2C device address for IQS5xx devices
pub const DEFAULT_I2C_ADDR: u8 = 0x74;

/// Product and version information of the IQS5xx device
pub struct DeviceInfo {
    /// Contents of the Product Number register
    pub product_number: u16,
    /// Contents of the Project Number register
    pub project_number: u16,
    /// Contents of the Major version register
    pub major_ver: u8,
    /// Contents of the Minor version register
    pub minor_ver: u8,
    /// Contents of the Bootloader status register
    pub bootloader_status: u8,
}

/// Fields describing a touch
pub struct Touch {
    pub abs_x: u16,
    pub abs_y: u16,
    pub strength: u16,
    pub size: u8,
}

impl Touch {
    fn from_iter<'a, I: core::iter::Iterator<Item = &'a u8>>(i: &mut I) -> Touch {
        Touch {
            abs_x: u16_be_from_iter(i),
            abs_y: u16_be_from_iter(i),
            strength: u16_be_from_iter(i),
            size: *i.next().unwrap(),
        }
    }
}

/// Single tap gesture
const GESTURE_EVENTS0_SINGLE_TAP: u8 = 1 << 0;
/// Press hold gesture
const GESTURE_EVENTS0_PRESS_HOLD: u8 = 1 << 1;
/// Swipe left gesture
const GESTURE_EVENTS0_SWIPE_X_N: u8 = 1 << 2;
/// Swipe right gesture
const GESTURE_EVENTS0_SWIPE_X_P: u8 = 1 << 3;
/// Swipe down gesture
const GESTURE_EVENTS0_SWIPE_Y_P: u8 = 1 << 4;
/// Swipe up gesture
const GESTURE_EVENTS0_SWIPE_Y_N: u8 = 1 << 5;
/// Two finger tap
const GESTURE_EVENTS1_TWO_FINGER_TAP: u8 = 1 << 0;
/// Two finger swipe to scroll
const GESTURE_EVENTS1_SCROLL: u8 = 1 << 1;
/// Two finger pinch to zoom
const GESTURE_EVENTS1_ZOOM: u8 = 1 << 2;

/// Status report from trackpad
pub struct Report {
    /// Single finger gesture events (see GESTURE_EVENTS0_xxx)
    pub events0: u8,
    /// Two finger gesture events (see GESTURE_EVENTS0_xxx)
    pub events1: u8,
    /// System info 0 register
    pub sys_info0: u8,
    /// System info 1 register
    pub sys_info1: u8,
    /// Number of active touches
    pub num_fingers: u8,
    /// Rel X value
    pub rel_x: i16,
    /// Rel Y value
    pub rel_y: i16,
    /// Per-touch information
    pub touches: [Touch; 5],
}

/// Events derived from status report
#[derive(Debug, Format, Eq, PartialEq)]
pub enum Event {
    /// No event
    None,
    /// A single finger movement of (x, y)
    Move { x: i16, y: i16 },
    /// A single finger tap at (x, y)
    SingleTap { x: u16, y: u16 },
    /// A single finger hold at (x, y)
    PressHold { x: i16, y: i16 },
    /// A single finger horizontal swipe
    SwipeX(i16),
    /// A single finger vertical swipe
    SwipeY(i16),
    /// A two finger tap
    TwoFingerTap,
    /// A two finger scroll
    Scroll { x: i16, y: i16 },
    /// A two finger zoom
    Zoom(i16),
    /// An invalid event
    Invalid,
}

impl From<&Report> for Event {
    /// Extract events from status report
    fn from(report: &Report) -> Self {
        match (report.events0, report.events1) {
            (0, 0) if report.rel_x == 0 && report.rel_y == 0 => Self::None,
            (0, 0) => Self::Move {
                x: report.rel_x,
                y: report.rel_y,
            },
            (GESTURE_EVENTS0_SINGLE_TAP, 0) => Self::SingleTap {
                x: report.touches[0].abs_x as u16,
                y: report.touches[0].abs_y as u16,
            },
            (GESTURE_EVENTS0_PRESS_HOLD, 0) => Self::PressHold {
                x: report.rel_x,
                y: report.rel_y,
            },
            (GESTURE_EVENTS0_SWIPE_X_P, 0) => Self::SwipeX(report.rel_x),
            (GESTURE_EVENTS0_SWIPE_X_N, 0) => Self::SwipeX(report.rel_x),
            (GESTURE_EVENTS0_SWIPE_Y_P, 0) => Self::SwipeY(report.rel_y),
            (GESTURE_EVENTS0_SWIPE_Y_N, 0) => Self::SwipeY(report.rel_y),
            (0, GESTURE_EVENTS1_TWO_FINGER_TAP) => Self::TwoFingerTap,
            (0, GESTURE_EVENTS1_SCROLL) => Self::Scroll {
                x: report.rel_x,
                y: report.rel_y,
            },
            (0, GESTURE_EVENTS1_ZOOM) => Self::Zoom(report.rel_x),

            _ => Self::Invalid,
        }
    }
}

impl<I2C, RDY, RST> IQS5xx<I2C, RDY, RST> {
    /// Create a new IQS5xx device
    ///
    /// `i2c` is the I2C device
    /// `rdy_pin` is a floating input GPIO
    /// `rst_pin` is a push pull output GPIO
    pub fn new(i2c: I2C, addr: u8, rdy_pin: RDY, rst_pin: RST) -> IQS5xx<I2C, RDY, RST> {
        IQS5xx {
            i2c,
            addr,
            rdy_pin,
            rst_pin,
        }
    }
}

impl<I2C, RDY, RST> IQS5xx<I2C, RDY, RST>
where
    RST: embedded_hal::digital::v2::OutputPin,
{
    /// Reset the IQS5xx device by pulsing the reset pin
    pub fn reset(&mut self, delay: &mut dyn DelayMs<u32>) -> Result<()> {
        self.rst_pin.set_low().map_err(|_| Error::GPIOError)?;
        delay.delay_ms(10);
        self.rst_pin.set_high().map_err(|_| Error::GPIOError)?;
        delay.delay_ms(10);
        Ok(())
    }
}

impl<I2C, RDY, RST> IQS5xx<I2C, RDY, RST>
where
    RDY: embedded_hal::digital::v2::InputPin,
{
    /// Test if the IQS5xx device indicates ready
    ///
    /// Return `true` if ready
    pub fn try_poll_ready(&mut self) -> Result<bool> {
        self.rdy_pin.is_high().map_err(|_| Error::GPIOError)
    }

    /// Wait until the IQS5xx device indicates ready (ready pin high)
    ///
    /// Return `Ok(())` when ready, `Err(Error::Timeout)` if not ready after 100ms
    pub fn poll_ready(&mut self, delay: &mut dyn DelayMs<u32>) -> Result<()> {
        for _ in 0..100 {
            if self.rdy_pin.is_high().map_err(|_| Error::GPIOError)? {
                return Ok(());
            }
            delay.delay_ms(1);
        }
        Err(Error::Timeout)
    }

    /// Wait until the IQS5xx device ready pin is low
    ///
    /// Return `Ok(())` when ready is low, `Err(Error::Timeout)` if not ready after 100ms
    pub fn poll_ready_low(&mut self, delay: &mut dyn DelayMs<u32>) -> Result<()> {
        for _ in 0..100 {
            if self.rdy_pin.is_low().map_err(|_| Error::GPIOError)? {
                return Ok(());
            }
            delay.delay_ms(1);
        }
        Err(Error::Timeout)
    }

    /// Call platform specific function to clear interrupt on ready pin
    ///
    /// This wrapper is needed as the IQS5xx driver owns the pin, so the
    /// platform can't maintain a mutable reference to it
    pub fn clear_irq<F: FnMut(&mut RDY) -> ()>(&mut self, mut f: F) -> () {
        f(&mut self.rdy_pin)
    }
}

impl<I2C, RDY, RST> IQS5xx<I2C, RDY, RST>
where
    I2C: embedded_hal::blocking::i2c::Write,
{
    /// Initialize the IQS5xx device
    pub fn init(&mut self) -> Result<()> {
        // ack reset
        self.write_reg_u8(registers::SYSTEM_CONTROL_0, 1 << 7)
    }

    /// End the communication window with the IQS5xx
    ///
    /// Signal to the devie that sensing and processing can continue
    pub fn end_session(&mut self) -> Result<()> {
        let mut msg = [0u8; 3];
        msg[0..2].copy_from_slice(&registers::END_WINDOW.to_be_bytes());
        self.i2c.write(self.addr, &msg).map_err(|_| Error::I2CError)
    }

    /// Write to a `u8` register
    pub fn write_reg_u8(&mut self, reg_num: u16, value: u8) -> Result<()> {
        let mut msg = [0u8; 3];
        msg[0..2].copy_from_slice(&reg_num.to_be_bytes());
        msg[2] = value;
        self.i2c.write(self.addr, &msg).map_err(|_| Error::I2CError)
    }

    /// Write to a `u16` register
    pub fn write_reg_u16(&mut self, reg_num: u16, value: u16) -> Result<()> {
        let mut msg = [0u8; 4];
        msg[0..2].copy_from_slice(&reg_num.to_be_bytes());
        msg[2..4].copy_from_slice(&value.to_be_bytes());
        self.i2c.write(self.addr, &msg).map_err(|_| Error::I2CError)
    }
}

impl<I2C, RDY, RST> IQS5xx<I2C, RDY, RST>
where
    I2C: embedded_hal::blocking::i2c::WriteRead,
{
    /// Read a `u8` register
    pub fn read_reg_u8(&mut self, reg_num: u16) -> Result<u8> {
        let mut rd_buf = [0u8; 1];
        self.i2c
            .write_read(self.addr, &reg_num.to_be_bytes(), &mut rd_buf)
            .map_err(|_| Error::I2CError)?;
        let r = rd_buf[0];
        Ok(r)
    }

    /// Read a `u16` register
    pub fn read_reg_u16(&mut self, reg_num: u16) -> Result<u16> {
        let mut rd_buf = [0u8; 2];
        self.i2c
            .write_read(self.addr, &reg_num.to_be_bytes(), &mut rd_buf)
            .map_err(|_| Error::I2CError)?;
        let r = u16::from_be_bytes(rd_buf);
        Ok(r)
    }

    /// Read the device information registers
    pub fn get_info(&mut self) -> Result<DeviceInfo> {
        let mut rd_buf = [0u8; 7];
        self.i2c
            .write_read(
                self.addr,
                &registers::PRODUCT_NUMBER_0.to_be_bytes(),
                &mut rd_buf,
            )
            .map_err(|_| Error::I2CError)?;
        let mut r = rd_buf.iter();
        Ok(DeviceInfo {
            product_number: u16_be_from_iter(&mut r),
            project_number: u16_be_from_iter(&mut r),
            major_ver: *r.next().unwrap(),
            minor_ver: *r.next().unwrap(),
            bootloader_status: *r.next().unwrap(),
        })
    }

    /// Read the event report registers
    pub fn get_report(&mut self) -> Result<Report> {
        let mut rd_buf = [0u8; 44];
        self.i2c
            .write_read(
                self.addr,
                &registers::GESTURE_EVENTS.to_be_bytes(),
                &mut rd_buf,
            )
            .map_err(|_| Error::I2CError)?;
        let mut r = rd_buf.iter();
        Ok(Report {
            events0: *r.next().unwrap(),
            events1: *r.next().unwrap(),
            sys_info0: *r.next().unwrap(),
            sys_info1: *r.next().unwrap(),
            num_fingers: *r.next().unwrap(),
            rel_x: i16_be_from_iter(&mut r),
            rel_y: i16_be_from_iter(&mut r),
            touches: [
                Touch::from_iter(&mut r),
                Touch::from_iter(&mut r),
                Touch::from_iter(&mut r),
                Touch::from_iter(&mut r),
                Touch::from_iter(&mut r),
            ],
        })
    }
}

impl<I2C, RDY, RST> IQS5xx<I2C, RDY, RST>
where
    RDY: embedded_hal::digital::v2::InputPin,
    I2C: embedded_hal::blocking::i2c::Write + embedded_hal::blocking::i2c::WriteRead,
{
    /// Perform a transaction with the IQS5xx device
    ///
    /// * Wait for the device to indicate ready
    /// * Call the passed function which may read/write device registers
    /// * Signal the end of the session
    /// * Wait for ready to clear
    ///
    /// The caller can use interrupts on the ready pin to avoid long waits
    /// while polling.  This function should be called from the interrupt
    /// handler or a task scheduled from the interrupt handler.  Interrupts
    /// should be cleared with `clear_irq()`.
    pub fn transact<T, F: FnMut(&mut Self) -> Result<T>>(
        &mut self,
        delay: &mut dyn DelayMs<u32>,
        mut f: F,
    ) -> Result<T> {
        self.poll_ready(delay).unwrap();
        let res = f(self);
        match res {
            Ok(_) => {
                self.end_session().unwrap();
                self.poll_ready_low(delay).unwrap();
            }
            Err(_) => {
                // TODO: retry
            }
        }
        res
    }

    /// Attempt a transaction with the IQS5xx device
    ///
    /// If the device to indicates ready:
    ///
    /// * Call the passed function which may read/write device registers
    /// * Signal the end of the session
    ///
    /// Returns immediately with `Ok(None)` if the device is not ready
    pub fn try_transact<T, F: FnMut(&mut Self) -> Result<T>>(
        &mut self,
        mut f: F,
    ) -> Result<Option<T>> {
        if self.try_poll_ready().unwrap() {
            match f(self) {
                Ok(v) => {
                    self.end_session().unwrap();
                    // self.poll_ready_low(delay).unwrap();
                    Ok(Some(v))
                }
                Err(e) => {
                    // TODO: retry
                    Err(e)
                }
            }
        } else {
            Ok(None)
        }
    }
}

macro_rules! register_read {
    ($name:ident, $sz:tt) => {
        $crate::paste::paste! {
            impl<I2C, RDY, RST> IQS5xx<I2C, RDY, RST>
            where
                I2C: embedded_hal::blocking::i2c::WriteRead,
            {
                #[doc="Read the " [<$name:upper>] " register"]
                pub fn [<read_ $name:lower>](&mut self) -> Result<$sz> {
                    self.[<read_reg_ $sz>](registers::[<$name:upper>])
                }
            }
        }
    };
}

macro_rules! register_write {
    ($name:ident, $sz:tt) => {
        $crate::paste::paste! {
            impl<I2C, RDY, RST> IQS5xx<I2C, RDY, RST>
            where
                I2C: embedded_hal::blocking::i2c::Write,
            {
                #[doc="Write the " [<$name:upper>] " register"]
                pub fn [<write_ $name:lower>](&mut self, v: $sz) -> Result<()> {
                    self.[<write_reg_ $sz>](registers::[<$name:upper>], v)
                }
            }
        }
    };
}

macro_rules! register_acc {
    ($name:ident, $sz:tt, ro) => {
        register_read!($name, $sz);
    };
    ($name:ident, $sz:tt, rw) => {
        register_read!($name, $sz);
        register_write!($name, $sz);
    };
}

register_acc!(REPORT_RATE_ACTIVE, u16, rw);
register_acc!(REPORT_RATE_IDLE_TOUCH, u16, rw);
register_acc!(REPORT_RATE_IDLE, u16, rw);
register_acc!(REPORT_RATE_LP1, u16, rw);
register_acc!(REPORT_RATE_LP2, u16, rw);
register_acc!(TIMEOUT_ACTIVE, u8, rw);
register_acc!(TIMEOUT_IDLE_TOUCH, u8, rw);
register_acc!(TIMEOUT_IDLE, u8, rw);
register_acc!(TIMEOUT_LP1, u8, rw);
register_acc!(REFERENCE_UPDATE_TIME, u8, rw);
register_acc!(SNAP_TIMEOUT, u8, rw);
register_acc!(I2C_TIMEOUT, u8, rw);
register_acc!(SINGLE_FINGER_GESTURES, u8, rw);
register_acc!(MULTI_FINGER_GESTURES, u8, rw);
register_acc!(TAP_TIME, u16, rw);
register_acc!(TAP_DISTANCE, u16, rw);
register_acc!(HOLD_TIME, u16, rw);
register_acc!(SWIPE_INITIAL_TIME, u16, rw);
register_acc!(SWIPE_INITIAL_DISTANCE, u16, rw);
register_acc!(SWIPE_CONSECUTIVE_TIME, u16, rw);
register_acc!(SWIPE_CONSECUTIVE_DISTANCE, u16, rw);
register_acc!(SWIPE_ANGLE, u8, rw);
register_acc!(SCROLL_INITIAL_DISTANCE, u16, rw);
register_acc!(SCROLL_ANGLE, u8, rw);
register_acc!(ZOOM_INITIAL_DISTANCE, u16, rw);
register_acc!(ZOOM_ANGLE, u8, rw);

fn u16_be_from_iter<'a, I: core::iter::Iterator<Item = &'a u8>>(i: &mut I) -> u16 {
    let h = *i.next().unwrap();
    let l = *i.next().unwrap();
    u16::from_be_bytes([h, l])
}

fn i16_be_from_iter<'a, I: core::iter::Iterator<Item = &'a u8>>(i: &mut I) -> i16 {
    let h = *i.next().unwrap();
    let l = *i.next().unwrap();
    i16::from_be_bytes([h, l])
}

// End of file
