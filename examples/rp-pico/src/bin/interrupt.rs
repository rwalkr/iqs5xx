//! IQS5xx capacitive touchpad controller device driver interrupt example
//!
//! Configures interrupts on the the IQS5xx RDY pin and reads reports and
//! logs them via defmt in the interrupt handler

#![no_std]
#![no_main]

// Provide an alias for our BSP so we can switch targets quickly.
use rp_pico as bsp;

use bsp::entry;
use defmt::*;
use defmt_rtt as _;
use fugit::RateExtU32;
use panic_probe as _;
use core::cell::RefCell;
use critical_section::Mutex;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    gpio,
    pac,
    pac::interrupt,
    sio::Sio,
    watchdog::Watchdog,
};

type RdyPin = gpio::Pin<gpio::bank0::Gpio3, gpio::FloatingInput>;
type RstPin = gpio::Pin<gpio::bank0::Gpio2, gpio::PushPullOutput>;
type I2CSDAPin = gpio::Pin<gpio::bank0::Gpio0, gpio::FunctionI2C>;
type I2CSCLPin = gpio::Pin<gpio::bank0::Gpio1, gpio::FunctionI2C>;
type IQS5xx = iqs5xx::IQS5xx<bsp::hal::I2C<pac::I2C0, (I2CSDAPin, I2CSCLPin)>, RdyPin, RstPin>;
static GLOBAL_DEV: Mutex<RefCell<Option<IQS5xx>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let clocks = init_clocks_and_plls(
        bsp::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Configure I²C interface
    let sda_pin = pins.gpio0.into_mode::<gpio::FunctionI2C>();
    let scl_pin = pins.gpio1.into_mode::<gpio::FunctionI2C>();
    let i2c = bsp::hal::I2C::i2c0(
        pac.I2C0,
        sda_pin,
        scl_pin,
        400.kHz(),
        &mut pac.RESETS,
        &clocks.system_clock,
    );

    // create ready pin handler - using IRQs and WFI
    let rdy_pin: RdyPin = pins.gpio3.into_floating_input();
    rdy_pin.set_interrupt_enabled(gpio::Interrupt::EdgeHigh, true);
    let rst_pin: RstPin = pins.gpio2.into_push_pull_output();

    let mut iqs = IQS5xx::new(i2c, iqs5xx::DEFAULT_I2C_ADDR, rdy_pin, rst_pin);

    info!("Starting");

    info!("IQS reset!");
    iqs.reset(&mut delay).unwrap();
    info!("IQS reset done!");

    info!("IQS init!");
    iqs.poll_ready(&mut delay).unwrap();
    iqs.init().unwrap();

    // show device information
    let res = iqs.transact(&mut delay, |iqs| {
        let info = iqs.get_info()?;
        let active_timeout = iqs.read_reg_u8(0x584).unwrap();
        let idle_touch_timeout = iqs.read_reg_u8(0x585).unwrap();
        let idle_timeout = iqs.read_reg_u8(0x586).unwrap();
        let i2c_timeout = iqs.read_reg_u8(0x58a).unwrap();
        Ok((info, active_timeout, idle_touch_timeout, idle_timeout, i2c_timeout))
    });
    match res {
        Ok((ver, active_timeout, idle_touch_timeout, idle_timeout, i2c_timeout)) => {
            info!(
                "IQS: {:04x}, {:04x}, {}.{:02} {:02x}",
                ver.product_number, ver.project_number, ver.major_ver, ver.minor_ver, ver.bootloader_status
            );
            info!(
                "IQS: {}, {}, {}, {}",
                active_timeout, idle_touch_timeout, idle_timeout, i2c_timeout);
        }
        Err(_) => {
            info!("retry (V)");
        }
    }

    // move to interrupt handler
    critical_section::with(|cs| {
        GLOBAL_DEV.borrow(cs).replace(Some(iqs));
    });

    // enable interrupts
    unsafe {
        pac::NVIC::unmask(pac::Interrupt::IO_IRQ_BANK0);
    }

    // all further action takes place in interrupt handler
    loop {
        cortex_m::asm::wfi();
    }
}

#[interrupt]
fn IO_IRQ_BANK0() {
    static mut IQS: Option<IQS5xx> = None;

    if IQS.is_none() {
        critical_section::with(|cs| {
            *IQS = GLOBAL_DEV.borrow(cs).take();
        });
    }

    if let Some(iqs) = IQS {
        // clear the IRQ
        iqs.clear_irq(|pin: &mut RdyPin| {
            if pin.interrupt_status(gpio::Interrupt::EdgeHigh) {
                pin.clear_interrupt(gpio::Interrupt::EdgeHigh);
            }
        });

        // read the report if available
        let res = iqs.try_transact(|iqs| { iqs.get_report() });
        match res {
            Ok(Some(report)) => {
                info!(
                    "{:02x}:{:02x}:{:02x}:{:02x}, {}: {}, {}",
                    report.events0, report.events1, report.sys_info0, report.sys_info1,
                    report.num_fingers, report.rel_x, report.rel_y
                );
                for i in 0..report.num_fingers as usize {
                    let t = &report.touches[i];
                    info!("{},{} : {} : {}", t.abs_x, t.abs_y, t.strength, t.size);
                }

                // interpret report as event
                let event = iqs5xx::Event::from(&report);
                if event != iqs5xx::Event::None {
                    info!("Event: {}", event);
                }
            }
            Ok(None) => {}
            Err(_) => {
                info!("retry (R)");
            }
        }
    }
}

