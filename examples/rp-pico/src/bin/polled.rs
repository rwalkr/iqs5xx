//! IQS5xx capacitive touchpad controller device driver polling example
//!
//! Polls the IQS5xx device in the main loop for reports and
//! logs them via defmt

#![no_std]
#![no_main]

// Provide an alias for our BSP so we can switch targets quickly.
use rp_pico as bsp;

use bsp::entry;
use defmt::*;
use defmt_rtt as _;
use fugit::RateExtU32;
use panic_probe as _;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    gpio,
    pac,
    sio::Sio,
    watchdog::Watchdog,
};

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
    let rdy_pin = pins.gpio3.into_floating_input();        
    let rst_pin = pins.gpio2.into_push_pull_output();
    let mut iqs = iqs5xx::IQS5xx::new(i2c, iqs5xx::DEFAULT_I2C_ADDR, rdy_pin, rst_pin);

    info!("Starting");

    info!("IQS reset!");
    iqs.reset(&mut delay).unwrap();

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

    // show reports
    loop {
        let res = iqs.transact(&mut delay, |iqs| { iqs.get_report() });
        match res {
            Ok(report) => {
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
            Err(_) => {
                info!("retry (R)");
            }
        }
    }
}

