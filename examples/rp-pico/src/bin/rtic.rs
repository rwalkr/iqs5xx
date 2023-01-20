//! IQS5xx capacitive touchpad controller device driver RTIC example
//!
//! Configures interrupts on the the IQS5xx RDY pin and reads reports and
//! logs them via defmt in the interrupt handler

#![no_std]
#![no_main]

use rtic::app;
use panic_probe as _;
use rp_pico as bsp;

#[app(device = rp_pico::hal::pac,
      peripherals = true,
      dispatchers = [DMA_IRQ_0])]
mod app {
    use defmt::*;
    use defmt_rtt as _;
    use fugit::RateExtU32;

    use crate::bsp::hal;
    use crate::bsp::hal::{
        clocks::Clock,
        gpio,
        pac,
    };

    type RdyPin = gpio::Pin<gpio::bank0::Gpio3, gpio::FloatingInput>;
    type RstPin = gpio::Pin<gpio::bank0::Gpio2, gpio::PushPullOutput>;
    type I2CSDAPin = gpio::Pin<gpio::bank0::Gpio0, gpio::FunctionI2C>;
    type I2CSCLPin = gpio::Pin<gpio::bank0::Gpio1, gpio::FunctionI2C>;
    type IQS5xx = iqs5xx::IQS5xx<hal::I2C<pac::I2C0, (I2CSDAPin, I2CSCLPin)>, RdyPin, RstPin>;

    #[shared]
    struct Shared {
        iqs: IQS5xx,
    }

    #[local]
    struct Local {
    }

    #[init]
    fn init(c: init::Context) -> (Shared, Local, init::Monotonics) {
        let mut resets = c.device.RESETS;
        let mut watchdog = hal::Watchdog::new(c.device.WATCHDOG);
        let clocks = hal::clocks::init_clocks_and_plls(
            crate::bsp::XOSC_CRYSTAL_FREQ,
            c.device.XOSC,
            c.device.CLOCKS,
            c.device.PLL_SYS,
            c.device.PLL_USB,
            &mut resets,
            &mut watchdog,
        )
        .ok()
        .unwrap();

        let mut delay = cortex_m::delay::Delay::new(c.core.SYST, clocks.system_clock.freq().to_Hz());

        let sio = hal::Sio::new(c.device.SIO);
        let pins = rp_pico::Pins::new(
            c.device.IO_BANK0,
            c.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut resets,
        );
        
        // Configure I²C interface
        let sda_pin = pins.gpio0.into_mode::<gpio::FunctionI2C>();
        let scl_pin = pins.gpio1.into_mode::<gpio::FunctionI2C>();
        let i2c = hal::I2C::i2c0(
            c.device.I2C0,
            sda_pin,
            scl_pin,
            400.kHz(),
            &mut resets,
            &clocks.system_clock,
        );

        // create ready pin handler - using IRQs and WFI
        let rdy_pin: RdyPin = pins.gpio3.into_floating_input();
        rdy_pin.set_interrupt_enabled(gpio::Interrupt::EdgeHigh, true);
        let rst_pin: RstPin = pins.gpio2.into_push_pull_output();

        let mut iqs = IQS5xx::new(i2c, iqs5xx::DEFAULT_I2C_ADDR, rdy_pin, rst_pin);

        info!("IQS reset!");
        iqs.reset(&mut delay).unwrap();
        info!("IQS reset done!");

        info!("IQS init!");
        iqs.poll_ready(&mut delay).unwrap();
        iqs.init().unwrap();

        // show device information
        let res = iqs.transact(&mut delay, |iqs| {
            let info = iqs.get_info()?;
            Ok(info)
        });
        match res {
            Ok(ver) => {
                info!(
                    "IQS: {:04x}, {:04x}, {}.{:02} {:02x}",
                    ver.product_number, ver.project_number, ver.major_ver, ver.minor_ver, ver.bootloader_status
                );
            }
            Err(_) => {
                info!("retry (V)");
            }
        }

        let shared = Shared {
            iqs
        };
        let local = Local {
        };
        (shared, local, init::Monotonics())
    }


    #[task(binds = IO_IRQ_BANK0, priority = 2, shared = [iqs])]
    fn io_irq(c: io_irq::Context) {
        let mut iqs = c.shared.iqs;

        iqs.lock(|iqs| {
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
        });
    }
}

