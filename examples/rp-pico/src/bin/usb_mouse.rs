//! IQS5xx capacitive touchpad controller device driver USB mouse example
//!
//! Configures the RP2040 as a USB HID device and translates the reports from
//! the IQS5xx to USB HID mouse packets

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

    // USB Device support
    use usb_device::{class_prelude::*, prelude::*};

    // USB Human Interface Device (HID) Class support
    use usbd_hid::descriptor::generator_prelude::*;
    use usbd_hid::descriptor::MouseReport;
    use usbd_hid::hid_class::HIDClass;
    
    type RdyPin = gpio::Pin<gpio::bank0::Gpio3, gpio::FloatingInput>;
    type RstPin = gpio::Pin<gpio::bank0::Gpio2, gpio::PushPullOutput>;
    type I2CSDAPin = gpio::Pin<gpio::bank0::Gpio0, gpio::FunctionI2C>;
    type I2CSCLPin = gpio::Pin<gpio::bank0::Gpio1, gpio::FunctionI2C>;
    type IQS5xx = iqs5xx::IQS5xx<hal::I2C<pac::I2C0, (I2CSDAPin, I2CSCLPin)>, RdyPin, RstPin>;

    type UsbClass = HIDClass<'static, hal::usb::UsbBus>;
    type UsbDevice = usb_device::device::UsbDevice<'static, hal::usb::UsbBus>;
    static mut USB_BUS: Option<UsbBusAllocator<hal::usb::UsbBus>> = None;

    #[shared]
    struct Shared {
        iqs: IQS5xx,
        usb_dev: UsbDevice,
        usb_hid: UsbClass,
    }

    #[local]
    struct Local {
        pressed_buttons: u8
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

        // Set up the USB driver
        let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
            c.device.USBCTRL_REGS,
            c.device.USBCTRL_DPRAM,
            clocks.usb_clock,
            true,
            &mut resets,
        ));
        let usb_bus = unsafe {
            // Note (safety): This is safe as interrupts haven't been started yet
            USB_BUS = Some(usb_bus);
            USB_BUS.as_ref().unwrap()
        };
        let usb_hid = HIDClass::new(usb_bus, MouseReport::desc(), 60);
        // Create a USB device with a fake VID and PID
        let usb_dev = UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x16c0, 0x27da))
            .manufacturer("None")
            .product("iqs5xx Mouse")
            .serial_number("TEST")
            .device_class(0)
            .build();

        info!("IQS reset!");
        iqs.reset(&mut delay).unwrap();
        info!("IQS reset done!");

        info!("IQS init!");
        iqs.poll_ready(&mut delay).unwrap();
        iqs.init().unwrap();

        // show device information
        let res = iqs.transact(&mut delay, |iqs| {
            let info = iqs.get_info()?;
            let tap_time = iqs.read_tap_time()?;
            let hold_time = iqs.read_hold_time()?;

            iqs.write_hold_time(100u16)?;
            Ok((info, tap_time, hold_time))
        });
        match res {
            Ok((info, tap_time, hold_time)) => {
                info!(
                    "IQS: {:04x}, {:04x}, {}.{:02} {:02x}",
                    info.product_number, info.project_number, info.major_ver, info.minor_ver, info.bootloader_status
                );
                info!("Tap: {}", tap_time);
                info!("Hold: {}", hold_time);
            }
            Err(_) => {
                info!("retry (V)");
            }
        }

        let shared = Shared {
            iqs,
            usb_dev,
            usb_hid,
        };
        let local = Local {
            pressed_buttons: 0u8
        };
        (shared, local, init::Monotonics())
    }

    #[task(binds = USBCTRL_IRQ, priority = 2, shared = [usb_dev, usb_hid])]
    fn usbctrl(c: usbctrl::Context) {
        let usb_dev = c.shared.usb_dev;
        let usb_hid = c.shared.usb_hid;
        (usb_dev, usb_hid).lock(|usb_dev, usb_hid| {
            usb_dev.poll(&mut [usb_hid]);
        });
    }

    #[task(binds = IO_IRQ_BANK0, priority = 2, shared = [iqs], local = [pressed_buttons])]
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
                    let event = iqs5xx::Event::from(&report);
                    info!("Event: {}", event);
                    let is_rhs = report.num_fingers == 1 && report.touches[0].abs_x > 900;
                    match event {
                        // interpret right hand edge as scroll up / down
                        iqs5xx::Event::Move{x: _, y: rel_y} | iqs5xx::Event::PressHold{x: _, y: rel_y} if is_rhs => {
                            let mut rep = make_report();
                            rep.wheel = -rel_y as i8;
                            send_report::spawn(rep).ok();
                        }
                        iqs5xx::Event::Move{x, y} => {
                            let mut rep = make_report();
                            rep.x = x as i8;
                            rep.y = y as i8;
                            send_report::spawn(rep).ok();
                        }
                        iqs5xx::Event::SingleTap{ .. } => {
                            let mut rep1 = make_report();
                            rep1.buttons = 1;
                            send_report::spawn(rep1).ok();
                            let rep2 = make_report();
                            send_report::spawn(rep2).ok();
                        },
                        iqs5xx::Event::PressHold{x, y} => {
                            let mut rep = make_report();
                            rep.x = x as i8;
                            rep.y = y as i8;
                            rep.buttons = 1;
                            send_report::spawn(rep).ok();
                        },
                        iqs5xx::Event::TwoFingerTap => {
                            let mut rep1 = make_report();
                            rep1.buttons = 2;
                            send_report::spawn(rep1).ok();
                            let rep2 = make_report();
                            send_report::spawn(rep2).ok();
                        },
                        iqs5xx::Event::Scroll{x, y: _} if x != 0 => {
                            let mut rep = make_report();
                            rep.pan = x as i8;
                            send_report::spawn(rep).ok();
                        },
                        iqs5xx::Event::Scroll{x: _, y} if y != 0 => {
                            let mut rep = make_report();
                            rep.wheel = -y as i8;
                            send_report::spawn(rep).ok();
                        },
                        _ => { }
                    };
                }
                Ok(None) => {}
                Err(_) => {
                    info!("retry (R)");
                }
            }
        });
    }

    #[task(shared = [usb_hid], capacity = 8)]
    fn send_report(
        c: send_report::Context,
        report: MouseReport,
    ) {
        let mut usb_hid = c.shared.usb_hid;
        usb_hid.lock(|usb_hid| {
            let res = usb_hid.push_input(&report);
            match res {
                Ok(_) => {},
                Err(UsbError::WouldBlock) => {
                    // no bytes written - rescedule to retry after next interrupt
                    send_report::spawn(report).ok();
                },
                Err(_) => {
                    warn!("Failed to send HID report");
                }
            }
        });
    }

    fn make_report() -> MouseReport {
        MouseReport {
            x: 0,
            y: 0,
            buttons: 0,
            wheel: 0,
            pan: 0,
        }
    }
}
