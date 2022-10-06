#![no_std]
#![no_main]

use embedded_hal::digital::v2::ToggleableOutputPin;
use rp_pico;
use rp_pico::entry;

use rp_pico::hal::gpio::bank0::Gpio4;
use rp_pico::hal::gpio::bank0::Gpio5;
use rp_pico::hal::gpio::FunctionI2C;

use rp_pico::hal::multicore::{Multicore, Stack};

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use rp_pico::hal::pac;

// A shorter alias for the Hardware Abstraction Layer, which provides
// higher-level drivers.
use rp_pico::hal;

use smart_leds::colors;
use smart_leds::SmartLedsWrite;
use tca9555::Tca9555;
// USB Device support
use usb_device::{class_prelude::*, prelude::*};

use usbd_hid::descriptor::KeyboardReport;
// USB Human Interface Device (HID) Class support
use usbd_hid::descriptor::generator_prelude::*;
use usbd_hid::hid_class::HIDClass;

static mut CORE1_STACK: Stack<4096> = Stack::new();

fn handle_button_press(
    mut tca: Tca9555<
        hal::I2C<
            rp_pico::pac::I2C0,
            (
                hal::gpio::Pin<Gpio4, FunctionI2C>,
                hal::gpio::Pin<Gpio5, FunctionI2C>,
            ),
        >,
    >,
    mut leds: apa102_spi::Apa102<hal::Spi<hal::spi::Enabled, pac::SPI0, 8>>,
) -> ! {
    let perhipherals = unsafe { pac::Peripherals::steal() };
    let mut sio = hal::Sio::new(perhipherals.SIO);

    loop {
        let mut colours = [colors::BLACK; 16];
        let pressed_buttons = tca.read_all().unwrap();
        sio.fifo.write_blocking(pressed_buttons.into());
        for i in 0..16 {
            if pressed_buttons & (0b1 << i) > 0 {
                colours[i] = colors::GREEN;
            }
        }

        leds.write(colours.into_iter()).unwrap();
    }
}

#[entry]
fn main() -> ! {
    let mut peripherals = pac::Peripherals::take().unwrap();
    let mut watchdog = hal::Watchdog::new(peripherals.WATCHDOG);

    let clocks = hal::clocks::init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        peripherals.XOSC,
        peripherals.CLOCKS,
        peripherals.PLL_SYS,
        peripherals.PLL_USB,
        &mut peripherals.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut sio = hal::Sio::new(peripherals.SIO);

    let pins = rp_pico::Pins::new(
        peripherals.IO_BANK0,
        peripherals.PADS_BANK0,
        sio.gpio_bank0,
        &mut peripherals.RESETS,
    );
    let _led_res = pins.led.into_push_pull_output().toggle();

    // Set up the USB driver
    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        peripherals.USBCTRL_REGS,
        peripherals.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut peripherals.RESETS,
    ));

    // Set up the USB HID Class Device driver, providing Keyboard Reports
    let mut usb_hid = HIDClass::new(&usb_bus, KeyboardReport::desc(), 8);

    // Create a USB device with a fake VID and PID
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27da))
        .manufacturer("Fake company")
        .product("RGB_Keyboard")
        .serial_number("TEST")
        .build();

    // Setup CORE1 peripherals
    let sda_pin = pins.gpio4.into_mode::<rp_pico::hal::gpio::FunctionI2C>();
    let scl_pin = pins.gpio5.into_mode::<rp_pico::hal::gpio::FunctionI2C>();

    let i2c = rp_pico::hal::I2C::i2c0(
        peripherals.I2C0,
        sda_pin,
        scl_pin,
        embedded_time::rate::Hertz::new(10_000_u32),
        &mut peripherals.RESETS,
        clocks.system_clock,
    );

    // Keyboard driver
    let mut tca = Tca9555::new(i2c, tca9555::DeviceAddr::default());
    tca.set_port_0_polarity_invert(u8::MAX).unwrap();
    tca.set_port_1_polarity_invert(u8::MAX).unwrap();

    let _mosi = pins.gpio19.into_mode::<rp_pico::hal::gpio::FunctionSpi>();
    let _sclk = pins.gpio18.into_mode::<rp_pico::hal::gpio::FunctionSpi>();
    let _cs = pins.gpio17.into_mode::<rp_pico::hal::gpio::FunctionSpi>();
    let spi = rp_pico::hal::Spi::<_, _, 8>::new(peripherals.SPI0);

    let spi = spi.init(
        &mut peripherals.RESETS,
        embedded_time::rate::Hertz::new(3_000_000_u32),
        embedded_time::rate::Hertz::new(96_000_u32),
        &embedded_hal::spi::MODE_0,
    );

    // Dotmatrix LED driver
    let apa102 = apa102_spi::Apa102::new_with_custom_postamble(spi, 4, true);

    // Setup and spawn CORE1 related functionality
    let mut mc = Multicore::new(&mut peripherals.PSM, &mut peripherals.PPB, &mut sio.fifo);
    let core1 = &mut mc.cores()[1];

    // SIO is stolen in CORE1 setup, so ensure no further use of SIO is required
    let _res = core1.spawn(unsafe { &mut CORE1_STACK.mem }, || {
        handle_button_press(tca, apa102)
    });

    // Back in CORE0, begin comms with host USB device.
    loop {
        usb_dev.poll(&mut [&mut usb_hid]);

        let mut keycodes: [u8; 6] = [0, 0, 0, 0, 0, 0];
        if sio.fifo.read_blocking() & (0b1 << 1) > 0 {
            keycodes[0] = 80; //left arrow
        }

        let report = KeyboardReport {
            reserved: 0,
            modifier: 0,
            leds: 0,
            keycodes,
        };
        usb_hid.push_input(&report).unwrap_or(0);
    }
}
