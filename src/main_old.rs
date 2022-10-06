#![no_std]
#![no_main]
//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.

mod keycodes;

use cortex_m::{delay::Delay, interrupt::InterruptNumber};
use defmt::info;
// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp_pico as bsp;
// use sparkfun_pro_micro_rp2040 as bsp;

use bsp::{
    entry,
    hal::{
        self,
        gpio::{FunctionI2C, FunctionSpi},
        spi::Enabled,
        usb::UsbBus,
        Clock, Spi, I2C,
    },
    pac::{interrupt, SPI0},
};
use defmt_rtt as _;
use embedded_time::{fixed_point::FixedPoint, rate::Extensions};
use panic_probe as _;

use bsp::hal::{clocks::init_clocks_and_plls, pac, sio::Sio, watchdog::Watchdog};
use smart_leds::SmartLedsWrite;
use tca9555::Tca9555;
use usb_device::{
    class_prelude::UsbBusAllocator,
    prelude::{UsbDevice, UsbDeviceBuilder, UsbVidPid},
};
// USB Human Interface Device (HID) Class support
use usbd_hid::descriptor::{generator_prelude::*, KeyboardReport, MouseReport};
use usbd_hid::hid_class::HIDClass;

/// The USB Device Driver (shared with the interrupt).
// static mut USB_DEVICE: Option<UsbDevice<hal::usb::UsbBus>> = None;

/// The USB Bus Driver (shared with the interrupt).
// static mut USB_BUS: Option<UsbBusAllocator<hal::usb::UsbBus>> = None;

/// The USB Human Interface Device Driver (shared with the interrupt).
// static mut USB_HID: Option<HIDClass<hal::usb::UsbBus>> = None;

#[entry]
fn main() -> ! {
    info!("Program start");
    // let (mut tca, mut leds, mut delay, mut usb_hid) = setup();

    let mut peripherals = pac::Peripherals::take().unwrap();
    let mut watchdog = Watchdog::new(peripherals.WATCHDOG);
    let sio = Sio::new(peripherals.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let clocks = init_clocks_and_plls(
        bsp::XOSC_CRYSTAL_FREQ,
        peripherals.XOSC,
        peripherals.CLOCKS,
        peripherals.PLL_SYS,
        peripherals.PLL_USB,
        &mut peripherals.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let core = pac::CorePeripherals::take().unwrap();
    let mut delay = Delay::new(core.SYST, clocks.system_clock.freq().integer());

    let pins = bsp::Pins::new(
        peripherals.IO_BANK0,
        peripherals.PADS_BANK0,
        sio.gpio_bank0,
        &mut peripherals.RESETS,
    );

    let sda_pin = pins.gpio4.into_mode::<FunctionI2C>();
    let scl_pin = pins.gpio5.into_mode::<FunctionI2C>();

    let i2c = I2C::i2c0(
        peripherals.I2C0,
        sda_pin,
        scl_pin,
        10_000.Hz(),
        &mut peripherals.RESETS,
        clocks.system_clock,
    );

    let mut tca = Tca9555::new(i2c, tca9555::DeviceAddr::default());
    tca.set_port_0_polarity_invert(u8::MAX).unwrap();
    tca.set_port_1_polarity_invert(u8::MAX).unwrap();

    let _mosi = pins.gpio19.into_mode::<FunctionSpi>();
    let _sclk = pins.gpio18.into_mode::<FunctionSpi>();
    let _cs = pins.gpio17.into_mode::<FunctionSpi>();
    let spi = Spi::<_, _, 8>::new(peripherals.SPI0);

    let spi = spi.init(
        &mut peripherals.RESETS,
        3_000_000.Hz(),
        96_000.Hz(),
        &embedded_hal::spi::MODE_0,
    );

    // let mut leds = apa102_spi::Apa102::new_with_custom_postamble(spi, 4, true);

    // Set up the USB driver
    let usb_bus = UsbBusAllocator::new(UsbBus::new(
        peripherals.USBCTRL_REGS,
        peripherals.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut peripherals.RESETS,
    ));

    // Set up the USB HID Class Device driver, providing Keyboard Reports
    let mut usb_human_interface_device = HIDClass::new(&usb_bus, KeyboardReport::desc(), 20);

    // Create a USB device with a fake VID and PID
    let mut usb_device = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0xF055, 0x27da))
        .composite_with_iads()
        .manufacturer("Thomas")
        .product("MacroPad_1")
        .serial_number("SOME_SN")
        .device_class(3)
        .device_protocol(1)
        .build();

    loop {
        let mut colors = [smart_leds::colors::BLACK; 16];
        let lit_buttons = 0; // tca.read_all().unwrap();
                             // delay.delay_ms(200);

        usb_device.poll(&mut [&mut usb_human_interface_device]);
        for i in 0..16 {
            let button_pressed = lit_buttons & (0b1 << i);
            if button_pressed > 0 {
                colors[i] = smart_leds::colors::DARK_ORANGE;
            }
            let rep_up = KeyboardReport {
                modifier: 0,
                reserved: 0,
                leds: 0,
                keycodes: [67, 80, 0, 0, 0, 0],
            };
            usb_human_interface_device
                .pull_raw_output(&mut [0; 64])
                .unwrap_or(0);
            usb_human_interface_device.push_input(&rep_up).unwrap_or(0);
        }
        // delay.delay_ms(4);
        // leds.write(colors.into_iter()).unwrap();
        // usb_hid.pull_raw_output(&mut [0; 64]).unwrap();
    }
}

fn debug_delay(mut delay: Delay) -> Delay {
    delay.delay_ms(200);
    delay
}
