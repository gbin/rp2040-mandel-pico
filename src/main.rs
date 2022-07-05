//! # Pico PWM Blink Example
//!
//! Fades the LED on a Pico board using the PWM peripheral.
//!
//! This will fade in/out the LED attached to GP25, which is the pin the Pico
//! uses for the on-board LED.
//!
//! See the `Cargo.toml` file for Copyright and license details.

#![no_std]
#![no_main]

mod mandel;
extern crate core;

use core::mem;
// The macro for our start-up function
use rp_pico::entry;

// GPIO traits
use embedded_hal::digital::v2::{InputPin, OutputPin};

// Time handling traits
use embedded_time::rate::*;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// Pull in any important traits
use rp_pico::hal::prelude::*;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use rp_pico::hal::gpio::FunctionSpi;

// A shorter alias for the Hardware Abstraction Layer, which provides
// higher-level drivers.
use rp_pico::hal;

use embedded_hal::spi::MODE_0;

use st7789::ST7789;

use fixed::types::{I16F16, I9F7};

use crate::mandel::{GRAPHIC_BUFFER_SIZE, SCREEN_HEIGHT, SCREEN_WIDTH};
use embedded_graphics_core::draw_target::DrawTarget;
use embedded_graphics_core::pixelcolor::Rgb565;
use rp_pico::hal::multicore::Multicore;
use rp_pico::hal::multicore::Stack;
use rp_pico::hal::sio::Sio;
use rp_pico::hal::{pac, Spi};

use crate::mandel::tools::Rectangle;

const SCREEN_FREQUENCY_HZ: u32 = 125_000_000u32;
const SCREEN_BAUDRATE: u32 = 16_000_000u32;

enum Protocol {
    ReadyMsg,
    DoneMsg,
}

/// entry point for second core
static mut CORE1_STACK: Stack<4096> = Stack::new();
fn core1_task(buffer_ptr: usize) -> ! {
    const CORE1_SCREEN_REGION: Rectangle<u16> =
        Rectangle::new(0, SCREEN_WIDTH / 2, SCREEN_HEIGHT, SCREEN_WIDTH);

    let pac = unsafe { pac::Peripherals::steal() };
    let mut sio = Sio::new(pac.SIO);
    let buff: &mut [u16; GRAPHIC_BUFFER_SIZE] =
        unsafe { &mut *(buffer_ptr as *mut [u16; GRAPHIC_BUFFER_SIZE]) };
    let mut mr: Rectangle<f32> = Rectangle::default();
    loop {
        sio.fifo.write(Protocol::ReadyMsg as u32);
        unsafe {
            mr.bx = mem::transmute::<u32, f32>(sio.fifo.read_blocking());
            mr.ex = mem::transmute::<u32, f32>(sio.fifo.read_blocking());
            mr.by = mem::transmute::<u32, f32>(sio.fifo.read_blocking());
            mr.ey = mem::transmute::<u32, f32>(sio.fifo.read_blocking());
        }

        mandel::draw_on_buffer_dispatch(mr, CORE1_SCREEN_REGION, buff);
        sio.fifo.write(Protocol::DoneMsg as u32);
    }
}

/// Entry point to our bare-metal application.
///
/// The `#[entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables are initialised.
///
/// The function configures the RP2040 peripherals.
#[entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut peripherals = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(peripherals.WATCHDOG);

    // Configure the clocks
    //
    // The default is to generate a 125 MHz system clock
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

    // The single-cycle I/O block controls our GPIO pins
    let mut sio = hal::Sio::new(peripherals.SIO);

    // Multicore spawning
    let mut mc = Multicore::new(&mut peripherals.PSM, &mut peripherals.PPB, &mut sio.fifo);
    let cores = mc.cores();
    let core1 = &mut cores[1];
    // end of multicore spawning

    // Set the pins up according to their function on this particular board
    let pins = rp_pico::Pins::new(
        peripherals.IO_BANK0,
        peripherals.PADS_BANK0,
        sio.gpio_bank0,
        &mut peripherals.RESETS,
    );

    // switch on the screen
    let mut pwr = pins.gpio22.into_push_pull_output();
    pwr.set_high().unwrap(); // needed
    let mut bl = pins.gpio4.into_push_pull_output();
    bl.set_high().unwrap(); // init the screen with no backlight so we cannot see the ugly memory dump at statup

    // Setup SPI on the GP0..GP5 pins
    let _ = pins.gpio2.into_mode::<FunctionSpi>();
    let _ = pins.gpio3.into_mode::<FunctionSpi>();
    let mut rst = pins.gpio0.into_push_pull_output();
    rst.set_low().unwrap();
    let mut dc = pins.gpio1.into_push_pull_output();
    dc.set_low().unwrap();
    let mut cs = pins.gpio5.into_push_pull_output();
    cs.set_high().unwrap();

    // Our LED output
    let mut led_pin = pins.led.into_push_pull_output();
    // Our button input
    let button_pin = pins.gpio6.into_pull_up_input();

    let spi = Spi::<_, _, 8>::new(peripherals.SPI0).init(
        &mut peripherals.RESETS,
        SCREEN_FREQUENCY_HZ.Hz(),
        SCREEN_BAUDRATE.Hz(),
        &MODE_0,
    );

    let di = display_interface_spi::SPIInterface::new(spi, dc, cs);

    let mut buffer: [u16; GRAPHIC_BUFFER_SIZE] = [0; GRAPHIC_BUFFER_SIZE];
    let buff_ptr = buffer.as_ptr() as usize;

    let _test = core1.spawn(unsafe { &mut CORE1_STACK.mem }, move || {
        core1_task(buff_ptr)
    });

    let mut lcd = ST7789::new(di, rst, SCREEN_HEIGHT as u16, SCREEN_WIDTH as u16);
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());
    lcd.init(&mut delay);
    lcd.clear(Rgb565::new(0, 0, 0));

    let mut mr = Rectangle {
        bx: -2.00f32,
        ex: 0.47f32,
        by: -1.12f32,
        ey: 1.12f32,
    };
    const CORE0_SCREEN_REGION: Rectangle<u16> = Rectangle {
        bx: 0,
        by: 0,
        ex: SCREEN_HEIGHT,
        ey: SCREEN_WIDTH / 2,
    };
    loop {
        sio.fifo.read_blocking(); // todo assert this it READY
        unsafe {
            sio.fifo.write(mem::transmute::<f32, u32>(mr.bx));
            sio.fifo.write(mem::transmute::<f32, u32>(mr.ex));
            sio.fifo.write(mem::transmute::<f32, u32>(mr.by));
            sio.fifo.write(mem::transmute::<f32, u32>(mr.ey));
        }
        mandel::draw_on_buffer_dispatch(mr, CORE0_SCREEN_REGION, &mut buffer);
        sio.fifo.read_blocking();

        // those constants are arbitrary offsets for the LCD (still haven't found where they are coming from or what the exact coordinate system is supposed to be).
        lcd.set_pixels(52, 40, SCREEN_HEIGHT + 51, SCREEN_WIDTH + 39, buffer);

        // Run forever, setting the LED according to the button
        loop {
            if button_pin.is_low().unwrap() {
                led_pin.set_high().unwrap();
                // change the drawing region
                mr.bx = mr.bx / 2.0 + 0.2;
                mr.ex = mr.ex / 2.0 + 0.2;
                mr.by = mr.by / 2.0 + 0.2;
                mr.ey = mr.ey / 2.0 + 0.2;
                break;
            } else {
                led_pin.set_low().unwrap();
            }
        }
    }
}
