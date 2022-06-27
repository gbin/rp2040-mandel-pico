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

// The macro for our start-up function
use rp_pico::entry;

// GPIO traits
use embedded_hal::digital::v2::{InputPin, OutputPin};


// GPIO traits
use embedded_hal::PwmPin;

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

use embedded_hal::spi::{MODE_0, MODE_1, MODE_3};
use embedded_time::rate::*;

use st7789::ST7789;
use st7789::Orientation;
use display_interface_spi::SPIInterface;
use fixed::FixedI16;
use rp_pico::hal::pio::PinState::Low;
use mandel::MAX_MANDEL_ITERATION;

use fixed::types::{I0F16, I16F0, I16F16, I9F7};
type FP = I9F7;


use embedded_graphics_core::pixelcolor::Rgb565;
use embedded_graphics_core::draw_target::DrawTarget;
use rp_pico::hal::{pac, Spi};
use rp_pico::hal::gpio::Pins;
use rp_pico::hal::sio::Sio;
use rp_pico::hal::multicore::Multicore;
use rp_pico::hal::multicore::Stack;
use crate::mandel::{SCREEN_HEIGHT, SCREEN_WIDTH, GRAPHIC_BUFFER_SIZE };

const SCREEN_FREQUENCY_HZ: u32 = 125_000_000u32;
const SCREEN_BAUDRATE: u32 = 16_000_000u32;

/// entry point for second core
static mut CORE1_STACK: Stack<4096> = Stack::new();
fn core1_task() -> ! { loop{} }

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

    /// Multicore spawning
    let mut mc = Multicore::new(&mut peripherals.PSM, &mut peripherals.PPB, &mut sio.fifo);
    let cores = mc.cores();
    let core1 = &mut cores[1];
    let _test = core1.spawn( unsafe { &mut CORE1_STACK.mem }, core1_task,);
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

    let spi = Spi::<_, _, 8>::new(peripherals.SPI0).init(&mut peripherals.RESETS, SCREEN_FREQUENCY_HZ.Hz(), SCREEN_BAUDRATE.Hz(), &MODE_0);

    let di = display_interface_spi::SPIInterface::new(spi, dc, cs);


    let mut buffer:[u16; GRAPHIC_BUFFER_SIZE] = [0;GRAPHIC_BUFFER_SIZE];
    let mut lcd = ST7789::new(di, rst, SCREEN_HEIGHT as u16, SCREEN_WIDTH as u16);
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());
    lcd.init(&mut delay);
    lcd.clear(Rgb565::new(255, 0, 0) );

    let mut bx = I16F16::from_num(-2.00);
    let mut ex = I16F16::from_num(0.47);
    let mut by = I16F16::from_num(-1.12);
    let mut ey = I16F16::from_num(1.12);
    loop {
        mandel::draw_on_buffer(bx, by, ex, ey, 0, 0, SCREEN_HEIGHT, SCREEN_WIDTH/2, &mut buffer);
        mandel::draw_on_buffer(bx, by, ex, ey, 0, SCREEN_WIDTH/2, SCREEN_HEIGHT, SCREEN_WIDTH, &mut buffer);
        lcd.set_pixels(52, 40, SCREEN_HEIGHT + 51 , SCREEN_WIDTH +39, buffer);


        // Run forever, setting the LED according to the button
        loop {
            if button_pin.is_low().unwrap() {
                led_pin.set_high().unwrap();
                bx = bx /2 + I16F16::from_num(0.2);
                ex = ex /2 + I16F16::from_num(0.2);
                by = by /2 + I16F16::from_num(0.2);
                ey = ey /2 + I16F16::from_num(0.2);
                break;
            } else {
                led_pin.set_low().unwrap();
            }
        }
    }
}
