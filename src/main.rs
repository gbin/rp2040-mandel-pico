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
use rp_pico::hal::{pac, Spi};
use rp_pico::hal::gpio::FunctionSpi;

// A shorter alias for the Hardware Abstraction Layer, which provides
// higher-level drivers.
use rp_pico::hal;

use embedded_hal::spi::{MODE_0, MODE_1, MODE_3};
use embedded_time::rate::*;

// The minimum PWM value (i.e. LED brightness) we want
const LOW: u16 = 0;

// The maximum PWM value (i.e. LED brightness) we want
const HIGH: u16 = 25000;

use st7789::ST7789;
use st7789::Orientation;
use display_interface_spi::SPIInterface;
use rp_pico::hal::pio::PinState::Low;

const MAX_MANDEL_ITERATION:i32 = 100;

// Mandel is in the square:
// x between (-2.00, 0.47)
// y between (-1.12, 1.12)
fn mandel(x:f64, y:f64) -> i32 {
    let mut u = 0.0;
    let mut v= 0.0;
    let mut u2 = 0.0;
    let mut v2 = 0.0;
    let mut k=0;
    while k< MAX_MANDEL_ITERATION && (u2+v2<4.0){
        v = 2.0 * u * v + y;
        u = u2 - v2 + x;
        u2 = u * u;
        v2 = v * v;
        k = k + 1;
    }
    k
}

const SCREEN_FREQUENCY_HZ: u32 = 125_000_000u32;
const SCREEN_BAUDRATE: u32 = 16_000_000u32;

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
    let sio = hal::Sio::new(peripherals.SIO);

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
    bl.set_high().unwrap(); // needed

    // Setup SPI on the GP0..GP5 pins
    let _ = pins.gpio2.into_mode::<FunctionSpi>();
    let _ = pins.gpio3.into_mode::<FunctionSpi>();
    let mut rst = pins.gpio0.into_push_pull_output();
    rst.set_low().unwrap();
    let mut dc = pins.gpio1.into_push_pull_output();
    dc.set_low().unwrap();
    let mut cs = pins.gpio5.into_push_pull_output();
    cs.set_high().unwrap();

    let spi = Spi::<_, _, 8>::new(peripherals.SPI0).init(&mut peripherals.RESETS, SCREEN_FREQUENCY_HZ.Hz(), SCREEN_BAUDRATE.Hz(), &MODE_0);

    let di = display_interface_spi::SPIInterface::new(spi, dc, cs);

    const h:u16 = 135;
    const w:u16 = 240;

    let mut lcd = ST7789::new(di, rst, h, w);

    // The delay object lets us wait for specified amounts of time (in
    // milliseconds)
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());

    //lcd.hard_reset(&mut delay);
    lcd.init(&mut delay);

    for px in 0..h {
        let y = (2.24 / h as f64) * (px as f64) - 1.12;
        for py in 0..w {
// x between (-2.00, 0.47)
// y between (-1.12, 1.12)
            let x = (2.47 / w as f64) * (py as f64) - 2.00;
            let iteration = mandel(x, y);
            if iteration == MAX_MANDEL_ITERATION {
                lcd.set_pixel(px+52, py+40, 0xFFFF);
            } else {
                lcd.set_pixel(px+52, py+40, (iteration%0xffff) as u16);
            }
        }
    }

    // Our LED output
    let mut led_pin = pins.led.into_push_pull_output();

    // Our button input
    let button_pin = pins.gpio6.into_pull_up_input();

    // Run forever, setting the LED according to the button
    loop {
        if button_pin.is_low().unwrap() {
            led_pin.set_high().unwrap();
        } else {
            led_pin.set_low().unwrap();
        }
    }
    /*
    // Init PWMs
    let mut pwm_slices = hal::pwm::Slices::new(peripherals.PWM, &mut peripherals.RESETS);

    // Configure PWM4
    let pwm = &mut pwm_slices.pwm4;
    pwm.set_ph_correct();
    pwm.enable();

    // Output channel B on PWM4 to the LED pin
    let channel = &mut pwm.channel_b;
    channel.output_to(pins.led);

    // Infinite loop, fading LED up and down
    loop {
        // Ramp brightness up
        for i in (LOW..=HIGH).skip(100) {
            delay.delay_us(8);
            channel.set_duty(i);
        }

        // Ramp brightness down
        for i in (LOW..=HIGH).rev().skip(100) {
            delay.delay_us(8);
            channel.set_duty(i);
        }

        delay.delay_ms(500);
    }*/
}