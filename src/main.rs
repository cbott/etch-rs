//! Blinks the LED on a Pico board
#![no_std]
#![no_main]

use bsp::entry;
use bsp::hal::gpio;
use core::cell::RefCell;
use critical_section::Mutex;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::InputPin;
use embedded_hal::digital::OutputPin;
use panic_probe as _;

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp_pico as bsp;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    sio::Sio,
    watchdog::Watchdog,
};

// Interrupt macro
use bsp::hal::pac::interrupt;

// Pin types quickly become very long!
// We'll create some type aliases using `type` to help with that
type EncoderAPin = gpio::Pin<gpio::bank0::Gpio13, gpio::FunctionSioInput, gpio::PullUp>;
type EncoderBPin = gpio::Pin<gpio::bank0::Gpio14, gpio::FunctionSioInput, gpio::PullUp>;

/// Since we're always accessing these pins together we'll store them in a tuple.
/// Giving this tuple a type alias means we won't need to use () when putting them
/// inside an Option. That will be easier to read.
type InterruptPins = (EncoderAPin, EncoderBPin);

/// This how we transfer our pins to the Interrupt Handler.
/// We'll have the option hold both using the InterruptPins type.
/// This will make it a bit easier to unpack them later.
static GLOBAL_PINS: Mutex<RefCell<Option<InterruptPins>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
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

    // Define I/O pins
    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut led_a = pins.gpio11.into_push_pull_output();
    let mut led_b = pins.gpio12.into_push_pull_output();
    let mut a_pin = pins.gpio13.into_pull_up_input();
    let mut b_pin = pins.gpio14.into_pull_up_input();

    // Configure interrupts for encoder
    a_pin.set_interrupt_enabled(gpio::Interrupt::EdgeHigh, true);
    b_pin.set_interrupt_enabled(gpio::Interrupt::EdgeHigh, true);

    // Give away our pins by moving them into the `GLOBAL_PINS` variable.
    // We won't need to access them in the main thread again
    critical_section::with(|cs| {
        GLOBAL_PINS.borrow(cs).replace(Some((a_pin, b_pin)));
    });

    // Unmask the IO_BANK0 IRQ so that the NVIC interrupt controller
    // will jump to the interrupt function when the interrupt occurs.
    // We do this last so that the interrupt can't go off while
    // it is in the middle of being configured
    unsafe {
        pac::NVIC::unmask(pac::Interrupt::IO_IRQ_BANK0);
    }

    // Clockwise
    // 00 10 11 01 ..
    let mut prev_a_state = false;
    let mut prev_b_state = false;
    let mut counter: i32 = 0;

    loop {
        let a_state = !a_pin.is_low().unwrap();
        let b_state = !b_pin.is_low().unwrap();

        if a_state && !prev_a_state {
            // Rising edge on pin A
            if b_state {
                // Counterclockwise transition
                counter -= 1;
            } else {
                // Clockwise transition
                counter += 1;
            }
        } else if b_state && !prev_b_state {
            // Rising edge on pin B
            if a_state {
                // Clockwise transition
                counter += 1;
            } else {
                // Counterclockwise transition
                counter -= 1;
            }
        }

        if a_state {
            led_a.set_high().unwrap();
        } else {
            led_a.set_low().unwrap();
        }

        if b_state {
            led_b.set_high().unwrap();
        } else {
            led_b.set_low().unwrap();
        }

        // info!("off!");info!("off!");
        info!("Counter: {}", counter);
        delay.delay_us(1);

        prev_a_state = a_state;
        prev_b_state = b_state;
    }
}

#[interrupt]
fn IO_IRQ_BANK0() {
    // The `#[interrupt]` attribute covertly converts this to `&'static mut Option<InterruptPins>`
    static mut INTERRUPT_PINS: Option<InterruptPins> = None;

    // This is one-time lazy initialisation. We steal the variables given to us
    // via `GLOBAL_PINS`.
    if INTERRUPT_PINS.is_none() {
        critical_section::with(|cs| {
            *INTERRUPT_PINS = GLOBAL_PINS.borrow(cs).take();
        });
    }

    if let Some(gpios) = INTERRUPT_PINS {
        // borrow led and button by *destructuring* the tuple
        // these will be of type `&mut XXPin` so we don't have
        // to move them back into the static after we use them
        let (a_pin, b_pin) = gpios;

        // Check if pin A was the source of the interrupt
        if a_pin.interrupt_status(gpio::Interrupt::EdgeHigh) {
            // toggle can't fail, but the embedded-hal traits always allow for it
            // we can discard the return value by assigning it to an unnamed variable
            let _ = led.toggle();

            // Our interrupt doesn't clear itself.
            // Do that now so we don't immediately jump back to this interrupt handler.
            a_pin.clear_interrupt(gpio::Interrupt::EdgeHigh);
        }

        // Check if pin B was the source of the interrupt
        if b_pin.interrupt_status(gpio::Interrupt::EdgeHigh) {
            // toggle can't fail, but the embedded-hal traits always allow for it
            // we can discard the return value by assigning it to an unnamed variable
            let _ = led.toggle();

            // Our interrupt doesn't clear itself.
            // Do that now so we don't immediately jump back to this interrupt handler.
            b_pin.clear_interrupt(gpio::Interrupt::EdgeHigh);
        }
    }
}

// End of file
