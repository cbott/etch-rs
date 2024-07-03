//! Run the etch-rs hardware
#![no_std]
#![no_main]

// Provide an alias for our BSP so we can switch targets quickly.
use rp_pico as bsp;

// TODO: organize and clean up imports
use bsp::entry;
use core::cell::RefCell;
use core::convert::Infallible;
use critical_section::Mutex;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::InputPin;
use embedded_hal::digital::OutputPin;
use panic_probe as _;

use bsp::hal::{
    self,
    clocks::{init_clocks_and_plls, Clock},
    gpio, pac,
    sio::Sio,
    watchdog::Watchdog,
};

// Needed for specifying SPI rates with MHz function
use hal::fugit::RateExtU32;

// Interrupt macro
use bsp::hal::pac::interrupt;
use core::ops::DerefMut;

mod encoder;
mod ra8875;

// Pin types quickly become very long!
// We'll create some type aliases using `type` to help with that
type Encoder1APin = gpio::Pin<gpio::bank0::Gpio7, gpio::FunctionSioInput, gpio::PullUp>;
type Encoder1BPin = gpio::Pin<gpio::bank0::Gpio8, gpio::FunctionSioInput, gpio::PullUp>;
type Encoder2APin = gpio::Pin<gpio::bank0::Gpio13, gpio::FunctionSioInput, gpio::PullUp>;
type Encoder2BPin = gpio::Pin<gpio::bank0::Gpio14, gpio::FunctionSioInput, gpio::PullUp>;
type LedPin = gpio::Pin<gpio::bank0::Gpio11, gpio::FunctionSioOutput, gpio::PullNone>;
type MyEncoder = encoder::Encoder<Encoder1APin, Encoder1BPin>;

/// Since we're always accessing these pins together we'll store them in a tuple.
/// Giving this tuple a type alias means we won't need to use () when putting them
/// inside an Option. That will be easier to read.
type InterruptPins = (LedPin, MyEncoder, Encoder2APin, Encoder2BPin);

/// This how we transfer our pins to the Interrupt Handler.
/// We'll have the option hold both using the InterruptPins type.
/// This will make it a bit easier to unpack them later.
static GLOBAL_PINS: Mutex<RefCell<Option<InterruptPins>>> = Mutex::new(RefCell::new(None));
static GLOBAL_VALUES: Mutex<RefCell<Option<(i16, i16)>>> = Mutex::new(RefCell::new(None));

// For gpio toggle()
use embedded_hal::digital::StatefulOutputPin;

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

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut led_pin = pins.led.into_push_pull_output();
    let led_a: LedPin = pins.gpio11.reconfigure();
    let pin_1a: Encoder1APin = pins.gpio7.reconfigure();
    let pin_1b: Encoder1BPin = pins.gpio8.reconfigure();
    let pin_2a: Encoder2APin = pins.gpio13.reconfigure();
    let pin_2b: Encoder2BPin = pins.gpio14.reconfigure();

    // Set up our GPIO interrupt stuff
    pin_1a.set_interrupt_enabled(gpio::Interrupt::EdgeHigh, true);
    pin_1a.set_interrupt_enabled(gpio::Interrupt::EdgeLow, true);
    pin_2a.set_interrupt_enabled(gpio::Interrupt::EdgeHigh, true);
    pin_2a.set_interrupt_enabled(gpio::Interrupt::EdgeLow, true);

    let mut encoder1 = encoder::Encoder::new(pin_1a, pin_1b);

    // Set up SPI
    let spi_mosi = pins.gpio19.into_function::<hal::gpio::FunctionSpi>();
    let spi_miso = pins.gpio16.into_function::<hal::gpio::FunctionSpi>();
    let spi_sclk = pins.gpio18.into_function::<hal::gpio::FunctionSpi>();
    let mut cs_pin = pins.gpio10.into_push_pull_output();
    let mut rst_pin = pins.gpio9.into_push_pull_output();
    let spi = hal::spi::Spi::<_, _, _, 8>::new(pac.SPI0, (spi_mosi, spi_miso, spi_sclk));

    // Equivalent of begin() method
    cs_pin.set_high().unwrap();
    rst_pin.set_low().unwrap();
    delay.delay_ms(100);
    rst_pin.set_high().unwrap();
    delay.delay_ms(100);

    // Exchange the uninitialised SPI driver for an initialised one
    let spi = spi.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        2.MHz(),
        embedded_hal::spi::MODE_0,
    );

    let mut tft = ra8875::RA8875::new(spi, cs_pin);

    let x: u8 = tft.read_reg(0);

    // Verify we're talking to the RA8875
    if x != 0x75 {
        loop {
            // infinite error blink
            led_pin.set_low().unwrap();
            delay.delay_ms(100);
            led_pin.set_high().unwrap();
            delay.delay_ms(100);
        }
    }

    tft.initialize();
    // initialize requires a delay after, should ideally enforce this but ok for now
    // TODO: verify delay. Adafruit library does 500ms but 50 seems to work
    delay.delay_ms(50);
    // done with begin() function

    tft.display_on(true);
    // Enable TFT - display enable tied to GPIOX
    tft.gpio_x(true);
    // PWM output for backlight
    tft.pwm1_config(true, ra8875::RA8875_PWM_CLK_DIV1024);
    tft.pwm1_out(255);

    // debug
    tft.fill_screen(ra8875::RA8875_YELLOW);
    delay.delay_ms(1000);

    // Give away our pins by moving them into the `GLOBAL_PINS` variable.
    // We won't need to access them in the main thread again
    critical_section::with(|cs| {
        GLOBAL_PINS
            .borrow(cs)
            .replace(Some((led_a, encoder1, pin_2a, pin_2b)));
        GLOBAL_VALUES.borrow(cs).replace(Some((25, 25)));
    });

    // Unmask the IO_BANK0 IRQ so that the NVIC interrupt controller
    // will jump to the interrupt function when the interrupt occurs.
    // We do this last so that the interrupt can't go off while
    // it is in the middle of being configured
    unsafe {
        pac::NVIC::unmask(pac::Interrupt::IO_IRQ_BANK0);
    }

    // Go back to doing LCD stuff
    tft.fill_screen(0xAD6F);
    led_pin.set_low().unwrap();
    delay.delay_ms(1000);

    loop {
        let mut x_pos: i16 = 0;
        let mut y_pos: i16 = 0;

        critical_section::with(|cs| {
            if let Some(ref mut value) = GLOBAL_VALUES.borrow(cs).borrow_mut().deref_mut() {
                (x_pos, y_pos) = *value;
            }
        });

        tft.draw_pixel(
            x_pos % ra8875::LCD_WIDTH,
            y_pos % ra8875::LCD_HEIGHT,
            ra8875::RA8875_BLACK,
        );
        delay.delay_ms(10)
    }
}

#[interrupt]
fn IO_IRQ_BANK0() {
    // The `#[interrupt]` attribute covertly converts this to `&'static mut Option<InterruptPins>`
    static mut INTERRUPT_PINS: Option<InterruptPins> = None;

    // Track current and previous encoder state
    static mut ENCODER_1_STATE: u8 = 0;
    static mut ENCODER_2_STATE: u8 = 0;

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
        let (led_a, encoder1, pin_2a, pin_2b) = gpios;

        // Process encoder 1 (X)
        let result = encoder1.process();
        if pin_1a.interrupt_status(gpio::Interrupt::EdgeHigh)
            | pin_1a.interrupt_status(gpio::Interrupt::EdgeLow)
        {
            // toggle can't fail, but the embedded-hal traits always allow for it
            // we can discard the return value by assigning it to an unnamed variable
            let _ = led_a.toggle();

            *ENCODER_1_STATE <<= 2;
            let a_state = pin_1a.is_high().unwrap();
            let b_state = pin_1b.is_high().unwrap();
            if a_state {
                *ENCODER_1_STATE |= 0b01;
            }
            if b_state {
                *ENCODER_1_STATE |= 0b10;
            }
            *ENCODER_1_STATE &= 0x0F;

            // Our interrupt doesn't clear itself.
            // Do that now so we don't immediately jump back to this interrupt handler.
            pin_1a.clear_interrupt(gpio::Interrupt::EdgeHigh);
            pin_1a.clear_interrupt(gpio::Interrupt::EdgeLow);
        }

        // Process encoder 2 (Y)
        if pin_2a.interrupt_status(gpio::Interrupt::EdgeHigh)
            | pin_2a.interrupt_status(gpio::Interrupt::EdgeLow)
        {
            // toggle can't fail, but the embedded-hal traits always allow for it
            // we can discard the return value by assigning it to an unnamed variable
            let _ = led_a.toggle();

            *ENCODER_2_STATE <<= 2;
            let a_state = pin_2a.is_high().unwrap();
            let b_state = pin_2b.is_high().unwrap();
            if a_state {
                *ENCODER_2_STATE |= 0b01;
            }
            if b_state {
                *ENCODER_2_STATE |= 0b10;
            }
            *ENCODER_2_STATE &= 0x0F;

            // Our interrupt doesn't clear itself.
            // Do that now so we don't immediately jump back to this interrupt handler.
            pin_2a.clear_interrupt(gpio::Interrupt::EdgeHigh);
            pin_2a.clear_interrupt(gpio::Interrupt::EdgeLow);
        }

        critical_section::with(|cs| {
            if let Some(ref mut xy) = GLOBAL_VALUES.borrow(cs).borrow_mut().deref_mut() {
                if *ENCODER_1_STATE == 0x09 {
                    (*xy).0 += 1;
                } else if *ENCODER_1_STATE == 0x03 {
                    (*xy).0 -= 1;
                }
                if *ENCODER_2_STATE == 0x09 {
                    (*xy).1 += 1;
                } else if *ENCODER_2_STATE == 0x03 {
                    (*xy).1 -= 1;
                }
            }
        });
    }
}
