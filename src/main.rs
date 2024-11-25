//! Run the etch-rs hardware
#![no_std]
#![no_main]

// Provide an alias for our BSP so we can switch targets quickly.
use rp_pico::{self as bsp, hal::fugit::Rate};

// TODO: organize and clean up imports
use bsp::entry;
use core::cell::RefCell;
use core::ops::DerefMut;
use critical_section::Mutex;
use defmt::*;
use defmt_rtt as _;
use panic_probe as _;

use bsp::hal::{
    self,
    clocks::{init_clocks_and_plls, Clock},
    fugit::HertzU32,
    fugit::RateExtU32, // For specifying SPI rates with MHz function
    gpio,
    pac,
    pac::interrupt, // Interrupt macro
    sio::Sio,
    watchdog::Watchdog,
};

use embedded_hal::digital::{
    OutputPin,
    StatefulOutputPin, // For gpio toggle()
};

// Local libraries
mod encoder;
use mpu_9250;
use ra8875;

// Define type aliases to shorten up definitions later
type EncoderX = encoder::Encoder<gpio::bank0::Gpio9, gpio::bank0::Gpio10>;
type EncoderY = encoder::Encoder<gpio::bank0::Gpio13, gpio::bank0::Gpio14>;

// Define the structures which will be passed in and out of the Interrupt Handler
// Objects will only be passed in once and not used by the main thread after
// while Values will be need to be read back in the main thread
static GLOBAL_OBJS: Mutex<RefCell<Option<(EncoderX, EncoderY)>>> = Mutex::new(RefCell::new(None));
static GLOBAL_VALUES: Mutex<RefCell<Option<(i16, i16)>>> = Mutex::new(RefCell::new(None));

// constants to define etch-rs behavior
const INITIAL_X_POS: i16 = ra8875::LCD_WIDTH / 2;
const INITIAL_Y_POS: i16 = ra8875::LCD_HEIGHT / 2;
const BACKGROUND_COLOR: u16 = 0x7BC7; // 0x7bc7 per william
const I2C_BAUD: HertzU32 = HertzU32::kHz(100);
const SPI_BAUD: HertzU32 = HertzU32::MHz(2);

const LOOP_DELAY_MS: u32 = 5;
const NUM_CROSSINGS: u32 = 6;
const ACCEL_DEADBAND: f32 = 0.5;
const NUM_SAMPLES: usize = 1000 / LOOP_DELAY_MS as usize;

// Determine if a time series of acceleration values contains a shaking motion
fn detect_shake(samples: &[f32]) -> bool {
    // First find the mean of the array
    let mut sum: f32 = 0.0;
    for val in samples {
        sum += val;
    }
    let mean: f32 = sum / (samples.len() as f32);
    // Then threshold the values above/below the mean, with some deadband
    // -1/0/1 for below/deadband/above
    // Then count the number of times we cross from above to below the mean
    let mut crossings: u32 = 0;
    let mut current_state: i8 = 0;
    for val in samples {
        // TODO: this is pretty messy
        let mut new_state: i8 = 0;
        let normalized_accel = val - mean;
        if normalized_accel > ACCEL_DEADBAND {
            new_state = 1;
        } else if normalized_accel < -ACCEL_DEADBAND {
            new_state = -1;
        }
        if new_state != 0 {
            // current_state tracks the last acceleration value that was above the threshold
            if new_state == -current_state {
                // If our new state is opposite, we've crossed over the deadband
                crossings += 1;
            }
            current_state = new_state;
        }
    }

    // If this exceeds a certain count then we have a "shake"
    return crossings >= NUM_CROSSINGS;
}

#[entry]
fn main() -> ! {
    info!("Program start");
    // Board setup
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

    // Configure pins

    // Set up display

    // Set up encoders

    // Set up IMU

    // Set up cursor

    // Enable interrupts (if not handled by encoder)

    // Start mainloop

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut led_pin = pins.led.into_push_pull_output();
    led_pin.set_high().unwrap();
    let pin_x1 = pins.gpio9.into_pull_up_input();
    let pin_x2 = pins.gpio10.into_pull_up_input();
    let pin_y1 = pins.gpio13.into_pull_up_input();
    let pin_y2 = pins.gpio14.into_pull_up_input();

    let mut encoder_x = encoder::Encoder::new(pin_x1, pin_x2);
    let mut encoder_y = encoder::Encoder::new(pin_y1, pin_y2);
    encoder_x.enable_interrupts();
    encoder_y.enable_interrupts();

    // Set up the MPU 9250 IMU over I2C
    let i2c_sda = pins.gpio4.into_function::<hal::gpio::FunctionI2C>();
    let i2c_scl = pins.gpio5.into_function::<hal::gpio::FunctionI2C>();
    let i2c = hal::i2c::I2C::new_controller(
        pac.I2C0,
        i2c_sda,
        i2c_scl,
        I2C_BAUD,
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
    );
    let mut imu = mpu_9250::MPU9250::new(i2c, mpu_9250::MPU9250_ADDRESS_AD0);

    // Verify we're talking to the MPU 9250
    // TODO: make "check whoami" method in the lib?
    // could return Ok() or the failing byte?
    let x: u8 = imu.read_byte(mpu_9250::MPU9250_ADDRESS_AD0, mpu_9250::WHO_AM_I_MPU9250);
    if x != 0x71 {
        // TODO: this should not block basic functioning of device, maybe display an error and move on
        loop {
            // infinite error blink
            led_pin.toggle().unwrap();
            delay.delay_ms(100);
        }
    }

    // Initialize imu for active mode read of acclerometer, gyroscope, and temperature
    imu.init();

    // Set up RA8875 LCD driver over SPI
    let spi_mosi = pins.gpio19.into_function::<hal::gpio::FunctionSpi>();
    let spi_miso = pins.gpio16.into_function::<hal::gpio::FunctionSpi>();
    let spi_sclk = pins.gpio18.into_function::<hal::gpio::FunctionSpi>();
    let mut cs_pin = pins.gpio17.into_push_pull_output();
    let mut rst_pin = pins.gpio20.into_push_pull_output();
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
        SPI_BAUD,
        embedded_hal::spi::MODE_0,
    );

    let mut tft = ra8875::RA8875::new(spi, cs_pin);

    // Verify we're talking to the RA8875
    let x: u8 = tft.read_reg(0);
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

    // Give away our ownership by moving them into the `GLOBAL_OBJS` variable.
    // We will take back ownership of the values later when we need to read them
    critical_section::with(|cs| {
        GLOBAL_OBJS.borrow(cs).replace(Some((encoder_x, encoder_y)));
        GLOBAL_VALUES
            .borrow(cs)
            .replace(Some((INITIAL_X_POS, INITIAL_Y_POS)));
    });

    // Unmask the IO_BANK0 IRQ so that the NVIC interrupt controller
    // will jump to the interrupt function when the interrupt occurs.
    // We do this last so that the interrupt can't go off while
    // it is in the middle of being configured
    unsafe {
        pac::NVIC::unmask(pac::Interrupt::IO_IRQ_BANK0);
    }

    // Go back to doing LCD stuff
    // TODO: make this a const
    tft.fill_screen(BACKGROUND_COLOR);
    led_pin.set_low().unwrap();
    delay.delay_ms(1000);

    let mut index: usize = 0;
    let mut accel_samples: [f32; NUM_SAMPLES] = [0.0; NUM_SAMPLES];

    let mut prev_x: i16 = INITIAL_X_POS;
    let mut prev_y: i16 = INITIAL_Y_POS;

    loop {
        let mut x_pos: i16 = 0;
        let mut y_pos: i16 = 0;
        critical_section::with(|cs| {
            if let Some(ref mut value) = GLOBAL_VALUES.borrow(cs).borrow_mut().deref_mut() {
                (x_pos, y_pos) = *value;
            }
        });
        x_pos = x_pos % ra8875::LCD_WIDTH;
        y_pos = y_pos % ra8875::LCD_HEIGHT;

        // TODO: figure out draw_line to avoid skipping pixels
        // tft.draw_line(prev_x, prev_y, x_pos, y_pos, ra8875::RA8875_BLACK);
        tft.draw_pixel(x_pos, y_pos, ra8875::RA8875_BLACK);
        prev_x = x_pos;
        prev_y = y_pos;

        // TODO: should we be checking has_data?
        let (_, _, z_accel) = imu.read_accel_data();
        accel_samples[index] = z_accel;
        index = (index + 1) % accel_samples.len();

        if detect_shake(&accel_samples) {
            // Blank the screen
            tft.fill_screen(BACKGROUND_COLOR);
            // Reset the sample buffer to avoid clearing again
            accel_samples.iter_mut().for_each(|x| *x = 0.0)
        }

        delay.delay_ms(LOOP_DELAY_MS);
    }
}

#[interrupt]
fn IO_IRQ_BANK0() {
    // The `#[interrupt]` attribute covertly converts this to `&'static mut Option<InterruptObjs>`
    static mut INTERRUPT_PINS: Option<(EncoderX, EncoderY)> = None;

    // This is one-time lazy initialisation. We steal the variables given to us via `GLOBAL_OBJS`.
    if INTERRUPT_PINS.is_none() {
        critical_section::with(|cs| {
            *INTERRUPT_PINS = GLOBAL_OBJS.borrow(cs).take();
        });
    }

    if let Some(gpios) = INTERRUPT_PINS {
        // mutable borrow of encoders to let us read and update them
        let (encoder_x, encoder_y) = gpios;

        // Process encoder 1 (X)
        let mut result_x = encoder::DIR_NONE;
        if encoder_x.pin1.interrupt_status(gpio::Interrupt::EdgeHigh)
            | encoder_x.pin1.interrupt_status(gpio::Interrupt::EdgeLow)
            | encoder_x.pin2.interrupt_status(gpio::Interrupt::EdgeHigh)
            | encoder_x.pin2.interrupt_status(gpio::Interrupt::EdgeLow)
        {
            result_x = encoder_x.process();
            encoder_x.pin1.clear_interrupt(gpio::Interrupt::EdgeHigh);
            encoder_x.pin1.clear_interrupt(gpio::Interrupt::EdgeLow);
            encoder_x.pin2.clear_interrupt(gpio::Interrupt::EdgeHigh);
            encoder_x.pin2.clear_interrupt(gpio::Interrupt::EdgeLow);
        }

        // Process encoder 2 (Y)
        let mut result_y = encoder::DIR_NONE;
        if encoder_y.pin1.interrupt_status(gpio::Interrupt::EdgeHigh)
            | encoder_y.pin1.interrupt_status(gpio::Interrupt::EdgeLow)
            | encoder_y.pin2.interrupt_status(gpio::Interrupt::EdgeHigh)
            | encoder_y.pin2.interrupt_status(gpio::Interrupt::EdgeLow)
        {
            result_y = encoder_y.process();
            encoder_y.pin1.clear_interrupt(gpio::Interrupt::EdgeHigh);
            encoder_y.pin1.clear_interrupt(gpio::Interrupt::EdgeLow);
            encoder_y.pin2.clear_interrupt(gpio::Interrupt::EdgeHigh);
            encoder_y.pin2.clear_interrupt(gpio::Interrupt::EdgeLow);
        }

        critical_section::with(|cs| {
            if let Some(ref mut xy) = GLOBAL_VALUES.borrow(cs).borrow_mut().deref_mut() {
                // Update X value
                if result_x == encoder::DIR_CW {
                    (*xy).0 -= 1;
                } else if result_x == encoder::DIR_CCW {
                    (*xy).0 += 1;
                }
                // Update Y value
                if result_y == encoder::DIR_CW {
                    (*xy).1 -= 1;
                } else if result_y == encoder::DIR_CCW {
                    (*xy).1 += 1;
                }
            }
        });
    }
}
