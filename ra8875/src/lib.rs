// Library for controlling a TFT display via the RA8875
// Reference https://github.com/adafruit/Adafruit_RA8875/tree/master
#![no_std]
#![no_main]

use embedded_hal::digital::OutputPin;
use embedded_hal::spi::SpiBus;

use cortex_m;

mod constants;
// re-export constants for use outside the library
pub use constants::*;

pub struct RA8875<SPI, CS>
where
    SPI: SpiBus,
    CS: OutputPin,
{
    spi_controller: SPI,
    cs_pin: CS,
}

impl<SPI, CS> RA8875<SPI, CS>
where
    SPI: SpiBus,
    CS: OutputPin,
{
    pub fn new(spi: SPI, cs: CS) -> Self {
        Self {
            spi_controller: spi,
            cs_pin: cs,
        }
    }

    // Read from a register
    pub fn read_reg(&mut self, reg: u8) -> u8 {
        self.write_command(reg);
        self.read_data()
    }

    // Write data to the specified register
    pub fn write_reg(&mut self, reg: u8, val: u8) {
        self.write_command(reg);
        self.write_data(val);
    }

    // Read the data from the current register
    pub fn read_data(&mut self) -> u8 {
        // set CS pin low
        self.cs_pin.set_low().unwrap();
        cortex_m::asm::delay(400);

        let _ = self.spi_controller.write(&[RA8875_DATAREAD]);

        let mut buf: [u8; 1] = [0x0];
        let _ = self.spi_controller.transfer_in_place(&mut buf);

        // set CS pin high
        cortex_m::asm::delay(400);
        self.cs_pin.set_high().unwrap();

        buf[0]
    }

    // Helper function to set chip select and write data over SPI
    fn write(&mut self, words: &[u8]) {
        // set CS pin low
        self.cs_pin.set_low().unwrap();
        // Delay for RA8875 to start listening after chip select is set
        // TODO: derive better value
        cortex_m::asm::delay(400);
        // Transfer the write command identifier, followed by the specified command
        let _ = self.spi_controller.write(words);
        // set CS pin high after RA8875 processing delay
        cortex_m::asm::delay(400);
        self.cs_pin.set_high().unwrap();
    }

    // Write a command to the current register
    pub fn write_command(&mut self, d: u8) {
        // Transfer the write command identifier, followed by the specified command
        self.write(&[RA8875_CMDWRITE, d]);
    }

    // Write data to the current register
    pub fn write_data(&mut self, d: u8) {
        // Transfer the write data identifier, followed by the specified data
        self.write(&[RA8875_DATAWRITE, d]);
    }

    // Initialise the PLL
    fn pll_init(&mut self) {
        // Settings are for 800x480 screen
        // TODO: account for other sizes
        self.write_reg(RA8875_PLLC1, RA8875_PLLC1_PLLDIV1 + 11);
        // Hack to delay for ~1ms since I don't want to figure out how to pass the
        // delay instance around. Pico clock is 125MHz
        cortex_m::asm::delay(125_000);

        self.write_reg(RA8875_PLLC2, RA8875_PLLC2_DIV4);
        cortex_m::asm::delay(125_000);
    }

    // Initializes the driver IC (clock setup, etc.)
    pub fn initialize(&mut self) {
        self.pll_init();
        self.write_reg(RA8875_SYSR, RA8875_SYSR_16BPP | RA8875_SYSR_MCU8);

        // size == RA8875_800x480
        let pixclk: u8 = RA8875_PCSR_PDATL | RA8875_PCSR_2CLK;
        let hsync_start: u8 = 32;
        let hsync_pw: u8 = 96;
        let hsync_finetune: u8 = 0;
        let hsync_nondisp: u8 = 26;
        let vsync_pw: u8 = 2;
        let vsync_nondisp: u16 = 32;
        let vsync_start: u16 = 23;

        self.write_reg(RA8875_PCSR, pixclk);
        // delay 1ms
        cortex_m::asm::delay(125_000);

        /* Horizontal settings registers */
        // H width: (HDWR + 1) * 8 = 480
        self.write_reg(RA8875_HDWR, ((LCD_WIDTH / 8) - 1) as u8);
        self.write_reg(RA8875_HNDFTR, RA8875_HNDFTR_DE_HIGH + hsync_finetune);
        // H non-display: HNDR * 8 + HNDFTR + 2 = 10
        self.write_reg(RA8875_HNDR, (hsync_nondisp - hsync_finetune - 2) / 8);
        // Hsync start: (HSTR + 1)*8
        self.write_reg(RA8875_HSTR, hsync_start / 8 - 1);
        // HSync pulse width = (HPWR+1) * 8
        self.write_reg(RA8875_HPWR, RA8875_HPWR_LOW + (hsync_pw / 8 - 1));

        /* Vertical settings registers */
        self.write_reg(RA8875_VDHR0, (LCD_HEIGHT - 1) as u8);
        self.write_reg(RA8875_VDHR1, ((LCD_HEIGHT - 1) >> 8) as u8);
        // V non-display period = VNDR + 1
        self.write_reg(RA8875_VNDR0, (vsync_nondisp - 1) as u8);
        self.write_reg(RA8875_VNDR1, (vsync_nondisp >> 8) as u8);
        // Vsync start position = VSTR + 1
        self.write_reg(RA8875_VSTR0, (vsync_start - 1) as u8);
        self.write_reg(RA8875_VSTR1, (vsync_start >> 8) as u8);
        // Vsync pulse width = VPWR + 1
        self.write_reg(RA8875_VPWR, RA8875_VPWR_LOW + vsync_pw - 1);

        /* Set active window X */
        // horizontal start point
        self.write_reg(RA8875_HSAW0, 0);
        self.write_reg(RA8875_HSAW1, 0);
        // horizontal end point
        self.write_reg(RA8875_HEAW0, (LCD_WIDTH - 1) as u8);
        self.write_reg(RA8875_HEAW1, ((LCD_WIDTH - 1) >> 8) as u8);

        /* Set active window Y */
        // vertical start point
        self.write_reg(RA8875_VSAW0, 0);
        self.write_reg(RA8875_VSAW1, 0);
        // vertical end point
        self.write_reg(RA8875_VEAW0, (LCD_HEIGHT - 1) as u8);
        self.write_reg(RA8875_VEAW1, ((LCD_HEIGHT - 1) >> 8) as u8);

        /* Clear the entire window */
        self.write_reg(RA8875_MCLR, RA8875_MCLR_START | RA8875_MCLR_FULL);
    }

    pub fn display_on(&mut self, on: bool) {
        if on {
            self.write_reg(RA8875_PWRR, RA8875_PWRR_NORMAL | RA8875_PWRR_DISPON);
        } else {
            self.write_reg(RA8875_PWRR, RA8875_PWRR_NORMAL | RA8875_PWRR_DISPOFF);
        }
    }

    // Set the Extra General Purpose IO Register
    pub fn gpio_x(&mut self, on: bool) {
        if on {
            self.write_reg(RA8875_GPIOX, 1);
        } else {
            self.write_reg(RA8875_GPIOX, 0);
        }
    }

    // Configure the PWM 1 Clock
    pub fn pwm1_config(&mut self, on: bool, clock: u8) {
        if on {
            self.write_reg(RA8875_P1CR, RA8875_P1CR_ENABLE | (clock & 0xF));
        } else {
            self.write_reg(RA8875_P1CR, RA8875_P1CR_DISABLE | (clock & 0xF));
        }
    }

    // Set the duty cycle of the PWM 1 Clock
    pub fn pwm1_out(&mut self, p: u8) {
        self.write_reg(RA8875_P1DCR, p);
    }

    // Draws a single pixel at the specified location
    pub fn draw_pixel(&mut self, x: i16, y: i16, color: u16) {
        self.write_reg(RA8875_CURH0, x as u8);
        self.write_reg(RA8875_CURH1, (x >> 8) as u8);
        self.write_reg(RA8875_CURV0, y as u8);
        self.write_reg(RA8875_CURV1, (y >> 8) as u8);
        self.write_command(RA8875_MRWC);
        self.write(&[RA8875_DATAWRITE, (color >> 8) as u8, color as u8]);
    }

    // Draws a HW accelerated line on the display
    pub fn draw_line(&mut self, x0: i16, y0: i16, x1: i16, y1: i16, color: u16) {
        /* Set X */
        self.write_command(0x91);
        self.write_data(x0 as u8);
        self.write_command(0x92);
        self.write_data((x0 >> 8) as u8);

        /* Set Y */
        self.write_command(0x93);
        self.write_data(y0 as u8);
        self.write_command(0x94);
        self.write_data((y0 >> 8) as u8);

        /* Set X1 */
        self.write_command(0x95);
        self.write_data(x1 as u8);
        self.write_command(0x96);
        self.write_data((x1 >> 8) as u8);

        /* Set Y1 */
        self.write_command(0x97);
        self.write_data(y1 as u8);
        self.write_command(0x98);
        self.write_data((y1 >> 8) as u8);

        /* Set Color */
        self.write_command(0x63);
        self.write_data(((color & 0xf800) >> 11) as u8);
        self.write_command(0x64);
        self.write_data(((color & 0x07e0) >> 5) as u8);
        self.write_command(0x65);
        self.write_data((color & 0x001f) as u8);

        /* Draw! */
        self.write_command(RA8875_DCR);
        self.write_data(0x80);

        /* Wait for the command to finish */
        self.wait_poll(RA8875_DCR, RA8875_DCR_LINESQUTRI_STATUS);
    }

    // Fills the screen with the spefied RGB565 color
    pub fn fill_screen(&mut self, color: u16) {
        self.rect_helper(0, 0, LCD_WIDTH - 1, LCD_HEIGHT - 1, color, true);
    }

    // Helper function for higher level rectangle drawing code
    fn rect_helper(&mut self, x: i16, y: i16, w: i16, h: i16, color: u16, filled: bool) {
        // Set X
        self.write_command(0x91);
        self.write_data(x as u8);
        self.write_command(0x92);
        self.write_data((x >> 8) as u8);

        // Set Y
        self.write_command(0x93);
        self.write_data(y as u8);
        self.write_command(0x94);
        self.write_data((y >> 8) as u8);

        // Set X1
        self.write_command(0x95);
        self.write_data(w as u8);
        self.write_command(0x96);
        self.write_data((w >> 8) as u8);

        // Set Y1
        self.write_command(0x97);
        self.write_data(h as u8);
        self.write_command(0x98);
        self.write_data((h >> 8) as u8);

        // Set Color
        self.write_command(0x63);
        self.write_data(((color & 0xf800) >> 11) as u8);
        self.write_command(0x64);
        self.write_data(((color & 0x07e0) >> 5) as u8);
        self.write_command(0x65);
        self.write_data((color & 0x001f) as u8);

        // Draw!
        self.write_command(RA8875_DCR);
        if filled {
            self.write_data(0xB0);
        } else {
            self.write_data(0x90);
        }

        /* Wait for the command to finish */
        self.wait_poll(RA8875_DCR, RA8875_DCR_LINESQUTRI_STATUS);
    }

    // Waits for screen to finish by polling the status
    // regname: The register name to check
    // waitflag: The value to wait for the status register to match
    fn wait_poll(&mut self, regname: u8, waitflag: u8) {
        /* Wait for the command to finish */
        loop {
            let temp: u8 = self.read_reg(regname);
            if (temp & waitflag) == 0 {
                return;
            }
        }
    }
}
