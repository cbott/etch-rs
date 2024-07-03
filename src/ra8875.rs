// Functions for controlling a TFT display via the RA8875
// Reference https://github.com/adafruit/Adafruit_RA8875/tree/master

use embedded_hal::digital::OutputPin;
use embedded_hal::spi::SpiBus;

use cortex_m;

pub const LCD_WIDTH: i16 = 800;
pub const LCD_HEIGHT: i16 = 480;

// Colors (RGB565)
pub const RA8875_BLACK: u16 = 0x0000;
pub const RA8875_BLUE: u16 = 0x001F;
pub const RA8875_RED: u16 = 0xF800;
pub const RA8875_GREEN: u16 = 0x07E0;
pub const RA8875_CYAN: u16 = 0x07FF;
pub const RA8875_MAGENTA: u16 = 0xF81F;
pub const RA8875_YELLOW: u16 = 0xFFE0;
pub const RA8875_WHITE: u16 = 0xFFFF;

// Command/Data pins for SPI
const RA8875_DATAWRITE: u8 = 0x00;
const RA8875_DATAREAD: u8 = 0x40;
const RA8875_CMDWRITE: u8 = 0x80;
const RA8875_CMDREAD: u8 = 0xC0;

// Registers & bits
const RA8875_PWRR: u8 = 0x01;
const RA8875_PWRR_DISPON: u8 = 0x80;
const RA8875_PWRR_DISPOFF: u8 = 0x00;
const RA8875_PWRR_SLEEP: u8 = 0x02;
const RA8875_PWRR_NORMAL: u8 = 0x00;
const RA8875_PWRR_SOFTRESET: u8 = 0x01;

const RA8875_MRWC: u8 = 0x02;

const RA8875_GPIOX: u8 = 0xC7;

const RA8875_PLLC1: u8 = 0x88;
const RA8875_PLLC1_PLLDIV2: u8 = 0x80;
const RA8875_PLLC1_PLLDIV1: u8 = 0x00;

const RA8875_PLLC2: u8 = 0x89;
const RA8875_PLLC2_DIV1: u8 = 0x00;
const RA8875_PLLC2_DIV2: u8 = 0x01;
const RA8875_PLLC2_DIV4: u8 = 0x02;
const RA8875_PLLC2_DIV8: u8 = 0x03;
const RA8875_PLLC2_DIV16: u8 = 0x04;
const RA8875_PLLC2_DIV32: u8 = 0x05;
const RA8875_PLLC2_DIV64: u8 = 0x06;
const RA8875_PLLC2_DIV128: u8 = 0x07;

const RA8875_SYSR: u8 = 0x10;
const RA8875_SYSR_8BPP: u8 = 0x00;
const RA8875_SYSR_16BPP: u8 = 0x0C;
const RA8875_SYSR_MCU8: u8 = 0x00;
const RA8875_SYSR_MCU16: u8 = 0x03;

const RA8875_PCSR: u8 = 0x04;
const RA8875_PCSR_PDATR: u8 = 0x00;
const RA8875_PCSR_PDATL: u8 = 0x80;
const RA8875_PCSR_CLK: u8 = 0x00;
const RA8875_PCSR_2CLK: u8 = 0x01;
const RA8875_PCSR_4CLK: u8 = 0x02;
const RA8875_PCSR_8CLK: u8 = 0x03;

const RA8875_HDWR: u8 = 0x14;

const RA8875_HNDFTR: u8 = 0x15;
const RA8875_HNDFTR_DE_HIGH: u8 = 0x00;
const RA8875_HNDFTR_DE_LOW: u8 = 0x80;

const RA8875_HNDR: u8 = 0x16;
const RA8875_HSTR: u8 = 0x17;
const RA8875_HPWR: u8 = 0x18;
const RA8875_HPWR_LOW: u8 = 0x00;
const RA8875_HPWR_HIGH: u8 = 0x80;

const RA8875_VDHR0: u8 = 0x19;
const RA8875_VDHR1: u8 = 0x1A;
const RA8875_VNDR0: u8 = 0x1B;
const RA8875_VNDR1: u8 = 0x1C;
const RA8875_VSTR0: u8 = 0x1D;
const RA8875_VSTR1: u8 = 0x1E;
const RA8875_VPWR: u8 = 0x1F;
const RA8875_VPWR_LOW: u8 = 0x00;
const RA8875_VPWR_HIGH: u8 = 0x80;

const RA8875_HSAW0: u8 = 0x30;
const RA8875_HSAW1: u8 = 0x31;
const RA8875_VSAW0: u8 = 0x32;
const RA8875_VSAW1: u8 = 0x33;

const RA8875_HEAW0: u8 = 0x34;
const RA8875_HEAW1: u8 = 0x35;
const RA8875_VEAW0: u8 = 0x36;
const RA8875_VEAW1: u8 = 0x37;

const RA8875_MCLR: u8 = 0x8E;
const RA8875_MCLR_START: u8 = 0x80;
const RA8875_MCLR_STOP: u8 = 0x00;
const RA8875_MCLR_READSTATUS: u8 = 0x80;
const RA8875_MCLR_FULL: u8 = 0x00;
const RA8875_MCLR_ACTIVE: u8 = 0x40;

const RA8875_DCR: u8 = 0x90;
const RA8875_DCR_LINESQUTRI_START: u8 = 0x80;
const RA8875_DCR_LINESQUTRI_STOP: u8 = 0x00;
const RA8875_DCR_LINESQUTRI_STATUS: u8 = 0x80;
const RA8875_DCR_CIRCLE_START: u8 = 0x40;
const RA8875_DCR_CIRCLE_STATUS: u8 = 0x40;
const RA8875_DCR_CIRCLE_STOP: u8 = 0x00;
const RA8875_DCR_FILL: u8 = 0x20;
const RA8875_DCR_NOFILL: u8 = 0x00;
const RA8875_DCR_DRAWLINE: u8 = 0x00;
const RA8875_DCR_DRAWTRIANGLE: u8 = 0x01;
const RA8875_DCR_DRAWSQUARE: u8 = 0x10;

const RA8875_ELLIPSE: u8 = 0xA0;
const RA8875_ELLIPSE_STATUS: u8 = 0x80;

const RA8875_MWCR0: u8 = 0x40;
const RA8875_MWCR0_GFXMODE: u8 = 0x00;
const RA8875_MWCR0_TXTMODE: u8 = 0x80;
const RA8875_MWCR0_CURSOR: u8 = 0x40;
const RA8875_MWCR0_BLINK: u8 = 0x20;

const RA8875_MWCR0_DIRMASK: u8 = 0x0C;
const RA8875_MWCR0_LRTD: u8 = 0x00;
const RA8875_MWCR0_RLTD: u8 = 0x04;
const RA8875_MWCR0_TDLR: u8 = 0x08;
const RA8875_MWCR0_DTLR: u8 = 0x0C;

const RA8875_BTCR: u8 = 0x44;
const RA8875_CURH0: u8 = 0x46;
const RA8875_CURH1: u8 = 0x47;
const RA8875_CURV0: u8 = 0x48;
const RA8875_CURV1: u8 = 0x49;

const RA8875_P1CR: u8 = 0x8A;
const RA8875_P1CR_ENABLE: u8 = 0x80;
const RA8875_P1CR_DISABLE: u8 = 0x00;
const RA8875_P1CR_CLKOUT: u8 = 0x10;
const RA8875_P1CR_PWMOUT: u8 = 0x00;

const RA8875_P1DCR: u8 = 0x8B;

const RA8875_P2CR: u8 = 0x8C;
const RA8875_P2CR_ENABLE: u8 = 0x80;
const RA8875_P2CR_DISABLE: u8 = 0x00;
const RA8875_P2CR_CLKOUT: u8 = 0x10;
const RA8875_P2CR_PWMOUT: u8 = 0x00;

const RA8875_P2DCR: u8 = 0x8D;

pub const RA8875_PWM_CLK_DIV1: u8 = 0x00;
pub const RA8875_PWM_CLK_DIV2: u8 = 0x01;
pub const RA8875_PWM_CLK_DIV4: u8 = 0x02;
pub const RA8875_PWM_CLK_DIV8: u8 = 0x03;
pub const RA8875_PWM_CLK_DIV16: u8 = 0x04;
pub const RA8875_PWM_CLK_DIV32: u8 = 0x05;
pub const RA8875_PWM_CLK_DIV64: u8 = 0x06;
pub const RA8875_PWM_CLK_DIV128: u8 = 0x07;
pub const RA8875_PWM_CLK_DIV256: u8 = 0x08;
pub const RA8875_PWM_CLK_DIV512: u8 = 0x09;
pub const RA8875_PWM_CLK_DIV1024: u8 = 0x0A;
pub const RA8875_PWM_CLK_DIV2048: u8 = 0x0B;
pub const RA8875_PWM_CLK_DIV4096: u8 = 0x0C;
pub const RA8875_PWM_CLK_DIV8192: u8 = 0x0D;
pub const RA8875_PWM_CLK_DIV16384: u8 = 0x0E;
pub const RA8875_PWM_CLK_DIV32768: u8 = 0x0F;

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

    // Write a command to the current register
    pub fn write_command(&mut self, d: u8) {
        // set CS pin low
        self.cs_pin.set_low().unwrap();
        cortex_m::asm::delay(400);

        // Transfer the write command identifier, followed by the specified command
        let _ = self.spi_controller.write(&[RA8875_CMDWRITE, d]);

        // set CS pin high
        cortex_m::asm::delay(400);
        self.cs_pin.set_high().unwrap();
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

    // Write data to the current register
    pub fn write_data(&mut self, d: u8) {
        // set CS pin low
        self.cs_pin.set_low().unwrap();
        cortex_m::asm::delay(400);

        // Transfer the write command identifier, followed by the specified command
        let _ = self.spi_controller.write(&[RA8875_DATAWRITE, d]);

        // set CS pin high
        cortex_m::asm::delay(400);
        self.cs_pin.set_high().unwrap();
    }

    // Initialise the PLL
    fn pll_init(&mut self) {
        // Settings are for 800x480 screen
        // TODO: account for other sizes
        self.write_reg(RA8875_PLLC1, RA8875_PLLC1_PLLDIV1 + 11);
        // Hack to delay for ~1ms since I don't want to figure out how to pass the
        // delay instance around. Pico clock iss 125MHz
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
        self.cs_pin.set_low().unwrap();
        cortex_m::asm::delay(400);
        let _ = self
            .spi_controller
            .write(&[RA8875_DATAWRITE, (color >> 8) as u8, color as u8]);
        cortex_m::asm::delay(400);
        self.cs_pin.set_high().unwrap();
    }

    // Fills the screen with the spefied RGB565 color
    pub fn fill_screen(&mut self, color: u16) {
        self.rect_helper(0, 0, LCD_WIDTH - 1, LCD_HEIGHT - 1, color, true);
    }

    // Helper function for higher level rectangle drawing code
    fn rect_helper(&mut self, x: i16, y: i16, w: i16, h: i16, color: u16, filled: bool) {
        // Not supporting rotation right now

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
