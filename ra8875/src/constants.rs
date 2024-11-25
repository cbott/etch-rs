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
pub const RA8875_DATAWRITE: u8 = 0x00;
pub const RA8875_DATAREAD: u8 = 0x40;
pub const RA8875_CMDWRITE: u8 = 0x80;
pub const RA8875_CMDREAD: u8 = 0xC0;

// Registers & bits
pub const RA8875_PWRR: u8 = 0x01;
pub const RA8875_PWRR_DISPON: u8 = 0x80;
pub const RA8875_PWRR_DISPOFF: u8 = 0x00;
pub const RA8875_PWRR_SLEEP: u8 = 0x02;
pub const RA8875_PWRR_NORMAL: u8 = 0x00;
pub const RA8875_PWRR_SOFTRESET: u8 = 0x01;

pub const RA8875_MRWC: u8 = 0x02;

pub const RA8875_GPIOX: u8 = 0xC7;

pub const RA8875_PLLC1: u8 = 0x88;
pub const RA8875_PLLC1_PLLDIV2: u8 = 0x80;
pub const RA8875_PLLC1_PLLDIV1: u8 = 0x00;

pub const RA8875_PLLC2: u8 = 0x89;
pub const RA8875_PLLC2_DIV1: u8 = 0x00;
pub const RA8875_PLLC2_DIV2: u8 = 0x01;
pub const RA8875_PLLC2_DIV4: u8 = 0x02;
pub const RA8875_PLLC2_DIV8: u8 = 0x03;
pub const RA8875_PLLC2_DIV16: u8 = 0x04;
pub const RA8875_PLLC2_DIV32: u8 = 0x05;
pub const RA8875_PLLC2_DIV64: u8 = 0x06;
pub const RA8875_PLLC2_DIV128: u8 = 0x07;

pub const RA8875_SYSR: u8 = 0x10;
pub const RA8875_SYSR_8BPP: u8 = 0x00;
pub const RA8875_SYSR_16BPP: u8 = 0x0C;
pub const RA8875_SYSR_MCU8: u8 = 0x00;
pub const RA8875_SYSR_MCU16: u8 = 0x03;

pub const RA8875_PCSR: u8 = 0x04;
pub const RA8875_PCSR_PDATR: u8 = 0x00;
pub const RA8875_PCSR_PDATL: u8 = 0x80;
pub const RA8875_PCSR_CLK: u8 = 0x00;
pub const RA8875_PCSR_2CLK: u8 = 0x01;
pub const RA8875_PCSR_4CLK: u8 = 0x02;
pub const RA8875_PCSR_8CLK: u8 = 0x03;

pub const RA8875_HDWR: u8 = 0x14;

pub const RA8875_HNDFTR: u8 = 0x15;
pub const RA8875_HNDFTR_DE_HIGH: u8 = 0x00;
pub const RA8875_HNDFTR_DE_LOW: u8 = 0x80;

pub const RA8875_HNDR: u8 = 0x16;
pub const RA8875_HSTR: u8 = 0x17;
pub const RA8875_HPWR: u8 = 0x18;
pub const RA8875_HPWR_LOW: u8 = 0x00;
pub const RA8875_HPWR_HIGH: u8 = 0x80;

pub const RA8875_VDHR0: u8 = 0x19;
pub const RA8875_VDHR1: u8 = 0x1A;
pub const RA8875_VNDR0: u8 = 0x1B;
pub const RA8875_VNDR1: u8 = 0x1C;
pub const RA8875_VSTR0: u8 = 0x1D;
pub const RA8875_VSTR1: u8 = 0x1E;
pub const RA8875_VPWR: u8 = 0x1F;
pub const RA8875_VPWR_LOW: u8 = 0x00;
pub const RA8875_VPWR_HIGH: u8 = 0x80;

pub const RA8875_HSAW0: u8 = 0x30;
pub const RA8875_HSAW1: u8 = 0x31;
pub const RA8875_VSAW0: u8 = 0x32;
pub const RA8875_VSAW1: u8 = 0x33;

pub const RA8875_HEAW0: u8 = 0x34;
pub const RA8875_HEAW1: u8 = 0x35;
pub const RA8875_VEAW0: u8 = 0x36;
pub const RA8875_VEAW1: u8 = 0x37;

pub const RA8875_MCLR: u8 = 0x8E;
pub const RA8875_MCLR_START: u8 = 0x80;
pub const RA8875_MCLR_STOP: u8 = 0x00;
pub const RA8875_MCLR_READSTATUS: u8 = 0x80;
pub const RA8875_MCLR_FULL: u8 = 0x00;
pub const RA8875_MCLR_ACTIVE: u8 = 0x40;

pub const RA8875_DCR: u8 = 0x90;
pub const RA8875_DCR_LINESQUTRI_START: u8 = 0x80;
pub const RA8875_DCR_LINESQUTRI_STOP: u8 = 0x00;
pub const RA8875_DCR_LINESQUTRI_STATUS: u8 = 0x80;
pub const RA8875_DCR_CIRCLE_START: u8 = 0x40;
pub const RA8875_DCR_CIRCLE_STATUS: u8 = 0x40;
pub const RA8875_DCR_CIRCLE_STOP: u8 = 0x00;
pub const RA8875_DCR_FILL: u8 = 0x20;
pub const RA8875_DCR_NOFILL: u8 = 0x00;
pub const RA8875_DCR_DRAWLINE: u8 = 0x00;
pub const RA8875_DCR_DRAWTRIANGLE: u8 = 0x01;
pub const RA8875_DCR_DRAWSQUARE: u8 = 0x10;

pub const RA8875_ELLIPSE: u8 = 0xA0;
pub const RA8875_ELLIPSE_STATUS: u8 = 0x80;

pub const RA8875_MWCR0: u8 = 0x40;
pub const RA8875_MWCR0_GFXMODE: u8 = 0x00;
pub const RA8875_MWCR0_TXTMODE: u8 = 0x80;
pub const RA8875_MWCR0_CURSOR: u8 = 0x40;
pub const RA8875_MWCR0_BLINK: u8 = 0x20;

pub const RA8875_MWCR0_DIRMASK: u8 = 0x0C;
pub const RA8875_MWCR0_LRTD: u8 = 0x00;
pub const RA8875_MWCR0_RLTD: u8 = 0x04;
pub const RA8875_MWCR0_TDLR: u8 = 0x08;
pub const RA8875_MWCR0_DTLR: u8 = 0x0C;

pub const RA8875_BTCR: u8 = 0x44;
pub const RA8875_CURH0: u8 = 0x46;
pub const RA8875_CURH1: u8 = 0x47;
pub const RA8875_CURV0: u8 = 0x48;
pub const RA8875_CURV1: u8 = 0x49;

pub const RA8875_P1CR: u8 = 0x8A;
pub const RA8875_P1CR_ENABLE: u8 = 0x80;
pub const RA8875_P1CR_DISABLE: u8 = 0x00;
pub const RA8875_P1CR_CLKOUT: u8 = 0x10;
pub const RA8875_P1CR_PWMOUT: u8 = 0x00;

pub const RA8875_P1DCR: u8 = 0x8B;

pub const RA8875_P2CR: u8 = 0x8C;
pub const RA8875_P2CR_ENABLE: u8 = 0x80;
pub const RA8875_P2CR_DISABLE: u8 = 0x00;
pub const RA8875_P2CR_CLKOUT: u8 = 0x10;
pub const RA8875_P2CR_PWMOUT: u8 = 0x00;

pub const RA8875_P2DCR: u8 = 0x8D;

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
