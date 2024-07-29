// Hardware constants for the MPU 9250 chip

pub const MPU9250_ADDRESS_AD0: u8 = 0x68; // Device address when ADO = 0
pub const MPU9250_ADDRESS_AD1: u8 = 0x69; // Device address when ADO = 1
pub const AK8963_ADDRESS: u8 = 0x0C; // Address of magnetometer

pub const SELF_TEST_X_GYRO: u8 = 0x00;
pub const SELF_TEST_Y_GYRO: u8 = 0x01;
pub const SELF_TEST_Z_GYRO: u8 = 0x02;

pub const SELF_TEST_X_ACCEL: u8 = 0x0D;
pub const SELF_TEST_Y_ACCEL: u8 = 0x0E;
pub const SELF_TEST_Z_ACCEL: u8 = 0x0F;

pub const SELF_TEST_A: u8 = 0x10;

pub const XG_OFFSET_H: u8 = 0x13; // User-defined trim values for gyroscope
pub const XG_OFFSET_L: u8 = 0x14;
pub const YG_OFFSET_H: u8 = 0x15;
pub const YG_OFFSET_L: u8 = 0x16;
pub const ZG_OFFSET_H: u8 = 0x17;
pub const ZG_OFFSET_L: u8 = 0x18;
pub const SMPLRT_DIV: u8 = 0x19;
pub const CONFIG: u8 = 0x1A;
pub const GYRO_CONFIG: u8 = 0x1B;
pub const ACCEL_CONFIG: u8 = 0x1C;
pub const ACCEL_CONFIG2: u8 = 0x1D;
pub const LP_ACCEL_ODR: u8 = 0x1E;
pub const WOM_THR: u8 = 0x1F;

// Duration counter threshold for motion interrupt generation, 1 kHz rate,
// LSB = 1 ms
pub const MOT_DUR: u8 = 0x20;
// Zero-motion detection threshold bits [7:0]
pub const ZMOT_THR: u8 = 0x21;
// Duration counter threshold for zero motion interrupt generation, 16 Hz rate,
// LSB = 64 ms
pub const ZRMOT_DUR: u8 = 0x22;

pub const FIFO_EN: u8 = 0x23;
pub const I2C_MST_CTRL: u8 = 0x24;
pub const I2C_SLV0_ADDR: u8 = 0x25;
pub const I2C_SLV0_REG: u8 = 0x26;
pub const I2C_SLV0_CTRL: u8 = 0x27;
pub const I2C_SLV1_ADDR: u8 = 0x28;
pub const I2C_SLV1_REG: u8 = 0x29;
pub const I2C_SLV1_CTRL: u8 = 0x2A;
pub const I2C_SLV2_ADDR: u8 = 0x2B;
pub const I2C_SLV2_REG: u8 = 0x2C;
pub const I2C_SLV2_CTRL: u8 = 0x2D;
pub const I2C_SLV3_ADDR: u8 = 0x2E;
pub const I2C_SLV3_REG: u8 = 0x2F;
pub const I2C_SLV3_CTRL: u8 = 0x30;
pub const I2C_SLV4_ADDR: u8 = 0x31;
pub const I2C_SLV4_REG: u8 = 0x32;
pub const I2C_SLV4_DO: u8 = 0x33;
pub const I2C_SLV4_CTRL: u8 = 0x34;
pub const I2C_SLV4_DI: u8 = 0x35;
pub const I2C_MST_STATUS: u8 = 0x36;
pub const INT_PIN_CFG: u8 = 0x37;
pub const INT_ENABLE: u8 = 0x38;
pub const DMP_INT_STATUS: u8 = 0x39; // Check DMP interrupt
pub const INT_STATUS: u8 = 0x3A;
pub const ACCEL_XOUT_H: u8 = 0x3B;
pub const ACCEL_XOUT_L: u8 = 0x3C;
pub const ACCEL_YOUT_H: u8 = 0x3D;
pub const ACCEL_YOUT_L: u8 = 0x3E;
pub const ACCEL_ZOUT_H: u8 = 0x3F;
pub const ACCEL_ZOUT_L: u8 = 0x40;
pub const TEMP_OUT_H: u8 = 0x41;
pub const TEMP_OUT_L: u8 = 0x42;
pub const GYRO_XOUT_H: u8 = 0x43;
pub const GYRO_XOUT_L: u8 = 0x44;
pub const GYRO_YOUT_H: u8 = 0x45;
pub const GYRO_YOUT_L: u8 = 0x46;
pub const GYRO_ZOUT_H: u8 = 0x47;
pub const GYRO_ZOUT_L: u8 = 0x48;
pub const EXT_SENS_DATA_00: u8 = 0x49;
pub const EXT_SENS_DATA_01: u8 = 0x4A;
pub const EXT_SENS_DATA_02: u8 = 0x4B;
pub const EXT_SENS_DATA_03: u8 = 0x4C;
pub const EXT_SENS_DATA_04: u8 = 0x4D;
pub const EXT_SENS_DATA_05: u8 = 0x4E;
pub const EXT_SENS_DATA_06: u8 = 0x4F;
pub const EXT_SENS_DATA_07: u8 = 0x50;
pub const EXT_SENS_DATA_08: u8 = 0x51;
pub const EXT_SENS_DATA_09: u8 = 0x52;
pub const EXT_SENS_DATA_10: u8 = 0x53;
pub const EXT_SENS_DATA_11: u8 = 0x54;
pub const EXT_SENS_DATA_12: u8 = 0x55;
pub const EXT_SENS_DATA_13: u8 = 0x56;
pub const EXT_SENS_DATA_14: u8 = 0x57;
pub const EXT_SENS_DATA_15: u8 = 0x58;
pub const EXT_SENS_DATA_16: u8 = 0x59;
pub const EXT_SENS_DATA_17: u8 = 0x5A;
pub const EXT_SENS_DATA_18: u8 = 0x5B;
pub const EXT_SENS_DATA_19: u8 = 0x5C;
pub const EXT_SENS_DATA_20: u8 = 0x5D;
pub const EXT_SENS_DATA_21: u8 = 0x5E;
pub const EXT_SENS_DATA_22: u8 = 0x5F;
pub const EXT_SENS_DATA_23: u8 = 0x60;
pub const MOT_DETECT_STATUS: u8 = 0x61;
pub const I2C_SLV0_DO: u8 = 0x63;
pub const I2C_SLV1_DO: u8 = 0x64;
pub const I2C_SLV2_DO: u8 = 0x65;
pub const I2C_SLV3_DO: u8 = 0x66;
pub const I2C_MST_DELAY_CTRL: u8 = 0x67;
pub const SIGNAL_PATH_RESET: u8 = 0x68;
pub const MOT_DETECT_CTRL: u8 = 0x69;
pub const USER_CTRL: u8 = 0x6A; // Bit 7 enable DMP, bit 3 reset DMP
pub const PWR_MGMT_1: u8 = 0x6B; // Device defaults to the SLEEP mode
pub const PWR_MGMT_2: u8 = 0x6C;
pub const DMP_BANK: u8 = 0x6D; // Activates a specific bank in the DMP
pub const DMP_RW_PNT: u8 = 0x6E; // Set read/write pointer to a specific start address in specified DMP bank
pub const DMP_REG: u8 = 0x6F; // Register in DMP from which to read or to which to write
pub const DMP_REG_1: u8 = 0x70;
pub const DMP_REG_2: u8 = 0x71;
pub const FIFO_COUNTH: u8 = 0x72;
pub const FIFO_COUNTL: u8 = 0x73;
pub const FIFO_R_W: u8 = 0x74;
pub const WHO_AM_I_MPU9250: u8 = 0x75; // Should return 0x71
pub const XA_OFFSET_H: u8 = 0x77;
pub const XA_OFFSET_L: u8 = 0x78;
pub const YA_OFFSET_H: u8 = 0x7A;
pub const YA_OFFSET_L: u8 = 0x7B;
pub const ZA_OFFSET_H: u8 = 0x7D;
pub const ZA_OFFSET_L: u8 = 0x7E;

//Magnetometer Registers
pub const WHO_AM_I_AK8963: u8 = 0x00; // (AKA WIA) should return 0x48
pub const INFO: u8 = 0x01;
pub const AK8963_ST1: u8 = 0x02; // data ready status bit 0
pub const AK8963_XOUT_L: u8 = 0x03; // data
pub const AK8963_XOUT_H: u8 = 0x04;
pub const AK8963_YOUT_L: u8 = 0x05;
pub const AK8963_YOUT_H: u8 = 0x06;
pub const AK8963_ZOUT_L: u8 = 0x07;
pub const AK8963_ZOUT_H: u8 = 0x08;
pub const AK8963_ST2: u8 = 0x09; // Data overflow bit 3 and data read error status bit 2
pub const AK8963_CNTL: u8 = 0x0A; // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
pub const AK8963_ASTC: u8 = 0x0C; // Self test control
pub const AK8963_I2CDIS: u8 = 0x0F; // I2C disable
pub const AK8963_ASAX: u8 = 0x10; // Fuse ROM x-axis sensitivity adjustment value
pub const AK8963_ASAY: u8 = 0x11; // Fuse ROM y-axis sensitivity adjustment value
pub const AK8963_ASAZ: u8 = 0x12; // Fuse ROM z-axis sensitivity adjustment value
