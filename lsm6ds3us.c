//-----------------------------------------------------------------------------
// Include Section
//-----------------------------------------------------------------------------
#include "lsm6ds3us.h"
#include "utility.h"
//-----------------------------------------------------------------------------
// Constants/Macros Definitions
//-----------------------------------------------------------------------------

#define SA0_PIN_STATE                           1u  //!< ADDR pin state used to set the slave address
#define LSM6DS3US_ADDRESS                    0x6Au  //!< 7 bit Device address

#define SLAVE_ADDRESS                 (LSM6DS3US_ADDRESS | SA0_PIN_STATE)<<1  //!< Slave address

// Registers addresses
#define FUNC_CFG_ACCESS_REG_ADDRESS          0x01u  //!< Enable embedded functions register address     
#define SENSOR_SYNC_TIME_FRAME_REG_ADDRESS   0x04u  //!< Sensor synchronization time frame register address                     (Read/Write)
#define FIFO_CTRL1_REG_ADDRESS               0x06u  //!< FIFO control register address                                          (Read/Write)
#define FIFO_CTRL2_REG_ADDRESS               0x07u  //!< FIFO control register address                                          (Read/Write)
#define FIFO_CTRL3_REG_ADDRESS               0x08u  //!< FIFO control register address                                          (Read/Write)
#define FIFO_CTRL4_REG_ADDRESS               0x09u  //!< FIFO control register address                                          (Read/Write)
#define FIFO_CTRL5_REG_ADDRESS               0x0Au  //!< FIFO control register address                                          (Read/Write)
#define DRDY_PULSE_CFG_G_REG_ADDRESS         0x0Bu  //!< DataReady configuration register address 								(Read/Write)
#define INT1_CTRL_REG_ADDRESS                0x0Du  //!< INT1 pad control register address                                      (Read/Write)
#define INT2_CTRL_REG_ADDRESS                0x0Eu  //!< INT2 pad control register address                                      (Read/Write)
#define WHO_AM_I_REG_ADDRESS                 0x0Fu  //!< Who_AM_I register address                                              (Read only)
#define CTRL1_XL_REG_ADDRESS                 0x10u  //!< Linear acceleration sensor control register 1 address                  (Read/Write)
#define CTRL2_G_REG_ADDRESS                  0x11u  //!< Angular rate sensor control register 2 address                         (Read/Write)
#define CTRL3_C_REG_ADDRESS                  0x12u  //!< Control register 3 address                                             (Read/Write)
#define CTRL4_C_REG_ADDRESS                  0x13u  //!< Control register 4 address                                             (Read/Write)
#define CTRL5_C_REG_ADDRESS                  0x14u  //!< Control register 5 address                                             (Read/Write)
#define CTRL6_C_REG_ADDRESS                  0x15u  //!< Angular rate sensor control register 6 address                         (Read/Write)
#define CTRL7_G_REG_ADDRESS                  0x16u  //!< Angular rate sensor control register 7 address                         (Read/Write)
#define CTRL8_XL_REG_ADDRESS                 0x17u  //!< Linear acceleration sensor control register 8 address                  (Read/Write)
#define CTRL9_XL_REG_ADDRESS                 0x18u  //!< Linear acceleration sensor control register 9 address                  (Read/Write)
#define CTRL10_C_REG_ADDRESS                 0x19u  //!< Control register 10 address                                            (Read/Write)
#define MASTER_CONFIG_REG_ADDRESS            0x1Au  //!< Master configuration register address                                  (Read/Write)
#define WAKE_UP_SRC_REG_ADDRESS              0x1Bu  //!< Wake up interrupt source register address                              (Read only)
#define TAP_SRC_REG_ADDRESS                  0x1Cu  //!< Tap source register address                                            (Read only)
#define D6D_SRC_REG_ADDRESS                  0x1Du  //!< Portrait, landscape, face-up and face-down source register address     (Read only)
#define STATUS_REG_ADDRESS                   0x1Eu  //!< Status register address                                                (Read only)
#define OUT_TEMP_L_REG_ADDRESS               0x20u  //!< Temperature data output register address                               (Read only)
#define OUT_TEMP_H_REG_ADDRESS               0x21u  //!< Temperature data output register address                               (Read only)
#define OUTX_L_G_REG_ADDRESS                 0x22u  //!< Low byte of angular rate X-axis output register address                (Read only)
#define OUTX_H_G_REG_ADDRESS                 0x23u  //!< High byte of angular rate X-axis output register address               (Read only)
#define OUTY_L_G_REG_ADDRESS                 0x24u  //!< Low byte of angular rate Y-axis output register address                (Read only)
#define OUTY_H_G_REG_ADDRESS                 0x25u  //!< High byte of angular rate Y-axis output register address               (Read only)
#define OUTZ_L_G_REG_ADDRESS                 0x26u  //!< Low byte of angular rate Z-axis output register address                (Read only)
#define OUTZ_H_G_REG_ADDRESS                 0x27u  //!< High byte of angular rate Z-axis output register address               (Read only)
#define OUTX_L_XL_REG_ADDRESS                0x28u  //!< Low byte of linear acceleration sensor X-axis output register address  (Read only)
#define OUTX_H_XL_REG_ADDRESS                0x29u  //!< High byte of linear acceleration sensor X-axis output register address (Read only)
#define OUTY_L_XL_REG_ADDRESS                0x2Au  //!< Low byte of linear acceleration sensor Y-axis output register address  (Read only)
#define OUTY_H_XL_REG_ADDRESS                0x2Bu  //!< High byte of linear acceleration sensor Y-axis output register address (Read only)
#define OUTZ_L_XL_REG_ADDRESS                0x2Cu  //!< Low byte of linear acceleration sensor Z-axis output register address  (Read only)
#define OUTZ_H_XL_REG_ADDRESS                0x2Du  //!< High byte of linear acceleration sensor Z-axis output register address (Read only)
#define SENSORHUB1_REG_ADDRESS				0x2Eu	//  First byte associated to external sensors. 
#define SENSORHUB2_REG_ADDRESS				0x2Fu	//  Second byte associated to external sensors. 
#define SENSORHUB3_REG_ADDRESS				0x30u	//  Third byte associated to external sensors. 
#define SENSORHUB4_REG_ADDRESS				0x31u	//  Fourth byte associated to external sensors. 
#define SENSORHUB5_REG_ADDRESS				0x32u	//  Fifth byte associated to external sensors. 
#define SENSORHUB6_REG_ADDRESS				0x33u	//  Sixth byte associated to external sensors. 
#define SENSORHUB7_REG_ADDRESS				0x34u	//  Seventh byte associated to external sensors.
#define SENSORHUB8_REG_ADDRESS				0x35u	//  Eighth byte associated to external sensors.  
#define SENSORHUB9_REG_ADDRESS				0x36u	//  Ninth byte associated to external sensors. 
#define SENSORHUB10_REG_ADDRESS				0x37u	//  Tenth byte associated to external sensors.
#define SENSORHUB11_REG_ADDRESS				0x38u	//  Eleventh byte associated to external sensors.
#define SENSORHUB12_REG_ADDRESS				0x39u	//  Twelfth byte associated to external sensors.
#define FIFO_STATUS1_REG_ADDRESS             0x3Au  //!< FIFO status control register 1 address                                 (Read only)
#define FIFO_STATUS2_REG_ADDRESS             0x3Bu  //!< FIFO status control register 2 address                                 (Read only)
#define FIFO_STATUS3_REG_ADDRESS             0x3Cu  //!< FIFO status control register 3 address                                 (Read only)
#define FIFO_STATUS4_REG_ADDRESS             0x3Du  //!< FIFO status control register 4 address                                 (Read only)
#define FIFO_DATA_OUT_L_REG_ADDRESS          0x3Eu  //!< Low byte of FIFO data output register address                          (Read only)
#define FIFO_DATA_OUT_H_REG_ADDRESS          0x3Fu  //!< High byte of FIFO data output register address                         (Read only)

#define FUNC_SRC_REG_ADDRESS				0x53u	// Significant motion, tilt, step detector, hard/soft-iron and sensor hub interrupt source register

// TODO last registers
#define TAP_CFG_REG_ADDRESS                  0x58u  //!< Tap recognition configuration register address                         (Read/Write)
#define TAP_THS_6D_REG_ADDRESS               0x59u  //!< Portrait/landscape position and tap threshold register address         (Read/Write)
#define INT_DUR2_REG_ADDRESS                 0x5Au  //!< Tap recognition register address                                       (Read/Write)
#define FREE_FALL_REG_ADDRESS                0x5Du  //!< Free-Fall register address                                             (Read/Write)
#define MD1_CFG_REG_ADDRESS                  0x5Eu  //!< Routing on INT1 register address                                       (Read/Write)
#define MD2_CFG_REG_ADDRESS                  0x5Fu  //!< Routing on INT2 register address                                       (Read/Write)

// BANK A REGISTERS
#define SLV0_ADD_REG_ADDRESS					0x02u	// I2C slave address of the first external sensor
#define SLV0_SUBADD_REG_ADDRESS					0x03u	// Address of register on the first external sensor
#define SLAVE0_CONFIG_REG_ADDRESS				0x04u	// First external sensor configuration and sensor hub settings register
#define SLV1_ADD_REG_ADDRESS					0x05u
#define SLV1_SUBADD_REG_ADDRESS					0x06u
#define SLAVE1_CONFIG_REG_ADDRESS				0x07u
#define SLV2_ADD_REG_ADDRESS					0x08u
#define SLV2_SUBADD_REG_ADDRESS					0x09u
#define SLAVE2_CONFIG_REG_ADDRESS				0x0Au
#define SLV3_ADD_REG_ADDRESS					0x0Bu
#define SLV3_SUBADD_REG_ADDRESS					0x0Cu
#define SLAVE3_CONFIG_REG_ADDRESS				0x0Du
#define DATAWRITE_SRC_MODE_SUB_SLV0_REG_ADDRESS 0x0Eu	// Data to be written into the slave device register
#define MAG_SI_XX_REG_ADDRESS					0x24u
#define MAG_SI_XY_REG_ADDRESS					0x25u
#define MAG_SI_XZ_REG_ADDRESS					0x26u
#define MAG_SI_YX_REG_ADDRESS					0x27u
#define MAG_SI_YY_REG_ADDRESS					0x28u
#define MAG_SI_YZ_REG_ADDRESS					0x29u
#define MAG_SI_ZX_REG_ADDRESS					0x2Au
#define MAG_SI_ZY_REG_ADDRESS					0x2Bu
#define MAG_SI_ZZ_REG_ADDRESS					0x2Cu
#define MAG_OFFX_L_REG_ADDRESS					0x2Du
#define MAG_OFFX_H_REG_ADDRESS					0x2Eu
#define MAG_OFFY_L_REG_ADDRESS					0x2Fu
#define MAG_OFFY_H_REG_ADDRESS					0x30u
#define MAG_OFFZ_L_REG_ADDRESS					0x31u
#define MAG_OFFZ_H_REG_ADDRESS					0x32u

// Manufacturer ID
#define MANUFACTURER_ID                      0x69u  //!< Manufacturer ID

// Register bits mask
// CTRL1_XL bits mask
#define ACC_ODR_XL_BIT_MASK                  0x0Fu  //!< Mask of bit ODR_XL

// CTRL2_G bits mask
#define GYR_ODR_G_BIT_MASK                   0x0Fu  //!< Mask of bit ODR_G
#define GYR_FS_G_BIT_MASK                    0xF3u  //!< Mask of bit FS_G

// CTRL10_C bits mask
#define GYR_EN_G_BIT_MASK                    0x07u  //!< Mask of bit EN_G (+ 2bits MSB)

// TAP_CFG bits mask
#define ACC_TAP_EN_BIT_MASK                  0xF1u  //!< Mask of bit TAP_x_EN

// TAP_THS_6D bits mask
#define TAP_THS_BIT_MASK                     0xE0u  //!< Mask of bit TAP_THS

// FIFO_CTRL3 bits mask
#define DEC_FIFO_GYRO_BIT_MASK               0x07u  //!< Mask of bit DEC_FIFO_GYRO

// FIFO_CTRL5 bits mask
#define FIFO_MODE_BIT_MASK                   0x78u  //!< Mask of bit FIFO_MODE
#define ODR_FIFO_BIT_MASK                    0x07u  //!< Mask of bit ODR_FIFO

// Register bits position
// CTRL1_XL bits position
#define ACC_ODR_XL_BIT_POS                      4u  //!< Position of LSB bit ODR_XL

// CTRL2_G bits position
#define GYR_ODR_G_BIT_POS                       4u  //!< Position of LSB bit ODR_G
#define GYR_FS_G_BIT_POS                        2u  //!< Position of LSB bit FS_G

// CTRL10_C bits position
#define GYR_EN_G_BIT_POS                        3u  //!< Position of LSB bit of EN_G

// TAP_CFG bits position
#define ACC_TAP_EN_BIT_POS                      1u  //!< Position of LSB bit TAP_x_EN

// FIFO_CTRL3 bits position
#define DEC_FIFO_GYRO_BIT_POS                   3u  //!< Position of LSB bit DEC_FIFO_GYRO

// FIFO_CTRL5 bits position
#define ODR_FIFO_BIT_POS                        3u  //!< Position of LSB bit ODR_FIFO

// Magnetometer registers
/** I2C Device Address 8 bit format **/
#define LIS2MDL_ADDR                 0x3C // 0x1E = 7 bit address
/** Device Identification (Who am I) **/
#define LIS2MDL_ID                      0x40U

//-----------------------------------------------------------------------------
// Types Definitions
//-----------------------------------------------------------------------------

enum
{
  E_Acc_OutputDataRate_PowerDown,
  E_Acc_OutputDataRate_12_5Hz,
  E_Acc_OutputDataRate_26Hz,
  E_Acc_OutputDataRate_52Hz,
  E_Acc_OutputDataRate_104Hz,
  E_Acc_OutputDataRate_208Hz,
  E_Acc_OutputDataRate_416Hz,
  E_Acc_OutputDataRate_833Hz,
  E_Acc_OutputDataRate_1660Hz,
  E_Acc_OutputDataRate_3330Hz,
  E_Acc_OutputDataRate_6660Hz
};
typedef uint8_t T_Acc_OutputDataRate;  //!< Accelerometer output data rate

enum
{
  E_Acc_Tap_DisableAll = 0x00,
  E_Acc_Tap_EnableZ    = 0x01,
  E_Acc_Tap_EnableY    = 0x02,
  E_Acc_Tap_EnableYZ   = 0x03,
  E_Acc_Tap_EnableX    = 0x04,
  E_Acc_Tap_EnableXZ   = 0x05,
  E_Acc_Tap_EnableXY   = 0x06,
  E_Acc_Tap_EnableAll  = 0x07
};
typedef uint8_t T_Acc_Tap;  //!< Accelerometer tap recognition

enum
{
  E_Gyro_OutputDataRate_PowerDown,
  E_Gyro_OutputDataRate_12_5Hz,
  E_Gyro_OutputDataRate_26Hz,
  E_Gyro_OutputDataRate_52Hz,
  E_Gyro_OutputDataRate_104Hz,
  E_Gyro_OutputDataRate_208Hz,
  E_Gyro_OutputDataRate_416Hz,
  E_Gyro_OutputDataRate_833Hz,
  E_Gyro_OutputDataRate_1660Hz,
  E_Gyro_OutputDataRate_3330Hz,
  E_Gyro_OutputDataRate_6660Hz
};
typedef uint8_t T_Gyro_OutputDataRate;  //!< Gyroscope output data rate

enum
{
  E_Gyro_FullScale_250dps,
  E_Gyro_FullScale_500dps,
  E_Gyro_FullScale_1000dps,
  E_Gyro_FullScale_2000dps,
  E_Gyro_FullScale_125dps
};
typedef uint8_t T_Gyro_FullScale;  //!< Gyroscope output full-scale

enum
{
  E_Gyro_Disable_All,
  E_Gyro_Enable_X,
  E_Gyro_Enable_Y,
  E_Gyro_Enable_XY,
  E_Gyro_Enable_Z,
  E_Gyro_Enable_ZX,
  E_Gyro_Enable_ZY,
  E_Gyro_Enable_All
};
typedef uint8_t T_Gyro_EnableAxis;  //!< Gyroscope enable/disable axis

//-----------------------------------------------------------------------------
// Exported Global Data
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Private Data
//-----------------------------------------------------------------------------


//-----------------------------------------------------------------------------
// Private Functions Prototypes
//-----------------------------------------------------------------------------

//! \brief     Update the accelerometer output data rate
//! \pre       None
//! \param     None
//! \return    None
static void UpdateAccOutputDataRate(T_Acc_OutputDataRate rate);

//! \brief     Update the gyroscope output data rate
//! \pre       None
//! \param     None
//! \return    None
static void UpdateGyroOutputDataRate(T_Gyro_OutputDataRate rate);

//! \brief     Update the gyroscope full-scale
//! \pre       None
//! \param     None
//! \return    None
static void UpdateGyroFullScale(T_Gyro_FullScale scale);

//! \brief     Enable/disable the gyroscope axis
//! \pre       None
//! \param     None
//! \return    None
static void EnableGyroAxis(T_Gyro_EnableAxis config);


//! \brief     Read the manufacturer ID
//! \pre       None
//! \param     None
//! \return    None
//! \image     html ReadAccManufacturerId.svg
static void ReadManufacturerId(uint8_t* data);

//-----------------------------------------------------------------------------
// Inline Code Definition
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Functions Implementation
//-----------------------------------------------------------------------------

uint8_t LSM6DS3US_I2C_ReadFromAddress(uint8_t addr, uint8_t reg, uint8_t* data, uint8_t num) {
	uint8_t i = 0;
	uint8_t ret = 0;
	ret = i2c_start(addr+I2C_WRITE);							// set device address and write mode
	if(ret) {
		return 1;
	}
	ret = i2c_write(reg);											// sends address to read from (X LSB)
	if(ret) {
		return 1;
	}
	ret = i2c_rep_start(addr+I2C_READ);						// set device address and read mode
	if(ret) {
		return 1;
	}
	for(i=0; i<(num-1); i++) {
		data[i] = i2c_readAck();								// read one byte at a time
	}
	data[i] = i2c_readNak();									// read last byte sending NACK
	i2c_stop();	
	return 0;
}
		
void LSM6DS3US_I2C_WriteToAddress(uint8_t addr, uint8_t reg, uint8_t* data, uint8_t num) {
	i2c_start(addr+I2C_WRITE);	// set device address and write mode
	i2c_write(reg);
	for(uint8_t i=0; i<num; i++) {
		i2c_write(data[i]);
	}
	i2c_stop();			// set stop condition = release bus
}
	  
//-----------------------------------------------------------------------------
// Accelerometer
//-----------------------------------------------------------------------------

void LSM6DS3US_InitAccelerometer(void)
{
  UpdateAccOutputDataRate(E_Acc_OutputDataRate_104Hz);
}

//_____________________________________________________________________________

static void UpdateAccOutputDataRate(T_Acc_OutputDataRate rate)
{
  uint8_t data[1] = {0x00u};

  if (rate <= E_Acc_OutputDataRate_6660Hz)
  {	  
    LSM6DS3US_I2C_ReadFromAddress(SLAVE_ADDRESS, CTRL1_XL_REG_ADDRESS, data, 1u);

    data[0] &= ACC_ODR_XL_BIT_MASK;
    data[0] |= (rate << ACC_ODR_XL_BIT_POS);

    LSM6DS3US_I2C_WriteToAddress(SLAVE_ADDRESS, CTRL1_XL_REG_ADDRESS, data, 1u);
  }
}

//_____________________________________________________________________________

void LSM6DS3US_ReadAcceleration(int8_t* data)
{
	LSM6DS3US_I2C_ReadFromAddress(SLAVE_ADDRESS, OUTX_L_XL_REG_ADDRESS, (uint8_t*)data, 6u);
}

void LSM6DS3US_ReadAccelerationXY(int8_t* data)
{
	LSM6DS3US_I2C_ReadFromAddress(SLAVE_ADDRESS, OUTX_L_XL_REG_ADDRESS, (uint8_t*)data, 4u);
}

//-----------------------------------------------------------------------------
// Gyroscope
//-----------------------------------------------------------------------------

void LSM6DS3US_InitGyroscope(int16_t offset)
{
	uint8_t data[1] = {0x00u};

	// Set BDU flag
	LSM6DS3US_I2C_ReadFromAddress(SLAVE_ADDRESS, CTRL3_C_REG_ADDRESS, data, 1u);
	data[0] |= 0x40;
	LSM6DS3US_I2C_WriteToAddress(SLAVE_ADDRESS, CTRL3_C_REG_ADDRESS, data, 1u);

	//EnableGyroAxis(E_Gyro_Enable_All);
	//UpdateGyroOutputDataRate(E_Gyro_OutputDataRate_104Hz);
	//UpdateGyroFullScale(E_Gyro_FullScale_500dps);
	
	data[0] = 0x44; // ODR=104hz, full-scale=500dps
	LSM6DS3US_I2C_WriteToAddress(SLAVE_ADDRESS, CTRL2_G_REG_ADDRESS, data, 1u);

	//data[0] = 0x80; // high performance disabled
	//LSM6DS3US_I2C_WriteToAddress(SLAVE_ADDRESS, CTRL7_G_REG_ADDRESS, data, 1u);

}

//_____________________________________________________________________________

static void UpdateGyroOutputDataRate(T_Gyro_OutputDataRate rate)
{
  uint8_t data[1] = {0x00u};

  if (rate <= E_Gyro_OutputDataRate_6660Hz)
  {
    LSM6DS3US_I2C_ReadFromAddress(SLAVE_ADDRESS, CTRL2_G_REG_ADDRESS, data, 1u);

    data[0] &= GYR_ODR_G_BIT_MASK;
    data[0] |= (rate << GYR_ODR_G_BIT_POS);

    LSM6DS3US_I2C_WriteToAddress(SLAVE_ADDRESS, CTRL2_G_REG_ADDRESS, data, 1u);
    //SetIntegratorFactors(rate);
  }
}

//_____________________________________________________________________________

static void UpdateGyroFullScale(T_Gyro_FullScale scale)
{
  uint8_t data[1] = {0x00u};

  if (scale <= E_Gyro_FullScale_2000dps)
  {
    LSM6DS3US_I2C_ReadFromAddress(SLAVE_ADDRESS, CTRL2_G_REG_ADDRESS, data, 1u);

    data[0] &= GYR_FS_G_BIT_MASK;
    data[0] |= (scale << GYR_FS_G_BIT_POS);

    LSM6DS3US_I2C_WriteToAddress(SLAVE_ADDRESS, CTRL2_G_REG_ADDRESS, data, 1u);
  }
  else if(scale == E_Gyro_FullScale_125dps)
  {
    LSM6DS3US_I2C_ReadFromAddress(SLAVE_ADDRESS, CTRL2_G_REG_ADDRESS, data, 1u);

    data[0] &= 0xF0;
    data[0] |= 0x02;

    LSM6DS3US_I2C_WriteToAddress(SLAVE_ADDRESS, CTRL2_G_REG_ADDRESS, data, 1u);
  }
}

//_____________________________________________________________________________

static void EnableGyroAxis(T_Gyro_EnableAxis config)
{
/*
  uint8_t data[1] = {0x00u};
  //seems gyro is always on, no need to handle this

  if (config <= E_Gyro_Enable_All)
  {
    LSM6DS3US_I2C_ReadFromAddress(SLAVE_ADDRESS, CTRL10_C_REG_ADDRESS, data, 1u);

    data[0] &= GYR_EN_G_BIT_MASK;
    data[0] |= (config << GYR_EN_G_BIT_POS);

    LSM6DS3US_I2C_WriteToAddress(SLAVE_ADDRESS, CTRL10_C_REG_ADDRESS, data, 1u);
  }
  else
  {
    ESP_LOGE(Tag, "Invalid gyroscope axis configuration: %d", config);
  }
*/
}

//_____________________________________________________________________________

void LSM6DS3US_ReadAngularVelocity(int8_t* data)
{
	LSM6DS3US_I2C_ReadFromAddress(SLAVE_ADDRESS, OUTX_L_G_REG_ADDRESS, (uint8_t *)data, 6u);
}

extern void LSM6DS3US_ReadAngularVelocityZ(int8_t* data) {
	LSM6DS3US_I2C_ReadFromAddress(SLAVE_ADDRESS, OUTZ_L_G_REG_ADDRESS, (uint8_t *)data, 2u);
}

//-----------------------------------------------------------------------------
// Common
//-----------------------------------------------------------------------------

uint8_t LSM6DS3US_CheckManufacturerId(void)
{
  uint8_t id[1] = {0x00u};

  ReadManufacturerId(id);

  if (id[0] != MANUFACTURER_ID)
  {
    return 1;
  }

  return 0;
}

//_____________________________________________________________________________

static void ReadManufacturerId(uint8_t* data)
{
  LSM6DS3US_I2C_ReadFromAddress(SLAVE_ADDRESS, WHO_AM_I_REG_ADDRESS, data, 1u);
}

//-----------------------------------------------------------------------------
// Magnetometer
//-----------------------------------------------------------------------------

void LSM6DS3US_InitLIS2ML(void) {
	uint8_t data[1] = {0x00u};
	
	data[0] = 0x00; // Turn off the accelerometer
	LSM6DS3US_I2C_WriteToAddress(SLAVE_ADDRESS, CTRL1_XL_REG_ADDRESS, data, 1);
	
	data[0] = 0x80; // Enable access to embedded functions registers (bank A)
	LSM6DS3US_I2C_WriteToAddress(SLAVE_ADDRESS, FUNC_CFG_ACCESS_REG_ADDRESS, data, 1);
	
	data[0] = LIS2MDL_ADDR+I2C_WRITE; // LIS2MDL slave address, Enable write operation (rw_0=0)
	LSM6DS3US_I2C_WriteToAddress(SLAVE_ADDRESS, SLV0_ADD_REG_ADDRESS, data, 1);

	data[0] = 0x60; // 60h is the LIS2MDL register to be written
	LSM6DS3US_I2C_WriteToAddress(SLAVE_ADDRESS, SLV0_SUBADD_REG_ADDRESS, data, 1);

	data[0] = 0x8C; // 8Ch is the value to be written in register 60h of LIS2MDL to configure it in continuous mode, ODR = 100 Hz, temperature compensation enabled
	LSM6DS3US_I2C_WriteToAddress(SLAVE_ADDRESS, DATAWRITE_SRC_MODE_SUB_SLV0_REG_ADDRESS, data, 1);
	
	data[0] = 0x10; // Set Aux_sens_on bits different from 00b
	LSM6DS3US_I2C_WriteToAddress(SLAVE_ADDRESS, SLAVE0_CONFIG_REG_ADDRESS, data, 1);

	data[0] = 0x20; // Enable write_once bit => it is not described in the datasheet, but without this register setting, the sensor cannot be configured correctly
	LSM6DS3US_I2C_WriteToAddress(SLAVE_ADDRESS, SLAVE1_CONFIG_REG_ADDRESS, data, 1);

	data[0] = 0x00; // Disable access to embedded functions registers (bank A)
	LSM6DS3US_I2C_WriteToAddress(SLAVE_ADDRESS, FUNC_CFG_ACCESS_REG_ADDRESS, data, 1);
	
	data[0] = 0x04; // Enable embedded functions (and disable gyro)
	LSM6DS3US_I2C_WriteToAddress(SLAVE_ADDRESS, CTRL10_C_REG_ADDRESS, data, 1);	

	int trials = 0;
	while(trials < 3) {
		
		data[0] = 0x09; // Enable internal pull-up on SDx/SCx lines, Sensor hub trigger signal is XL Data-Ready, Enable auxiliary I2C master
		LSM6DS3US_I2C_WriteToAddress(SLAVE_ADDRESS, MASTER_CONFIG_REG_ADDRESS, data, 1);

		data[0] = 0x80; // Turn on the accelerometer (for trigger signal)
		LSM6DS3US_I2C_WriteToAddress(SLAVE_ADDRESS, CTRL1_XL_REG_ADDRESS, data, 1);

		// Wait for the sensor hub communication to be concluded
		LSM6DS3US_I2C_ReadFromAddress(SLAVE_ADDRESS, FUNC_SRC_REG_ADDRESS, data, 1u);
		if((data[0]&0x01)==0x01) {
			break;
		}
	
		trials++;
	}

	data[0] = 0x00; // Disable embedded functions (and disable gyro)
	LSM6DS3US_I2C_WriteToAddress(SLAVE_ADDRESS, CTRL10_C_REG_ADDRESS, data, 1);	

	data[0] = 0x00; // Disable auxiliary I2C master
	LSM6DS3US_I2C_WriteToAddress(SLAVE_ADDRESS, MASTER_CONFIG_REG_ADDRESS, data, 1);
	
	data[0] = 0x00; // Turn off the accelerometer
	LSM6DS3US_I2C_WriteToAddress(SLAVE_ADDRESS, CTRL1_XL_REG_ADDRESS, data, 1);
	
	data[0] = 0x80; // Enable access to embedded functions registers (bank A)
	LSM6DS3US_I2C_WriteToAddress(SLAVE_ADDRESS, FUNC_CFG_ACCESS_REG_ADDRESS, data, 1);
	
	data[0] = LIS2MDL_ADDR+I2C_READ; // LIS2MDL slave address, Enable read operation (rw_0=1)
	LSM6DS3US_I2C_WriteToAddress(SLAVE_ADDRESS, SLV0_ADD_REG_ADDRESS, data, 1);
	
	data[0] = 0x68; // 68h is the first LIS2MDL output register to be read
	LSM6DS3US_I2C_WriteToAddress(SLAVE_ADDRESS, SLV0_SUBADD_REG_ADDRESS, data, 1);	

	data[0] = 0x06; // No decimation, 1 external sensor connected, Number of registers to read = 6
	LSM6DS3US_I2C_WriteToAddress(SLAVE_ADDRESS, SLAVE0_CONFIG_REG_ADDRESS, data, 1);

	data[0] = 0x00; // Disable access to embedded functions registers (bank A)
	LSM6DS3US_I2C_WriteToAddress(SLAVE_ADDRESS, FUNC_CFG_ACCESS_REG_ADDRESS, data, 1);

	data[0] = 0x3C; // Enable embedded functions and enable all gyro axes
	LSM6DS3US_I2C_WriteToAddress(SLAVE_ADDRESS, CTRL10_C_REG_ADDRESS, data, 1);

	data[0] = 0x09; // Enable internal pull-up on SDx/SCx lines, Sensor hub trigger signal is XL Data-Ready, Enable auxiliary I2C master
	LSM6DS3US_I2C_WriteToAddress(SLAVE_ADDRESS, MASTER_CONFIG_REG_ADDRESS, data, 1);

}

void LSM6DS3US_InitLIS2MLWithCal(void) {
	
	uint8_t data[1] = {0x00u};
	
	data[0] = 0x00; // Turn off the accelerometer
	LSM6DS3US_I2C_WriteToAddress(SLAVE_ADDRESS, CTRL1_XL_REG_ADDRESS, data, 1);	
	
	data[0] = 0x80; // Enable access to embedded functions registers (bank A)
	LSM6DS3US_I2C_WriteToAddress(SLAVE_ADDRESS, FUNC_CFG_ACCESS_REG_ADDRESS, data, 1);
	
	data[0] = LIS2MDL_ADDR+I2C_WRITE; // LIS2MDL slave address, Enable write operation (rw_0=0)
	LSM6DS3US_I2C_WriteToAddress(SLAVE_ADDRESS, SLV0_ADD_REG_ADDRESS, data, 1);

	data[0] = 0x60; // 60h is the LIS2MDL register to be written
	LSM6DS3US_I2C_WriteToAddress(SLAVE_ADDRESS, SLV0_SUBADD_REG_ADDRESS, data, 1);

	data[0] = 0x8C; // 8Ch is the value to be written in register 60h of LIS2MDL to configure it in continuous mode, ODR = 100 Hz, temperature compensation enabled
	LSM6DS3US_I2C_WriteToAddress(SLAVE_ADDRESS, DATAWRITE_SRC_MODE_SUB_SLV0_REG_ADDRESS, data, 1);
	
	data[0] = 0x10; // Set Aux_sens_on bits different from 00b
	LSM6DS3US_I2C_WriteToAddress(SLAVE_ADDRESS, SLAVE0_CONFIG_REG_ADDRESS, data, 1);

	data[0] = 0x20; // Enable write_once bit => it is not described in the datasheet, but without this register setting, the sensor cannot be configured correctly
	LSM6DS3US_I2C_WriteToAddress(SLAVE_ADDRESS, SLAVE1_CONFIG_REG_ADDRESS, data, 1);

	data[0] = 0x00; // Disable access to embedded functions registers (bank A)
	LSM6DS3US_I2C_WriteToAddress(SLAVE_ADDRESS, FUNC_CFG_ACCESS_REG_ADDRESS, data, 1);
	
	data[0] = 0x04; // Enable embedded functions (and disable gyro)
	LSM6DS3US_I2C_WriteToAddress(SLAVE_ADDRESS, CTRL10_C_REG_ADDRESS, data, 1);

	int trials = 0;
	while(trials < 3) {
		
		data[0] = 0x09; // Enable internal pull-up on SDx/SCx lines, Sensor hub trigger signal is XL Data-Ready, Enable auxiliary I2C master
		LSM6DS3US_I2C_WriteToAddress(SLAVE_ADDRESS, MASTER_CONFIG_REG_ADDRESS, data, 1);

		data[0] = 0x80; // Turn on the accelerometer (for trigger signal)
		LSM6DS3US_I2C_WriteToAddress(SLAVE_ADDRESS, CTRL1_XL_REG_ADDRESS, data, 1);

		// Wait for the sensor hub communication to be concluded
		LSM6DS3US_I2C_ReadFromAddress(SLAVE_ADDRESS, FUNC_SRC_REG_ADDRESS, data, 1u);
		if((data[0]&0x01)==0x01) {
			break;
		}
		
		trials++;
	}

	data[0] = 0x00; // Disable embedded functions (and disable gyro)
	LSM6DS3US_I2C_WriteToAddress(SLAVE_ADDRESS, CTRL10_C_REG_ADDRESS, data, 1);

	data[0] = 0x00; // Disable auxiliary I2C master
	LSM6DS3US_I2C_WriteToAddress(SLAVE_ADDRESS, MASTER_CONFIG_REG_ADDRESS, data, 1);

	data[0] = 0x00; // Turn off the accelerometer
	LSM6DS3US_I2C_WriteToAddress(SLAVE_ADDRESS, CTRL1_XL_REG_ADDRESS, data, 1);
		
	data[0] = 0x80; // Enable access to embedded functions registers (bank A)
	LSM6DS3US_I2C_WriteToAddress(SLAVE_ADDRESS, FUNC_CFG_ACCESS_REG_ADDRESS, data, 1);
	
	data[0] = LIS2MDL_ADDR+I2C_READ; // LIS2MDL slave address, Enable read operation (rw_0=1)
	LSM6DS3US_I2C_WriteToAddress(SLAVE_ADDRESS, SLV0_ADD_REG_ADDRESS, data, 1);
	
	data[0] = 0x68; // 68h is the first LIS2MDL output register to be read
	LSM6DS3US_I2C_WriteToAddress(SLAVE_ADDRESS, SLV0_SUBADD_REG_ADDRESS, data, 1);

	data[0] = 0x06; // No decimation, 1 external sensor connected, Number of registers to read = 6
	LSM6DS3US_I2C_WriteToAddress(SLAVE_ADDRESS, SLAVE0_CONFIG_REG_ADDRESS, data, 1);

/*
	// 4171
	// Hard-iron compensation
	// X=3.43, Y=-14.87, Z=28.29 uT
	// => uT to mGauss => multiply by 10
	// X=34, Y=-149, Z=283
	data[0] = 0x00;
	LSM6DS3US_I2C_WriteToAddress(SLAVE_ADDRESS,  MAG_OFFX_H_REG_ADDRESS, data, 1); // X offset value initialization
	data[0] = 0x22;
	LSM6DS3US_I2C_WriteToAddress(SLAVE_ADDRESS,  MAG_OFFX_L_REG_ADDRESS, data, 1); // X offset value initialization
	data[0] = 0xFF;
	LSM6DS3US_I2C_WriteToAddress(SLAVE_ADDRESS,  MAG_OFFY_H_REG_ADDRESS, data, 1); // Y offset value initialization
	data[0] = 0x6B;
	LSM6DS3US_I2C_WriteToAddress(SLAVE_ADDRESS,  MAG_OFFY_L_REG_ADDRESS, data, 1); // Y offset value initialization
	data[0] = 0x01;
	LSM6DS3US_I2C_WriteToAddress(SLAVE_ADDRESS,  MAG_OFFZ_H_REG_ADDRESS, data, 1); // Z offset value initialization
	data[0] = 0x1B;
	LSM6DS3US_I2C_WriteToAddress(SLAVE_ADDRESS,  MAG_OFFZ_L_REG_ADDRESS, data, 1); // Z offset value initialization	
	
	// Soft-iron compensation
	//	+0.982	+0.014	+0.021
	//	+0.014	+0.978	-0.002
	//	+0.021	-0.002	+1.042
	// Multiplied by 8 in order to get the LSB values to be written in the soft-iron correction registers:
	//	8	0	0
	//	0	8	0
	//	0	0	8
	data[0] = 0x08;
	LSM6DS3US_I2C_WriteToAddress(SLAVE_ADDRESS,  MAG_SI_XX_REG_ADDRESS, data, 1); // XX soft-iron element
	data[0] = 0x00;
	LSM6DS3US_I2C_WriteToAddress(SLAVE_ADDRESS,  MAG_SI_XY_REG_ADDRESS, data, 1); // XY soft-iron element
	data[0] = 0x00;
	LSM6DS3US_I2C_WriteToAddress(SLAVE_ADDRESS,  MAG_SI_XZ_REG_ADDRESS, data, 1); // XZ soft-iron element
	data[0] = 0x00;
	LSM6DS3US_I2C_WriteToAddress(SLAVE_ADDRESS,  MAG_SI_YX_REG_ADDRESS, data, 1); // YX soft-iron element
	data[0] = 0x08;
	LSM6DS3US_I2C_WriteToAddress(SLAVE_ADDRESS,  MAG_SI_YY_REG_ADDRESS, data, 1); // YY soft-iron element
	data[0] = 0x00;
	LSM6DS3US_I2C_WriteToAddress(SLAVE_ADDRESS,  MAG_SI_YZ_REG_ADDRESS, data, 1); // YZ soft-iron element
	data[0] = 0x00;
	LSM6DS3US_I2C_WriteToAddress(SLAVE_ADDRESS,  MAG_SI_ZX_REG_ADDRESS, data, 1); // ZX soft-iron element
	data[0] = 0x00;
	LSM6DS3US_I2C_WriteToAddress(SLAVE_ADDRESS,  MAG_SI_ZY_REG_ADDRESS, data, 1); // ZY soft-iron element
	data[0] = 0x08;
	LSM6DS3US_I2C_WriteToAddress(SLAVE_ADDRESS,  MAG_SI_ZZ_REG_ADDRESS, data, 1); // ZZ soft-iron element				
*/

/*
	// 4170 - 1
	// Hard-iron compensation
	// X=-3.12, Y=-34.21, Z=9.35 uT
	// => uT to mGauss => multiply by 10
	// X=-31, Y=-342, Z=93
	data[0] = 0xFF;
	LSM6DS3US_I2C_WriteToAddress(SLAVE_ADDRESS,  MAG_OFFX_H_REG_ADDRESS, data, 1); // X offset value initialization
	data[0] = 0xE1;
	LSM6DS3US_I2C_WriteToAddress(SLAVE_ADDRESS,  MAG_OFFX_L_REG_ADDRESS, data, 1); // X offset value initialization
	data[0] = 0xFE;
	LSM6DS3US_I2C_WriteToAddress(SLAVE_ADDRESS,  MAG_OFFY_H_REG_ADDRESS, data, 1); // Y offset value initialization
	data[0] = 0xAA;
	LSM6DS3US_I2C_WriteToAddress(SLAVE_ADDRESS,  MAG_OFFY_L_REG_ADDRESS, data, 1); // Y offset value initialization
	data[0] = 0x00;
	LSM6DS3US_I2C_WriteToAddress(SLAVE_ADDRESS,  MAG_OFFZ_H_REG_ADDRESS, data, 1); // Z offset value initialization
	data[0] = 0x5D;
	LSM6DS3US_I2C_WriteToAddress(SLAVE_ADDRESS,  MAG_OFFZ_L_REG_ADDRESS, data, 1); // Z offset value initialization
*/

/*	
	// Soft-iron compensation
	//	+0.959	+0.010	+0.010
	//	+0.010	+1.003	-0.015
	//	+0.010	-0.015	+1.040
	// Multiplied by 8 in order to get the LSB values to be written in the soft-iron correction registers:
	//	8	0	0
	//	0	8	0
	//	0	0	8
	data[0] = 0x08;
	LSM6DS3US_I2C_WriteToAddress(SLAVE_ADDRESS,  MAG_SI_XX_REG_ADDRESS, data, 1); // XX soft-iron element
	data[0] = 0x00;
	LSM6DS3US_I2C_WriteToAddress(SLAVE_ADDRESS,  MAG_SI_XY_REG_ADDRESS, data, 1); // XY soft-iron element
	data[0] = 0x00;
	LSM6DS3US_I2C_WriteToAddress(SLAVE_ADDRESS,  MAG_SI_XZ_REG_ADDRESS, data, 1); // XZ soft-iron element
	data[0] = 0x00;
	LSM6DS3US_I2C_WriteToAddress(SLAVE_ADDRESS,  MAG_SI_YX_REG_ADDRESS, data, 1); // YX soft-iron element
	data[0] = 0x08;
	LSM6DS3US_I2C_WriteToAddress(SLAVE_ADDRESS,  MAG_SI_YY_REG_ADDRESS, data, 1); // YY soft-iron element
	data[0] = 0x00;
	LSM6DS3US_I2C_WriteToAddress(SLAVE_ADDRESS,  MAG_SI_YZ_REG_ADDRESS, data, 1); // YZ soft-iron element
	data[0] = 0x00;
	LSM6DS3US_I2C_WriteToAddress(SLAVE_ADDRESS,  MAG_SI_ZX_REG_ADDRESS, data, 1); // ZX soft-iron element
	data[0] = 0x00;
	LSM6DS3US_I2C_WriteToAddress(SLAVE_ADDRESS,  MAG_SI_ZY_REG_ADDRESS, data, 1); // ZY soft-iron element
	data[0] = 0x08;
	LSM6DS3US_I2C_WriteToAddress(SLAVE_ADDRESS,  MAG_SI_ZZ_REG_ADDRESS, data, 1); // ZZ soft-iron element
*/
/*
	// 4170 - 3
	// Hard-iron compensation
	// X=1.84, Y=-35.05, Z=-14.69 uT
	// => uT to mGauss => multiply by 10
	// X=18, Y=-350, Z=-147
	data[0] = 0x00;
	LSM6DS3US_I2C_WriteToAddress(SLAVE_ADDRESS,  MAG_OFFX_H_REG_ADDRESS, data, 1); // X offset value initialization
	data[0] = 0x12;
	LSM6DS3US_I2C_WriteToAddress(SLAVE_ADDRESS,  MAG_OFFX_L_REG_ADDRESS, data, 1); // X offset value initialization
	data[0] = 0xFE;
	LSM6DS3US_I2C_WriteToAddress(SLAVE_ADDRESS,  MAG_OFFY_H_REG_ADDRESS, data, 1); // Y offset value initialization
	data[0] = 0xA2;
	LSM6DS3US_I2C_WriteToAddress(SLAVE_ADDRESS,  MAG_OFFY_L_REG_ADDRESS, data, 1); // Y offset value initialization
	data[0] = 0xFF;
	LSM6DS3US_I2C_WriteToAddress(SLAVE_ADDRESS,  MAG_OFFZ_H_REG_ADDRESS, data, 1); // Z offset value initialization
	data[0] = 0x6D;
	LSM6DS3US_I2C_WriteToAddress(SLAVE_ADDRESS,  MAG_OFFZ_L_REG_ADDRESS, data, 1); // Z offset value initialization
*/
	data[0] = magOffset[0]>>8;
	LSM6DS3US_I2C_WriteToAddress(SLAVE_ADDRESS,  MAG_OFFX_H_REG_ADDRESS, data, 1); // X offset value initialization
	data[0] = magOffset[0]&0xFF;
	LSM6DS3US_I2C_WriteToAddress(SLAVE_ADDRESS,  MAG_OFFX_L_REG_ADDRESS, data, 1); // X offset value initialization
	data[0] = magOffset[1]>>8;
	LSM6DS3US_I2C_WriteToAddress(SLAVE_ADDRESS,  MAG_OFFY_H_REG_ADDRESS, data, 1); // Y offset value initialization
	data[0] = magOffset[1]&0xFF;
	LSM6DS3US_I2C_WriteToAddress(SLAVE_ADDRESS,  MAG_OFFY_L_REG_ADDRESS, data, 1); // Y offset value initialization
	data[0] = magOffset[2]>>8;
	LSM6DS3US_I2C_WriteToAddress(SLAVE_ADDRESS,  MAG_OFFZ_H_REG_ADDRESS, data, 1); // Z offset value initialization
	data[0] = magOffset[2]&0xFF;
	LSM6DS3US_I2C_WriteToAddress(SLAVE_ADDRESS,  MAG_OFFZ_L_REG_ADDRESS, data, 1); // Z offset value initialization

	data[0] = 0x00; // Disable access to embedded functions registers (bank A)
	LSM6DS3US_I2C_WriteToAddress(SLAVE_ADDRESS, FUNC_CFG_ACCESS_REG_ADDRESS, data, 1);

	data[0] = 0x3C; // Enable embedded functions and enable all gyro axes
	LSM6DS3US_I2C_WriteToAddress(SLAVE_ADDRESS, CTRL10_C_REG_ADDRESS, data, 1);
	
	data[0] = 0x0B; // Enable internal pull-up on SDx/SCx lines, Sensor hub trigger signal is XL Data-Ready, Enable hard-iron correction, Enable auxiliary I2C master
	LSM6DS3US_I2C_WriteToAddress(SLAVE_ADDRESS, MASTER_CONFIG_REG_ADDRESS, data, 1);

	//data[0] = 0x3C; // Enable all acc axes + enable soft-iron correction
	//LSM6DS3US_I2C_WriteToAddress(SLAVE_ADDRESS, CTRL9_XL_REG_ADDRESS, data, 1);

}

void LSM6DS3US_ReadLIS2ML(int8_t* data) {
	LSM6DS3US_I2C_ReadFromAddress(SLAVE_ADDRESS, SENSORHUB1_REG_ADDRESS, (uint8_t*)data, 6u);
}

uint8_t LSM6DS3US_ReadAll(int8_t* data) {
	return LSM6DS3US_I2C_ReadFromAddress(SLAVE_ADDRESS, OUTX_L_G_REG_ADDRESS, (uint8_t *)data, 18);
}
