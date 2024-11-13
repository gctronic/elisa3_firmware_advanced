/**
 * \file sensors.h
 * \brief LSM6DS3TR acc+gyro module
 * \author Stefano Morgani <stefano@gctronic.com>
 * \version 1.0
 * \date 05.11.24
 * \copyright GNU GPL v3
*/

#ifndef LSM6DS3TR_H_
#define LSM6DS3TR_H_

//-----------------------------------------------------------------------------
// Include Section
//-----------------------------------------------------------------------------

#include "twimaster.h"

//-----------------------------------------------------------------------------
// Constants/Macros Definitions
//-----------------------------------------------------------------------------
#define GYRO_BUFFER_SIZE 128

//-----------------------------------------------------------------------------
// Types Definitions
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Exported Global Data
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Inline Code Definition
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Exported Functions Prototypes
//-----------------------------------------------------------------------------

//! \brief     Initialize the accelerometer of the LSM6DS3TR device
//! \pre       None
//! \param     None
//! \return    None
extern void LSM6DS3TR_InitAccelerometer(void);

//! \brief     Get the acceleration in [???]
//! \pre       First initialize the LSM6DS3TR device
//! \param     acceleration - Acceleration
//! \return    None
//! \image     html ReadAcceleration.svg
extern void LSM6DS3TR_ReadAcceleration(int8_t* data);
extern void LSM6DS3TR_ReadAccelerationXY(int8_t* data);

//! \brief     Get the acceleration tap source
//! \pre       First initialize the LSM6DS3TR device
//! \param     source - Source of the tap
//! \return    None
extern void LSM6DS3TR_ReadTapSource(uint8_t* source);

//! \brief     Initialize the gyroscope of the LSM6DS3TR device
//! \pre       None
//! \param     None
//! \return    None
extern void LSM6DS3TR_InitGyroscope(int16_t offset);

//! \brief     Get the angular velocity in [???]
//! \pre       First initialize the LSM6DS3TR device
//! \param     angularPosition - Angular position
//! \return    None
//! \image     html ReadAngle.svg
extern void LSM6DS3TR_ReadAngularVelocity(int8_t* data);
extern void LSM6DS3TR_ReadAngularVelocityZ(int8_t* data);

//! \brief     Read the buffered data from the gyroscope
//! \pre       First initialize the LSM6DS3TR device
//! \param     None
//! \return    Number of samples read
extern uint16_t LSM6DS3TR_ReadBufferedAngularPosition(void);

extern void LSM6DS3TR_SetOffset(int32_t offset);

//! \brief     Check the manufacturer ID
//! \pre       None
//! \param     None
//! \return    E_Error_None if no error, otherwise E_Error_Acc_InvalidID
extern uint8_t LSM6DS3TR_CheckManufacturerId(void);

void LSM6DS3TR_InitLIS2ML(void);
void LSM6DS3TR_InitLIS2MLWithCal(void);
void LSM6DS3TR_ReadLIS2ML(int8_t* data);
uint8_t LSM6DS3TR_ReadAll(int8_t* data);

#endif // LSM6DS3TR_H_
