/**
 * \file sensors.h
 * \brief LSM6DS3US acc+gyro module
 * \author Stefano Morgani <stefano@gctronic.com>
 * \version 1.0
 * \date 05.11.24
 * \copyright GNU GPL v3
*/

#ifndef LSM6DS3US_H_
#define LSM6DS3US_H_

//-----------------------------------------------------------------------------
// Include Section
//-----------------------------------------------------------------------------

#include "twimaster.h"

//-----------------------------------------------------------------------------
// Constants/Macros Definitions
//-----------------------------------------------------------------------------

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

//! \brief     Initialize the accelerometer of the LSM6DS3US device
//! \pre       None
//! \param     None
//! \return    None
extern void LSM6DS3US_InitAccelerometer(void);

//! \brief     Get the acceleration in [???]
//! \pre       First initialize the LSM6DS3US device
//! \param     acceleration - Acceleration
//! \return    None
//! \image     html ReadAcceleration.svg
extern void LSM6DS3US_ReadAcceleration(int8_t* data);
extern void LSM6DS3US_ReadAccelerationXY(int8_t* data);

//! \brief     Initialize the gyroscope of the LSM6DS3US device
//! \pre       None
//! \param     None
//! \return    None
extern void LSM6DS3US_InitGyroscope(int16_t offset);

//! \brief     Get the angular velocity in [???]
//! \pre       First initialize the LSM6DS3US device
//! \param     angularPosition - Angular position
//! \return    None
//! \image     html ReadAngle.svg
extern void LSM6DS3US_ReadAngularVelocity(int8_t* data);
extern void LSM6DS3US_ReadAngularVelocityZ(int8_t* data);

//! \brief     Check the manufacturer ID
//! \pre       None
//! \param     None
//! \return    E_Error_None if no error, otherwise E_Error_Acc_InvalidID
extern uint8_t LSM6DS3US_CheckManufacturerId(void);

void LSM6DS3US_InitLIS2ML(void);
void LSM6DS3US_InitLIS2MLWithCal(void);
void LSM6DS3US_ReadLIS2ML(int8_t* data);
uint8_t LSM6DS3US_ReadAll(int8_t* data);

#endif // LSM6DS3US_H_
