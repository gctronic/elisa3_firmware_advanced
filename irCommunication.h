
#ifndef IR_COMMUNICATION_H
#define IR_COMMUNICATION_H


/**
 * \file usart.h
 * \brief Usart module
 * \author Stefano Morgani <stefano@gctronic.com>
 * \version 1.0
 * \date 01.02.12
 * \copyright GNU GPL v3

 The usart peripheral is used primarly for debugging purposes; it's initialized to work at 57600 baud that 
 is the maximum throughput usable with the main clock at 8 MHz. An interrupt is generated at each character 
 reception; a function for transfer data is also available.
*/


#include "variables.h"
#include "utility.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief Initialize the IR communication either in reception or transmission mode.
 * \ param mode: IRCOMM_MODE_RECEIVE or IRCOMM_MODE_TRANSMIT
 * \return none
 */

void irCommInit();

void irCommInitReceiver();

void irCommInitTransmitter();

/**
 * \brief Stop the IR communication, only sensors sampling for obstacle and cliff avoidance is enabled.
 * \return none
 */
void irCommDeinit();

/**
 * \brief Handle the internal state machine of the IR communication. Need to be called repeatedly from the main loop.
 * \return none
 */
void irCommTasks();

/**
 * \brief Set the data to be sent through IR; the obstacle avoidance and cliff avoidance will be disabled during the transmission.
 * \param value: byte to be sent; sensor: sensor id mask (bit 0 corresponds to sensor 0 in front of robot, id increases clockwise).
 * \return none
 */
void irCommSendData(unsigned char value, unsigned char sensorMask);

/**
 * \brief Tell whether the last transmission is terminated or not.
 * \return 1 if the last byte is sent, 0 otherwise
 */
unsigned char irCommDataSent();

/**
 * \brief Tell whether incoming data are received or not.
 * \return 1 if one byte is received, 0 otherwise
 */
unsigned char irCommDataAvailable();

/**
 * \brief Get the last byte received through IR.
 * \return the value of the byte received
 */
unsigned char irCommReadData();

/**
 * \brief Get the last sensor id that receives the message.
 * \return the sensor id (0..7); 0 is the front sensor, sensors id increases clockwise
 */
signed char irCommReceivingSensor();


#ifdef __cplusplus
} // extern "C"
#endif

#endif
