
#include <avr\io.h>
#include <avr\interrupt.h>

#include <stdlib.h>
#include "variables.h"
#include "utility.h"
#include "speed_control.h"
#include "nRF24L01.h"
#include "behaviors.h"
#include "sensors.h"


int main(void) {

	unsigned long int startTime = 0, endTime = 0;
	unsigned char prevSelector=0;
	unsigned int i=0;

	initPeripherals();

	calibrateSensors();

	initBehaviors();

	startTime = getTime100MicroSec();


	while(1) {

		currentSelector = getSelector();	// update selector position

		readAccelXYZ();						// update accelerometer values to compute the angle

		computeAngle();
	
		endTime = getTime100MicroSec();
		if((endTime-startTime) >= (PAUSE_2_SEC)) {
			readBatteryLevel();				// the battery level is updated every two seconds
             		
			if(currentSelector==4 || currentSelector==5 || currentSelector==7) {
				pwm_red = rand() % 255;
				pwm_green = rand() % 255;
				pwm_blue = rand() % 255;
			} else if(currentSelector==6) {
				if(menuChoice==1 && rfFlags<=1) {
					if(rgbState == 0) {
						pwm_red = 254;
						pwm_green = 255;
						pwm_blue = 255;
						rgbState = 1;
					} else if(rgbState == 1) {
						pwm_red = 255;
						pwm_green = 254;
						pwm_blue = 255;
						rgbState = 2;
					} else if(rgbState == 2) {
						pwm_red = 255;
						pwm_green = 255;
						pwm_blue = 254;
						rgbState = 0;
					}
				}
			}

			startTime = getTime100MicroSec();
		}


		handleIRRemoteCommands();


		handleRFCommands();

		if(currentSelector != 6) {
			usart0Transmit(currentSelector,0);		// send the current selector position through uart as debug info
		}

		switch(currentSelector) {
    
			case 0:	// motors in direct power control (no speed control)
					handleMotorsWithNoController();
					break;
             
			case 1:	// obstacle avoidance enabled (the robot does not move untill commands are 
					// received from the radio or tv remote)
             		enableObstacleAvoidance();
					break;
             
			case 2:	// cliff avoidance enabled (the robot does not move untill commands are 
					// received from the radio or tv remote)
             		enableCliffAvoidance();
					break;
    
			case 3:	// both obstacle and cliff avoidance enabled (the robot does not move untill commands are
					// received from the radio or tv remote)
            		enableObstacleAvoidance();
					enableCliffAvoidance();
					break;
            
			case 4:	// random colors on RGB leds; small green leds turned on
					GREEN_LED0_ON;
					GREEN_LED1_ON;
					GREEN_LED2_ON;
					GREEN_LED3_ON;
					GREEN_LED4_ON;
					GREEN_LED5_ON;
					GREEN_LED6_ON;
					GREEN_LED7_ON;
					updateRedLed(pwm_red);
					updateGreenLed(pwm_green);
					updateBlueLed(pwm_blue);
					break;
             
			case 5:	// random colors on RGB leds; obstacle avoidance enabled; robot start moving automatically
					// (motors speed setting)
					updateRedLed(pwm_red);
					updateGreenLed(pwm_green);
					updateBlueLed(pwm_blue);
					enableObstacleAvoidance();
					setLeftSpeed(25);
					setRightSpeed(25);
					break;

			case 6:	// robot testing
					switch(menuChoice) {
						case 0:
							setRightSpeed(0);
							setLeftSpeed(0);
							turnOffGreenLeds();
							pwm_red=255;
							pwm_green=255;
							pwm_blue=255;
							updateRedLed(pwm_red);
							updateGreenLed(pwm_green);
							updateBlueLed(pwm_blue);
							LED_IR1_HIGH;
							LED_IR2_HIGH;
							break;

						case 1:	// send sensors data and activate actuators
							//setRightSpeed(20);
							//setLeftSpeed(20);
							turnOnGreenLeds();
							updateRedLed(pwm_red);
							updateGreenLed(pwm_green);
							updateBlueLed(pwm_blue);
							LED_IR1_LOW;
							LED_IR2_LOW;
							
							if(getDataNow) {
								getDataNow = 0;	
								for(i=0; i<12; i++) {
									usart0Transmit(proximityResult[i]&0xFF,1);
									usart0Transmit(proximityResult[i]>>8,1);
									usart0Transmit(proximityValue[i*2]&0xFF,1);
									usart0Transmit(proximityValue[i*2]>>8,1);
								}
								usart0Transmit(accX&0xFF,1);
								usart0Transmit(accX>>8,1);
								usart0Transmit(accY&0xFF,1);
								usart0Transmit(accY>>8,1);
								usart0Transmit(accZ&0xFF,1);
								usart0Transmit(accZ>>8,1);
								usart0Transmit(irCommand,1);
								usart0Transmit(currentSelector,1);
								usart0Transmit(BUTTON0,1);
								usart0Transmit(rfFlags,1);
							}

							break;

						case 2:	// address writing in eeprom
							if(addressReceived) {
								turnOnGreenLeds();
								eeprom_write_word((uint16_t*)4094, rfAddress);
								turnOffGreenLeds();
								usart0Transmit(0xAA, 1);	// successfully written
								addressReceived = 0;
								menuChoice = 0;
							}
							break;
					}
					break;
			
			case 7:
					switch(demoState) {
						case 0:	// move around
							turnOnGreenLeds();
							lineFound = 0;
							enableObstacleAvoidance();
							setRightSpeed(20);
							setLeftSpeed(20);
							demoEndTime = getTime100MicroSec();
							if((demoEndTime-demoStartTime) >= (PAUSE_20_SEC)) {
								demoState = 1;
							}
							//pwm_red = 0;
							//pwm_green = 255;
							//pwm_blue = 255;
							break;

						case 1:	// search for a line
							turnOffGreenLeds();
							outOfLine = 0;
							enableObstacleAvoidance();
							setRightSpeed(20);
							setLeftSpeed(20);
							if(proximityResult[9]<LINE_IN_THR || proximityResult[10]<LINE_IN_THR) {
								lineFound++;
								if(lineFound > 10) {
									outOfLine = 0;
									chargeContact = 0;
									demoStartTime = getTime100MicroSec();
									demoState = 2;
									break;
								}
							} else {
								lineFound = 0;
							}
							/*
							if(CHARGE_ON) {
								chargeContact++;
								if(chargeContact > 20) {
									setLeftSpeed(0);
									setRightSpeed(0);
									demoStartTime = getTime100MicroSec();
									chargeContact = 0;
									demoState = 3;
								}
							} else {
								chargeContact = 0;
							}
							*/
							//pwm_red = 255;
							//pwm_green = 0;
							//pwm_blue = 255;
							break;

						case 2:	// line found, follow it
							turnOnGreenLeds();
							disableObstacleAvoidance();

							demoEndTime = getTime100MicroSec();
							if((demoEndTime-demoStartTime) >= (PAUSE_20_SEC)) {	// the robot seems to be blocked somehow
								// go back for a while
								setRightSpeed(-20);
								setLeftSpeed(-20);
								demoStartTime = getTime100MicroSec();
								demoState = 4;
								break;
							}

							if(CHARGE_ON) {
								outOfLine = 0;
								chargeContact++;
								if(chargeContact > 20) {
									setLeftSpeed(0);
									setRightSpeed(0);
									demoStartTime = getTime100MicroSec();
									demoState = 3;
									break;
								}
							} else {
								chargeContact = 0;

								if(proximityResult[9]>LINE_OUT_THR && proximityResult[10]>LINE_OUT_THR) {
									outOfLine++;
									if(outOfLine > 250) {
										chargeContact = 0;
										demoState = 1;
										break;
									}
								} else {
									outOfLine = 0;
								}
							}
	
							if(proximityResult[9]>LINE_OUT_THR) {	// center left is leaving the line => turn right
								setLeftSpeed(20);
								setRightSpeed(-10);
								//outOfLine++;
								//if(outOfLine > 250) {
								//	demoState = 1;
								//}
							} else if(proximityResult[10]>LINE_OUT_THR) {	// center right is leaving the lnie => turn left
								setLeftSpeed(-10);
								setRightSpeed(20);
								//outOfLine++;
								//if(outOfLine > 250) {
								//	demoState = 1;
								//}
							} else {
								setRightSpeed(20);
								setLeftSpeed(20);
								//outOfLine = 0;
								/*
								if(CHARGE_ON) {
									outOfLine = 0;
									chargeContact++;
									if(chargeContact > 20) {
										setLeftSpeed(0);
										setRightSpeed(0);
										demoStartTime = getTime100MicroSec();
										demoState = 3;
									}
								} else {
									chargeContact = 0;
								}
								*/
							}
							//pwm_red = 255;
							//pwm_green = 255;
							//pwm_blue = 0;
							break;

						case 3:	// charge for some time
							turnOffGreenLeds();
							demoEndTime = getTime100MicroSec();
							if((demoEndTime-demoStartTime) >= (PAUSE_30_SEC)) {
								if(batteryLevel<800) {	// stay in charge if too much discharged
									demoStartTime = getTime100MicroSec();
									break;
								} else {
									setRightSpeed(-20);
									setLeftSpeed(-20);
									demoStartTime = getTime100MicroSec();
									demoState = 4;
									break;
								}
							}
							if(!CHARGE_ON) {
								chargeContact = 0;
								outOfLine = 0;
								demoState = 2;
								demoStartTime = getTime100MicroSec();
								break;						
							}	
							//pwm_red = 0;
							//pwm_green = 255;
							//pwm_blue = 0;
							break;
						
						case 4: // go back from charger
							turnOnGreenLeds();
							demoEndTime = getTime100MicroSec();
							if((demoEndTime-demoStartTime) >= (PAUSE_1_SEC)) {
								setRightSpeed(20);
								setLeftSpeed(-20);								
								demoStartTime = getTime100MicroSec();
								demoState = 5;							
							}	
							//pwm_red = 0;
							//pwm_green = 0;
							//pwm_blue = 255;													
							break;

						case 5:	// turn around
							turnOffGreenLeds();
							demoEndTime = getTime100MicroSec();
							if((demoEndTime-demoStartTime) >= (PAUSE_750_MSEC)) {
								demoStartTime = getTime100MicroSec();
								demoState = 0;							
							}	
							//pwm_red = 255;
							//pwm_green = 0;
							//pwm_blue = 0;													
							break;							
					}

					updateRedLed(pwm_red);
					updateGreenLed(pwm_green);
					updateBlueLed(pwm_blue);

					break;
  
		}

		if(currentSelector != 0) {
			handleMotorsWithSpeedController();  
		}

		if(prevSelector != currentSelector) {	// in case the selector is changed, reset the robot state
			disableObstacleAvoidance();
			disableCliffAvoidance();
			GREEN_LED0_OFF;
			GREEN_LED1_OFF;
			GREEN_LED2_OFF;
			GREEN_LED3_OFF;
			GREEN_LED4_OFF;
			GREEN_LED5_OFF;
			GREEN_LED6_OFF;
			GREEN_LED7_OFF;
			pwm_red = 255;
			pwm_green = 255;
			pwm_blue = 255;
			updateRedLed(pwm_red);
			updateGreenLed(pwm_green);
			updateBlueLed(pwm_blue);
			setRightSpeed(0);
			setLeftSpeed(0);
		}
		prevSelector = currentSelector;


	} // while(1)

}
