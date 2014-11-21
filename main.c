
#include <avr\io.h>
#include <avr\interrupt.h>

#include <stdlib.h>
#include "variables.h"
#include "utility.h"
#include "speed_control.h"
#include "nRF24L01.h"
#include "behaviors.h"
#include "sensors.h"
#include "irCommunication.h"

int main(void) {

	unsigned long int startTime = 0, endTime = 0, turnOffLedsTime = 0;
	unsigned char prevSelector=0;
	unsigned int i=0;
	unsigned int currRand=0, currRand2=0;

	initPeripherals();

	calibrateSensors();

	initBehaviors();

	startTime = getTime100MicroSec();

	speedStepCounter = getTime100MicroSec();

/*
	unsigned int j=0;
	for(i=0; i<9; i++) {
		for(j=0; j<8; j++) {
			calibration[i][j]=0xFF;
		}
	}
	eeprom_update_block(calibration, (uint8_t*) CALIB_DATA_START_ADDR, 144);
*/

	while(1) {

		currentSelector = getSelector();	// update selector position

		readAccelXYZ();						// update accelerometer values to compute the angle

		computeAngle();

		// turn off the rgb leds after one half second in the "charger demo"
		if(currentSelector==7) {
			if((getTime100MicroSec()-turnOffLedsTime) > PAUSE_500_MSEC) {
				pwm_red = 255;
				pwm_green = 255;
				pwm_blue = 255;			
			}
		}

		endTime = getTime100MicroSec();
		if((endTime-startTime) >= (PAUSE_2_SEC)) {
			readBatteryLevel();				// the battery level is updated every two seconds
             		
			if(currentSelector==4 || currentSelector==5) {
				currRand = (currRand + rand()%30) % 128;	// 0 to 255 is the maximum, we use 128 to 255 to reduce consumption

				switch(rgbState) {
					case 0:
						pwm_red = 255 - currRand;	
						rgbState = 1;
						break;

					case 1:
						pwm_green = 255 - currRand;
						rgbState = 2;
						break;

					case 2:
						pwm_blue = 255 - currRand;
						rgbState = 0;
						break;
				}
				
				if(currRand<32) {
					if(pwm_green!=255 && pwm_blue!=255) {	// do not turn on all leds
						pwm_red = 255;
					}
				} else if(currRand<64) {
					if(pwm_red!=255 && pwm_blue!=255) {
						pwm_green = 255;
					}
				} else if (currRand<96) {
					if(pwm_red!=255 && pwm_green!=255) {
						pwm_blue = 255;
					}
				} else {	// do nothing => all 3 leds turned on

				}

			} else if(currentSelector==7) {

				srand(TCNT3);
				currRand = (currRand + rand()%30) % 128;	// 0 to 255 is the maximum, we use 128 to 255 to reduce consumption

				pwm_red = 255 - currRand;
				
				srand(TCNT3);
				currRand = (currRand + rand()%30) % 128;	
				pwm_green = 255 - currRand;

				srand(TCNT3);
				currRand = (currRand + rand()%30) % 128;
				pwm_blue = 255 - currRand;

			
				srand(TCNT3);
				currRand2 = rand()%128;
				if(currRand2<32) {
					pwm_red = 255;
				} else if(currRand2<64) {
					pwm_green = 255;
				} else if (currRand2<96) {
					pwm_blue = 255;
				} else {	// do nothing => all 3 leds turned on

				}

				turnOffLedsTime = getTime100MicroSec();

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

		
		//if(calibrateOdomFlag==0) {
			handleIRRemoteCommands();
		//}


		//if(calibrateOdomFlag==0) {
			handleRFCommands();
		//}


		if(calibrateOdomFlag==0) {
			if((getTime100MicroSec()-speedStepCounter) >= SPEED_STEP_DELAY) {
				speedStepCounter = getTime100MicroSec();

				if(softAccEnabled) {
					if(pwm_right_desired == 0) {
						pwm_intermediate_right_desired = 0;
					} else if((pwm_right_desired*pwm_intermediate_right_desired) < 0) {
						pwm_intermediate_right_desired = 0;
					} else if(pwm_right_desired > pwm_intermediate_right_desired) {
						pwm_intermediate_right_desired += speedStep;
						if(pwm_intermediate_right_desired > pwm_right_desired) {
							pwm_intermediate_right_desired = pwm_right_desired;
						}
					} else if(pwm_right_desired < pwm_intermediate_right_desired) {
						pwm_intermediate_right_desired -= speedStep;
						if(pwm_intermediate_right_desired < pwm_right_desired) {
							pwm_intermediate_right_desired = pwm_right_desired;
						}					
					}
	
					if(pwm_left_desired == 0) {
						pwm_intermediate_left_desired = 0;
					} else if((pwm_left_desired*pwm_intermediate_left_desired) < 0) {
						pwm_intermediate_left_desired = 0;
					} else if(pwm_left_desired > pwm_intermediate_left_desired) {
						pwm_intermediate_left_desired += speedStep;
						if(pwm_intermediate_left_desired > pwm_left_desired) {
							pwm_intermediate_left_desired = pwm_left_desired;
						}
					} else if(pwm_left_desired < pwm_intermediate_left_desired) {
						pwm_intermediate_left_desired -= speedStep;
						if(pwm_intermediate_left_desired < pwm_left_desired) {
							pwm_intermediate_left_desired = pwm_left_desired;
						}					
					}
				} else {
					pwm_intermediate_right_desired = pwm_right_desired;
					pwm_intermediate_left_desired = pwm_left_desired;
				}

			}
		}

		//if(currentSelector!=6 && currentSelector!=15) {
		//	usart0Transmit(currentSelector,0);		// send the current selector position through uart as debug info
		//}

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
								usart0Transmit(((unsigned int)(theta*573.0))&0xFF,1);	// radians to degrees => 573 = 1800/PI
								usart0Transmit(((unsigned int)(theta*573.0))>>8,1);
								usart0Transmit(((unsigned int)xPos)&0xFF,1);
								usart0Transmit(((unsigned int)xPos)>>8,1);
								usart0Transmit(((unsigned int)yPos)&0xFF,1);
								usart0Transmit(((unsigned int)yPos)>>8,1);
								usart0Transmit(((signed long int)leftMotSteps)&0xFF,1);
								usart0Transmit(((signed long int)leftMotSteps)>>8,1);
								usart0Transmit(((signed long int)leftMotSteps)>>16,1);
								usart0Transmit(((signed long int)leftMotSteps)>>24,1);
								usart0Transmit(((signed long int)rightMotSteps)&0xFF,1);
								usart0Transmit(((signed long int)rightMotSteps)>>8,1);
								usart0Transmit(((signed long int)rightMotSteps)>>16,1);
								usart0Transmit(((signed long int)rightMotSteps)>>24,1);
								usart0Transmit(pwm_left_desired>>2, 1);
								usart0Transmit(pwm_right_desired>>2, 1);

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
							turnOffGreenLeds();
							GREEN_LED0_ON;
							//GREEN_LED1_ON;
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
							GREEN_LED2_ON;
							//GREEN_LED3_ON;
							outOfLine = 0;
							enableObstacleAvoidance();
							setRightSpeed(15);
							setLeftSpeed(15);
							if(proximityResult[9]<LINE_IN_THR || proximityResult[10]<LINE_IN_THR || proximityResult[8]<LINE_IN_THR || proximityResult[11]<LINE_IN_THR) {
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
							turnOffGreenLeds();
							GREEN_LED4_ON;
							//GREEN_LED5_ON;
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
							
							if(proximityResult[8]<LINE_OUT_THR && proximityResult[9]>LINE_OUT_THR && proximityResult[10]>LINE_OUT_THR && proximityResult[11]>LINE_OUT_THR) {	// left ground is the only within the black line => turn left
								setLeftSpeed(-10);
								setRightSpeed(15);
							} else if(proximityResult[11]<LINE_OUT_THR && proximityResult[8]>LINE_OUT_THR && proximityResult[9]>LINE_OUT_THR && proximityResult[10]>LINE_OUT_THR) {	// right ground is the only within the black line => turn right
								setLeftSpeed(15);
								setRightSpeed(-10);
							} else if(proximityResult[9]>LINE_OUT_THR) {	// center left is leaving the line => turn right
								setLeftSpeed(15);
								setRightSpeed(-5);
								//outOfLine++;
								//if(outOfLine > 250) {
								//	demoState = 1;
								//}
							} else if(proximityResult[10]>LINE_OUT_THR) {	// center right is leaving the lnie => turn left
								setLeftSpeed(-5);
								setRightSpeed(15);
								//outOfLine++;
								//if(outOfLine > 250) {
								//	demoState = 1;
								//}
							} else {
								setRightSpeed(15);
								setLeftSpeed(15);
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
							GREEN_LED6_ON;
							//GREEN_LED7_ON;
							demoEndTime = getTime100MicroSec();
							if((demoEndTime-demoStartTime) >= (PAUSE_30_SEC)) {
								if(batteryLevel<890) {//860) {	// stay in charge if too much discharged (consider the fact that the robot
														// is still in charge thus the battery value measured is higher)
									demoStartTime = getTime100MicroSec();
									break;
								} else {
									setRightSpeed(-13);
									setLeftSpeed(-13);
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
							turnOffGreenLeds();
							GREEN_LED6_ON;
							GREEN_LED7_ON;
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
							GREEN_LED6_ON;
							GREEN_LED7_ON;
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
  
			case 8:	// motors calibration
					irEnabled = 1;
					if(calibrateOdomFlag==1) {
						handleCalibration();
					}
					break;

			case 9:	// write default calibration values; wait 2 seconds before start writing the calibration values
					// in eeprom in order to avoid rewriting the data involuntarily when moving the selector and passing 
					// through selector position 9
					switch(demoState) {
						case 0:
							demoStartTime = getTime100MicroSec();
							demoState = 1;
							break;

						case 1:
							if((getTime100MicroSec()-demoStartTime) >= (PAUSE_2_SEC)) {
								demoState = 2;
							}
							break;						

						case 2:
							if(!calibrationWritten) {
								calibrationWritten = 1;
								writeDefaultCalibration();
							}							
							break;
					}
					break;

			case 10:// obstacle avoidance with random colors; 4 seconds motion and 10 seconds pause					
					switch(demoState) {
						case 0: // get first clock tick and start moving the robot with obstacle avoidance enabled
							demoStartTime = getTime100MicroSec();
							demoEndTime = getTime100MicroSec();							
							demoStartTime2 = getTime100MicroSec();
							enableObstacleAvoidance();
							setLeftSpeed(20);
							setRightSpeed(20);
							demoState = 1;
							break;
						case 1: // motion					
							if((getTime100MicroSec()-demoStartTime) >= (PAUSE_4_SEC)) {
								setRightSpeed(0);
								setLeftSpeed(0);								
								demoStartTime = getTime100MicroSec();
								demoState = 2;							
							}
							break;

						case 2: // pause							
							if((getTime100MicroSec()-demoStartTime) >= (PAUSE_10_SEC)) {
								setRightSpeed(20);
								setLeftSpeed(20);								
								demoStartTime = getTime100MicroSec();								
								demoState = 1;							
							}
							break;
					}	
					
					switch(rgbLedState) {
						case 0:							
							pwm_red = 255;
							pwm_green = 255;
							pwm_blue = 255;
							if((getTime100MicroSec()-demoEndTime) >= (PAUSE_100_MSEC)) {
								setGreenLed(greenLedState, 0);
								greenLedState++;
								if(greenLedState > 7) {
									greenLedState = 0;
								}
								setGreenLed(greenLedState, 1);
								demoEndTime = getTime100MicroSec();											
							}
							if((getTime100MicroSec()-demoStartTime2) >= (PAUSE_2_SEC)) {
								rgbLedState = 1;
								demoStartTime2 = getTime100MicroSec();
							}
							updateRedLed(pwm_red);
							updateGreenLed(pwm_green);
							updateBlueLed(pwm_blue);
							break;

						case 1:		
							turnOffGreenLeds();					
							currRand = rand()% 128;	// 0 to 255 is the maximum, we use 0 to 127 to get brighter colors
							if(currRand > 95) {
								pwm_red = 255;
							} else {
								pwm_red = currRand;
							}

							currRand = rand()% 128;
							if(currRand > 95) {
								pwm_green = 255;
							} else {
								pwm_green = currRand;
							}

							currRand = rand()% 128;
							if(currRand > 95) {
								pwm_blue = 255;
							} else {
								pwm_blue = currRand;
							}

							currRand = rand()% 200;
							if(currRand<50) {
								pwm_red = 255;
							} else if(currRand<100) {
								pwm_green = 255;
							} else if (currRand<150) {	
								pwm_blue = 255;
							}
							updateRedLed(pwm_red);
							updateGreenLed(pwm_green);
							updateBlueLed(pwm_blue);
							rgbLedState = 2;
							break;

						case 2:
							if((getTime100MicroSec()-demoStartTime2) >= (PAUSE_500_MSEC)) {
								rgbLedState = 0;
								demoStartTime2 = getTime100MicroSec();
								demoEndTime = getTime100MicroSec();
								greenLedState = 0;
								setGreenLed(greenLedState, 1);
							}
							break;
		
					}
					

					break;

			case 11:if(leftMotSteps <= 5000) {
						setLeftSpeed(40);
						setRightSpeed(40);
					} else {
						setLeftSpeed(0);
						setRightSpeed(0);
					}
					break;

			case 12:if(leftMotSteps <= 6000) {
						setLeftSpeed(40);
						setRightSpeed(40);
					} else {
						setLeftSpeed(0);
						setRightSpeed(0);
					}
					break;

			case 13: // IR comm transmitter
				switch(demoState) {
					case 0:
						irCommInit(IRCOMM_MODE_TRANSMIT_ONLY);
						//enableObstacleAvoidance();
						//setLeftSpeed(25);
						//setRightSpeed(25);
						demoState = 1;
						break;

					case 1:						
						irCommSendData(irCommRxByteExpected, 0x01);												
						demoState = 2;
						break;

					case 2:
						irCommTasks();
						if(irCommDataSent()==1) {
							demoState = 1;
							// send commands to turn on green leds
							//irCommRxByteExpected = 0;
							//turnOffGreenLeds();
							//setGreenLed(irCommRxByteExpected, 1);						
							//irCommRxByteExpected++;
							//if(irCommRxByteExpected==8) {
							//	irCommRxByteExpected=0;
							//}
							if(irCommRxByteExpected<255) {
								irCommRxByteExpected++;
							} else {
								irCommRxByteExpected = 0;
							}
						}
						break;
				}
				break;
			
			case 14: // IR comm receiver
				switch(demoState) {
					case 0:
						irCommInit(IRCOMM_MODE_RECEIVE);
						//enableObstacleAvoidance();
						//setLeftSpeed(25);
						//setRightSpeed(25);
						demoState = 1;
						break;

					case 1:
						irCommTasks();
						if(irCommDataAvailable()==1) {
							i = irCommReadData();
							//turnOffGreenLeds();
							//setGreenLed(currRand, 1);
							//currRand++;
							//if(currRand==8) {
							//	currRand=0;
							//}

							/*
							irCommRxByteExpected = irCommReadData();
							turnOffGreenLeds();
							setGreenLed(irCommRxByteExpected, 1);
							*/
							/*
							i = irCommReceivingSensor();
							turnOffGreenLeds();
							setGreenLed(i, 1);
							switch(irCommRxByteExpected) {
								case 0: 
									updateRedLed(255);
									updateGreenLed(255);
									updateBlueLed(255);
									break;
								case 1: 
									updateRedLed(0);
									updateGreenLed(255);
									updateBlueLed(255);
									break;	
								case 2: 
									updateRedLed(255);
									updateGreenLed(0);
									updateBlueLed(255);
									break;
								case 3: 
									updateRedLed(255);
									updateGreenLed(255);
									updateBlueLed(0);
									break;
								case 4: 
									updateRedLed(0);
									updateGreenLed(0);
									updateBlueLed(255);
									break;
								case 5: 
									updateRedLed(0);
									updateGreenLed(255);
									updateBlueLed(0);
									break;
								case 6: 
									updateRedLed(255);
									updateGreenLed(0);
									updateBlueLed(0);
									break;
								case 7: 
									updateRedLed(0);
									updateGreenLed(0);
									updateBlueLed(0);
									break;
								default:
									break;
							}
							*/
						}
						break;
				}									
				break;

			case 15:// clock calibration
					//usart0Transmit(irCommand,1);
					//currentOsccal = OSCCAL;
					//usart0Transmit(currentOsccal,1);
					break;

		}

		if(currentSelector!=0) {
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
			rgbState = 0;
			calibrationWritten = 0;
			demoState = 0;

			irCommState = 0;
		}
		prevSelector = currentSelector;


	} // while(1)

}
