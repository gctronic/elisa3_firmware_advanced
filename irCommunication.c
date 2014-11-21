#include "irCommunication.h"

void irCommInit(unsigned char mode) {
	if(mode==IRCOMM_MODE_RECEIVE) {
		irCommProxValuesAdc = irCommProxValuesBuff1;
		irCommProxValuesCurr = irCommProxValuesBuff2;
		irCommMaxSensorValueAdc = irCommMaxSensorValueBuff1;
		irCommMaxSensorValueCurr = irCommMaxSensorValueBuff2;
		irCommMinSensorValueAdc = irCommMinSensorValueBuff1;
		irCommMinSensorValueCurr = irCommMinSensorValueBuff2;
		memset(irCommMaxSensorValueAdc, 0x00, 16);
		memset(irCommMinSensorValueAdc, 0xFF, 16);
		irCommEnabled = IRCOMM_MODE_RECEIVE;
		irCommState = IRCOMM_RX_IDLE_STATE;
	} else if(mode==IRCOMM_MODE_TRANSMIT) {
		irCommEnabled = IRCOMM_MODE_TRANSMIT;
		irCommState = IRCOMM_TX_IDLE_STATE;
	} else if(mode==IRCOMM_MODE_RECEIVE_ONLY) {
		irCommProxValuesAdc = irCommProxValuesBuff1;
		irCommProxValuesCurr = irCommProxValuesBuff2;
		irCommMaxSensorValueAdc = irCommMaxSensorValueBuff1;
		irCommMaxSensorValueCurr = irCommMaxSensorValueBuff2;
		irCommMinSensorValueAdc = irCommMinSensorValueBuff1;
		irCommMinSensorValueCurr = irCommMinSensorValueBuff2;
		memset(irCommMaxSensorValueAdc, 0x00, 16);
		memset(irCommMinSensorValueAdc, 0xFF, 16);
		irCommEnabled = IRCOMM_MODE_RECEIVE_ONLY;
		irCommState = IRCOMM_RX_IDLE_STATE;
	} else if(mode==IRCOMM_MODE_TRANSMIT_ONLY) {
		irCommEnabled = IRCOMM_MODE_TRANSMIT_ONLY;
		irCommState = IRCOMM_TX_IDLE_STATE;
	}
}

void irCommDeinit() {
	irCommEnabled = IRCOMM_MODE_SENSORS_SAMPLING;
	irCommMode = IRCOMM_MODE_SENSORS_SAMPLING;
}

void irCommTasks() {
	int i = 0;

	if(irCommMode==IRCOMM_MODE_RECEIVE) {

		switch(irCommState) {
			case IRCOMM_RX_IDLE_STATE:				
				break;

			case IRCOMM_RX_MAX_SENSOR_STATE:
				irCommRxMaxDiff = 0;
    			irCommRxMaxSensor = -1;
				for(i=0; i<8; i++) {
					if ((signed int)(irCommMaxSensorValueCurr[i]-irCommMinSensorValueCurr[i]) > irCommRxMaxDiff) {
						irCommRxMaxDiff = irCommMaxSensorValueCurr[i]-irCommMinSensorValueCurr[i];
						irCommRxMaxSensor = i;
					}
				}
				irCommRxMaxSensorTemp = irCommRxMaxSensor;
				irCommRxMaxDiffTemp = irCommRxMaxDiff;
				irCommMaxSensorValueCurrTemp = irCommMaxSensorValueCurr[irCommRxMaxSensor];
				irCommMinSensorValueCurrTemp = irCommMinSensorValueCurr[irCommRxMaxSensor];


				//if(irCommRxMaxSensor == -1) {
				//	updateRedLed(0);
				//}
				if(irCommRxMaxDiff >= IRCOMM_DETECTION_AMPLITUDE_THR) {
					irCommState = IRCOMM_RX_DETECT_START_BIT_STATE;
				} else {
					irCommRxStartBitDetected = 0;
					if(irCommEnabled == IRCOMM_MODE_RECEIVE) {
						currentProx = 0;
						adcSaveDataTo = SKIP_SAMPLE;
						adcSamplingState = 0;
						irCommMode=IRCOMM_MODE_SENSORS_SAMPLING;
					}
					irCommState = IRCOMM_RX_IDLE_STATE;
				}						
				break;

			case IRCOMM_RX_DETECT_START_BIT_STATE:
				// extract signal from the sensor with higher amplitude and compute the signal mean
				irCommProxSum = 0;
				irCommTempMin = 1024;
				irCommTempMax = 0;
				irCommShiftCount = 0;
				irCommComputeShift = 1;
				for(i=0; i<IRCOMM_SAMPLING_WINDOW; i++) {
					irCommMaxSensorSignal[i] = irCommProxValuesCurr[irCommRxMaxSensor+i*8];
					irCommMaxSensorSignalTemp[i] = irCommMaxSensorSignal[i];
					irCommProxSum += irCommMaxSensorSignal[i];
					if(irCommComputeShift == 1) {	// compute the shift between the rx sampling and tx signal
						irCommShiftCount++;								
						if(irCommTempMin > irCommMaxSensorSignal[i]) {
							irCommTempMin = irCommMaxSensorSignal[i];
						}
						if(irCommTempMax < irCommMaxSensorSignal[i]) {
							irCommTempMax = irCommMaxSensorSignal[i];
						}
						irCommTempValue = (irCommTempMax - irCommTempMin);
						if((irCommTempMax - irCommTempMin) >= IRCOMM_DETECTION_AMPLITUDE_THR) {
							if(irCommMaxSensorSignal[i] == irCommTempMax) {
								irCommRxPeakHighToLow = 0;
							} else if(irCommMaxSensorSignal[i] == irCommTempMin) {
								irCommRxPeakHighToLow = 1;
							} else {
								updateRedLed(0);
							}
							irCommTempMin = 1024;
							irCommTempMax = 0;
							irCommComputeShift = 2;
							irCommShiftCount--;	// the current sample is already part of the signal start thus do not skip it
							irCommRxStartPeakDuration = 0;
						}
					 } else if(irCommComputeShift == 2) {		
					 	irCommRxStartPeakDuration++;					
						if(irCommTempMin > irCommMaxSensorSignal[i]) {
							irCommTempMin = irCommMaxSensorSignal[i];
						}
						if(irCommTempMax < irCommMaxSensorSignal[i]) {
							irCommTempMax = irCommMaxSensorSignal[i];
						}	
						if((irCommTempMax - irCommTempMin) >= IRCOMM_DETECTION_AMPLITUDE_THR) {
							if((irCommMaxSensorSignal[i]==irCommTempMax) && (irCommRxPeakHighToLow==1)) {
								irCommComputeShift = 0;
							} else if((irCommMaxSensorSignal[i]==irCommTempMin) && (irCommRxPeakHighToLow==0)) {
								irCommComputeShift = 0;
							}
							
						}											
					 }

				}
				
				//if(irCommComputeShift == 1) {	// it should not be never 1 because the difference between min and max in the current signal
				//	updateRedLed(0);			// is at least IRCOMM_DETECTION_AMPLITUDE_THR (checked in the previous state)
				//}
				irCommProxMean = (int)(irCommProxSum / IRCOMM_SAMPLING_WINDOW);

				// substract mean from signal
				for(i=0; i<IRCOMM_SAMPLING_WINDOW; i++) {
					irCommMaxSensorSignal[i] -= irCommProxMean;
				}
						
				// start counting number of switch around mean signal
				if(irCommMaxSensorSignal[0] > 0) {
					irCommSignalState = 1;
				} else {
					irCommSignalState = -1;
				}
				irCommSwitchCount = 0;
				for(i=1; i<IRCOMM_SAMPLING_WINDOW; i++) {
					if(irCommMaxSensorSignal[i] > 0) {
						if(irCommSignalState < 0) {
							irCommSignalState = 1;
							irCommSwitchCount++;
						}
					} else {
						if(irCommSignalState > 0) {
							irCommSignalState = -1;
							irCommSwitchCount++;
						}
					}
				}
				// check if a start bit is detected											
				//if(irCommSwitchCount >= IRCOMM_START_BIT_MIN_SWITCH_COUNT) {
				if((irCommSwitchCount==2) && (irCommRxStartPeakDuration>=9) && (irCommRxStartPeakDuration<=11)) {
					if(irCommRxPeakHighToLow == 1) {
						irCommShiftCount = IRCOMM_SAMPLING_WINDOW/2 + irCommShiftCount;
					} else {
						if(irCommRxStartBitDetected == 1) {
							irCommShiftCount = irCommShiftCount;
						} else {
							irCommShiftCount = IRCOMM_SAMPLING_WINDOW + irCommShiftCount;
						}
					}
					irCommShiftCountTemp = irCommShiftCount;
					irCommRxPeakHighToLow = 0;
					irCommRxStartBitDetected = 0;
					irCommSecondBitSkipped = 0;
					irCommShiftCounter = 0;
					irCommRxBitCount = 0;	
					irCommRxCrc = 0;	
					irCommRxByte = 0;
					irCommState = IRCOMM_RX_SYNC_SIGNAL;																										
				} else {
					if(irCommSwitchCount == 1) {
						irCommRxStartBitDetected = 1;
					}
					if(irCommEnabled == IRCOMM_MODE_RECEIVE) {
						currentProx = 0;
						adcSaveDataTo = SKIP_SAMPLE;
						adcSamplingState = 0;
						irCommMode=IRCOMM_MODE_SENSORS_SAMPLING;
					}
					irCommState = IRCOMM_RX_IDLE_STATE;										
				}
				break;
				
			case IRCOMM_RX_SYNC_SIGNAL:
				break;

			case IRCOMM_RX_WAITING_BIT:
				break;

			case IRCOMM_RX_READ_BIT:
				// extract signal from the sensor with higher amplitude and compute the signal mean
				irCommProxSum = 0;
				irCommTempMin = 1024;
				irCommTempMax = 0;
				for(i=0; i<IRCOMM_SAMPLING_WINDOW; i++) {
					irCommMaxSensorSignal[i] = irCommProxValuesCurr[irCommRxMaxSensor+i*8];
					irCommProxSum += irCommMaxSensorSignal[i];
				}
				irCommProxMean = (int)(irCommProxSum / IRCOMM_SAMPLING_WINDOW);

				// substract mean from signal
				for(i=0; i<IRCOMM_SAMPLING_WINDOW; i++) {
					irCommMaxSensorSignal[i] -= irCommProxMean;
				}
						
				// start counting number of switch around mean signal
				if(irCommMaxSensorSignal[0] > 0) {
					irCommSignalState = 1;
				} else {
					irCommSignalState = -1;
				}
				irCommSwitchCount = 0;
				for(i=1; i<IRCOMM_SAMPLING_WINDOW; i++) {
					if(irCommMaxSensorSignal[i] > 0) {
						if(irCommSignalState < 0) {
							irCommSignalState = 1;
							irCommSwitchCount++;
						}
					} else {
						if(irCommSignalState > 0) {
							irCommSignalState = -1;
							irCommSwitchCount++;
						}
					}
				}
				// check whether we received either a "0" or a "1"
				if(irCommSwitchCount >= (IRCOMM_BIT0_SWITCH_COUNT-2)) {
					irCommRxBitReceived[irCommRxBitCount] = 0;
					if(irCommRxBitCount<8) {	// do not consider the crc for byte interpretation
						irCommRxByte = irCommRxByte<<1;	// bit0, only shift
					}
				} else if(irCommSwitchCount >= (IRCOMM_BIT1_SWITCH_COUNT-1)) {
					irCommRxBitReceived[irCommRxBitCount] = 1;
					if(irCommRxBitCount<8) {	// do not consider the crc for byte interpretation
						irCommRxCrc++;
						irCommRxByte = irCommRxByte<<1;	// bit1, shift and add 1
						irCommRxByte += 1;
					}
				} else {	// error...no significant signal perceived
					//irCommRxBitReceived[irCommRxBitCount] = 0xFF;
					if(irCommEnabled == IRCOMM_MODE_RECEIVE) {
						currentProx = 0;
						adcSaveDataTo = SKIP_SAMPLE;
						adcSamplingState = 0;
						irCommMode=IRCOMM_MODE_SENSORS_SAMPLING;
					}
					irCommState = IRCOMM_RX_IDLE_STATE;
				}
				irCommRxBitCount++;
				if(irCommRxBitCount == 10) {	// received 8 bit of data + 2 bit of crc
					irCommState = IRCOMM_RX_CHECK_CRC;
				} else {
					irCommState = IRCOMM_RX_WAITING_BIT;
				}							
				break;

			case IRCOMM_RX_CHECK_CRC:
				irCommRxCrcError = (irCommRxCrc + (irCommRxBitReceived[8]<<1) + irCommRxBitReceived[9])&0x03;
				if(irCommRxCrcError==0) {
					irCommRxLastDataReceived = irCommRxByte;
					irCommRxReceivingSensor = irCommRxMaxSensor;
					irCommRxDataAvailabled = 1;					
				}
				/*
				if((irCommRxByte!=irCommRxByteExpected) || (irCommRxCrcError!=0)) {	// if there is an error
					// debug...
				}
				if(irCommRxByteExpected == 255) {
					irCommRxByteExpected = 0;
					irCommRxSequenceCount++;
				} else {
					irCommRxByteExpected++;
				}
				*/				
				if(irCommEnabled == IRCOMM_MODE_RECEIVE) {
					currentProx = 0;
					adcSaveDataTo = SKIP_SAMPLE;
					adcSamplingState = 0;
					irCommMode=IRCOMM_MODE_SENSORS_SAMPLING;
				}
				irCommState = IRCOMM_RX_IDLE_STATE;				
				break;
					
		}

	} else if(irCommMode==IRCOMM_MODE_TRANSMIT) {

		switch(irCommState) {
			case IRCOMM_TX_IDLE_STATE:					
				break;

			case IRCOMM_TX_PREPARE_TRANSMISSION:
				if((getTime100MicroSec() - irCommTxWaitStartTime) < PAUSE_500_MSEC) {
					break;
				}
				updateBlueLed(0);
				irCommTickCounter = getTime100MicroSec()-irCommTickCounter2;
				irCommTickCounter2 = getTime100MicroSec();
				irCommTxBitToTransmit[0] = 2;	// start bit
				irCommTxBitToTransmit[1] = 2;	// start bit
				irCommTxCrc = 0;
				for(i=0; i<8; i++) {
					irCommTempValue = (irCommTxByte>>i)&0x01;
					irCommTxBitToTransmit[9-i] = irCommTempValue;
					if(irCommTempValue==1) {
						irCommTxCrc++;
					}
				}
				irCommTxCrc = irCommTxCrc % 4;
				irCommTxCrc = 4 - irCommTxCrc;
				irCommTxBitToTransmit[10] = (irCommTxCrc>>1)&0x01;
				irCommTxBitToTransmit[11] = irCommTxCrc&0x01;	
				irCommTxBitCount = 0;							
				irCommTxPulseState = 0;	
				irCommState = IRCOMM_TX_COMPUTE_TIMINGS;				
				break;

			case IRCOMM_TX_COMPUTE_TIMINGS:
				if(irCommTxBitToTransmit[irCommTxBitCount] == 2) {
					irCommTxDuration = IRCOMM_BIT_START_DURATION;					
					irCommTxSwitchCount = IRCOMM_BIT_START_SWITCH_COUNT;
				} else if(irCommTxBitToTransmit[irCommTxBitCount] == 1) {
					irCommTxDuration = IRCOMM_BIT1_DURATOIN;					
					irCommTxSwitchCount = IRCOMM_BIT1_SWITCH_COUNT;
				} else {
					irCommTxDuration = IRCOMM_BIT0_DURATION;						
					irCommTxSwitchCount = IRCOMM_BIT0_SWITCH_COUNT;
				}
				irCommTxDurationCycle = 0;
				irCommTxSwitchCounter = 0;						
				irCommState = IRCOMM_TX_TRANSMIT_BIT;
				irCommAdcTxState = IRCOMM_TX_ADC_TRANSMISSION_SEQ1;
				break;

			case IRCOMM_TX_TRANSMIT_BIT:	// wait for bit to be transmitted
				break;

		}

	}

}


void irCommSendData(unsigned char value, unsigned char sensorMask) {
	irCommTxByte = value;
	irCommTxByteEnqueued = 1;
	irCommTxSensorMask = sensorMask;
}

unsigned char irCommDataSent() {
	if(irCommTxByteEnqueued==1) {
		return 0;
	} else {
		return 1;
	}
}

unsigned char irCommDataAvailable() {
	return irCommRxDataAvailabled;
}

unsigned char irCommReadData() {
	irCommRxDataAvailabled = 0;
	return irCommRxLastDataReceived;
}

signed char irCommReceivingSensor() {
	return irCommRxReceivingSensor;
}
