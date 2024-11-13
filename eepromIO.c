

#include "eepromIO.h"

void writeCalibrationToFlash() {	
	eeprom_update_block(calibration, (uint8_t*) CALIB_DATA_START_ADDR, 144);
	eeprom_update_word ((uint16_t*) CALIB_CHECK_ADDRESS, 0xAA55);   // to let know the calibration data are valid
}

void readCalibrationFromFlash() {
	eeprom_read_block (calibration, (uint8_t*) CALIB_DATA_START_ADDR, 144);
}

void writeMagCalibToFlash() {
	eeprom_update_block(magOffset, (uint8_t*) MAG_CALIB_START_ADDR, 6);
	eeprom_update_word ((uint16_t*) MAG_CALIB_CHECK_ADDRESS, 0xAA55);   // to let know the calibration data are valid
}

void readMagCalibFromFlash() {
	eeprom_read_block(magOffset, (uint8_t*) MAG_CALIB_START_ADDR, 6);
}

void resetMagCalib() {
	uint8_t temp[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
	eeprom_update_block(temp, (uint8_t*) MAG_CALIB_CHECK_ADDRESS, 8);
}
