avrdude -p m2560 -P COM39 -b 57600 -c stk500v2 -D -Ueeprom:r:Elisa3-eeprom.hex:i -v
pause