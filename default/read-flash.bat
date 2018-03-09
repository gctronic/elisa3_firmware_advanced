avrdude -p m2560 -c stk500v2 -P COM348 -D -U flash:r:bootloader+firmware.hex:i
pause