c:\windows\system32\mode.com com169: dtr=on
avrdude -p m2560 -P COM169 -b 57600 -c stk500v2 -D -Uflash:w:Elisa3-avr-studio.hex:i -v
pause