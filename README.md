# MAVLinkAudioVario
A simple Arduino MAVLink audio variometer for FPV soaring

+ based on KapteinKUK's simple DIY RC variometer code
+ tested on Arduino ProMini
+ uses MAVLink telemetry data
+ uses either baro pressure (`scaled_pressure_get_press_abs`) or climbrate (`vfr_hud_get_climb`)
+ uses TEC compensated climbrate when used with ArduPlane soaring feature enabled


## Connections

![MAVLinkVario](https://github.com/vierfuffzig/MAVLinkAudioVario/blob/main/MAVLinkVario.jpeg)


+ MAVLink telemetry input defaults RX pin
+ no bidirectional connection required
+ default input baudrate 57.600 baud
+ debug console output via softserial TX on pin 9 at 57.600 baud
+ audio out on pin D2, best used with 10kΩ / 2kΩ voltage divider to feed standard VTx audio input pin
+ best used with relevant autopilot MAVLink streamrates (EXTRA2 and RAW_SENSORS) set to 10 Hz


## Credentials

 Main vario code by Rolf R Bakke, Oct 2012 https://www.rcgroups.com/forums/showthread.php?1749208-DIY-simple-and-inexpensive-Arduino-based-sailplane-variometer
  
 MAVLink code from GhettoStation by Guillaume S https://code.google.com/p/ghettostation/ published under The GNU Public License (GPL) Version 3
 
 Exponential filter from Meguino codebase https://www.megunolink.com/
 
 adopted for use with MAVLink input by vierfuffzig, Feb 2021
