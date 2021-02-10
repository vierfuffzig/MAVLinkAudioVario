/**************************************************************************** 
 * Main vario code by Rolf R Bakke, Oct 2012
 *  
 * MAVLink code from GhettProxy
 *
 * @file       GhettoStation.ino
 * @author     Guillaume S
 * @brief      Arduino based antenna tracker & telemetry display for UAV projects.
 * @project    https://code.google.com/p/ghettostation/
 *
 * @see        The GNU Public License (GPL) Version 3
 * 
 * Exponential filter from Meguino codebase
 * 
 * adopted for use with MAVLink input by vierfuffzig, Feb 2021
 *
 *****************************************************************************
*/

#include <FastSerial.h>
#include <avr/pgmspace.h>
#include <Arduino.h>

#include "Filter.h"

#include <AP_Math.h>
#include "Tones.h"

#include "Config.h"

#include <AltSoftSerial.h>

//#include <Metro.h>
//#include "GhettoStation.h"


#ifdef DEBUG
#include <MemoryFree.h>
#endif

FastSerialPort0(Serial);
AltSoftSerial SerialPort2;


#ifdef PROTOCOL_MAVLINK
#include <AP_Common.h>
#include <GCS_MAVLink.h>
#include "Mavlink.cpp"
#endif

//################################### SETTING OBJECTS ###############################################

#define Audiopin 2    // defaults to D2 on ProMini

const byte led = 13;

int melody[] = { 
  NOTE_G4, NOTE_C5, NOTE_E5, NOTE_G5, 0, NOTE_E5, NOTE_G5
};

// note durations: 4 = quarter note, 8 = eighth note, etc.:
int noteDurations[] = {
  8, 8, 8, 8, 8, 8, 1
};

unsigned long time = 0;
static boolean      enable_frame_request = 0;

float toneFreq, toneFreqLowpass, pressure, lowpassFast, lowpassSlow;
int ddsAcc;

ExponentialFilter<float> FilteredClimbrate(10, 0);


//#################################### SETUP ####################################################

void setup() {
    //start serial com  
    init_serial();
    #ifdef PROTOCOL_MAVLINK
    mavlink_comm_0_port = &Serial;
    #endif

    SerialPort2.println(F("MAVLink Audio Vario"));
    SerialPort2.println(F("Main vario code by Rolf R Bakke, Oct 2012"));
    SerialPort2.println(F("Main MAVLink code from GhettoStation by Guillaume S"));
    SerialPort2.println(F("adopted for use with ArduPilot MAVLink input by vierfuffzig, Feb 2021"));

    for (int thisNote = 0; thisNote < 8; thisNote++) {

        int noteDuration = 1000 / noteDurations[thisNote];
        tone(Audiopin, melody[thisNote], noteDuration);
        int pauseBetweenNotes = noteDuration * 1.30;
        delay(pauseBetweenNotes);
        noTone(Audiopin);
    }
    delay(1000);

    pressure = uav_pressure;
    lowpassFast = lowpassSlow = pressure;
}


//######################################## MAIN LOOP #####################################################################

void loop() {
    get_telemetry(); 

    play_beep();
 
    #ifdef DEBUG
    if (loopDebug.check()) {
        debug_proxy();
    }
   #endif

}

//#######################################  VARIO TONES ##########################################################

void play_beep() {
    #if defined(USE_CLIMBRATE)

    uav_climbrate = constrain(uav_climbrate, -8, 8);
    float RawClimbrate = uav_climbrate;
    FilteredClimbrate.Filter(RawClimbrate);
    float SmoothClimbrate = FilteredClimbrate.Current();
    toneFreq = SmoothClimbrate * 35; // scale for max reslution at +- 5 m/s
    toneFreq = constrain(toneFreq, -500, 500);
    ddsAcc += toneFreq * 100 + 2000;
    
    if ((SmoothClimbrate > -0.25) && (SmoothClimbrate <  0.25)) {
      SmoothClimbrate = 0;
    }
    
    if (toneFreq < 0 || ddsAcc > 0) {
        tone(Audiopin, toneFreq + 510);  
    }
    else {
        noTone(Audiopin);
    }
  
    #endif

    #if defined(USE_PRESSURE)
  
    pressure = uav_pressure * 100;
    lowpassFast = lowpassFast + (pressure - lowpassFast) * 0.1;
    lowpassSlow = lowpassSlow + (pressure - lowpassSlow) * 0.05;
    toneFreq = (lowpassSlow - lowpassFast) * 50;
    toneFreqLowpass = toneFreqLowpass + (toneFreq - toneFreqLowpass) * 0.1;
    toneFreq = constrain(toneFreqLowpass, -500, 500);
    ddsAcc += toneFreq * 100 + 2000;
    
    if (toneFreq < 0 || ddsAcc > 0) {
      tone(Audiopin, toneFreq + 510);  
    }
    else {
        noTone(Audiopin);
    }
    #endif
    
    ledOff(); 
    while (millis() < time);
    time += 20;
    ledOn();
}


void ledOn()
{
    digitalWrite(led,1);
}


void ledOff()
{
    digitalWrite(led,0);
}

//######################################## TELEMETRY FUNCTIONS #############################################

void init_serial() {
    
    Serial.begin(INPUT_BAUD);
    SerialPort2.begin(OUTPUT_BAUD);
}


void get_telemetry() {
        
    #if defined(PROTOCOL_MAVLINK)
    if(enable_frame_request == 1){
        enable_frame_request = 0;
        if (!PASSIVEMODE) {
           request_mavlink_rates();
        }
    }
    read_mavlink(); 
    #endif
}

#ifdef DEBUG

void debug_proxy() {
    SerialPort2.print("uav_climbrate = ");
    SerialPort2.println(uav_climbrate);
    SerialPort2.print("uav_pressure = ");
    SerialPort2.println(uav_pressure);
    SerialPort2.print("tone freq = ");
    SerialPort2.println(toneFreq);
    //SerialPort2.print("softserial_delay = ");
    //SerialPort2.println(softserial_delay);
    //SerialPort2.print("packet_drops = ");
    //SerialPort2.println(packet_drops);
    //SerialPort2.print("parse_error = ");
    //SerialPort2.println(parse_error);
}

#endif
