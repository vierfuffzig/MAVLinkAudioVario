/*****************************************************************************************************************  
 *   MAVLINK telemetry
 *   get raw baro pressure or climbrate
 *   
 *   pressure from RAW_SENSORS -> PRESSURE_ABS
 *   climbrate from EXTRA2 -> vfr.HUD
 *   
 *   allows to use TEC-compensated climbrate
 *   when SOAR_ENABLED = 1
 *   
 *****************************************************************************************************************
 */
 
#ifdef PROTOCOL_MAVLINK
#include "../GCS_MAVLink/include/mavlink/v1.0/mavlink_types.h"
#include "../GCS_MAVLink/include/mavlink/v1.0/ardupilotmega/mavlink.h"

int             softserial_delay = (int)round(10000000.0f/(OUTPUT_BAUD)); // time to wait between each byte sent.
float           uav_climbrate;
float           uav_pressure;

long            lastpacketreceived;

// static boolean  enable_frame_request = 0;

boolean getBit(byte Reg, byte whichBit) {
    boolean State;
    State = Reg & (1 << whichBit);
    return State;
}

byte setBit(byte &Reg, byte whichBit, boolean stat) {
    if (stat) {
        Reg = Reg | (1 << whichBit);
    } 
    else {
        Reg = Reg & ~(1 << whichBit);
    }
    return Reg;
}



// true when we have received at least 1 MAVLink packet
static bool       mavlink_active;
static uint8_t    crlf_count = 0;
static boolean    mavbeat = 0;
static uint8_t    apm_mav_type;

static int        packet_drops = 0;
static int        parse_error = 0;

mavlink_message_t msg; 
mavlink_status_t status;


void request_mavlink_rates()
{
    const int  maxStreams = 6;
    const uint8_t MAVStreams[maxStreams] = {
        MAV_DATA_STREAM_RAW_SENSORS,
        MAV_DATA_STREAM_EXTENDED_STATUS,
        MAV_DATA_STREAM_RC_CHANNELS,
        MAV_DATA_STREAM_POSITION,
        MAV_DATA_STREAM_EXTRA1, 
        MAV_DATA_STREAM_EXTRA2};
    const uint16_t MAVRates[maxStreams] = {0x05, 0x01, 0x01, 0x01, 0x01, 0x05};
    for (int i=0; i < maxStreams; i++) {
        mavlink_msg_request_data_stream_pack(127, 0, &msg, 7, 1, MAVStreams[i], MAVRates[i], 1);
        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
        uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
        Serial.write(buf, len);
    }
}

void read_mavlink(){
    mavlink_message_t msg; 
    mavlink_status_t status;

    //grabbing data 
    while(Serial.available() > 0) { 
        uint8_t c = Serial.read();
        
        if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
            mavlink_active = 1;
            lastpacketreceived = millis();
            //handle msg
            switch(msg.msgid) {
                case MAVLINK_MSG_ID_HEARTBEAT:
                    {
                        mavbeat = 1;
                    }
                break;
            case MAVLINK_MSG_ID_VFR_HUD:
                {
                    uav_climbrate = (float)mavlink_msg_vfr_hud_get_climb(&msg);
                }
                break;
            case MAVLINK_MSG_ID_SCALED_PRESSURE:
                {
                    uav_pressure = (float)mavlink_msg_scaled_pressure_get_press_abs(&msg);
                }
                break;
           } 
        }
        //delayMicroseconds(138);
    }
    // Update global packet drops counter
    packet_drops += status.packet_rx_drop_count;
    parse_error += status.parse_error;

}
    

#endif
