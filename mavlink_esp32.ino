#include <MAVLink.h>
#include <SoftwareSerial.h>

// #define SOFT_SERIAL_DEBUGGING   // Comment this line if no serial debugging is needed
// #ifdef SOFT_SERIAL_DEBUGGING
//   // Library to use serial debugging with a second board
//   #include <SoftwareSerial.h>
//   SoftwareSerial mySerial(6,7); // RX, TX
//   SoftwareSerial pxSerial(9,10);   // RX, TX
// #endif

void printHex(uint8_t num) {
  char hexCar[2];

  sprintf(hexCar, "%02X", num);
  Serial.print(hexCar);
}
unsigned long previousMillisMAVLink = 0;     // will store last time MAVLink was transmitted and listened
unsigned long next_interval_MAVLink = 1000;  // next interval to count
const int num_hbs = 60;                      // # of heartbeats to wait before activating STREAMS from Pixhawk. 60 = one minute.
int num_hbs_pasados = num_hbs;

SoftwareSerial mySerial(6,7); 

void setup() {
  // MAVLink interface start
  Serial.begin(57600);
  Serial.println("MAVLink starting.");
  mySerial.begin(57600);

}

void loop() {

  
  int sysid = 1;                   ///< ID 20 for this airplane. 1 PX, 255 ground station
  int compid = 158;                ///< The component sending the message
  int type = MAV_TYPE_QUADROTOR;   ///< This system is an airplane / fixed wing
 
  // Define the system type, in this case an airplane -> on-board controller
  // uint8_t system_type = MAV_TYPE_FIXED_WING;
  uint8_t system_type = MAV_TYPE_GENERIC;
  uint8_t autopilot_type = MAV_AUTOPILOT_INVALID;
 
  uint8_t system_mode = MAV_MODE_PREFLIGHT; ///< Booting up
  uint32_t custom_mode = 0;                 ///< Custom mode, can be defined by user/adopter
  uint8_t system_state = MAV_STATE_STANDBY; ///< System ready for flight
  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
 
  // Pack the message
  //mavlink_msg_heartbeat_pack(sysid,compid, &msg, type, autopilot_type, system_mode, custom_mode, system_state);
  mavlink_msg_heartbeat_pack(1,0, &msg, type, autopilot_type, system_mode, custom_mode, system_state);
 
  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
 

  unsigned long currentMillisMAVLink = millis();
  if (currentMillisMAVLink - previousMillisMAVLink >= next_interval_MAVLink) {
  
    previousMillisMAVLink = currentMillisMAVLink;


    mySerial.write(buf, len);
     for(int i=0; i<len; i++){
    printHex(buf[i]);}


    //Mav_Request_Data();
    num_hbs_pasados++;
    if(num_hbs_pasados>=num_hbs) {
      // Request streams from Pixhawk

      Serial.println("Streams requested!");

      Mav_Request_Data();
      num_hbs_pasados=0;
    }

  }

  // Check reception buffer
  comm_receive();
}

void Mav_Request_Data()
{
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];


  // STREAMS that can be requested
  /*
   * Definitions are in common.h: enum MAV_DATA_STREAM
   *   
   * MAV_DATA_STREAM_ALL=0, // Enable all data streams
   * MAV_DATA_STREAM_RAW_SENSORS=1, /* Enable IMU_RAW, GPS_RAW, GPS_STATUS packets.
   * MAV_DATA_STREAM_EXTENDED_STATUS=2, /* Enable GPS_STATUS, CONTROL_STATUS, AUX_STATUS
   * MAV_DATA_STREAM_RC_CHANNELS=3, /* Enable RC_CHANNELS_SCALED, RC_CHANNELS_RAW, SERVO_OUTPUT_RAW
   * MAV_DATA_STREAM_RAW_CONTROLLER=4, /* Enable ATTITUDE_CONTROLLER_OUTPUT, POSITION_CONTROLLER_OUTPUT, NAV_CONTROLLER_OUTPUT.
   * MAV_DATA_STREAM_POSITION=6, /* Enable LOCAL_POSITION, GLOBAL_POSITION/GLOBAL_POSITION_INT messages.
   * MAV_DATA_STREAM_EXTRA1=10, /* Dependent on the autopilot
   * MAV_DATA_STREAM_EXTRA2=11, /* Dependent on the autopilot
   * MAV_DATA_STREAM_EXTRA3=12, /* Dependent on the autopilot
   * MAV_DATA_STREAM_ENUM_END=13,
   * 
   * Data in PixHawk available in:
   *  - Battery, amperage and voltage (SYS_STATUS) in MAV_DATA_STREAM_EXTENDED_STATUS
   *  - Gyro info (IMU_SCALED) in MAV_DATA_STREAM_EXTRA1
   */

  // To be setup according to the needed information to be requested from the Pixhawk
  const int  maxStreams = 2;
  const uint8_t MAVStreams[maxStreams] = {MAV_DATA_STREAM_RAW_SENSORS, MAV_DATA_STREAM_EXTRA1};
  const uint16_t MAVRates[maxStreams] = {0x02,0x05};

    
  for (int i=0; i < maxStreams; i++) {
  
    mavlink_msg_request_data_stream_pack(2, 200, &msg, 1, 0, MAVStreams[i], MAVRates[i], 1);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial.write(buf,len);

    mySerial.write(buf, len);

  }
  
}



void comm_receive() {
 
  mavlink_message_t msg;
  mavlink_status_t status;
  

  // Serial.println("---Start---");
  // delay(500);


  while(mySerial.available()>0) {
    uint8_t c = mySerial.read();


    // Try to get a new message
    if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {

      // Handle message
      switch(msg.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT:  // #0: Heartbeat
          {
            // E.g. read GCS heartbeat and go into
            // comm lost mode if timer times out

            Serial.println("PX HB");

          }
         

        case MAVLINK_MSG_ID_SYS_STATUS:  // #1: SYS_STATUS
          {
            /* Message decoding: PRIMITIVE
             *    mavlink_msg_sys_status_decode(const mavlink_message_t* msg, mavlink_sys_status_t* sys_status)
             */
            //mavlink_message_t* msg;
            mavlink_sys_status_t sys_status;
            mavlink_msg_sys_status_decode(&msg, &sys_status);

            Serial.print("PX SYS STATUS: ");
            Serial.print("[Bat (V): ");
            Serial.print(sys_status.voltage_battery);
            Serial.print("], [Bat (A): ");
            Serial.print(sys_status.current_battery);
            Serial.print("], [Comms loss (%): ");
            Serial.print(sys_status.drop_rate_comm);
            Serial.println("]");
            Serial.print("[Batt remaining: ");
            Serial.print(sys_status.battery_remaining); 
            Serial.println("]");
          }
        

        case MAVLINK_MSG_ID_PARAM_VALUE:  // #22: PARAM_VALUE
          {
          
            mavlink_param_value_t param_value;
            mavlink_msg_param_value_decode(&msg, &param_value);

            Serial.println("PX PARAM_VALUE");
            Serial.println(param_value.param_value);
            Serial.println(param_value.param_count);
            Serial.println(param_value.param_index);
            Serial.println(param_value.param_id);
            Serial.println(param_value.param_type);
            Serial.println("------ Fin -------");

          }
       

        case MAVLINK_MSG_ID_RAW_IMU:  // #27: RAW_IMU
          {
            /* Message decoding: PRIMITIVE
             *    static inline void mavlink_msg_raw_imu_decode(const mavlink_message_t* msg, mavlink_raw_imu_t* raw_imu)
             */
            mavlink_raw_imu_t raw_imu;
            mavlink_msg_raw_imu_decode(&msg, &raw_imu);

            Serial.println("PX RAW IMU");
            Serial.println(raw_imu.xacc);

          }
         

        case MAVLINK_MSG_ID_ATTITUDE:  // #30
          {
            /* Message decoding: PRIMITIVE
             *    mavlink_msg_attitude_decode(const mavlink_message_t* msg, mavlink_attitude_t* attitude)
             */
            mavlink_attitude_t attitude;
            mavlink_msg_attitude_decode(&msg, &attitude);

            Serial.println("PX ATTITUDE");
            Serial.println(attitude.roll);
          }
        
        
          
        case  MAVLINK_MSG_ID_DISTANCE_SENSOR_LEN:
          {
            mavlink_distance_sensor_t sys_status;
            mavlink_msg_distance_sensor_decode(&msg, &sys_status);

            Serial.print("[Dist: ");
            Serial.print(sys_status.current_distance); 
            Serial.println("]");
          }
       default:

          Serial.print("--- Others: ");
          Serial.print("[ID: ");
          Serial.print(msg.msgid);
          Serial.print("], [seq: ");
          Serial.print(msg.seq);
          Serial.println("]");

          break;
      }
    }
  }
}
