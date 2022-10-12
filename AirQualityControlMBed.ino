/* 
    Made by: AGH Solar Plane 2022
    For Rapsberry Pico - BJP project 
    Raspberry Pico pinout PDF:
    https://datasheets.raspberrypi.com/pico/Pico-R3-A4-Pinout.pdf
*/

/*______________________________________ INCLUDE ______________________________________*/

#include <mavlink.h>
#include <Arduino.h>
#include <SFE_BMP180.h>
#include <Wire.h>
#include <MQ135.h>
#include <DFRobot_OzoneSensor.h>

/*______________________________________ DEFINE ______________________________________*/

#define OZONE_COLLECT_NUMBER   20                                       // collect number, the collection range is 1-100
#define Ozone_IICAddress 0x73
/*   iic slave Address, The default is ADDRESS_3
       ADDRESS_0               0x70       // iic device address
       ADDRESS_1               0x71
       ADDRESS_2               0x72
       ADDRESS_3               0x73
*/
#define BMP180_ALTITUDE 1655.0                                    // Altitude of SparkFun's HQ in Boulder, CO. in meters (relative altitude for BMP180 sensor)
#define MQ_7_PIN 26                                               // TODO: change that
#define MQ135_PIN 27
#define SENSOR_NUM 4
/*______________________________________ TYPES ______________________________________*/

typedef struct CoordsRPY{
  double roll=0.0;
  double pitch=0.0;
  double yaw=0.0;
};

typedef int16_t OzoneReading;
typedef int     MQ7Reading;
typedef struct  MQ135Reading{
  float rzero = 0.0;
  float correctedRZero = 0.0;
  float resistance = 0.0;
  float ppm = 0.0;
  float correctedPPM = 0.0;
};

typedef unsigned long int timeMicroseconds;

typedef struct Measurements{
  CoordsRPY rpy;
  OzoneReading ozone;
  MQ7Reading mq7;
  MQ135Reading mq135;
};

typedef struct ExecutionTimes{
  timeMicroseconds rpy_time = 0;
  timeMicroseconds ozone_time = 0;
  timeMicroseconds mq7_time = 0;
  timeMicroseconds mq135_time = 0;
};

/*______________________________________ GLOBAL ______________________________________*/

/* Sensor list - used as indexes in sensor active flag table below */
enum sensor_list{
  OZONE_SENSOR,
  MQ7_SENSOR,
  MQ135_SENSOR,
  BMP180_SENSOR
};

/* Sensor active - flag table. If value is true - sensor is properly initailized and active */
bool sensor_active[SENSOR_NUM] = {false};

UART Serial2(8,9,0,0);

DFRobot_OzoneSensor sensor_ozone;
MQ135               sensor_mq135(MQ135_PIN);
SFE_BMP180          sensor_bmp180;

Measurements measurements_global; // here all measurements will be collected for global use
ExecutionTimes execution_times_global; // here all execution times are stored for global use

/*_______________________________ FUNCTION DEFINITIONS ________________________________*/

void serialInit();                            // For Serial Communication to make sure both cores initialize it first
void requestPixStream();                      // Request needed stream from PIXHAWK
void LOG(String topic, String message);       // Logging information to Serial (PC).
void sensorInit();                            // Initialize all sensors

void getAllMeasurements();
void logAllMeasurements();

timeMicroseconds getCoordinatesFromPix();                 // Receives frames containing coordinates from PIXHAWK and saves it in global variable
timeMicroseconds getDataOZONE();                          // Get measurements from Ozone sensor
timeMicroseconds getDataMQ7();                            // Get measurements from MQ7 sensor
timeMicroseconds getDataMQ135();                          // Get measurements from MQ135 sensor


/*______________________________________ SETUP ______________________________________*/

void setup() {
  // Initialize serial communication
  serialInit();
  // Initialize all sensors (check sensor_active for sensor status)
  sensorInit();
  // Request PIX stream with GPS, altitude and RPY data
  requestPixStream();
  delay(5000);
}

/*______________________________________ LOOP ______________________________________*/

void loop() {
  getAllMeasurements();
  logAllMeasurements();
  delay(500);
}

/*______________________________ FUNCTION BODIES ____________________________________*/

void serialInit(){
  // Serial will be used to communicate and log current situation into PC
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  // Serial1 is UART1 on Pico, will be used to communicate with PIXHAWK flight controller
  // Ports used: Tx at GP8, Rx at GP9
  Serial2.begin(57600);
  while (!Serial2) {
    ; // wait for serial1 port to connect. Needed for native USB port only
  }
}

void sensorInit(){
  // Initialize OZONE Sensor
  if(!sensor_ozone.begin(Ozone_IICAddress)) {
    LOG("OZONE","I2C connection fail");
    sensor_active[OZONE_SENSOR] = false;
  } else {
    LOG("OZONE","I2C connection succeeded");
    sensor_ozone.setModes(MEASURE_MODE_PASSIVE);
    sensor_active[OZONE_SENSOR] = true;
  }

  // Initialize MQ_7 Sensor
  sensor_active[MQ7_SENSOR] = true;
  LOG("MQ7","Initialization succeeded");

  // Initialize MQ_135 Sensor;
  sensor_active[MQ135_SENSOR] = true;
  LOG("MQ135","Initialization succeeded");
  
  // Initialize BMP180 Sensor
  if (!sensor_bmp180.begin()) {
    LOG("BMP180","Initialization failed.");
    sensor_active[BMP180_SENSOR] = false;
  } else {
    LOG("BMP180","Initialization succeeded.");
    sensor_active[BMP180_SENSOR] = true;
  }
}

void requestPixStream(){
  //Request Data from Pixhawk
  uint8_t _system_id = 255; // id of computer which is sending the command (ground control software has id of 255)
  uint8_t _component_id = 2; // seems like it can be any # except the number of what Pixhawk sys_id is
  uint8_t _target_system = 1; // Id # of Pixhawk (should be 1)
  uint8_t _target_component = 0; // Target component, 0 = all (seems to work with 0 or 1)
  uint8_t _req_stream_id = MAV_DATA_STREAM_RAW_SENSORS;
  uint16_t _req_message_rate = 0x01; //number of times per second to request the data in hex
  uint8_t _start_stop = 1; // 1 = start, 0 = stop
 

  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
 
  // Pack the message
  mavlink_msg_request_data_stream_pack(_system_id, _component_id, &msg, _target_system, _target_component, _req_stream_id, _req_message_rate, _start_stop);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);  // Send the message (.write sends as bytes)
 
  Serial2.write(buf, len); //Write data to serial port
}

void LOG(String topic, String message){
  String log = String("[" + topic + "]: " + message);
  Serial.println(log);
}

timeMicroseconds getCoordinatesFromPix(){
  timeMicroseconds start_time = micros();
  CoordsRPY coords_output;
  mavlink_message_t msg;
  mavlink_status_t status;
  bool reading_ready = false;

  while(!reading_ready){
    if(Serial2.available()){
      uint8_t c = Serial2.read();

      if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)){
        if(msg.msgid == MAVLINK_MSG_ID_ATTITUDE){
          mavlink_attitude_t at;
          mavlink_msg_attitude_decode(&msg, &at);
          coords_output.roll = at.roll;
          coords_output.pitch = at.pitch;
          coords_output.yaw = at.yaw;
          reading_ready = true;
        }
      }
    }
  }
  measurements_global.rpy = coords_output;
  timeMicroseconds end_time = micros();
  return end_time-start_time;
}

timeMicroseconds getDataOZONE(){
  timeMicroseconds start_time = micros();
  /*   Smooth data collection
       COLLECT_NUMBER                    // The collection range is 1-100
  */
  measurements_global.ozone = (OzoneReading)sensor_ozone.readOzoneData(OZONE_COLLECT_NUMBER);
  timeMicroseconds end_time = micros();
  return end_time-start_time;
}

timeMicroseconds getDataMQ7(){
  timeMicroseconds start_time = micros();
  measurements_global.mq7 = (MQ7Reading)analogRead(MQ_7_PIN);
  timeMicroseconds end_time = micros();
  return end_time-start_time;
}

timeMicroseconds getDataMQ135(){
  timeMicroseconds start_time = micros();
  MQ135Reading output;
  float temperature = 21.0; // Assume current temperature. Recommended to measure with DHT22
  float humidity = 25.0; // Assume current humidity. Recommended to measure with DHT22
  output.rzero = sensor_mq135.getRZero();
  output.correctedRZero = sensor_mq135.getCorrectedRZero(temperature, humidity);
  output.resistance = sensor_mq135.getResistance();
  output.ppm = sensor_mq135.getPPM();
  output.correctedPPM = sensor_mq135.getCorrectedPPM(temperature, humidity);
  measurements_global.mq135 = output;
  timeMicroseconds end_time = micros();
  return end_time-start_time;
}

void getAllMeasurements(){
  // Measure coordinates with PIXHAWK
  execution_times_global.rpy_time = getCoordinatesFromPix();
  
  // Measure from Ozone sensor
  execution_times_global.ozone_time = getDataOZONE();

  // Measure from MQ7 sensor
  execution_times_global.mq7_time = getDataMQ7();

  // Measure from MQ135 sensor
  execution_times_global.mq135_time = getDataMQ135();
}

void logAllMeasurements(){
  // Prepare logs for PIXHAWK readings
  String pix_roll_str = String("Roll: " + String(measurements_global.rpy.roll));
  String pix_pitch_str = String("Pitch: " + String(measurements_global.rpy.pitch));
  String pix_yaw_str = String("Yaw: " + String(measurements_global.rpy.yaw));
  String pix_log_str = String(pix_roll_str + " , " + pix_pitch_str + " , " + pix_yaw_str + " | Time: "  + \
  String(execution_times_global.rpy_time/1e3) + "ms");

  // Prepare logs for OZONE readings PPM
  String ozone_log_str = String("Measurement: " + String(measurements_global.ozone/1e3) + " | Time: " + \
  String(execution_times_global.ozone_time/1e3) + "ms");

  // Prepare logs for MQ7 readings PPM
  String mq7_log_str = String("Measurement: " + String(measurements_global.mq7) + " | Time: " + \
  String(execution_times_global.mq7_time/1e3) + "ms");

  // Prepare logs for MQ135 readings PPM
  // String mq135_rzero_str = String("RZero: " + String(measurements_global.mq135.rzero));
  // String mq135_correctedRZero_str = String("CorrectedRZero: " + String(measurements_global.mq135.correctedRZero));
  // String mq135_resistance_str = String("Resistance: " + String(measurements_global.mq135.resistance));
  // String mq135_ppm_str = String("PPM: " + String(measurements_global.mq135.ppm));
  String mq135_correctedPPM_str = String("CorrectedPPM: " + String(measurements_global.mq135.correctedPPM));
  String mq135_log_str = String(mq135_rzero_str + " , " + mq135_correctedRZero_str + " , " + mq135_resistance_str + " , " + mq135_ppm_str + " , " + mq135_correctedPPM_str + \
  " | Time: " + String(execution_times_global.mq135_time/1e3) + "ms");

  // Log all prepared strings

  Serial.println("----------------------------MEASUREMENTS----------------------------");
  LOG("PIX",pix_log_str);  
  LOG("OZONE",ozone_log_str);
  LOG("MQ7",mq7_log_str);
  LOG("MQ135",mq135_log_str);
  Serial.println("--------------------------------------------------------------------");
}
