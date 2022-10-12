/* 
    Made by: AGH Solar Plane 2022
    Requires Earle Philhower's Arduino Pico core:
    https://github.com/earlephilhower/arduino-pico
    Documentation:
    https://arduino-pico.readthedocs.io/en/latest/index.html
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
#include <SPI.h>
#include <RP2040_SD.h>

/*______________________________________ DEFINE ______________________________________*/

#define OZONE_COLLECT_NUMBER 20  // collect number, the collection range is 1-100
#define Ozone_IICAddress 0x73
/*   iic slave Address, The default is ADDRESS_3
       ADDRESS_0               0x70       // iic device address
       ADDRESS_1               0x71
       ADDRESS_2               0x72
       ADDRESS_3               0x73
*/
#define MQ_7_PIN 26
#define MQ135_PIN 27

#define _RP2040_SD_LOGLEVEL_ 0

#define PIN_SD_MOSI PIN_SPI0_MOSI
#define PIN_SD_MISO PIN_SPI0_MISO
#define PIN_SD_SCK PIN_SPI0_SCK
#define PIN_SD_SS PIN_SPI0_SS

/*______________________________________ TYPES ______________________________________*/

struct CoordsGPS {
  double lat = 0.0;
  double lon = 0.0;
  double alt = 0.0;
};

typedef double OzoneReading;
typedef int MQ7Reading;
struct MQ135Reading {
  float ppm = 0.0;
  float correctedPPM = 0.0;
};
struct BMP180Reading {
  double Temp = 0.0;
  double Pressure = 0.0;
};

typedef unsigned long int timeMicroseconds;

struct Measurements {
  CoordsGPS gps;
  OzoneReading ozone;
  MQ7Reading mq7;
  MQ135Reading mq135;
  BMP180Reading bmp180;
};

struct ExecutionTimes {
  timeMicroseconds gps_time = 0;
  timeMicroseconds ozone_time = 0;
  timeMicroseconds mq7_time = 0;
  timeMicroseconds mq135_time = 0;
  timeMicroseconds bmp180_time = 0;
};

/*______________________________________ GLOBAL ______________________________________*/

/* Sensor list - used as indexes in sensor active flag table below */
enum sensor_list {
  OZONE_SENSOR,
  MQ7_SENSOR,
  MQ135_SENSOR,
  BMP180_SENSOR,
  END
};

/* Sensor active - flag table. If value is true - sensor is properly initailized and active */
bool sensor_active[END - OZONE_SENSOR] = { false };

DFRobot_OzoneSensor sensor_ozone;
// MQ135               sensor_mq135(MQ135_PIN);
SFE_BMP180 sensor_bmp180;

Measurements measurements_global;       // here all measurements will be collected for global use
ExecutionTimes execution_times_global;  // here all execution times are stored for global use

String sd_file_name;
String sd_file_column_tags = "lat, lon, alt, ozone, mq7, mq135_ppm, mq135_cor, bmp180_temp, bmp180_press";

/*______________________________________ FUNCTION DEFINITIONS ______________________________________*/

void serialInit();                       // For Serial Communication to make sure both cores initialize it first
void sensorInit();                       // Initialize all sensors
void memoryInit();                       // Initialize SD card
void requestPixStream();                 // Request needed stream from PIXHAWK
void LOG(String topic, String message);  // Logging information to Serial (PC).


void getAllMeasurements();
void logAllMeasurements();

timeMicroseconds getCoordinatesFromPix();  // Receives frames containing coordinates from PIXHAWK and saves it in global variable
timeMicroseconds getDataOZONE();           // Get measurements from Ozone sensor
timeMicroseconds getDataMQ7();             // Get measurements from MQ7 sensor
timeMicroseconds getDataMQ135();           // Get measurements from MQ135 sensor
timeMicroseconds getDataBMP180();          // Get measurements from BMP180 sensor

void createSDFile(String filename, String variables);  // Create SD Card file on which data will be stored
void writeLog(String filename);                        // Write data into SD card


/*______________________________________ SETUP ______________________________________*/

void setup() {
  // Initialize serial communication
  serialInit();
  // Initialize all sensors (check sensor_active for sensor status)
  sensorInit();
  // Initialize SD card memory
  memoryInit();
  // Create SD card file where measurements will be stored
  randomSeed(analogRead(MQ_7_PIN));
  int file_num = (int)random(1000);
  sd_file_name = String("TEST" + String(file_num) + ".txt");
  createSDFile(sd_file_name, sd_file_column_tags);
  // Request PIX stream with GPS, altitude and RPY data
  requestPixStream();
  delay(5000);
}

/*______________________________________ LOOP ______________________________________*/

void loop() {
  getAllMeasurements();
  logAllMeasurements();
  writeLog(sd_file_name);
  delay(100);
}

/*______________________________________ FUNCTION BODIES ______________________________________*/

void serialInit() {
  // Serial will be used to communicate and log current situation into PC
  Serial.begin(9600);
  while (!Serial) {
    ;  // wait for serial port to connect. Needed for native USB port only
  }
  // Serial1 is UART1 on Pico, will be used to communicate with PIXHAWK flight controller
  // Ports used: Tx at GP8, Rx at GP9
  Serial1.begin(57600);
  while (!Serial1) {
    LOG("PIX", "Serial not initialized!");
    delay(1000);
  }
}

void sensorInit() {
  // Initialize OZONE Sensor
  if (!sensor_ozone.begin(Ozone_IICAddress)) {
    LOG("OZONE", "I2C connection fail");
    sensor_active[OZONE_SENSOR] = false;
  } else {
    LOG("OZONE", "I2C connection succeeded");
    sensor_ozone.setModes(MEASURE_MODE_PASSIVE);
    sensor_active[OZONE_SENSOR] = true;
  }

  // Initialize MQ_7 Sensor
  sensor_active[MQ7_SENSOR] = true;
  LOG("MQ7", "Initialization succeeded");

  // Initialize MQ_135 Sensor;
  sensor_active[MQ135_SENSOR] = true;
  LOG("MQ135", "Initialization succeeded");

  // Initialize BMP180 Sensor
  if (!sensor_bmp180.begin()) {
    LOG("BMP180", "Initialization failed.");
    sensor_active[BMP180_SENSOR] = false;
  } else {
    LOG("BMP180", "Initialization succeeded.");
    sensor_active[BMP180_SENSOR] = true;
  }
}

void memoryInit() {
  // Initialize SD card
  // It will be connected to default Pico SPI ports:
  // MOSI : PIN 19
  // MISO : PIN 16
  // SCK  : PIN 18
  // CS   : PIN 17
  if (!SD.begin(PIN_SD_SS)) {
    LOG("SD", "Initialization failed.");
  } else {
    LOG("SD", "Initialization succeeded.");
  }
}

void requestPixStream() {
  //Request Data from Pixhawk
  uint8_t _system_id = 255;       // id of computer which is sending the command (ground control software has id of 255)
  uint8_t _component_id = 2;      // seems like it can be any # except the number of what Pixhawk sys_id is
  uint8_t _target_system = 1;     // Id # of Pixhawk (should be 1)
  uint8_t _target_component = 0;  // Target component, 0 = all (seems to work with 0 or 1)
  uint8_t _req_stream_id = MAV_DATA_STREAM_RAW_SENSORS;
  uint16_t _req_message_rate = 0x01;  //number of times per second to request the data in hex
  uint8_t _start_stop = 1;            // 1 = start, 0 = stop


  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
  mavlink_msg_request_data_stream_pack(_system_id, _component_id, &msg, _target_system, _target_component, _req_stream_id, _req_message_rate, _start_stop);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);  // Send the message (.write sends as bytes)

  Serial1.write(buf, len);  //Write data to serial port
}

void LOG(String topic, String message) {
  String log = String("[" + topic + "]: " + message);
  Serial.println(log);
}

timeMicroseconds getCoordinatesFromPix() {
  timeMicroseconds start_time = micros();
  CoordsGPS coords_output;
  mavlink_message_t msg;
  mavlink_status_t status;
  bool reading_ready = false;
  int timeout = 0;

  while (!reading_ready && timeout < 8e4) {
    if (Serial1.available()) {
      uint8_t c = Serial1.read();

      if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
        if (msg.msgid == MAVLINK_MSG_ID_GPS_RAW_INT) {
          mavlink_gps_raw_int_t at;
          mavlink_msg_gps_raw_int_decode(&msg, &at);
          coords_output.lat = ((double)at.lat) / 1e7;
          coords_output.lon = ((double)at.lon) / 1e7;
          coords_output.alt = ((double)at.alt) / 1e3;
          reading_ready = true;
        }
      }
    }
    timeout++;
  }
  measurements_global.gps = coords_output;
  timeMicroseconds end_time = micros();
  return end_time - start_time;
}

timeMicroseconds getDataOZONE() {
  timeMicroseconds start_time = micros();
  /*   Smooth data collection
       COLLECT_NUMBER                    // The collection range is 1-100
  */
  measurements_global.ozone = ((OzoneReading)sensor_ozone.readOzoneData(OZONE_COLLECT_NUMBER)) / 1000;
  timeMicroseconds end_time = micros();
  return end_time - start_time;
}

timeMicroseconds getDataMQ7() {
  timeMicroseconds start_time = micros();
  measurements_global.mq7 = (MQ7Reading)analogRead(MQ_7_PIN);
  timeMicroseconds end_time = micros();
  return end_time - start_time;
}

timeMicroseconds getDataMQ135() {
  timeMicroseconds start_time = micros();
  MQ135Reading output;
  float temperature = 21.0;  // Assume current temperature. Recommended to measure with DHT22
  float humidity = 25.0;     // Assume current humidity. Recommended to measure with DHT22
  // output.ppm = sensor_mq135.getPPM();
  // output.correctedPPM = sensor_mq135.getCorrectedPPM(temperature, humidity);
  measurements_global.mq135 = output;
  timeMicroseconds end_time = micros();
  return end_time - start_time;
}

timeMicroseconds getDataBMP180() {
  timeMicroseconds start_time = micros();
  BMP180Reading output;
  char status;
  double T, P;
  status = sensor_bmp180.startTemperature();
  if (status != 0) {
    delay(status);
    status = sensor_bmp180.getTemperature(T);
    output.Temp = T;
    if (status != 0) {
      delay(status);
      status = sensor_bmp180.startPressure(3);
      if (status != 0) {
        delay(status);
        status = sensor_bmp180.getPressure(P, T);
        output.Pressure = P;
        if (status != 0) {
          delay(status);
        } else {
          ;
        }
      } else {
        ;
      }
    } else {
      ;
    }
  } else {
    ;
  }
  measurements_global.bmp180 = output;
  timeMicroseconds end_time = micros();
  return end_time - start_time;
}

void getAllMeasurements() {
  // Measure coordinates with PIXHAWK
  execution_times_global.gps_time = getCoordinatesFromPix();

  // Measure from Ozone sensor
  execution_times_global.ozone_time = getDataOZONE();

  // Measure from MQ7 sensor
  execution_times_global.mq7_time = getDataMQ7();

  // Measure from MQ135 sensor
  execution_times_global.mq135_time = getDataMQ135();

  // Measure from BMP180 sensor
  execution_times_global.bmp180_time = getDataBMP180();
}

void logAllMeasurements() {
  // Prepare logs for PIXHAWK readings
  String pix_lat_str = String("Latitude: " + String(measurements_global.gps.lat));
  String pix_lon_str = String("Longitude: " + String(measurements_global.gps.lon));
  String pix_alt_str = String("Altitude: " + String(measurements_global.gps.alt));
  String pix_log_str = String(pix_lat_str + " , " + pix_lon_str + " , " + pix_alt_str + " | Time: " + String(execution_times_global.gps_time / 1e3) + "ms");

  // Prepare logs for OZONE readings
  String ozone_log_str = String("Measurement: " + String(measurements_global.ozone) + " ppm | Time: " + String(execution_times_global.ozone_time / 1e3) + "ms");

  // Prepare logs for MQ7 readings
  String mq7_log_str = String("Measurement: " + String(measurements_global.mq7) + " ppm | Time: " + String(execution_times_global.mq7_time / 1e3) + "ms");

  // Prepare logs for MQ135 readings
  String mq135_ppm_str = String("Measurement: " + String(measurements_global.mq135.ppm) + " ppm");
  String mq135_correctedPPM_str = String("CorrectedMeasurement: " + String(measurements_global.mq135.correctedPPM) + " ppm");
  String mq135_log_str = String(mq135_ppm_str + " , " + mq135_correctedPPM_str + " | Time: " + String(execution_times_global.mq135_time / 1e3) + "ms");

  // Prepare logs for BMP180 readings
  String bmp180_temp_str = String("Temperature: " + String(measurements_global.bmp180.Temp) + " C");
  String bmp180_press_str = String("Pressure: " + String(measurements_global.bmp180.Pressure) + " hPa");
  String bmp180_log_str = String(bmp180_temp_str + " , " + bmp180_press_str + " | Time: " + String(execution_times_global.bmp180_time / 1e3) + "ms");

  // Prepare logs for SD card
  String sd_card_log_str = String("SD file name: " + sd_file_name);

  // Log all prepared strings

  Serial.println("----------------------------MEASUREMENTS----------------------------");
  LOG("SD",sd_card_log_str);
  LOG("PIX", pix_log_str);
  LOG("OZONE", ozone_log_str);
  LOG("MQ7", mq7_log_str);
  LOG("MQ135", mq135_log_str);
  LOG("BMP180", bmp180_log_str);
  Serial.println("--------------------------------------------------------------------");
}

void createSDFile(String filename, String variables) {
  filename.toUpperCase();
  File logfile = SD.open(filename, FILE_WRITE);
  logfile.println(variables);
  logfile.close();
}

void writeLog(String filename) {
  String dataString = String(measurements_global.gps.lat, 6) + ", " + \
  String(measurements_global.gps.lon, 6) + ", " + \
  String(measurements_global.gps.alt, 6) + ", " + \
  String(measurements_global.ozone, 6) + ", " + \
  String(measurements_global.mq7, 6) + ", " + \
  String(measurements_global.mq135.ppm, 6) + ", " + \
  String(measurements_global.mq135.correctedPPM, 6) + ", " + \
  String(measurements_global.bmp180.Temp, 6) + ", " + \
  String(measurements_global.bmp180.Pressure, 6);
  // sd_file_name.toUpperCase();
  File logfile = SD.open(filename, FILE_WRITE);
  logfile.println(dataString);
  logfile.close();
}