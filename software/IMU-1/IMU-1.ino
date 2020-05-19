
// Need libraries :
// https://github.com/adafruit/Adafruit_MPU6050
// https://github.com/adafruit/Adafruit_LIS3DH

// Need Board :
// ESP32-WROOM-32D

/////////////////////////////////////////////////////////////////////////
//////////////////////// Configuration //////////////////////////////////
/////////////////////////////////////////////////////////////////////////
#define DEVICE_NAME "IMU-1"

// Uncomment to enable the features
#define ENABLE_OTA
#define ENABLE_SERIAL

/************************* WiFi Access Point *********************************/

#include <WiFi.h>        // Include the Wi-Fi library
#include <ESPmDNS.h>
#include <WiFiUdp.h>

#ifdef ENABLE_OTA
  #include <ArduinoOTA.h>
#endif

#include "credentials.h"        // Include Credentials (you need to create that file in the same folder if you cloned it from git)
/*
Content of "credentials.h" that matters for this section

// WIFI Credentials

#define WIFI_SSID        "[REPLACE BY YOUR WIFI SSID (2G)]"     // The SSID (name) of the Wi-Fi network you want to connect to
#define WIFI_PASSWORD    "[REPLACE BY YOUR WIFI PASSWORD]"      // The password of the Wi-Fi 
*/

const char* ssid     = WIFI_SSID;         // The SSID (name) of the Wi-Fi network you want to connect to
const char* password = WIFI_PASSWORD;     // The password of the Wi-Fi 


/************************* MQTT Setup *********************************/

#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include "credentials.h"

/*
// MQTT Credentials

Content of "credentials.h" that matters for this section

#define AIO_SERVER      "[REPLACE BY YOUR MQTT SERVER IP ADDRESS OR ITS FQDN]"
#define AIO_SERVERPORT  [REPLACE BY THE PORT NUMBER USED FOR THE MQTT SERVICE ON YOUR MQTT SERVEUR (DEFAULT IS 1883)]       // use 8883 for SSL"
#define AIO_USERNAME    ""  // USE THIS IF YOU HAVE USERNAME AND PASSWORD ENABLED ON YOUR MQTT SERVER
#define AIO_KEY         ""  // USE THIS IF YOU HAVE USERNAME AND PASSWORD ENABLED ON YOUR MQTT SERVER
*/

/************ Global State (you don't need to change this!) ******************/

// Create an ESP8266 WiFiClient class to connect to the MQTT server.
WiFiClient client;
// or... use WiFiFlientSecure for SSL
//WiFiClientSecure client;

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

/****************************** Feeds ***************************************/

// Setup a feeds for publishing.
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>
Adafruit_MQTT_Publish stat_ADXL337_acc_raw_x = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/" DEVICE_NAME "/stat/ADXL337/acc_raw/x"); 
Adafruit_MQTT_Publish stat_ADXL337_acc_raw_y = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/" DEVICE_NAME "/stat/ADXL337/acc_raw/y"); 
Adafruit_MQTT_Publish stat_ADXL337_acc_raw_z = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/" DEVICE_NAME "/stat/ADXL337/acc_raw/z"); 

Adafruit_MQTT_Publish stat_MPU6050_acc_x = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/" DEVICE_NAME "/stat/MPU6050/acc/x");
Adafruit_MQTT_Publish stat_MPU6050_acc_y = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/" DEVICE_NAME "/stat/MPU6050/acc/y");
Adafruit_MQTT_Publish stat_MPU6050_acc_z = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/" DEVICE_NAME "/stat/MPU6050/acc/z");
Adafruit_MQTT_Publish stat_MPU6050_rot_x = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/" DEVICE_NAME "/stat/MPU6050/rot/x");
Adafruit_MQTT_Publish stat_MPU6050_rot_y = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/" DEVICE_NAME "/stat/MPU6050/rot/y");
Adafruit_MQTT_Publish stat_MPU6050_rot_z = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/" DEVICE_NAME "/stat/MPU6050/rot/z");
Adafruit_MQTT_Publish stat_MPU6050_temp = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/" DEVICE_NAME "/stat/MPU6050/temp");

Adafruit_MQTT_Publish stat_LIS3DH_acc_raw_x = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/" DEVICE_NAME "/stat/LIS3DH/acc_raw/x");
Adafruit_MQTT_Publish stat_LIS3DH_acc_raw_y = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/" DEVICE_NAME "/stat/LIS3DH/acc_raw/y");
Adafruit_MQTT_Publish stat_LIS3DH_acc_raw_z = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/" DEVICE_NAME "/stat/LIS3DH/acc_raw/z");
Adafruit_MQTT_Publish stat_LIS3DH_acc_x = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/" DEVICE_NAME "/stat/LIS3DH/acc/x");
Adafruit_MQTT_Publish stat_LIS3DH_acc_y = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/" DEVICE_NAME "/stat/LIS3DH/acc/y");
Adafruit_MQTT_Publish stat_LIS3DH_acc_z = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/" DEVICE_NAME "/stat/LIS3DH/acc/z");

// Bug workaround for Arduino 1.6.6, it seems to need a function declaration
// for some reason (only affects ESP8266, likely an arduino-builder bug).
void MQTT_connect();


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////// PIN connections ////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*


                                                  ESP32-WROOM-32D
                                        +---------------------------------+                        
                                        |                                 |      
                                        | [ ]3v3                   GND[ ] |
             GND  <-   -|10uF|+   <-    | [ ]EN                   IO23[ ] |
                                        | [ ]SENSOR VP            IO22[ ] |   ->  ITG/MPU6050_SDL / LIS3DH_SDL
                                        | [ ]SENSOR VN            TXD0[ ] |
      ADXL337_X       (ANALOG)    <-    | [ ]IO34                 RXD0[ ] |
      ADXL337_Y       (ANALOG)    <-    | [ ]IO35                 IO21[ ] |   ->  ITG/MPU6050_SDA / LIS3DH_SDA
      ADXL337_Z       (ANALOG)    <-    | [ ]IO32                  GND[ ] |
      ITG/MPU6050_INT (INTERUPT)  <-    | [ ]IO33                 IO19[ ] |
      LIS3DH_INT      (INTERUPT)  <-    | [ ]IO25                 IO18[ ] |
                                        | [ ]IO26                  IO5[ ] |
                                        | [ ]IO27                 IO17[ ] |
                                        | [ ]IO14                 IO16[ ] |
                                        | [ ]IO12                  IO4[ ] |        
                                        | [ ]GND                   IO0[ ] |
                                        | [ ]IO13                  IO2[ ] |
                                        | [ ]SD2                  IO15[ ] |
                                        | [ ]SD3                   SD1[ ] |
                                        | [ ]CMD                   SD0[ ] |
                                        | [ ]5v                    CLK[ ] |
                                        |                                 |
                                        |   +-----+             +-----+   |
                                        |   |FLASH|             |RESET|   |
                                        |   +-----+  +-------+  +-----+   |
                                        +------------|  USB  |------------+
                                                     +-------+
*/

const int ADXL337_X_PIN = 34; // ADXL337 accelerometer axis X 
const int ADXL337_Y_PIN = 35; // ADXL337 accelerometer axis Y 
const int ADXL337_Z_PIN = 32; // ADXL337 accelerometer axis Z 

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////// END PIN connections //////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////// STUFF required for MPU6050 /////////////

// Basic demo for accelerometer readings from Adafruit MPU6050

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

///////////// END of STUFF required for MPU6050 /////////////



///////////// STUFF required for LIS3DH /////////////

// #include <Wire.h>    // => already included
// #include <Adafruit_Sensor.h>   // => already included
#include <Adafruit_LIS3DH.h>

Adafruit_LIS3DH lis = Adafruit_LIS3DH();

///////////// END of STUFF required for MPU6050 /////////////



///////////////////////////////////////////////////////
//////  ///////////////////////////////////////////////
//////  ///////////////////////////////////////////////
//////  ///////////                         ///////////
//////  ///////////                         ///////////
//\        ////////          SETUP          ///////////
///\      /////////                         ///////////
////\    //////////                         ///////////
/////\  ///////////////////////////////////////////////
///////////////////////////////////////////////////////
void setup() {

///////////////////////////////////////////////////////
///////////////////// Start Serial ////////////////////
///////////////////////////////////////////////////////
Serial.begin(115200);

///////////////////////////////////////////////////////
//////////////////// Start Wifi ///////////////////////
///////////////////////////////////////////////////////

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

  ////////////////// Initialize OTA /////////////////////
  // Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname(DEVICE_NAME);

  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();

  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());


#ifdef ENABLE_SERIAL
  Serial.println('\n');
  Serial.println("Connection established!");  
  Serial.print("IP address:\t");
  Serial.println(WiFi.localIP());         // Send the IP address of the ESP8266 to the computer
#endif

  
////////////////////// MPU6050 INITIALIZATION ////////////////////

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  Serial.println("");


//////////////////////// END OF MPU 6050 Initialization ////////////////////////


//////////////////////// LIS3DH Initialization ////////////////////////


  if (! lis.begin(0x18)) {   // change this to 0x19 for alternative i2c address
    Serial.println("Couldnt start");
    while (1) yield();
  }
  Serial.println("LIS3DH found!");

  // lis.setRange(LIS3DH_RANGE_4_G);   // 2, 4, 8 or 16 G!

  Serial.print("Range = "); Serial.print(2 << lis.getRange());
  Serial.println("G");

  // lis.setDataRate(LIS3DH_DATARATE_50_HZ);
  Serial.print("Data rate set to: ");
  switch (lis.getDataRate()) {
    case LIS3DH_DATARATE_1_HZ: Serial.println("1 Hz"); break;
    case LIS3DH_DATARATE_10_HZ: Serial.println("10 Hz"); break;
    case LIS3DH_DATARATE_25_HZ: Serial.println("25 Hz"); break;
    case LIS3DH_DATARATE_50_HZ: Serial.println("50 Hz"); break;
    case LIS3DH_DATARATE_100_HZ: Serial.println("100 Hz"); break;
    case LIS3DH_DATARATE_200_HZ: Serial.println("200 Hz"); break;
    case LIS3DH_DATARATE_400_HZ: Serial.println("400 Hz"); break;

    case LIS3DH_DATARATE_POWERDOWN: Serial.println("Powered Down"); break;
    case LIS3DH_DATARATE_LOWPOWER_5KHZ: Serial.println("5 Khz Low Power"); break;
    case LIS3DH_DATARATE_LOWPOWER_1K6HZ: Serial.println("16 Khz Low Power"); break;
    }

//////////////////////// END of LIS3DH Initialization ////////////////////////

  delay(5000);
}
///////////////////////////////////////////////////////
///  //////////////////////////////////////////////////
///  //////////////////////////////////////////////////
///  /// ///////////                        ///////////
///  ///   /////////                        ///////////
///          ///////      END OF SETUP      ///////////
////////   /////////                        ///////////
//////// ///////////                        ///////////
///////////////////////////////////////////////////////
///////////////////////////////////////////////////////


 ///////////////////////////////////////////////////////
//////  ///////////////////////////////////////////////
//////  ///////////////////////////////////////////////
//////  ///////////                         ///////////
//////  ///////////                         ///////////
//\        ////////           LOOP          ///////////
///\      /////////                         ///////////
////\    //////////                         ///////////
/////\  ///////////////////////////////////////////////
///////////////////////////////////////////////////////
void loop() {

#ifdef ENABLE_OTA
  ArduinoOTA.handle();
#endif

// Ensure the connection to the MQTT server is alive (this will make the first
// connection and automatically reconnect when disconnected).  See the MQTT_connect
// function definition further below.
MQTT_connect();

//////////////// ADXL3367 Reading ///////////////////////
  
int ADXL337_RAW_X = analogRead(ADXL337_X_PIN); //read from xpin
delay(1); //
int ADXL337_RAW_Y = analogRead(ADXL337_Y_PIN); //read from ypin
delay(1);
int ADXL337_RAW_Z = analogRead(ADXL337_Z_PIN); //read from zpin

////////////////// END of ADXL337 Reading //////////////////

delay(10); //wait for 10 milliseconds

////////////////// MPU6050 Reading /////////////////////////

/* Get new sensor events with the readings */
sensors_event_t MPU6050_accelerometer, MPU6050_gyroscope, MPU6050_thermometer;
mpu.getEvent(&MPU6050_accelerometer, &MPU6050_gyroscope, &MPU6050_thermometer);

//////////////// END of MPU 6050 Reading //////////////////

delay(10); //wait for 10 milliseconds

//////////////// LIS3DH Reading //////////////////

lis.read();      // get X Y and Z data at once

/* Or....get a new sensor event, normalized */
sensors_event_t LIS3DH_accelerometer;
lis.getEvent(&LIS3DH_accelerometer);

//////////////// END of LIS3DH Reading //////////////////

delay(10); //wait for 10 milliseconds

////////////////////////////////////////////////////////////
//////////////////////// data out //////////////////////////
////////////////////////////////////////////////////////////

// ADXL337 
Serial.print("AD_acc_raw X: ");                                           // ADXL337_X
Serial.print(ADXL337_RAW_X); //print x value on serial monitor
Serial.print("\t");
Serial.print("Y: ");
Serial.print(ADXL337_RAW_Y); //print y value on serial monitor
Serial.print("\t");
Serial.print("Z: ");
Serial.print(ADXL337_RAW_Z); //print z value on serial monitor
Serial.print("\t");
//Serial.print("\n");


// MPU6050
  /* Print out the values */
Serial.print("MP_acc (m/s^2) X: ");
Serial.print(MPU6050_accelerometer.acceleration.x);
Serial.print("\t");
Serial.print("Y: ");
Serial.print(MPU6050_accelerometer.acceleration.y);
Serial.print("\t");
Serial.print("Z: ");
Serial.print(MPU6050_accelerometer.acceleration.z);
Serial.print("\t");

Serial.print("MP_rot (rad/s) X: ");
Serial.print(MPU6050_gyroscope.gyro.x);
Serial.print("Y: ");
Serial.print(MPU6050_gyroscope.gyro.y);
Serial.print("Z: ");
Serial.print(MPU6050_gyroscope.gyro.z);
Serial.print("\t");

Serial.print("MP_temp (degC) : ");
Serial.print(MPU6050_thermometer.temperature);
Serial.print("\t");

//Serial.println("");

//LIS3DH

// Then print out the raw data
Serial.print("LI_acc_raw X: "); 
Serial.print(lis.x);
Serial.print("\t");
Serial.print("Y: "); 
Serial.print(lis.y);
Serial.print("\t");
Serial.print("Z: "); 
Serial.print(lis.z);
Serial.print("\t");

/* Display the results (acceleration is measured in m/s^2) */
Serial.print("LI_acc (m/s^2) X: "); 
Serial.print(LIS3DH_accelerometer.acceleration.x);
Serial.print("\t");
Serial.print("Y: "); 
Serial.print(LIS3DH_accelerometer.acceleration.y);
Serial.print("\t");
Serial.print("Z: "); 
Serial.print(LIS3DH_accelerometer.acceleration.z);
Serial.print("\t");

Serial.println();

  /// we repport status and publish to mqtt 
  stat_ADXL337_acc_raw_x.publish(ADXL337_RAW_X);
  stat_ADXL337_acc_raw_y.publish(ADXL337_RAW_Y);
  stat_ADXL337_acc_raw_z.publish(ADXL337_RAW_Z);

  stat_MPU6050_acc_x.publish(MPU6050_accelerometer.acceleration.x);
  stat_MPU6050_acc_y.publish(MPU6050_accelerometer.acceleration.y);
  stat_MPU6050_acc_z.publish(MPU6050_accelerometer.acceleration.z);
  stat_MPU6050_rot_x.publish(MPU6050_gyroscope.gyro.x);
  stat_MPU6050_rot_y.publish(MPU6050_gyroscope.gyro.y);
  stat_MPU6050_rot_z.publish(MPU6050_gyroscope.gyro.z);
  stat_MPU6050_temp.publish(MPU6050_thermometer.temperature);

  stat_LIS3DH_acc_raw_x.publish(lis.x);
  stat_LIS3DH_acc_raw_y.publish(lis.y);
  stat_LIS3DH_acc_raw_z.publish(lis.z);
  stat_LIS3DH_acc_x.publish(LIS3DH_accelerometer.acceleration.x);
  stat_LIS3DH_acc_y.publish(LIS3DH_accelerometer.acceleration.y);
  stat_LIS3DH_acc_z.publish(LIS3DH_accelerometer.acceleration.z);

delay(10); //wait for 10 milliseconds
}
///////////////////////////////////////////////////////
///  //////////////////////////////////////////////////
///  //////////////////////////////////////////////////
///  /// ///////////                        ///////////
///  ///   /////////                        ///////////
///          ///////       END OF LOOP      ///////////
////////   /////////                        ///////////
//////// ///////////                        ///////////
///////////////////////////////////////////////////////
///////////////////////////////////////////////////////


///////////////////////////////////////////////////////
//////  ///////////////////////////////////////////////
//////  ///////////////////////////////////////////////
//////  ///////////                         ///////////
//////  ///////////                         ///////////
//\        ////////        FUNCTIONS        ///////////
///\      /////////                         ///////////
////\    //////////                         ///////////
/////\  ///////////////////////////////////////////////
///////////////////////////////////////////////////////


///////////////////////////////////////////////////////
//////////////// MQTT_connect Function ////////////////
///////////////////////////////////////////////////////

// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

#ifdef ENABLE_SERIAL
  Serial.print("Connecting to MQTT... ");
#endif

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
#ifdef ENABLE_SERIAL
       Serial.println(mqtt.connectErrorString(ret));
       Serial.println("Retrying MQTT connection in 5 seconds...");
#endif
       mqtt.disconnect();
       delay(250);  // wait 5 seconds
       retries--;
       if (retries == 0) {
         // basically die and wait for WDT to reset me
         while (1);
       }
  }
#ifdef ENABLE_SERIAL
  Serial.println("MQTT Connected!");
#endif
}

///////////////////////////////////////////////////////
////////////// REPORT THE SYSTEM STATUS ///////////////
///////////// ON THE CONSOLE AND ON MQTT //////////////
///////////////////////////////////////////////////////

#define REPORT_STATUS_PERIOD_MS 500
void report_status()
{
  static unsigned long previous_millis = 0;
  if (millis() - previous_millis < REPORT_STATUS_PERIOD_MS) {
    // too early to report the status, abort now
    return;
  }

#ifdef ENABLE_SERIAL

#endif
  previous_millis = millis();
}

///////////////////////////////////////////////////////
///  //////////////////////////////////////////////////
///  //////////////////////////////////////////////////
///  /// ///////////                        ///////////
///  ///   /////////                        ///////////
///          ///////    END OF FUNCTIONS    ///////////
////////   /////////                        ///////////
//////// ///////////                        ///////////
///////////////////////////////////////////////////////
///////////////////////////////////////////////////////


/// Sources
// http://www.esp32learning.com/code/esp32-and-adxl335-accelerometer-example.php
