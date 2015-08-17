#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <Adafruit_LSM303.h>
#include <Adafruit_BMP085.h>
#include <SoftwareSerial.h>

#define GPSECHO false
#define chipSelect 10

SoftwareSerial mySerial(8, 7); // RX, TX

Adafruit_GPS GPS(&mySerial);
Adafruit_LSM303 lsm;
Adafruit_BMP085 bmp;

uint32_t timer = millis();
boolean usingInterrupt = false;
boolean running = true;
long delaySeconds = 1;
File logfile;

int trigger = 0;

void setup(){
  //Serial.begin(9600);
      
  initSensors();
  
  // SD card
  if (!SD.begin(chipSelect)) {
    //Serial.println("Card init. failed!");
    //while(1);
  }

  char filename[15];
  strcpy(filename, "GPSLOG00.TXT");
  for (uint8_t i = 0; i < 100; i++) {
    filename[6] = '0' + i/10;
    filename[7] = '0' + i%10;
    // create if does not exist, do not open existing, write, sync after write
    if (! SD.exists(filename)) {
      break;
    }
  }

  logfile = SD.open(filename, FILE_WRITE);
  if( ! logfile ) {
    //Serial.print("Couldn't create "); Serial.println(filename);
    //while(1);
  }
  
  // GPS setup
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PGCMD_ANTENNA);
  delay(1000);
  mySerial.println(PMTK_Q_RELEASE);

//  for(int i = 0; i < 100; i++){
    String h = "date,time,lat,latitude,lon,longitude,altitude,mph,fix quality,satellites,temp,roll,pitch,heading,trigger";
    logfile.println(h);
    logfile.flush();
//    //Serial.println(h);
//  }
}

void loop(){  
  char c = GPS.read();
  
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA())){
//      logfile.println("parse failed");
    }
  }

//  if(running || trigger != 0){
    if(readSensors(trigger)){
//      Serial.print("trigger = "); 
//      Serial.println(trigger);
    }
//  }
  
  trigger = 0; 
  
//  String h = "date,time,lat,latitude,lon,longitude,altitude,mph,fix quality,satellites,temp,roll,pitch,heading,trigger";
//  logfile.println(h);
//  logfile.flush();

  delay(1);
}

void initSensors(){
  if (!lsm.begin()){
//    Serial.println("lsm");//("Oops ... unable to initialize the LSM303. Check your wiring!");
    //while (1);
  }

  if (!bmp.begin()) {
//	Serial.println("bmp");//("Could not find a valid BMP085 sensor, check wiring!");
	//while (1) {}
  }
}

boolean readSensors(int trigger){
  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis()){
    timer = millis();
  }

  if ((millis() - timer) > (delaySeconds * 1000)){
    timer = millis(); // reset the timer
    
    lsm.read();
    float aX = lsm.accelData.x;
    float aY = lsm.accelData.y;
    float aZ = lsm.accelData.z;
    
    float mX = lsm.magData.x;
    float mY = lsm.magData.y;
    float mZ = lsm.magData.z;
    
    float roll;
    float pitch;
    float heading;
    
    getOrientation(aX, aY, aZ, mX, mY, mZ, roll, pitch, heading);
     
    float temperature = bmp.readTemperature();
    temperature = ((temperature * 9.0) / 5.0) + 32;
    
    // read the GPS
//    char c = GPS.read();
//    if (GPS.newNMEAreceived()) {
//      if (!GPS.parse(GPS.lastNMEA())){
////        Serial.println("parse failed");
//          logfile.println("parse failed");  
////        return false;
//      }
//    }
    
    float lat = GPS.latitude;
    if(GPS.lat == 'S'){
      lat *= -1; 
    }    
    
    float lon = GPS.longitude;
    if(GPS.lon == 'W'){
      lon *= -1; 
    }
    
    float mph = GPS.speed * 1.15078;
        
    logfile.print("20");
    logfile.print(GPS.year);
    logfile.print("-");
    logfile.print(GPS.month);
    logfile.print("-");
    logfile.print(GPS.day);
    logfile.print(",");
    logfile.print(GPS.hour);
    logfile.print(":");
    logfile.print(GPS.minute);
    logfile.print(":");
    logfile.print(GPS.seconds);
    logfile.print(",");
    logfile.print(GPS.lat);
    logfile.print(",");
    logfile.print(GPS.latitude);
    logfile.print(",");
    logfile.print(GPS.lon);
    logfile.print(",");
    logfile.print(GPS.longitude);
    logfile.print(",");
    logfile.print(GPS.altitude);
    logfile.print(",");
    logfile.print(mph);
    logfile.print(",");
    logfile.print(GPS.fixquality);
    logfile.print(",");
    logfile.print(GPS.satellites);
    logfile.print(",");
    logfile.print(temperature);
    logfile.print(",");
    logfile.print(roll);
    logfile.print(",");
    logfile.print(pitch);
    logfile.print(",");
    logfile.print(heading);
    logfile.print(",");
    logfile.print(trigger);
    
    logfile.println();
    logfile.flush();
    
//    Serial.print("20");
//    Serial.print(GPS.year);
//    Serial.print("-");
//    Serial.print(GPS.month);
//    Serial.print("-");
//    Serial.print(GPS.day);
//    Serial.print(",");
//    Serial.print(GPS.hour);
//    Serial.print(":");
//    Serial.print(GPS.minute);
//    Serial.print(":");
//    Serial.print(GPS.seconds);
//    Serial.print(",");
//    Serial.print(GPS.lat);
//    Serial.print(",");
//    Serial.print(GPS.latitude, DEC);
//    Serial.print(",");
//    Serial.print(GPS.lon);
//    Serial.print(",");
//    Serial.print(GPS.longitude, DEC);
//    Serial.print(",");
//    Serial.print(GPS.altitude, DEC);
//    Serial.print(",");
//    Serial.print(GPS.speed, DEC);
//    Serial.print(",");
//    Serial.print(GPS.fixquality, DEC);
//    Serial.print(",");
//    Serial.print(GPS.satellites, DEC);
//    Serial.print(",");
//    Serial.print(temperature);
//    Serial.print(",");
//    Serial.print(roll);
//    Serial.print(",");
//    Serial.print(pitch);
//    Serial.print(",");
//    Serial.print(heading);
//    Serial.print(",");
//    Serial.print(trigger, DEC);
    
    return true;
  }

  return false;
}

void getOrientation(float aX, float aY, float aZ, float mX, float mY, float mZ, float &roll, float &pitch, float &heading){
  float const PI_F = 3.14159265F;

  /* roll: Rotation around the X-axis. -180 <= roll <= 180                                          */
  /* a positive roll angle is defined to be a clockwise rotation about the positive X-axis          */
  /*                                                                                                */
  /*                    y                                                                           */
  /*      roll = atan2(---)                                                                         */
  /*                    z                                                                           */
  /*                                                                                                */
  /* where:  y, z are returned value from accelerometer sensor                                      */
  roll = (float)atan2(aY, aZ);

  /* pitch: Rotation around the Y-axis. -180 <= roll <= 180                                         */
  /* a positive pitch angle is defined to be a clockwise rotation about the positive Y-axis         */
  /*                                                                                                */
  /*                                 -x                                                             */
  /*      pitch = atan(-------------------------------)                                             */
  /*                    y * sin(roll) + z * cos(roll)                                               */
  /*                                                                                                */
  /* where:  x, y, z are returned value from accelerometer sensor                                   */
  if (aY * sin(roll) + aZ * cos(roll) == 0)
    pitch = aX > 0 ? (PI_F / 2) : (-PI_F / 2);
  else
    pitch = (float)atan(-aX / (aY * sin(roll) + aZ * cos(roll)));

  /* heading: Rotation around the Z-axis. -180 <= roll <= 180                                       */
  /* a positive heading angle is defined to be a clockwise rotation about the positive Z-axis       */
  /*                                                                                                */
  /*                                       z * sin(roll) - y * cos(roll)                            */
  /*   heading = atan2(--------------------------------------------------------------------------)  */
  /*                    x * cos(pitch) + y * sin(pitch) * sin(roll) + z * sin(pitch) * cos(roll))   */
  /*                                                                                                */
  /* where:  x, y, z are returned value from magnetometer sensor                                    */
  heading = (float)atan2(mZ * sin(roll) - mY * cos(roll), 
                         mX * cos(pitch) + mY * sin(pitch) * sin(roll) + mZ * sin(pitch) * cos(roll));


  /* Convert angular data to degree */
  roll = roll * 180 / PI_F;
  pitch = pitch * 180 / PI_F;
  heading = heading * 180 / PI_F;
}

