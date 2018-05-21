#include "Arduino.h"
#include "MPU9250.h"
#include <Adafruit_BMP280.h>
#include "math.h"
#include "SD.h"

#define SD_ERROR_LED PC13
#define IMU_ERROR_LED PC14
#define BMP_ERROR_LED PC15
#define LAUNCH_DETECTENCE_ACCELERATION_THRESHOLD 3
#define LAUNCH_DETECTENCE_ACCELERATION_DURATION_MILLIS 500
#define SD_BUFFER_SIZE 1024
#define ALTITUDE_PRE_LAUNCH_BUFFER_SIZE 100

MPU9250 imu;
MPU9250_Data imuData;

Adafruit_BMP280 pressure;
File flightData;


float startPressure = 0; //pressure when the rocket is initialized
unsigned long systemTime; //millis for recording data
float currentAlt = 0; //current altitude of the rocket
long launchMillis = 0; //time at which the rocket launched




// function prototype
void addToPreLaunchAltitudeBuffer(float altitude);
String getFileName();
void writeData(String toWrite, bool force);
bool initIMU();
bool initBMP();
void reportLaunch();
void recordLaunch();
float getAccelMagnitude();
void waitForLaunchOverDuration();

float altitudePreLaunchBuffer[ALTITUDE_PRE_LAUNCH_BUFFER_SIZE];
int altitudePreLaunchBufferIndex = 0;

void addToPreLaunchAltitudeBuffer(float altitude){
  altitudePreLaunchBuffer[altitudePreLaunchBufferIndex] = altitude;
  if(altitudePreLaunchBufferIndex + 1 < ALTITUDE_PRE_LAUNCH_BUFFER_SIZE) altitudePreLaunchBufferIndex ++;
  else altitudePreLaunchBufferIndex = 0;
}




/*
  getFileName()

  returns the next available file name on the SD card following "fdat(number).txt"

  @return String: file name that's available on the SD card, "" if non are avialble
*/


String getFileName(){
  String base = "fdat";
  int counter = 0;
  while(counter < 10){
    if(!SD.exists(base + String(counter) + ".txt")){
      return base + String(counter) + ".txt";
    }
    else counter ++;
  }
  return "";
}

char writeBuff[SD_BUFFER_SIZE] = {};
int buffIndex = 0;

void writeData(String toWrite, bool force){
  if(buffIndex + toWrite.length() + 1 >= SD_BUFFER_SIZE || force){
    flightData.write(writeBuff,buffIndex);
    flightData.flush();
    for(int i = 0; i < buffIndex; i ++){
      writeBuff[i] = '\0';
    }
    buffIndex = 0;
  }
  for(uint i = 0; i < toWrite.length(); i ++){
    writeBuff[buffIndex] = toWrite.charAt(i);
    buffIndex ++;
  }
}


/*
  initIMU

  Initializes and calibrates the IMU and returns false if the IMU fails

  @return bool: A true or false dependent upon the success of the IMU initializing
*/

bool initIMU(){
  if(imu.begin(MPU9250_GYRO_RANGE_2000_DPS, MPU9250_ACCEL_RANGE_16_GPS) < 0){
    return false;
  }
  imu.calibrateGyroOffsets();
  imu.zero();
  imu.calibrateAccel();
  imu.setMagnetometerCalibrationOffsets(-36.35, 40.36, -140.07);
  imu.setMagnetometerCalibrationScales(1.0, 1.03, .92);
  imu.setDataFuseMode(MPU9250_DATA_FUSE_GYRO_MAG_AUTO_ACCEL);
  return true;
}

/*
  initBMP

  Initializes the BMP and returns false if the BMP fails

  @return bool: A true or false dependent upon the success of the BMP initializing
*/

bool initBMP(){
  if (pressure.begin())
  {
    return true;
  }
  return false;
}

void reportLaunch(){
  digitalWrite(SD_ERROR_LED, HIGH);
  digitalWrite(BMP_ERROR_LED, HIGH);
  digitalWrite(IMU_ERROR_LED, HIGH);
}

void recordPreLaunchBuffer(){
  for(int i = 0; i < ALTITUDE_PRE_LAUNCH_BUFFER_SIZE; i ++){
    writeData("@{PA:" + String(altitudePreLaunchBuffer[(altitudePreLaunchBufferIndex + i) % ALTITUDE_PRE_LAUNCH_BUFFER_SIZE]) + ";TS:" + String(systemTime) + ";}@\n",false);
  }
}

/*
  waitForLaunch

  waits until a launch is detected by monitoring the acceleration of the rocket
*/

void waitForLaunch(){
  imu.getData(&imuData);
  float normalMag = sqrt(pow(imuData.accel.x,2) + pow(imuData.accel.y,2) + pow(imuData.accel.z,2));
  Serial.println("Accel: " + String(normalMag));
  while(fabs(1.0 - normalMag) <= LAUNCH_DETECTENCE_ACCELERATION_THRESHOLD){
    Serial.println("Accel: " + String(normalMag));
    imu.getData(&imuData);
    normalMag = sqrt(pow(imuData.accel.x,2) + pow(imuData.accel.y,2) + pow(imuData.accel.z,2));
  }
  reportLaunch();
}

void setup() {
    delay(5000);
    Serial.begin(115200);
    Serial2.begin(9600);
    pinMode(IMU_ERROR_LED, OUTPUT);
    pinMode(BMP_ERROR_LED, OUTPUT);
    pinMode(SD_ERROR_LED, OUTPUT);
    digitalWrite(IMU_ERROR_LED, LOW);
    digitalWrite(BMP_ERROR_LED,LOW);
    digitalWrite(SD_ERROR_LED, LOW);
    if(!initIMU()){
      digitalWrite(IMU_ERROR_LED, HIGH);
      while(true){
          Serial.println("IMU INIT FAILED! STOPPING!");
          delay(20);

      }
    }
    Serial.println("IMU INIT SUCCESS!");    if(!initBMP()){
        digitalWrite(BMP_ERROR_LED,HIGH);
        while(true){
          Serial.println("BMP FAILURE!");
          delay(20);
        }
    }
    if(!SD.begin(SPI_FULL_SPEED,PA4)){
      digitalWrite(SD_ERROR_LED, HIGH);
      while(true){
        Serial.println("SD Begin FAILED!");
        delay(20);
      }
    }
    String fileName = getFileName();
    if(fileName.equals("")){
      digitalWrite(SD_ERROR_LED, HIGH);
      digitalWrite(IMU_ERROR_LED, HIGH);
      while(true){
        Serial.println("SD Card FULL!");

        delay(20);
      }
    }
    flightData = SD.open(fileName,O_CREAT | O_WRITE);
    delay(10);
    if(flightData){
      Serial.println("Opened!");
    }
    else{
      while(true){
        Serial.println("Failed to open!" + fileName + "n");
        delay(20);
      }
    }
    startPressure = pressure.readPressure()/100.0;
    systemTime = millis();
    recordLaunch();
}

/*
  sendDataThroughRadio

  Sends the data through Serial2 which the radio is connected to on

*/

void sendDataThroughRadio(String what){
  Serial2.println(what);
}

/*
  recordLaunch

  records flight data of the craft
*/

void recordLaunch(){
  waitForLaunch();
  launchMillis = millis();
  while(true){
    systemTime = millis() - launchMillis;
    currentAlt = pressure.readAltitude(startPressure);
    writeData("@{PA:" + String(currentAlt) + ";TS:" + String(systemTime) + ";}@\n",false);
    sendDataThroughRadio("@{PA:" + String(currentAlt) + ";TS:" + String(systemTime) + ";}@\n");
    imu.getData(&imuData);
    writeData("@{OX:" + String(imuData.orientation.x) + ";OY:" + String(imuData.orientation.y) + ";OZ:" + String(imuData.orientation.z) + ";TS:" + String(systemTime) + ";}@\n",false);
    writeData("@{AX:" + String(imuData.accel.x) + ";AY:" + String(imuData.accel.y) + ";AZ:" + String(imuData.accel.z) + ";TS:" + String(systemTime) + ";}@\n",false);

  }
}

void loop() {
}


/*
  getAccelMagnitude

  returns the total acceleration of the rocket

  @return float: the total acceleration of the rocket
*/

float getAccelMagnitude(){
  imu.getData(&imuData);
  return sqrt(pow(imuData.accel.x,2) + pow(imuData.accel.y,2) + pow(imuData.accel.z,2));
}

/*
  waitForLaunchOverDuration

  Waits for a launch as defined as crossing an acceleration threshold and a time under acceleration

*/

void waitForLaunchOverDuration(){
  while(true){
    while(fabs(1-getAccelMagnitude()) < LAUNCH_DETECTENCE_ACCELERATION_THRESHOLD);
    long timeBegin = millis();
    while(fabs(1-getAccelMagnitude()) >= LAUNCH_DETECTENCE_ACCELERATION_THRESHOLD){
      if (millis() - timeBegin > LAUNCH_DETECTENCE_ACCELERATION_DURATION_MILLIS){
        return;
      }
    }
  }
}
