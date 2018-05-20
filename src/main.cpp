#include "Arduino.h"
#include "MPU9250.h"
#include "BMP180.h"
#include "math.h"
#include "SD.h"

#define SD_ERROR_LED PC13
#define IMU_ERROR_LED PC14
#define BMP_ERROR_LED PC15
#define THRESHOLD 3
#define THRESHOLD_DELAY 500

//double T, P, p0, a;
unsigned long timeStamp;

#define SD_BUFFER_SIZE 1024

File flightData;
MPU9250 imu;
BMP180 pressure;

String fileName;

MPU9250_Data imuData;
float currentAlt = 0;
long timeStart = 0;


double tempPressure = 0;
bool isNewPressure = false;
long launchMillis = 0;
#define ALTITUDE_PRE_LAUNCH_BUFFER_SIZE 100
float altitudePreLaunchBuffer[ALTITUDE_PRE_LAUNCH_BUFFER_SIZE];
int altitudePreLaunchBufferIndex = 0;
void addToPreLaunchAltitudeBuffer(float altitude){
  altitudePreLaunchBuffer[altitudePreLaunchBufferIndex] = altitude;
  if(altitudePreLaunchBufferIndex + 1 < ALTITUDE_PRE_LAUNCH_BUFFER_SIZE) altitudePreLaunchBufferIndex ++;
  else altitudePreLaunchBufferIndex = 0;
}


int SD_FAILURE =        0b00000001;
int IMU_FAILURE =       0b00000010;
int BMP_FAILURE =       0b00000100;
int SD_NO_SPACE =       0b00001000;
int LAUNCH_DETECTED =   0b00010000;

int currentErrors = 0;

void addError(int what){
  currentErrors |= what;
}

void handleErrors(){
  if(currentErrors &= (0b11111111 | SD_FAILURE)){
    Serial.println("SD_FAILURE!");
  }
}

void removeError(int what){
  currentErrors &= what;
}





void recordLaunch();


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
    //Serial.println("Buff Size: " + String(buffIndex));
    for(int i = 0; i < buffIndex; i ++){
      writeBuff[i] = '\0';
    }
    buffIndex = 0;
    //if(!force) writeData("Buffer wrote!\n", false);
    //Serial.println("Writing buff!");
  }
  for(uint i = 0; i < toWrite.length(); i ++){
    writeBuff[buffIndex] = toWrite.charAt(i);
    buffIndex ++;
  }
}

bool initIMU(){
  if(imu.begin(MPU9250_GYRO_RANGE_2000_DPS, MPU9250_ACCEL_RANGE_16_GPS) < 0){
    //Serial.println("IMU init Fail!");
    //digitalWrite(IMUFail, HIGH);
    return false;
  }
  //Serial.println("IMU init Success!");
  //Serial.println("Calibrating Gyroscope!");
  imu.calibrateGyroOffsets();
  imu.zero();
  imu.calibrateAccel();
  imu.setMagnetometerCalibrationOffsets(-36.35, 40.36, -140.07);
  imu.setMagnetometerCalibrationScales(1.0, 1.03, .92);
  imu.setDataFuseMode(MPU9250_DATA_FUSE_GYRO_MAG_AUTO_ACCEL);
  //imu.calibrateMagnetometer();
  return true;
}

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
    writeData("@{PA:" + String(altitudePreLaunchBuffer[(altitudePreLaunchBufferIndex + i) % ALTITUDE_PRE_LAUNCH_BUFFER_SIZE]) + ";TS:" + String(timeStamp) + ";}@\n",false);
  }
}

void waitForLaunch(){
  imu.getData(&imuData);
  float normalMag = sqrt(pow(imuData.accel.x,2) + pow(imuData.accel.y,2) + pow(imuData.accel.z,2));
  Serial.println("Accel: " + String(normalMag));
  while(fabs(1.0 - normalMag) <= THRESHOLD){
    Serial.println("Accel: " + String(normalMag));
    imu.getData(&imuData);
    normalMag = sqrt(pow(imuData.accel.x,2) + pow(imuData.accel.y,2) + pow(imuData.accel.z,2));
  }
  reportLaunch();
}

void setup() {
    // put your setup code here, to run once:
    delay(5000);
    Serial.begin(115200);
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
    fileName = getFileName();
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
    timeStamp = millis();
    recordLaunch();
}

//MPU9250_Raw_Data imuData;

void recordLaunch(){
  waitForLaunch();
  launchMillis = millis();
  while(true){
    timeStamp = millis() - launchMillis;
    isNewPressure = pressure.getPressureAsync(&tempPressure);
    //Serial.println("Pressure Delay: " + String(millis() - launchMillis - timeStamp));
    if(isNewPressure){
      currentAlt = pressure.altitude(tempPressure);
      writeData("@{PA:" + String(currentAlt) + ";TS:" + String(timeStamp) + ";}@\n",false);
    }
    imu.getData(&imuData);
    //flightData.println("High");
    writeData("@{OX:" + String(imuData.orientation.x) + ";OY:" + String(imuData.orientation.y) + ";OZ:" + String(imuData.orientation.z) + ";TS:" + String(timeStamp) + ";}@\n",false);

    writeData("@{AX:" + String(imuData.accel.x) + ";AY:" + String(imuData.accel.y) + ";AZ:" + String(imuData.accel.z) + ";TS:" + String(timeStamp) + ";}@\n",false);
  }
}
void loop() {
}


float getAccelMagnitude(){
  imu.getData(&imuData);
  return sqrt(pow(imuData.accel.x,2) + pow(imuData.accel.y,2) + pow(imuData.accel.z,2));
}

void waitForLaunchOverDuration(){
  while(true){
    while(fabs(1-getAccelMagnitude()) < THRESHOLD);
    long timeBegin = millis();
    while(fabs(1-getAccelMagnitude()) >= THRESHOLD){
      if (millis() - timeBegin > THRESHOLD_DELAY){
        return;
      }
    }
  }
}
