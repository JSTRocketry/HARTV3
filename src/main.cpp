#include "Arduino.h"
#include "MPU9250.h"
#include <Adafruit_BMP280.h>
#include "math.h"
#include "SD.h"

#define SD_ERROR_LED PC14
#define IMU_ERROR_LED PC15
#define BMP_ERROR_LED PB1
#define RADIO_SET_PIN PC13
#define LAUNCH_DETECTENCE_ACCELERATION_THRESHOLD 3
#define LAUNCH_DETECTENCE_ACCELERATION_DURATION_MILLIS 500
#define SD_BUFFER_SIZE 1024
#define ALTITUDE_PRE_LAUNCH_BUFFER_SIZE 100
#define SECOND_STAGE_IGNITION_DELAY 4000
#define FIRST_STAGE_LAUNCH_PIN PC15
#define SECOND_STAGE_LAUNCH_PIN PC15


//#define USE_GPS


enum fixQuan{
  GPS_FIX_INVALID, GPS_FIX, DGPS_FIX, PPS_Fix, Real_Time_Kinematic, Float_RTK, Estimated_Dead_Reckoning, manual_input_mode, simulation_mode
};

MPU9250 imu;
MPU9250_Data imuData;

Adafruit_BMP280 pressure;
File flightData;

struct GGA_GPS{
  long timeStamp;
  float latitude;
  float longitude;
  fixQuan fixQuality;
  int numberOfSatilites;
  double horizontalDilution;
  float altitude;
  float heightOfGeoid;
  long timeSinceUpdate;
  int DGPSid;
  int checkSumData;

};

float startPressure = 0; //pressure when the rocket is initialized
unsigned long systemTime; //millis for recording data
float currentAlt = 0; //current altitude of the rocket
long launchMillis = 0; //time at which the rocket launched
GGA_GPS location;



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
void sendDataThroughRadio(String what);
void writeToSD();
float DMStoDecimal(float dms);
int parseGPS(String line, GGA_GPS *gpsData);
void updateGPS();
void initGPS();
bool waitForRadioLaunch();
void recordFlightData();
bool getRadioLine(String *str);

struct altData{
  float altitude = 0;
  long timeStamp = 0;
};

altData altitudePreLaunchBuffer[ALTITUDE_PRE_LAUNCH_BUFFER_SIZE];
int altitudePreLaunchBufferIndex = 0;
altData tempAltitudeData;
void addToPreLaunchAltitudeBuffer(float altitude, long timeStamp){
  tempAltitudeData.altitude = altitude;
  tempAltitudeData.timeStamp = timeStamp;
  altitudePreLaunchBuffer[altitudePreLaunchBufferIndex] = tempAltitudeData;
  if(altitudePreLaunchBufferIndex + 1 < ALTITUDE_PRE_LAUNCH_BUFFER_SIZE) altitudePreLaunchBufferIndex ++;
  else altitudePreLaunchBufferIndex = 0;
}

void savePreLaunchBuffer(){
  for(int i = 0; i < ALTITUDE_PRE_LAUNCH_BUFFER_SIZE; i ++){
    int index = altitudePreLaunchBufferIndex + i % ALTITUDE_PRE_LAUNCH_BUFFER_SIZE;
    writeData("@{PA:" + String(altitudePreLaunchBuffer[index].altitude) + ";TS:" + String(altitudePreLaunchBuffer[index].timeStamp) + ";}@\n",false);
  }
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

void writeToSD(){
  while(true){
      sendDataThroughRadio("@{PA:" + String(currentAlt) + ";TS:" + String(systemTime) + ";}@\n");
      Serial.println("Sending Radio Alt");

  writeData("@{PA:" + String(currentAlt) + ";TS:" + String(systemTime) + ";}@\n",false);
  imu.getData(&imuData);
  writeData("@{OX:" + String(imuData.orientation.x) + ";OY:" + String(imuData.orientation.y) + ";OZ:" + String(imuData.orientation.z) + ";TS:" + String(systemTime) + ";}@\n",false);
  writeData("@{AX:" + String(imuData.accel.x) + ";AY:" + String(imuData.accel.y) + ";AZ:" + String(imuData.accel.z) + ";TS:" + String(systemTime) + ";}@\n",false);

}
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



/*
  waitForLaunch

  waits until a launch is detected by monitoring the acceleration of the rocket
*/

void waitForLaunch(){
  imu.getData(&imuData);
  float normalMag = sqrt(pow(imuData.accel.x,2) + pow(imuData.accel.y,2) + pow(imuData.accel.z,2));
  Serial.println("Accel: " + String(normalMag));
  //for(int i = 0; i < ALTITUDE_PRE_LAUNCH_BUFFER_SIZE; i ++){
  //  addToPreLaunchAltitudeBuffer(pressure.readAltitude(startPressure), millis());
  //}
  while(fabs(1.0 - normalMag) <= LAUNCH_DETECTENCE_ACCELERATION_THRESHOLD){
    Serial.println("Accel: " + String(normalMag));
    imu.getData(&imuData);
    normalMag = sqrt(pow(imuData.accel.x,2) + pow(imuData.accel.y,2) + pow(imuData.accel.z,2));
    addToPreLaunchAltitudeBuffer(pressure.readAltitude(startPressure), millis());
  }
  //savePreLaunchBuffer();
  reportLaunch();
}

void launchFirstStage(){
  pinMode(FIRST_STAGE_LAUNCH_PIN, OUTPUT);
  digitalWrite(FIRST_STAGE_LAUNCH_PIN, HIGH);
  long start = millis();
  while(millis() - start < 1000){
    recordFlightData();
    delay(10);
  }
}

bool waitForMECO(){
  long start = millis();
  while(millis() - start < SECOND_STAGE_IGNITION_DELAY){
    recordFlightData();
    delay(10);
  }
  if(pressure.readAltitude(startPressure) > 25){
    return true;
  }
  return false;
}

void launchSecondStage(){
  pinMode(SECOND_STAGE_LAUNCH_PIN, OUTPUT);
  digitalWrite(SECOND_STAGE_LAUNCH_PIN, HIGH);
  long start = millis();
  while(millis() - start < 1000){
    recordFlightData();
    delay(10);
  }
}


void syncRadio(){
  String msg = "";
  while(msg.indexOf("ECHO") == -1){
    while(!getRadioLine(&msg)){
      sendDataThroughRadio("PING!");
      Serial.println("Sending Radio Ping");
      delay(1000);
    }
    Serial.println("Recieved from radio: " + msg);
  }

}

void initPins(){
  pinMode(RADIO_SET_PIN,OUTPUT);
  digitalWrite(RADIO_SET_PIN,HIGH);
  pinMode(IMU_ERROR_LED, OUTPUT);
  pinMode(BMP_ERROR_LED, OUTPUT);
  pinMode(SD_ERROR_LED, OUTPUT);
  digitalWrite(IMU_ERROR_LED, LOW);
  digitalWrite(BMP_ERROR_LED,LOW);
  digitalWrite(SD_ERROR_LED, LOW);
}

void initSensors(){
  if(!initIMU()){
    digitalWrite(IMU_ERROR_LED, HIGH);
    while(true){
        Serial.println("IMU INIT FAILED! STOPPING!");
        delay(20);
    }
  }
  Serial.println("IMU init Sucess");
  if(!initBMP()){
      digitalWrite(BMP_ERROR_LED,HIGH);
      while(true){
        Serial.println("BMP FAILURE!");
        delay(20);
      }
  }
  Serial.println("BMP init Sucess");

}

void openFlightDataFile(){
  if(!SD.begin(SPI_HALF_SPEED,PA4)){
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
  //if(flightData){
  //  Serial.println("Opened!");
  //}
  if(!flightData){
    while(true){
      Serial.println("Failed to open!" + fileName + "n");
      delay(20);
    }
  }
}

void setup() {
    Serial.begin(115200);
    Serial2.begin(9600);
    Serial3.begin(4800);
    initPins();
    delay(3000);
    Serial.println("Starting IMU");
    initSensors();
    //openFlightDataFile();
    #ifdef USE_GPS
      initGPS();
    #endif
    syncRadio();
    startPressure = pressure.readPressure()/100.0;
    Serial.println("ready for launch!");
    systemTime = millis();
    if(!waitForRadioLaunch()){
        while(true){
        }
    }
    launchFirstStage();
    waitForMECO();
    launchSecondStage();

    /*
    recordLaunch();
    long airTime = millis();
    while (!launchSecondStage(airTime)){
      airTime = millis();
    }
    if (launchSecondStage(airTime)){
      digitalWrite(launchPin, HIGH);
    }
    */
}


String radioBuffer;
char tempRadioChar;

bool getRadioLine(String *str){
  if(Serial3.available()){
    while(Serial3.available()){
      tempRadioChar = (char)Serial3.read();
      //Serial.print(temp);
      if(tempRadioChar == '\n' || tempRadioChar == '\r'){
        *str = radioBuffer;
        radioBuffer = "";
        return true;
      }
      else radioBuffer += tempRadioChar;
    }
  }
  return false;
}

void turnOnBuzzer(){

}

void turnOffBuzzer(){

}

bool waitForRadioLaunch(){
  bool hasRecievedOkay = false;
  String radioMessage;
  while(!hasRecievedOkay){
    if(getRadioLine(&radioMessage)){
      if(radioMessage.indexOf("ARM") != -1){
        turnOnBuzzer();
        sendDataThroughRadio("ARM sequence start");
        delay(5000);
        while(true){
          if(getRadioLine(&radioMessage)){
            if(radioMessage.indexOf("LAUNCH") != -1){
              return true;
            }
            else if(radioMessage.indexOf("OFF") != -1){
              return false;
            }
          }
        }
      }
      if(radioMessage.indexOf("STAND DOWN") != -1){
        return false;
      }
    }
  }
}

/*
  sendDataThroughRadio

  Sends the data through Serial2 which the radio is connected to on

*/

void sendDataThroughRadio(String what){
  Serial3.println(what);
}

long lastAltSend = 0;
long altRadioDelay = 150;
void recordFlightData(){
  systemTime = millis() - launchMillis;
  currentAlt = pressure.readAltitude(startPressure);
  updateGPS();
  if((long)millis() - lastAltSend > altRadioDelay){
    sendDataThroughRadio("@{PA:" + String(currentAlt) + ";TS:" + String(systemTime) + ";}@\n");
    Serial.println("Sending Radio Alt");
    lastAltSend = millis();
    initGPS();
    sendDataThroughRadio("Latitude: " + String(location.latitude));
    sendDataThroughRadio("Longitude: " + String(location.longitude));
  }
  writeData("@{PA:" + String(currentAlt) + ";TS:" + String(systemTime) + ";}@\n",false);
  imu.getData(&imuData);
  writeData("@{OX:" + String(imuData.orientation.x) + ";OY:" + String(imuData.orientation.y) + ";OZ:" + String(imuData.orientation.z) + ";TS:" + String(systemTime) + ";}@\n",false);
  writeData("@{AX:" + String(imuData.accel.x) + ";AY:" + String(imuData.accel.y) + ";AZ:" + String(imuData.accel.z) + ";TS:" + String(systemTime) + ";}@\n",false);
  writeData("Timestamp: " + String(location.timeStamp), false);
  writeData("Latitude: " + String(location.latitude), false);
  writeData("Longitude: " + String(location.longitude), false);
  //Serial.println("Looping@");
}

/*
  recordLaunch

  records flight data of the craft
*/

void recordLaunch(){
  String gpsLine;
  GGA_GPS location;
  waitForLaunch();
  launchMillis = millis();
  lastAltSend = millis();
  while(true){
    systemTime = millis() - launchMillis;
    currentAlt = pressure.readAltitude(startPressure);
    if((long)millis() - lastAltSend > altRadioDelay){
      sendDataThroughRadio("@{PA:" + String(currentAlt) + ";TS:" + String(systemTime) + ";}@\n");
      Serial.println("Sending Radio Alt");
      lastAltSend = millis();
      //initGPS();
      #ifdef USE_GPS
        sendDataThroughRadio("Latitude: " + String(location.latitude));
        sendDataThroughRadio("Longitude: " + String(location.longitude));
      #endif
    }

    writeData("@{PA:" + String(currentAlt) + ";TS:" + String(systemTime) + ";}@\n",false);
    imu.getData(&imuData);
    writeData("@{OX:" + String(imuData.orientation.x) + ";OY:" + String(imuData.orientation.y) + ";OZ:" + String(imuData.orientation.z) + ";TS:" + String(systemTime) + ";}@\n",false);
    writeData("@{AX:" + String(imuData.accel.x) + ";AY:" + String(imuData.accel.y) + ";AZ:" + String(imuData.accel.z) + ";TS:" + String(systemTime) + ";}@\n",false);
    //writeData("Timestamp: " + String(location.timeStamp), false);
    #ifdef USE_GPS
      updateGPS();
      writeData("Latitude: " + String(location.latitude), false);
      writeData("Longitude: " + String(location.longitude), false);
    #endif
    //Serial.println("Looping@");
    if(Serial.available()){
      char c = (char)Serial.read();
      if(c == 'p'){
        digitalWrite(PB0, LOW);
      }
      else if(c == 'n'){
        digitalWrite(PB0,HIGH);
      }
      else{
          Serial3.write(c);
      }

    }
    if(Serial3.available()){
      Serial.print((char)Serial3.read());
    }
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

  int addon;
  int sumon;


  /*
  DMStoDecimal()

  returns gps coordinates in deciaml form. a positive latitude indicates north
  while a negative latitude indicates south. a positive longitude indicates east
  while a negative longitude indicates west.

  */
  float DMStoDecimal(float dms){
    char brokenDMS[6];
    String(dms).toCharArray(brokenDMS, 6);
    float degree = atof((String(brokenDMS[0]) + String(brokenDMS[1])).c_str());
    float minutes = atof((String(brokenDMS[2]) + String(brokenDMS[3])).c_str());
    float secconds = atof((String(brokenDMS[4]) + String(brokenDMS[5]) + String(brokenDMS[6])).c_str());
    float decimal = degree + (minutes/60) + (secconds/3600);
    return decimal;
  }


  /*
  pareGPS()

  stores the decimal value of latitude and longitude into the GGA_GPS struct named gpsData.

  */
  int parseGPS(String line, GGA_GPS *gpsData){
    if (line.indexOf("$GPGGA") == -1){
      return -1;
    }
    int commaCount[13];
    commaCount[0] = line.indexOf(",");
    for (int i = 1; i < 13; i++){
      commaCount[i] = line.indexOf(",", commaCount[i-1]+1);
      if(commaCount[i] < 0 || commaCount[i] >= (int)line.length()){
        Serial.println("Something wrong with gps parsing");
        return -1;
      }
    }
    if (strcmp(line.substring(commaCount[2] + 1, commaCount[3]).c_str(),  String("N").c_str()) == 0){
      addon = 1;
    }
    else{
      addon = -1;
    }
    if (strcmp(line.substring(commaCount[4] + 1, commaCount[5]).c_str(), String("E").c_str()) == 0){
      sumon = 1;
    }
    else{
      sumon = -1;
    }
    gpsData->timeStamp = atol(line.substring((commaCount[0] + 1), commaCount[1]).c_str());
    gpsData->latitude = DMStoDecimal((atof(line.substring((commaCount[1] + 1), commaCount[2]).c_str()))) * addon;
    gpsData->longitude = DMStoDecimal((atof(line.substring((commaCount[3] + 1), commaCount[4]).c_str()))) * sumon;
    gpsData->altitude = atof((line.substring(commaCount[8] + 1, commaCount[9])).c_str());
    gpsData->horizontalDilution = atof((line.substring(commaCount[7] + 1, commaCount[8])).c_str());
    switch(atoi((line.substring(commaCount[5] + 1, commaCount[6])).c_str())){
      case 0:
        gpsData->fixQuality = GPS_FIX_INVALID;
        break;
      case 1:
        gpsData->fixQuality = GPS_FIX;
        break;
      case 2:
        gpsData->fixQuality = DGPS_FIX;
        break;
      case 3:
        gpsData->fixQuality = PPS_Fix;
        break;
      case 4:
        gpsData->fixQuality = Real_Time_Kinematic;
        break;
      case 5:
        gpsData->fixQuality = Float_RTK;
        break;
      case 6:
        gpsData->fixQuality = Estimated_Dead_Reckoning;
        break;
      case 7:
        gpsData->fixQuality = manual_input_mode;
        break;
      case 8:
        gpsData->fixQuality = simulation_mode;
        break;
    }
    gpsData->numberOfSatilites = atoi((line.substring(commaCount[6] + 1, commaCount[7])).c_str());
    gpsData->heightOfGeoid = atof((line.substring(commaCount[10] + 1, commaCount[11])).c_str());
    return 0;

  }

  String currentBuffer;
  char temp;

  bool getLine(String *str){
    if(Serial2.available()){
      while(Serial2.available()){
        temp = (char)Serial2.read();
        //Serial.print(temp);
        if(temp == '\n' || temp == '\r'){
          *str = currentBuffer;
          currentBuffer = "";
          return true;
        }
        else currentBuffer += temp;
      }
    }
    return false;
  }

  String gpsLine;
  void updateGPS(){
    if(getLine(&gpsLine)){
      //Serial.println("New GPS Data! " + gpsLine);
      if(gpsLine.indexOf("$GPGGA") != -1){
        Serial.println("New GPS Data! " + gpsLine);
        parseGPS(gpsLine, &location);
      }
    }
  }

  void initGPS() {
      location.fixQuality = GPS_FIX_INVALID;
      while(location.fixQuality != GPS_FIX){
        updateGPS();
      }
  }
