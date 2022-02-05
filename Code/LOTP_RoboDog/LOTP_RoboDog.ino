#include <SoftwareSerial.h>
#include <MPU6050_light.h>
#include <ArduinoJson.h>
#include <TinyGPS++.h>
#include <nRF24L01.h>
#include <Servo.h>
#include <RF24.h>
#include <math.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>

int Leg1 = 50;
int Leg2 = 125;
int Leg3 = 125;

SoftwareSerial ss(0, 1); //rx, tx
RF24 radio(7, 8); // CE, CSN
MPU6050 mpu(Wire);
TinyGPSPlus gps;
Servo FLLeg[3];
Servo FRLeg[3];
Servo RLLeg[3];
Servo RRLeg[3];

const uint64_t tAddress = 0xE8E8F0F0E0LL;
const uint64_t rAddress = 0xE8E8F0F0E1LL;

int joystick[6] = {512, 512, 512, 512, 512, 512};
int oldJoystick[6] = {512, 512, 512, 512, 512, 512};
float gyro[3] = {0, 0, 0};
int maxForce[4] = {800, 900, 900, 300};
float FLLegPress;
float FRLegPress;
float RLLegPress;
float RRLegPress;
double gyroLast; 
double limitMove[7];
float Ag[3] = {0, 0, 0};
int curser = 6;
int moveSideA = 0; 
double Sz;
double disLimit[4] = {700, 700, 700, 700};
double LidarForce[4];

int Yin;
int Xin;
int Zin;

double Ax;
double Ay;
double Az;
    
char oldStand;

int LegtoOrg[3] = {150,95,0};

int FLLegPins[3] = {6,25,26};
int FRLegPins[3] = {5,24,4};
int RLLegPins[3] = {31,29,32};
int RRLegPins[3] = {30,28,27};

float FLLegPos[3] = {0,100,0};
float FRLegPos[3] = {0,100,0};
float RLLegPos[3] = {0,100,0};
float RRLegPos[3] = {0,100,0};

//float FLLegPos[3] = {125,125,0};
//float FRLegPos[3] = {125,125,0};
//float RLLegPos[3] = {125,125,0};
//float RRLegPos[3] = {125,125,0};

float FLLegAdd[3] = {0,10,0};
float FRLegAdd[3] = {5,0,0};
float RLLegAdd[3] = {5,15,0};
float RRLegAdd[3] = {0,0,10};

union{
  byte array[4];
  float bigNum;  
} H2;

union{
  byte array[4];
  float bigNum;  
} LPG;

union{
  byte array[4];
  float bigNum;  
} CH4;

union{
  byte array[4];
  float bigNum;  
} CO;

union{
  byte array[4];
  float bigNum;  
} Alchol;

union{
  byte array[4];
  float bigNum;  
} xForce;

union{
  byte array[4];
  float bigNum;  
} xnForce;

union{
  byte array[4];
  float bigNum;  
} yForce;

union{
  byte array[4];
  float bigNum;  
} ynForce;

struct Package{
  int16_t joystick[6];
  bool gpsRequest;
  bool gasRequest;

  bool standMode;
  bool walkMode;

  bool Gps;
  bool Lidar;
  bool Drone;
  bool Gas;

  bool restoreConfig;

  byte joystickSmoothness;
  byte path;
  byte stepH;
  bool gyroAssist;
  bool forceAssist;
  bool WalkMode;
  
  byte lidarSwitch;
  byte droneSwitch;
};

struct GPSPackage{
  byte staller;

  byte hour;
  byte minute;

  int16_t year;
  byte month;
  byte day;

  float longti;
  float lati;
};

struct GasPackage{
  float H2;
  float LPG;
  float CH4;
  float CO;
  float Alchol;
};

struct Config {
  byte joystickSmoothness;
  byte path;
  byte stepH;
  byte gyroAssist;
  byte forceAssist;
  byte lidarSwitch;
  byte droneSwitch;
  bool WalkMode;
};


const char *filename = "/config.txt";  // <- SD library uses 8.3 filenames
Config config;

Package package;
Package oldPackage;

GPSPackage gpsPackage;
GasPackage gasPackage;

int DroneAdress = 3;
int GasAdress = 5;
int LidarAdress = 9;
double lidarTime = 0;
double gasTime = 0;

// |h|h|s|p|p|p|
// |p|p|p|h|h|s|

float hoverTime = 2;
float stepDelay = 1;
float pushTime = hoverTime + stepDelay; 

int maxStepX=  150;
int maxStepZ = 100;
int stepOffset = 0;
int tilt = 10;

float sTime[4];

float W2s[4] = {0, pushTime, pushTime, 0};
float W4s[4] = {0, pushTime, 2 * pushTime, 3 * pushTime};

void setup() {
  Serial.begin(115200);
  ss.begin(9600);
  Wire.begin();
  Wire1.begin();

  //while(!Serial){}
  
  while (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println(F("Failed to initialize SD library"));
    delay(1000);
  }

  mpu.begin();
  Serial.println("MPU6050 Found!");
  
  loadConfiguration(filename, config);

  for(int i = 0; i < 4; i++){
    sTime[i] = W2s[i];
  }

  for(int i = 0; i < 4; i++){
    sTime[i] *= config.path;
  }
  
  for(int i = 0; i < 3; i++){
    FLLeg[i].attach(FLLegPins[i]); 
    FLLeg[i].write(90 + FLLegAdd[i]);
  }
  for(int i = 0; i < 3; i++){
    FRLeg[i].attach(FRLegPins[i]);
    FRLeg[i].write(90 + FRLegAdd[i]);
  }
  for(int i = 0; i < 3; i++){
    RLLeg[i].attach(RLLegPins[i]);
    RLLeg[i].write(90 + RLLegAdd[i]);
  }
  for(int i = 0; i < 3; i++){
    RRLeg[i].attach(RRLegPins[i]);
    RRLeg[i].write(90 + RRLegAdd[i]);
  }

  radio.begin();
  //radio.setPackageRate( RF24_250KBPS );
  radio.openWritingPipe(tAddress);
  radio.openReadingPipe(1,rAddress);
  //radio.setPALevel(RF24_PA_MIN);

  moveLegsPositions();

  //restoreConfig();
  delay(2500);
  mpu.calcOffsets();
  
  radio.startListening();
  delay(1);
}

void loop() {
  //delay(25);

  int oldDrone = package.Drone;
  int oldLidar = package.Lidar;
  int oldGas = package.Gas;
  getPackage();

  //Serial.println(package.WalkMode);
  
  getJoystick();
  
  mpu.update();
  getGyro();

  if(oldLidar != package.Lidar){
    if(package.Lidar){
      Wire1.beginTransmission(LidarAdress);
      Wire1.write(1);
      Wire1.endTransmission();
    }
    else{
      Wire1.beginTransmission(LidarAdress);
      Wire1.write(0);
      Wire1.endTransmission();
    }
  }
  if(package.Lidar and millis() - lidarTime > 500){
    lidarTime = millis();
    getLidar();
  }

  if(oldDrone != package.Drone){
    if(package.Drone){
      Wire1.beginTransmission(DroneAdress);
      Wire1.write(1);
      Wire1.endTransmission();
    }
    else{
      Wire1.beginTransmission(DroneAdress);
      Wire1.write(0);
      Wire1.endTransmission();
    }
  }

  if(oldGas != package.Gas){
    if(package.Gas){
      Wire1.beginTransmission(GasAdress);
      Wire1.write(1);
      Wire1.endTransmission();
    }
    else{
      Wire1.beginTransmission(GasAdress);
      Wire1.write(0);
      Wire1.endTransmission();
    }
  }
  if(package.Gas and millis() - gasTime > 500){
    gasTime = millis();
    getGas();
  }
  if(package.gasRequest){
    delay(1);
    radio.stopListening();
    radio.write(&gasPackage, sizeof(gasPackage));
    //Serial.println("Sended");
    radio.startListening();
    delay(1);
  }
  
  if(package.Gps){
    while(ss.available() > 0){
      if(gps.encode(ss.read())){
        if(gps.location.isValid()){
          //Serial.println(gps.location.lat(),6);
          gpsPackage.lati = gps.location.lat();
          gpsPackage.longti = gps.location.lng();
        }

        if(gps.time.isValid()){
          gpsPackage.hour = gps.time.hour();
          gpsPackage.minute = gps.time.minute();
        }

        if(gps.date.isValid()){
          gpsPackage.year = gps.date.year();
          gpsPackage.month = gps.date.month();
          gpsPackage.day = gps.date.day();
        }

        if(gps.satellites.isValid()){
          //Serial.println(gps.satellites.value());
          gpsPackage.staller = gps.satellites.value();
        }
        
        //for(int i = 0; i < 29; i++){
        //  Serial.print(GPSPackage[i]);
        //}
        //Serial.println("GPS");

      if(package.gpsRequest){
        delay(1);
        radio.stopListening();
        radio.write(&gpsPackage, sizeof(gpsPackage));
        //Serial.println("Sended");
        radio.startListening();
        delay(1);
        }
      }
    }
  }
  
  if(package.standMode and !package.walkMode){
    if(oldStand != package.standMode){
      int Bpos[12];
      for(int i = 0; i < 3; i++){
        Bpos[i] = FLLegPos[i];
      }
      for(int i = 0; i < 3; i++){
        Bpos[i + 3] = FRLegPos[i];
      }
      for(int i = 0; i < 3; i++){
        Bpos[i + 6] = RLLegPos[i];
      }
      for(int i = 0; i < 3; i++){
        Bpos[i + 9] = RRLegPos[i];
      }
      
  
      int sitPoint[3] = {0,175,0};
      int line[3];
      for(int i = 0; i < 100; i++){
        delay(5);
        for(int v = 0; v < 3; v++){
        line[v] = Bpos[v] - sitPoint[v];
  
        FLLegPos[v] = Bpos[v] - line[v] * i / 100 ;
        }
  
        for(int v = 0; v < 3; v++){
        line[v] = Bpos[v + 3] - sitPoint[v];
  
        FRLegPos[v] = Bpos[v + 3] - line[v] * i / 100 ;
        }
  
        for(int v = 0; v < 3; v++){
        line[v] = Bpos[v + 6] - sitPoint[v];
  
        RLLegPos[v] = Bpos[v + 6] - line[v] * i / 100 ;
        }
  
        for(int v = 0; v < 3; v++){
        line[v] = Bpos[v + 9] - sitPoint[v];
  
        RRLegPos[v] = Bpos[v + 9] - line[v] * i / 100 ;
        }
        
        moveLegsPositions();
      }
      oldStand = package.standMode;
    }
    Yin = 200 - map(joystick[5], 0, 1023, 50, 0);
    Xin = map(joystick[4], 0, 1023, -50, 50);
    Zin = map(joystick[3], 0, 1023, -45, 45);

    Ax = map(joystick[0], 0, 1023, -25, 25);
    Ay = map(joystick[1], 0, 1023, -15, 15);
    Az = map(joystick[2], 0, 1023, -15, 15);

    gyro[0] += Ax; 
    gyro[1] += Ay; 
    int p;
    if(millis() - gyroLast and millis() - gyroLast < 200){
      p =  (millis() - gyroLast) * 50;
    }
    else{
      p = 100;
    }
    for(int i = 0; i < 2 and config.gyroAssist; i++){
      
      if(abs(gyro[i]) > 2 and abs(Ag[i] + gyro[i] / p) <= 15) {Ag[i] += gyro[i] / p; gyroLast = millis();}
      
      
    }
    
    Ax += Ag[0];
    Ay += Ag[1];
    Zin += sin(radians(Ag[0])) * Yin; 
    Xin += sin(radians(Ag[1])) * Yin;
    
    standingMove();
    
    if(config.forceAssist){
      if(analogRead(A1) > maxForce[0] and -FLLegPress < 50){
        FLLegPress -= analogRead(A1) / maxForce[0];
      }
      else if(abs(FLLegPress) > 1){
        FLLegPress += 1;
      }
      FLLegPos[1] += FLLegPress;
  
      if(analogRead(A0) > maxForce[1] and -FRLegPress < 50){
        FRLegPress -= analogRead(A0) / maxForce[1];
      }
      else if(abs(FRLegPress) > 1){
        FRLegPress += 1;
      }
      FRLegPos[1] += FRLegPress;
  
      if(analogRead(A3) > maxForce[2] and -RLLegPress < 50){
        RLLegPress -= analogRead(A3) / maxForce[2];
      }
      else if(abs(RLLegPress) > 1){
        RLLegPress += 1;
      }
      RLLegPos[1] += RLLegPress;
  
      if(analogRead(A2) > maxForce[3] and -RRLegPress < 50){
        RRLegPress -= analogRead(A2) / maxForce[3];
      }
      else if(abs(RRLegPress) > 1){
        RRLegPress += 1;
      }
      RRLegPos[1] += RRLegPress;
    }

    moveLegsPositions();
    
  }
  
  if(!package.standMode and oldStand != package.standMode){

    int Bpos[12];
    for(int i = 0; i < 3; i++){
      Bpos[i] = FLLegPos[i];
    }
    for(int i = 0; i < 3; i++){
      Bpos[i + 3] = FRLegPos[i];
    }
    for(int i = 0; i < 3; i++){
      Bpos[i + 6] = RLLegPos[i];
    }
    for(int i = 0; i < 3; i++){
      Bpos[i + 9] = RRLegPos[i];
    }
    

    int sitPoint[3] = {0,100,0};
    int line[3];
    for(int i = 0; i < 100; i++){
      delay(5);
      for(int v = 0; v < 3; v++){
      line[v] = Bpos[v] - sitPoint[v];

      FLLegPos[v] = Bpos[v] - line[v] * i / 100 ;
      }

      for(int v = 0; v < 3; v++){
      line[v] = Bpos[v + 3] - sitPoint[v];

      FRLegPos[v] = Bpos[v + 3] - line[v] * i / 100 ;
      }

      for(int v = 0; v < 3; v++){
      line[v] = Bpos[v + 6] - sitPoint[v];

      RLLegPos[v] = Bpos[v + 6] - line[v] * i / 100 ;
      }

      for(int v = 0; v < 3; v++){
      line[v] = Bpos[v + 9] - sitPoint[v];

      RRLegPos[v] = Bpos[v + 9] - line[v] * i / 100 ;
      }
      
      moveLegsPositions();
    }
  }
  double Sy = 200 - map(joystick[5], 0, 1023, 50, 0);
  
  if(package.standMode and package.walkMode){
    
    double Wx = map(joystick[1], 0, 1023, maxStepX, -maxStepX);
    double Wz = map(joystick[0], 0, 1023, -maxStepZ, maxStepZ);

    double Fz = map(joystick[3], 0, 1023, -maxStepZ, maxStepZ) + Wz;
    double Rz = -Fz + 2*Wz;

    if(package.Lidar){
      Serial.println(xForce.bigNum);
      Serial.println(xnForce.bigNum);
      Serial.println(yForce.bigNum);
      Serial.println(ynForce.bigNum);
      Serial.println(" ");

      if(xForce.bigNum < disLimit[0]){
        Wz += 35;
      }
      if(xnForce.bigNum < disLimit[1]){
        Wz -= 35;
      }
      if(yForce.bigNum < disLimit[2]){
        Wx -= 35;
      }
      if(ynForce.bigNum < disLimit[3]){
        Wx += 35;
      }
    }

    

    if(sqrt(sq(Wx) + sq(Wz)) > 10 or abs(Fz) > 10){
      FLLegPos[0] -= stepOffset;
      FRLegPos[0] -= stepOffset;
      RLLegPos[0] -= stepOffset;
      RRLegPos[0] -= stepOffset;

      FLLegPos[1] -= tilt;
      RLLegPos[1] -= tilt;
      if(0 <= sTime[0] / config.path and sTime[0] / config.path < hoverTime / 2){
        FLLegPos[0] = FLLegPos[0] + ((Wx - FLLegPos[0]) * (1 / ((config.path * (hoverTime / 2)) - sTime[0])));
        FLLegPos[1] = tilt + FLLegPos[1] + ((Sy - config.stepH - FLLegPos[1]) * (1 / ((config.path * (hoverTime / 2)) - sTime[0])));
        FLLegPos[2] = FLLegPos[2] + ((Fz - FLLegPos[2]) * (1 / ((config.path * (hoverTime / 2)) - sTime[0]))); 
      }
      if(hoverTime / 2 <= sTime[0] / config.path and sTime[0] / config.path < hoverTime){
        FLLegPos[0] = Wx;
        FLLegPos[1] = tilt + FLLegPos[1] + ((Sy - FLLegPos[1]) * (1 / ((config.path * hoverTime / 2) - (sTime[0] - (hoverTime / 2) * config.path))));
        FLLegPos[2] = Fz; 
      }
      if(hoverTime <= sTime[0] / config.path and sTime[0] / config.path < 2 * pushTime){
        FLLegPos[0] = FLLegPos[0] + ((-Wx - FLLegPos[0]) * (1 / ((config.path * (pushTime + stepDelay)) - (sTime[0] - hoverTime * config.path))));
        FLLegPos[1] = tilt + FLLegPos[1] + ((Sy - FLLegPos[1]) * (1 / ((config.path * (pushTime + stepDelay)) - (sTime[0] - hoverTime * config.path))));
        FLLegPos[2] = FLLegPos[2] + ((-Fz - FLLegPos[2]) * (1 / ((config.path * (pushTime + stepDelay)) - (sTime[0] - hoverTime * config.path)))); 
      }
  
  
      if(0 <= sTime[1] / config.path and sTime[1] / config.path < hoverTime / 2){
        FRLegPos[0] = FRLegPos[0] + ((Wx - FRLegPos[0]) * (1 / ((config.path * (hoverTime / 2)) - sTime[1])));
        FRLegPos[1] = FRLegPos[1] + ((Sy - config.stepH - FRLegPos[1]) * (1 / ((config.path * (hoverTime / 2)) - sTime[1])));
        FRLegPos[2] = FRLegPos[2] + ((-Fz - FRLegPos[2]) * (1 / ((config.path * (hoverTime / 2)) - sTime[1]))); 
      }
      if(hoverTime / 2 <= sTime[1] / config.path and sTime[1] / config.path < hoverTime){
        FRLegPos[0] = Wx;
        FRLegPos[1] = FRLegPos[1] + ((Sy - FRLegPos[1]) * (1 / ((config.path * hoverTime / 2) - (sTime[1] - (hoverTime / 2) * config.path))));
        FRLegPos[2] = -Fz; 
      }
      if(hoverTime <= sTime[1] / config.path and sTime[1] / config.path < 2 * pushTime){
        FRLegPos[0] = FRLegPos[0] + ((-Wx - FRLegPos[0]) * (1 / ((config.path * (pushTime + stepDelay)) - (sTime[1] - hoverTime * config.path))));
        FRLegPos[1] = FRLegPos[1] + ((Sy - FRLegPos[1]) * (1 / ((config.path * (pushTime + stepDelay)) - (sTime[1] - hoverTime * config.path))));
        FRLegPos[2] = FRLegPos[2] + ((Fz - FRLegPos[2]) * (1 / ((config.path * (pushTime + stepDelay)) - (sTime[1] - hoverTime * config.path)))); 
      }

  
      if(0 <= sTime[2] / config.path and sTime[2] / config.path < hoverTime / 2){
        RLLegPos[0] = RLLegPos[0] + ((Wx - RLLegPos[0]) * (1 / ((config.path * (hoverTime / 2)) - sTime[2])));
        RLLegPos[1] = tilt + RLLegPos[1] + ((Sy - config.stepH - RLLegPos[1]) * (1 / ((config.path * (hoverTime / 2)) - sTime[2])));
        RLLegPos[2] = RLLegPos[2] + ((Rz - RLLegPos[2]) * (1 / ((config.path * (hoverTime / 2)) - sTime[2]))); 
      }
      if(hoverTime / 2 <= sTime[2] / config.path and sTime[2] / config.path < hoverTime){
        RLLegPos[0] = Wx;
        RLLegPos[1] = tilt + RLLegPos[1] + ((Sy - RLLegPos[1]) * (1 / ((config.path * hoverTime / 2) - (sTime[2] - (hoverTime / 2) * config.path))));
        RLLegPos[2] = Rz; 
      }
      if(hoverTime <= sTime[2] / config.path and sTime[2] / config.path < 2 * pushTime){
        RLLegPos[0] = RLLegPos[0] + ((-Wx - RLLegPos[0]) * (1 / ((config.path * (pushTime + stepDelay)) - (sTime[2] - hoverTime * config.path))));
        RLLegPos[1] = tilt + RLLegPos[1] + ((Sy - RLLegPos[1]) * (1 / ((config.path * (pushTime + stepDelay)) - (sTime[2] - hoverTime * config.path))));
        RLLegPos[2] = RLLegPos[2] + ((-Rz - RLLegPos[2]) * (1 / ((config.path * (pushTime + stepDelay)) - (sTime[2] - hoverTime * config.path)))); 
      }
  
      
      if(0 <= sTime[3] / config.path and sTime[3] / config.path < hoverTime / 2){
        RRLegPos[0] = RRLegPos[0] + ((Wx - RRLegPos[0]) * (1 / ((config.path * (hoverTime / 2)) - sTime[3])));
        RRLegPos[1] = RRLegPos[1] + ((Sy - config.stepH - RRLegPos[1]) * (1 / ((config.path * (hoverTime / 2)) - sTime[3])));
        RRLegPos[2] = RRLegPos[2] + ((-Wz - RRLegPos[2]) * (1 / ((config.path * (hoverTime / 2)) - sTime[3]))); 
      }
      if(hoverTime / 2 <= sTime[3] / config.path and sTime[3] / config.path < hoverTime){
        RRLegPos[0] = Wx;
        RRLegPos[1] = RRLegPos[1] + ((Sy - RRLegPos[1]) * (1 / ((config.path * hoverTime / 2) - (sTime[3] - (hoverTime / 2) * config.path))));
        RRLegPos[2] = -Wz; 
      }
      if(hoverTime <= sTime[3] / config.path and sTime[3] / config.path < 2 * pushTime){
        RRLegPos[0] = RRLegPos[0] + ((-Wx - RRLegPos[0]) * (1 / ((config.path * (pushTime + stepDelay)) - (sTime[3] - hoverTime * config.path))));
        RRLegPos[1] = RRLegPos[1] + ((Sy - RRLegPos[1]) * (1 / ((config.path * (pushTime + stepDelay)) - (sTime[3] - hoverTime * config.path))));
        RRLegPos[2] = RRLegPos[2] + ((Wz - RRLegPos[2]) * (1 / ((config.path * (pushTime + stepDelay)) - (sTime[3] - hoverTime * config.path)))); 
      }

      FLLegPos[0] += stepOffset;
      FRLegPos[0] += stepOffset;
      RLLegPos[0] += stepOffset;
      RRLegPos[0] += stepOffset;
      
      for(int i = 0; i < 4; i++){
        if((sTime[i] / config.path >= 2 * pushTime and !package.WalkMode) or (sTime[i] / config.path >= 5 * pushTime and package.WalkMode)){
          sTime[i] = 0;
        }
        else{
          sTime[i] ++;
        }
      }
    }
    else{
      FLLegPos[1] = Sy + tilt;
      FRLegPos[1] = Sy;
      RLLegPos[1] = Sy+ tilt;
      RRLegPos[1] = Sy;
      if(package.WalkMode){
        for(int i = 0; i < 4; i++){
          sTime[i] = W4s[i] * config.path;
        }
      }
      else{
        for(int i = 0; i < 4; i++){
          sTime[i] = W2s[i] * config.path;
        }
      }
    }

    moveLegsPositions();
  }

  oldStand = package.standMode;
}

void moveLegsPositions(){
  //Front Left Leg------------
  double Hz = sqrt(sq(FLLegPos[2] + Leg1) + sq(FLLegPos[1]));
  double Hx = sqrt(sq(FLLegPos[0]) + sq(Hz));

  double a3 = acos( (sq(Leg2) + sq(Leg3) - sq(Hx)) / (2 * Leg2 * Leg3) ) * 57296 / 1000;
  double a2 = 90 + acos( (sq(Hx) + sq(Leg2) - sq(Leg3)) / (2 * Hx * Leg2)) * 57296 / 1000 - asin(FLLegPos[0] / Hx) * 57296 / 1000;
  double a1 = atan((FLLegPos[2] + Leg1) / FLLegPos[1])* 57296 / 1000 - atan(Leg1 / FLLegPos[1])* 57296 / 1000;

  //Serial.println(FLLegPos[0]);
  //Serial.println(Hx);
  //Serial.println(a3);
  
  FLLeg[2].write(map(a3, 35, 135, 180, 0) + FLLegAdd[2]);
  FLLeg[1].write(a2 + FLLegAdd[1]);
  FLLeg[0].write(180 - (90 - a1) + FLLegAdd[0]);

  //Front Right Leg-----------
  Hz = sqrt(sq(FRLegPos[2] + Leg1) + sq(FRLegPos[1]));
  Hx = sqrt(sq(FRLegPos[0]) + sq(Hz));

  a3 = acos( (sq(Leg2)  + sq(Leg3) - sq(Hx)) / (2 * Leg2 * Leg3) ) * 57296 / 1000;
  a2 = 90 + acos( (sq(Hx) + sq(Leg2) - sq(Leg3)) / (2 * Hx * Leg2)) * 57296 / 1000 - asin(FRLegPos[0] / Hx) * 57296 / 1000;
  a1 = atan((FRLegPos[2] + Leg1) / FRLegPos[1])* 57296 / 1000 - atan(Leg1 / FRLegPos[1])* 57296 / 1000;
  
  FRLeg[2].write(map(a3, 35, 135, 0, 180) + FRLegAdd[2]);
  FRLeg[1].write(180 - a2  + FRLegAdd[1]);
  FRLeg[0].write(180 - (90 + a1)  + FRLegAdd[0]);

  //Rear Left Leg------------
  Hz = sqrt(sq(RLLegPos[2] + Leg1) + sq(RLLegPos[1]));
  Hx = sqrt(sq(RLLegPos[0]) + sq(Hz));

  a3 = acos( (sq(Leg2) + sq(Leg3) - sq(Hx)) / (2 * Leg2 * Leg3) ) * 57296 / 1000;
  a2 = 90 + acos( (sq(Hx) + sq(Leg2) - sq(Leg3)) / (2 * Hx * Leg2)) * 57296 / 1000 - asin(RLLegPos[0] / Hx) * 57296 / 1000;
  a1 = atan((RLLegPos[2] + Leg1) / RLLegPos[1])* 57296 / 1000 - atan(Leg1 / RLLegPos[1])* 57296 / 1000;
  
  RLLeg[2].write(map(a3, 35, 135, 180, 0) + RLLegAdd[2]);
  RLLeg[1].write(a2 + RLLegAdd[1]);
  RLLeg[0].write((90 - a1) + RLLegAdd[0]);

  //Rear Right Leg-----------
  Hz = sqrt(sq(RRLegPos[2] + Leg1) + sq(RRLegPos[1]));
  Hx = sqrt(sq(RRLegPos[0]) + sq(Hz));

  a3 = acos( (sq(Leg2) + sq(Leg3) - sq(Hx)) / (2 * Leg2 * Leg3) ) * 57296 / 1000;
  a2 = 90 + acos( (sq(Hx) + sq(Leg2) - sq(Leg3)) / (2 * Hx * Leg2)) * 57296 / 1000 - asin(RRLegPos[0] / Hx) * 57296 / 1000;
  a1 = atan((RRLegPos[2] + Leg1) / RRLegPos[1])* 57296 / 1000 - atan(Leg1 / RRLegPos[1])* 57296 / 1000;
  
  RRLeg[2].write(map(a3, 35, 135, 0, 180) + RRLegAdd[2]);
  RRLeg[1].write(180 - a2 + RRLegAdd[1]);
  RRLeg[0].write((90 + a1) + RRLegAdd[0]);
}

void getPackage(){
  for(int i = 0; i < 5; i++){
    if ( radio.available() ) {
      i = 5;
      radio.read(&package, sizeof(package));
      //Serial.println(package.joystick[1]);
    }
  }
  if(package.restoreConfig){
    setConfig();
  }
}

void getJoystick(){
    
    for(int ax = 0; ax < 6; ax++){
      joystick[ax] = package.joystick[ax];
    }
    
    for(int j = 0; j < 6; j++){
      if(joystick[j] - oldJoystick[j] > config.joystickSmoothness){
        delay(config.joystickSmoothness/10);
        joystick[j] = oldJoystick[j] +config.joystickSmoothness;
      }
      else if(joystick[j] - oldJoystick[j] < -config.joystickSmoothness){
        delay(config.joystickSmoothness/10);
        joystick[j] = oldJoystick[j] -config.joystickSmoothness;
      }
      oldJoystick[j] = joystick[j];
    }
 // for(int ax = 0; ax < 6; ax++){
    //Serial.print(joystick[ax]);
 //   Serial.print(" ");
 // }
  //Serial.println("");

}

void moveTwoLeg(){
  FLLeg[0].write(map(joystick[2],0,1023,45,180));
  FLLeg[1].write(map(joystick[0],0,1023,0,180));
  FLLeg[2].write(map(joystick[1],0,1023,0,180));
  FRLeg[0].write(180-map(joystick[5],0,1023,45,180));
  FRLeg[1].write(map(joystick[3],0,1023,0,180));
  FRLeg[2].write(map(joystick[4],0,1023,0,180));
}

void addToServos(){
  FLLeg[0].write(FLLeg[0].read() + 0);
  FLLeg[1].write(FLLeg[1].read() + 0);
  FLLeg[2].write(FLLeg[2].read() + 0);

  FRLeg[0].write(FRLeg[0].read() + 0);
  FRLeg[1].write(FRLeg[1].read() + 0);
  FRLeg[2].write(FRLeg[2].read() + 0);
  
  RLLeg[0].write(RLLeg[0].read() + 0);
  RLLeg[1].write(RLLeg[1].read() + 0);
  RLLeg[2].write(RLLeg[2].read() + 0);

  RRLeg[0].write(RRLeg[0].read() + 0);
  RRLeg[1].write(RRLeg[1].read() + 0);
  RRLeg[2].write(RRLeg[2].read() + 0);
}

void setLegstoFL(){
  FRLegPos[1] = FLLegPos[1];
  FRLegPos[0] = FLLegPos[0];
  FRLegPos[2] = FLLegPos[2];

  RLLegPos[1] = FLLegPos[1];
  RLLegPos[0] = FLLegPos[0];
  RLLegPos[2] = FLLegPos[2];

  RRLegPos[1] = FLLegPos[1];
  RRLegPos[0] = FLLegPos[0];
  RRLegPos[2] = FLLegPos[2];
}

void loadConfiguration(const char *filename, Config &config) {
  File file = SD.open(filename);
  StaticJsonDocument<512> doc;
  DeserializationError error = deserializeJson(doc, file);
  if (error)
    Serial.println(F("Failed to read file, using default configuration"));

  config.joystickSmoothness = doc["joystickSmoothness"];
  config.path = doc["path"];
  config.stepH = doc["stepH"];
  config.lidarSwitch = doc["lidarSwith"];
  config.gyroAssist = doc["gyroAssist"];
  config.droneSwitch = doc["droneSwith"];
  config.forceAssist = doc["forceAssist"];

  file.close();
}

void restoreConfig(){
  while(!radio.write(&config, sizeof(config))){}
}

void setConfig(){
  config.joystickSmoothness = package.joystickSmoothness;

  for(int i = 0; i < 2; i++){
    sTime[i] /= config.path;
  }
  config.path = package.path;
  if(package.WalkMode){
    for(int i = 0; i < 4; i++){
      sTime[i] = W4s[i];
    }
  }
  else{
    for(int i = 0; i < 4; i++){
      sTime[i] = W2s[i];
    }
  }
  for(int i = 0; i < 2; i++){
    sTime[i] *= config.path;
  }
  config.stepH = package.stepH;
  config.lidarSwitch = package.lidarSwitch;
  config.gyroAssist = package.gyroAssist;
  config.forceAssist = package.forceAssist;
  config.droneSwitch = package.droneSwitch;
  config.WalkMode = package.WalkMode;

  saveConfiguration(filename, config);
}

void saveConfiguration(const char *filename, const Config &config) {
  SD.remove(filename);

  File file = SD.open(filename, FILE_WRITE);
  if (!file) {
    Serial.println(F("Failed to create file"));
    return;
  }
  StaticJsonDocument<256> doc;

  doc["joystickSmoothness"] = config.joystickSmoothness;
  doc["path"] = config.path;
  doc["stepH"] = config.stepH;
  doc["lidarSwith"] = config.lidarSwitch;
  doc["gyroAssist"] = config.gyroAssist;
  doc["droneSwith"] = config.droneSwitch;
  doc["forceAssist"] = config.forceAssist;


  if (serializeJson(doc, file) == 0) {
    Serial.println(F("Failed to write to file"));
  }
  file.close();
}


void getGyro(){
  gyro[0] = -mpu.getAngleX();
  gyro[1] = -mpu.getAngleY();
  gyro[2] = -mpu.getAngleZ();
}

void getTemperature(){
  Serial.println(mpu.getTemp());
}

void getLidar(){
  int i = 0;
  int d = 10;
  Wire1.requestFrom(LidarAdress, 4);
  while(Wire1.available()){
    xForce.array[i] = Wire1.read();
    i++;
  }delay(d);
  
  Wire1.requestFrom(LidarAdress, 4);
  i = 0;
  while(Wire1.available()){
    yForce.array[i] = Wire1.read();
    i++;
  }delay(d);

  Wire1.requestFrom(LidarAdress, 4);
  i = 0;
  while(Wire1.available()){
    xnForce.array[i] = Wire1.read();
    i++;
  }delay(d);
  
  Wire1.requestFrom(LidarAdress, 4);
  i = 0;
  while(Wire1.available()){
    ynForce.array[i] = Wire1.read();
    i++;
  }delay(d);
  //Serial.println("Read");
}

void getGas(){
  Wire1.requestFrom(GasAdress, 4);
  
  int i = 0;
  int d = 5;
  while(Wire1.available()){
    H2.array[i] = Wire1.read();
    i++;
  }delay(d);
  Serial.println(H2.bigNum);

  Wire1.requestFrom(GasAdress, 4);
  i = 0;
  while(Wire1.available()){
    LPG.array[i] = Wire1.read();
    i++;
  }delay(d);
  Serial.println(LPG.bigNum);

  Wire1.requestFrom(GasAdress, 4);
  i = 0;
  while(Wire1.available()){
    CH4.array[i] = Wire1.read();
    i++;
  }delay(d);
  Serial.println(CH4.bigNum);

  Wire1.requestFrom(GasAdress, 4);
  i = 0;
  while(Wire1.available()){
    CO.array[i] = Wire1.read();
    i++;
  }delay(d);
  Serial.println(CO.bigNum);

  Wire1.requestFrom(GasAdress, 4);
  i = 0;
  while(Wire1.available()){
    Alchol.array[i] = Wire1.read();
    i++;
  }delay(d);
  Serial.println(Alchol.bigNum);

  gasPackage.H2 = H2.bigNum;
  gasPackage.LPG = LPG.bigNum;
  gasPackage.CH4 = CH4.bigNum;
  gasPackage.CO = CO.bigNum;
  gasPackage.Alchol = Alchol.bigNum;
}

void standingMove(){
  double r = sqrt(sq(LegtoOrg[0]) + sq(Yin));
  double Al = degrees(acos((sq(r) + sq(Yin) - sq(LegtoOrg[0])) / (2 * r * Yin)));
    
  double Ar = 270 - Ay - Al;
  double Af = 270 + Ay - Al;

  double Yr = r * sin(radians(Ar)) + sin(radians(Ax)) * Xin + sin(radians(Az)) * Zin;
  double Yf = r * sin(radians(Af)) + sin(radians(Ax)) * Xin + sin(radians(Az)) * Zin;

  double Xf = LegtoOrg[0] + r * cos(radians(Af));
  double Xr = LegtoOrg[0] + r * cos(radians(Ar));

  double Xz = cos(radians(32.34 + Az)) * sqrt(sq(LegtoOrg[1]) + sq(LegtoOrg[0])) - 150;
  double Zz = sin(radians(32.34 + Az)) * sqrt(sq(LegtoOrg[1]) + sq(LegtoOrg[0])) - 95;

    //----------------

  r = sqrt(sq(LegtoOrg[1]) + sq(Yf));
  Al = degrees(acos((sq(r) + sq(Yf) - sq(LegtoOrg[1])) / (2 * r * Yf)));
    
  double Ari = 270 - Ax - Al;
  double Ale = 270 + Ax - Al;

  double Yri = r * sin(radians(Ari));
  double Yle = r * sin(radians(Ale));

  double Zri = LegtoOrg[1] + r * cos(radians(Ari));
  double Zle = LegtoOrg[1] + r * cos(radians(Ale));
    
  FLLegPos[0] = -Xf + cos(radians(Ax)) * Xin + Xz;
  FLLegPos[1] = Yle;
  FLLegPos[2] = -Zle - cos(radians(Az)) * Zin + Zz;
    
  FRLegPos[0] = -Xf + cos(radians(Ax)) * Xin - Xz;
  FRLegPos[1] = Yri;
  FRLegPos[2] = -Zri + cos(radians(Az)) * Zin - Zz;

  r = sqrt(sq(LegtoOrg[1]) + sq(Yr));
  Al = degrees(acos((sq(r) + sq(Yr) - sq(LegtoOrg[1])) / (2 * r * Yr)));
  
  Ari = 270 - Ax - Al;
  Ale = 270 + Ax - Al;

  Yri = r * sin(radians(Ari));
  Yle = r * sin(radians(Ale));

  Zri = LegtoOrg[1] + r * cos(radians(Ari));
  Zle = LegtoOrg[1] + r * cos(radians(Ale));
    
  RLLegPos[0] = -Xf + cos(radians(Ax)) * Xin + Xz;
  RLLegPos[1] = Yle;
  RLLegPos[2] = -Zle - cos(radians(Az)) * Zin - Zz;
    
  RRLegPos[0] = -Xf + cos(radians(Ax)) * Xin - Xz;
  RRLegPos[1] = Yri;
  RRLegPos[2] = -Zri + cos(radians(Az)) * Zin + Zz;
}
