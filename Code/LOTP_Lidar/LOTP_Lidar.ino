#include <VL53L0X.h>
#include <Wire.h>

double pastTime = 0;
bool R;
float Map[200];
float Sides[8];
float Force[4];
int req = 0;
double lastReq = 0;
int d = 0;
bool turn = 0;

union {
   byte array[4];
   float bigNum;
} xPackage;

union {
   byte array[4];
   float bigNum;
} xnPackage;

union {
   byte array[4];
   float bigNum;
} yPackage;

union {
   byte array[4];
   float bigNum;
} ynPackage;

int Adress = 9;

VL53L0X sensor;

void setup() {
  //Serial.begin(9600);
  Wire.begin(Adress);

  sensor.init();
  
  pinMode(13, OUTPUT);
  pinMode(12, INPUT);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);

  sensor.startContinuous();
}

void loop() {
  R = digitalRead(12);
  digitalWrite(13, turn);

  if(millis() - lastReq > 200){
    req = 0;
  }

  if(turn){
    Map[d] = sensor.readRangeContinuousMillimeters();
    d++;
    
    if((R and millis() - pastTime > 500) or d == 200){
      //Serial.println(d);
      pastTime = millis();
      
      for(int i = 0; i < 8; i++){
        for(int p = 0; p < d/8; p++){
          Sides[i] += Map[p + (d/8) * i];
        } 
        Sides[i] /= d/8;
        //Serial.print(Sides[i]);
        //Serial.print(" ");
      }
      //Serial.println("");

      for(int i = 0; i < 4; i++){
        Force[i] = 0;
      }

      Force[2] += Sides[3];
      Force[2] += Sides[4];
      Force[3] += Sides[0];
      Force[3] += Sides[7];

      Force[0] += Sides[5];
      Force[0] += Sides[6];
      Force[1] += Sides[1];
      Force[1] += Sides[2];

      for(int i = 0; i < 4; i++){
        Force[i] = Force[i] / 2;
      }

      //Serial.print(Force[0]);
      //Serial.print(" ");
      //Serial.print(Force[1]);
      //Serial.println(" ");

      xPackage.bigNum = Force[0];
      xnPackage.bigNum = Force[1];
      yPackage.bigNum = Force[2];
      ynPackage.bigNum = Force[3];
      
      d = 0;
      
    }
  }
  
}

void receiveEvent() {
  while(Wire.available()) {
    turn = Wire.read();
  }
}
 
void requestEvent() {
  lastReq = millis();
  if(req == 0){
    for(int i = 0; i < 4; i++){
      Wire.write(xPackage.array[i]);
    }
    req = 1;
  }
  else if(req == 1){
    for(int i = 0; i < 4; i++){
      Wire.write(yPackage.array[i]);
    }
    req = 2;
  }
  else if(req == 2){
    for(int i = 0; i < 4; i++){
      Wire.write(xnPackage.array[i]);
    }
    req = 3;
  }
  else if(req == 3){
    for(int i = 0; i < 4; i++){
      Wire.write(ynPackage.array[i]);
    }
    req = 0;
  }
}
