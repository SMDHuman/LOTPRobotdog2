#include <MQUnifiedsensor.h>
#include <Wire.h>

byte Package[5];
int Adress = 5;
bool turn = 0;
int req = 0;
double lastReq = 0;
double lastBuzz = 0;
int Buzz = 12;
int Red = 10;
int Green = 11;
int danger = 0;

union {
   byte array[4];
   float bigNum;
} H2;

union {
   byte array[4];
   float bigNum;
} LPG;

union {
   byte array[4];
   float bigNum;
} CH4;

union {
   byte array[4];
   float bigNum;
} CO;

union {
   byte array[4];
   float bigNum;
} Alcohol;

MQUnifiedsensor MQ7("Arduino Pro Mini", 5, 10, A0, "MQ-7");

void setup() {
  Serial.begin(9600);
  Wire.begin(Adress);

  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);

  MQ7.setRegressionMethod(1);
  MQ7.init(); 

  for(int i = 0; i < 3; i++){
    pinMode(i+10, OUTPUT);
  }

  Serial.print("Calibrating please wait.");
  float calcR0 = 0;
  for(int i = 1; i<=10; i ++)
  {
    delay(500)
    MQ7.update(); // Update data, the arduino will be read the voltage on the analog pin
    calcR0 += MQ7.calibrate(27.5);
    Serial.print(".");
  }
  MQ7.setR0(calcR0/10); 
  Serial.println("  done!.");

  if(isinf(calcR0)) {Serial.println("Warning: Conection issue founded, R0 is infite (Open circuit detected) please check your wiring and supply"); while(1);}
  if(calcR0 == 0){Serial.println("Warning: Conection issue founded, R0 is zero (Analog pin with short circuit to ground) please check your wiring and supply"); while(1);}
}

void loop() {
  danger = 0;
  MQ7.update();
  digitalWrite(Green, turn);
  
  if(millis() - lastReq > 100){
    req = 0;
  }
  
  if(turn){    
    MQ7.setA(69.014); MQ7.setB(-1.374);
    H2.bigNum = MQ7.readSensor(); //H2
    //if(H2.bigNum > 100){danger ++;}
    //Serial.print(H2.bigNum);
    //Serial.print(" H2 ");
 
    MQ7.setA(700000000); MQ7.setB(-7.703);
    LPG.bigNum = MQ7.readSensor(); //LPG
    //if(LPG.bigNum > 100000){danger ++;}
    //Serial.print(LPG.bigNum);
    //Serial.print( " LPG ");

    MQ7.setA(60000000000000); MQ7.setB(-10.54);
    CH4.bigNum = MQ7.readSensor(); //CH4
    if(CH4.bigNum > 10000){danger ++;}
    //Serial.print(CH4.bigNum);
    //Serial.print(" CH4 ");

    MQ7.setA(99.042); MQ7.setB(-1.518);
    CO.bigNum = MQ7.readSensor(); //CO
    if(CO.bigNum > 50){danger ++;}
    //Serial.print(CO.bigNum);
    //Serial.print( " CO ");

    MQ7.setA(40000000000000000); MQ7.setB(-12.35);
    Alcohol.bigNum = MQ7.readSensor(); //Alcohol
    //if(Alcohol.bigNum > 50000){danger ++;}
    //Serial.print(Alcohol.bigNum);
    //Serial.print(" Alcohol ");
    //Serial.println(" ");
    
  }
  if(danger > 0){
    if(millis() - lastBuzz < 250){
      digitalWrite(Buzz, 1);
    }
    else{
      digitalWrite(Buzz, 0);
    }
    if(millis() - lastBuzz > 500){
      lastBuzz = millis();
    }
    digitalWrite(Red, 1);
  }
  else{
    digitalWrite(Buzz, 0);
    digitalWrite(Red, 0);
  }
  
}

void receiveEvent() {
  while(Wire.available()) {
    turn = Wire.read();
  }
  Serial.println("Rec");
}
 
void requestEvent() {
  lastReq = millis();
  if(req == 0){
    for(int i = 0; i < 4; i++){
      Wire.write(H2.array[i]);
    }
    req = 1;
  }
  else if(req == 1){
    for(int i = 0; i < 4; i++){
      Wire.write(LPG.array[i]);
    }
    req = 2;
  }
  else if(req == 2){
    for(int i = 0; i < 4; i++){
      Wire.write(CH4.array[i]);
    }
    req = 3;
  }
  else if(req == 3){
    for(int i = 0; i < 4; i++){
      Wire.write(CO.array[i]);
    }
    req = 4;
  }
  else if(req == 4){
    for(int i = 0; i < 4; i++){
      Wire.write(Alcohol.array[i]);
    }
    req = 0;
  }
  //Serial.print(req);
  //Serial.println("Req");
}
