#include <Wire.h>

int Adress = 3;
bool turn = 0;
bool oldTurn = 3;

void setup() {
  Serial.begin(9600);
  Wire.begin(Adress);

  Wire.onReceive(receiveEvent);

  for(int i = 0; i < 3; i++){
    pinMode(i+11, OUTPUT);
  }
  pinMode(10, INPUT);
}

void loop() {
  if(oldTurn != turn){
    if(turn){
      digitalWrite(13,1);
      digitalWrite(11,1);
      delay(25*1000);
      digitalWrite(11,0);
    }
    else{
      digitalWrite(13,0);
      while(!digitalRead(10)){
        digitalWrite(12,1);
      }
      digitalWrite(12,0);
    }
    oldTurn = turn;
  }
}

void receiveEvent() {
  while(Wire.available()) {
    turn = Wire.read();
  }
  Serial.println("Rec");
}
