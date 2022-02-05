#include <LiquidCrystal.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(6, 7); // CE, CSN
LiquidCrystal lcd(13, 4, 3, 2, 0, 1);

int joystickZero[6] = {10,10,10,10,10,10};

char joypack[24];
int menuCursor = 0;
int BinMode = 5;
int SinMode = 8;
int Switches[4];
int oldSwitches[4];
int switchPins[4] = {12,11,10,9};
bool Bnew[3] = {1,1,1};
bool bEnter = 0;
int menuLoc[3] = {0,0,0};
int buttonPins[3] = {11,10,9};
int Cstat = 2;
int CurserX = 9;
int menuLength = 4;
bool tStatus = 0;
bool rStatus = 0;
bool recive = 0;
bool doOnes[2] = {0,0};

char configPackage[80];
int packageCharSize = 5;

String menu[4] = {"-GPS     ", "-Modules", "-Settings", "-About  "};
String menuGPS[4] = {"< Back", "-Location", "-Time ", "         "};
String menuSettings[8] = {"< Back      ", "Restore^    ", "J Smoth     ", "Path        ", "Step H      ", "gAssist     ", "fAssist     ", "Walk Mode   "};
String menuModules[4] = {"< Back", "Lidar", "Drone ", "DGM  "};
String menuLidar[4] = {"< Back     ", "Turn Off", "Switch None", "        "};
String menuDrone[4] = {"< Back     ", "Turn Off", "Switch None", "        "};
String menuGas[4] = {"< Back       ", "Alert Off", "Gas Variables", "         "};

String menuAbout[6] = {"< Back   ", "LOTP RoboDog  ", "Prototype", "V2            ", "Made By :", "Halid Yildirim"};

const uint64_t tAddress = 0xE8E8F0F0E1LL;
const uint64_t rAddress = 0xE8E8F0F0E0LL;
const uint64_t tDrone = 0xE8E8F0F1E1LL;

unsigned long pTime = millis();
unsigned long cTime = millis();
unsigned long dTime = 1000;

byte conChar[8] = {
  B00000,
  B01110,
  B10001,
  B00100,
  B01010,
  B00000,
  B00100,
};

byte disconChar[8] = {
  B00000,
  B00000,
  B00000,
  B00000,
  B00000,
  B00000,
  B00100,
};

byte blank[8] = {
  B00000,
  B00000,
  B00000,
  B00000,
  B00000,
  B00000,
  B00000,
};

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

struct Config{
  byte joystickSmoothness;
  byte path;
  byte stepH;
  bool gyroAssist;
  bool forceAssist;
  byte lidarSwitch;
  byte droneSwitch;
};

Package package;

GPSPackage gpsPackage;

GasPackage gasPackage;

Config config;

void setup(){
  Serial.begin(9600);

  pinMode(SinMode, OUTPUT);
  pinMode(BinMode, OUTPUT);

  for(int i = 0; i < 4; i++){
    pinMode(switchPins[i], INPUT);
  }
  
  lcd.begin(16, 2);
  lcd.createChar(1,disconChar);
  lcd.createChar(2,conChar);
  lcd.createChar(0,blank);
  
  radio.begin();
  radio.openWritingPipe(tAddress);
  radio.openReadingPipe(1,rAddress);
  radio.setPALevel(RF24_PA_MIN);
  
  lcd.setCursor(0, 0);
  lcd.print("LOTP RoboDog");
  lcd.setCursor(1, 1);
  lcd.print("V2");

  delay(1500);
  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print("Searching for");
  lcd.setCursor(0, 1);
  lcd.print("config    Skip <");

  delay(5);
  digitalWrite(BinMode, 1);
  radio.startListening();
  while(!radio.available() and !digitalRead(buttonPins[1])){}
  for(int i = 0; i < 5; i++){
    if(radio.available()){
      lcd.clear();
      radio.read(&config, sizeof(config));
      lcd.setCursor(0, 0);
      lcd.print("Config Recived");
      setConfig();
      delay(750);
      i = 5;
    }
    else{
      setConfigDefault();
      Bnew[1] = 0;
    }
  }
  if(package.gyroAssist){
    menuSettings[5] = "gAssist On  "; 
  }
  else{
    menuSettings[5] = "gAssist Off "; 
  }
  if(package.forceAssist){
    menuSettings[6] = "fAssist On  "; 
  }
  else{
    menuSettings[6] = "fAssist Off "; 
  }
  if(package.WalkMode){
    menuSettings[7] = "Walk Mode 4s"; 
  }
  else{
    menuSettings[7] = "Walk Mode 2s"; 
  }
  
  radio.stopListening();
  digitalWrite(BinMode, 0);
  delay(50);

  
  lcd.clear();
}

void loop(){
  cTime = millis();
  Serial.println(package.WalkMode);
  
  package.restoreConfig = 0;
  
  lcd.setCursor(CurserX, menuCursor%2);
  lcd.write(byte(0));
  
  digitalWrite(BinMode, 1);
  //Up Button
  if(digitalRead(buttonPins[0]) and menuCursor > 0 and Bnew[0]){
    menuCursor--;
    Bnew[0] = 0;
  }
  else if(!digitalRead(buttonPins[0])){
    Bnew[0] = 1;
  }

  //Down Button
  if(digitalRead(buttonPins[2]) and menuCursor < menuLength-1 and Bnew[2]){
    menuCursor++;
    Bnew[2] = 0;
  }
  else if(!digitalRead(buttonPins[2])){
    Bnew[2] = 1;
  }

  //Enter Button
  if(digitalRead(buttonPins[1]) and Bnew[1]){
    Bnew[1] = 0;
    bEnter = 1;
  }
  else if(!digitalRead(buttonPins[1]) and !Bnew[1]){
    lcd.clear();
    updateConnectionStat();
    Bnew[1] = 1;
    bEnter = 0;
  }
  digitalWrite(BinMode, 0);

  if(menuLoc[0] == 0){
    CurserX = 9;
    mainMenu();
    menuLength = 4;
  }
  
  if(menuLoc[0] == 1){
    package.Gps = 1;
    recive = 1;
    if(menuLoc[1] == 0){
      CurserX = 9;
      menuLength = 3;
      GPSMenu();
    }
    
    if(menuLoc[1] == 1){
      CurserX = 13;
      LocMenu();
    }
    if(menuLoc[1] == 2){
      CurserX = 13;
      TimeMenu();
    }
    
    if(cTime - pTime > dTime){
      if(cTime - pTime < dTime + 10){
        rStatus = 0 ;
      }
      package.gpsRequest = 1;
      if(cTime - pTime > dTime + 100){
        pTime = cTime;
      }
    }
    else{
      package.gpsRequest = 0;
    }
  }
  else{
    rStatus = 0;
    package.Gps = 0;
  }
  
  if(menuLoc[0] == 2){
    if(menuLoc[1] == 0){
      CurserX = 10;
      menuLength = 4;
      modulesMenu();
    }

    if(menuLoc[1] == 1){
      CurserX = 11;
      menuLength = 3;
      lidarMenu();
    }
    if(menuLoc[1] == 2){
      CurserX = 11;
      menuLength = 3;
      droneMenu();
    }
    if(menuLoc[1] == 3){
      CurserX = 13;
      menuLength = 3;
      if(menuLoc[2] == 0){
        gasMenu();
      }
      if(menuLoc[2] == 1){
        gasVarMenu();
      }
      
    }
    
  }

  if(package.Gas){
    gasAlert();
    if(cTime - pTime > dTime){
      if(cTime - pTime < dTime + 10){
        rStatus = 0 ;
      }
      package.gasRequest = 1;
      if(cTime - pTime > dTime + 100){
        pTime = cTime;
      }
    }
    else{
      package.gasRequest = 0;
    }
  }

  if(menuLoc[0] == 3){
    
    if(menuLoc[1] == 0){
      CurserX = 12;
      menuLength = 8;
      
      settingsMenu();
    }
    if(menuLoc[1] == 1){
      RestoreConfig();
      menuLoc[1] = 0;
      menuCursor = 1;
      bEnter = 0;
    }
    
    if(menuLoc[1] == 2){
      menuLength = 512;
      if(doOnes[0] == 0){
        doOnes[0] = 1;
        menuCursor = -package.joystickSmoothness + 256;
      }
      lcd.setCursor(7, 0);
      lcd.print("  ");
      lcd.setCursor(7, 0);
      lcd.print(-menuCursor + 256);
      if(bEnter == 1){
        lcd.setCursor(7, 0);
        lcd.print("  ");
        package.joystickSmoothness = -menuCursor + 256;
        menuLoc[1] = 0;
        menuCursor = 2;
        bEnter = 0;
        doOnes[0] = 0;
      }
    }
    if(menuLoc[1] == 3){
      menuLength = 512;
      if(doOnes[0] == 0){
        doOnes[0] = 1;
        menuCursor = -package.path + 256;
      }
      lcd.setCursor(7, 0);
      lcd.print("  ");
      lcd.setCursor(7, 0);
      lcd.print(-menuCursor + 256);
      if(bEnter == 1){
        lcd.setCursor(7, 0);
        lcd.print("  ");
        package.path = -menuCursor + 256;
        menuLoc[1] = 0;
        menuCursor = 3;
        bEnter = 0;
        doOnes[0] = 0;
      }
    }
    if(menuLoc[1] == 4){
      menuLength = 512;
      if(doOnes[0] == 0){
        doOnes[0] = 1;
        menuCursor = -package.stepH + 256;
      }
      lcd.setCursor(7, 0);
      lcd.print("  ");
      lcd.setCursor(7, 0);
      lcd.print(-menuCursor + 256);
      if(bEnter == 1){
        lcd.setCursor(7, 0);
        lcd.print("  ");
        package.stepH = -menuCursor + 256;
        menuLoc[1] = 0;
        menuCursor = 4;
        bEnter = 0;
        doOnes[0] = 0;
      }
    }
    if(menuLoc[1] == 5){
      menuLength = 1;
      if(package.gyroAssist){
        package.gyroAssist = 0;
        menuSettings[5] = "gAssist Off "; 
      }
      else{
        package.gyroAssist = 1;
        menuSettings[5] = "gAssist On  "; 
      }
      menuLoc[1] = 0;
      menuCursor = 5;
      bEnter = 0;
      doOnes[0] = 0;
    }
    if(menuLoc[1] == 6){
      menuLength = 1;
      if(package.forceAssist){
        package.forceAssist = 0;
        menuSettings[6] = "fAssist Off "; 
      }
      else{
        package.forceAssist = 1;
        menuSettings[6] = "fAssist On  "; 
      }
      menuLoc[1] = 0;
      menuCursor = 6;
      bEnter = 0;
      doOnes[0] = 0;
    }
    if(menuLoc[1] == 7){
      menuLength = 1;
      if(package.WalkMode){
        package.WalkMode = 0;
        menuSettings[7] = "Walk Mode 2s"; 
      }
      else{
        package.WalkMode = 1;
        menuSettings[7] = "Walk Mode 4s"; 
      }
      menuLoc[1] = 0;
      menuCursor = 7;
      bEnter = 0;
      doOnes[0] = 0;
    }
    
    
  }
  
  if(menuLoc[0] == 4){
    CurserX = 12;
    menuLength = 5;
      
    aboutMenu();
  }
  
  //Get joystick and add in to Package
  zipJoystick();

  //Get Switch mode and add in to Package
  getSwitch();
  
  package.standMode = Switches[0];
  package.walkMode = Switches[1];

  if(package.lidarSwitch == 3 and oldSwitches[2] != Switches[2]){
    package.Lidar = Switches[2];
  }
  if(package.lidarSwitch == 4 and oldSwitches[3] != Switches[3]){
    package.Lidar = Switches[3];
  }

  if(package.droneSwitch == 3 and oldSwitches[2] != Switches[2]){
    package.Drone = Switches[2];
  }
  if(package.droneSwitch == 4 and oldSwitches[3] != Switches[3]){
    package.Drone = Switches[3];
  }

  for(int i = 0; i < 4; i++){
    oldSwitches[i] = Switches[i];
  }

  //Send Package and check for connection
  radio.stopListening();
  tStatus = radio.write(&package, sizeof(package));
  if(package.gpsRequest){
    getGpsPackage();
  }
  if(package.gasRequest){
    getGasPackage();
  }
  updateConnectionStat();
}

void zipJoystick(){
  int joystick[6] = {analogRead(A0),analogRead(A1),analogRead(A2),analogRead(A3),analogRead(A4),analogRead(A5)};

  for(int i = 0; i < 6; i++){
    if(abs(joystick[i] - 512) < joystickZero[i]){
      joystick[i] = 512;
    }
    package.joystick[i] = joystick[i];
  }
}

void getSwitch(){
  digitalWrite(SinMode, 1);
  for(int i = 0; i < 4; i++){
    Switches[i] = digitalRead(switchPins[i]);
  }
  digitalWrite(SinMode, 0);
}

void updateConnectionStat(){
  lcd.setCursor(14,0);
  lcd.write("T");
  lcd.setCursor(15,0);
  if(tStatus){
    lcd.write(byte(2));
  }
  else{
    lcd.write(byte(1));
  }
  lcd.setCursor(14,1);
  lcd.write("R");
  lcd.setCursor(15,1);
  if(rStatus){
    lcd.write(byte(2));
  }
  else{
    lcd.write(byte(1));
  }
}

void mainMenu(){
  lcd.setCursor(CurserX, menuCursor%2);
  lcd.write(char(60));
  lcd.setCursor(0, 0);
  lcd.print(menu[int(menuCursor/2)*2]);
  lcd.setCursor(0, 1);
  lcd.print(menu[int(menuCursor/2)*2+1]);

  if(bEnter == 1){
    menuLoc[0] = menuCursor + 1;
    menuCursor = 0;
    bEnter = 0;
  }
}

void GPSMenu(){
  lcd.setCursor(CurserX, menuCursor%2);
  lcd.write(char(60));
  lcd.setCursor(0, 0);
  lcd.print(menuGPS[int(menuCursor/2)*2]);
  lcd.setCursor(0, 1);
  lcd.print(menuGPS[int(menuCursor/2)*2+1]);

  if(bEnter == 1 and menuCursor == 0){
    menuLoc[0] = 0;
    menuLoc[1] = 0;
    menuLoc[2] = 0;
    menuCursor = 0;
    bEnter = 0;
  }
  else if(bEnter == 1){
    menuLoc[1] = menuCursor;
    menuCursor = 0;
    bEnter = 0;
  }
}

void LocMenu(){
  lcd.setCursor(0, 0);
  lcd.print("Lati");
  for(int i = 0; i < 8; i++){
    lcd.setCursor(i + 5, 0);
    lcd.print(gpsPackage.lati);
  }
  
  lcd.setCursor(0, 1);
  lcd.print("Long");
  for(int i = 0; i < 8; i++){
    lcd.setCursor(i + 5, 1);
    lcd.print(gpsPackage.longti);
  }

  if(bEnter == 1){
    menuLoc[1] = 0;
    menuCursor = 1;
    bEnter = 0;
  }
}

void TimeMenu(){
  lcd.setCursor(0, 0);
  lcd.print("Time");

  lcd.setCursor(7, 0);
  lcd.print(gpsPackage.hour);
    
  lcd.setCursor(9, 0);
  lcd.print(":");
  
  lcd.setCursor(10, 0);
  lcd.print(gpsPackage.minute);
  
  
  lcd.setCursor(2, 1);
  lcd.print(gpsPackage.day);
  
  lcd.setCursor(4, 1);
  lcd.print(".");
  
  lcd.setCursor(5, 1);
  lcd.print(gpsPackage.month);

  lcd.setCursor(7, 1);
  lcd.print(".");
  
  lcd.setCursor(8, 1);
  lcd.print(gpsPackage.year);
  

  if(bEnter == 1){
    menuLoc[1] = 0;
    menuCursor = 2;
    bEnter = 0;
  }
}

void aboutMenu(){
  lcd.setCursor(CurserX, menuCursor%2);
  lcd.write(char(60));
  lcd.setCursor(0, 0);
  lcd.print(menuAbout[int(menuCursor/2)*2]);
  lcd.setCursor(0, 1);
  lcd.print(menuAbout[int(menuCursor/2)*2+1]);

  if(bEnter == 1 and menuCursor == 0){
    menuLoc[0] = 0;
    menuLoc[1] = 0;
    menuLoc[2] = 0;
    menuCursor = 3;
    bEnter = 0;
  }
}

void getGpsPackage(){
    delay(1);
    
    radio.startListening();
    for(int i = 0; i < 5; i++){
      if(radio.available()){
        i = 5;
        rStatus = 1;
        radio.read(&gpsPackage, sizeof(gpsPackage));
      }
    }
    delay(1);
}

void getGasPackage(){
    delay(1);
    
    radio.startListening();
    for(int i = 0; i < 5; i++){
      if(radio.available()){
        i = 5;
        rStatus = 1;
        radio.read(&gasPackage, sizeof(gasPackage));
      }
    }
    delay(1);
}

void settingsMenu(){
  lcd.setCursor(CurserX, menuCursor%2);
  lcd.write(char(60));
  lcd.setCursor(0, 0);
  lcd.print(menuSettings[int(menuCursor/2)*2]);
  lcd.setCursor(0, 1);
  lcd.print(menuSettings[int(menuCursor/2)*2+1]);

  if(bEnter == 1 and menuCursor == 0){
    menuLoc[0] = 0;
    menuLoc[1] = 0;
    menuLoc[2] = 0;
    menuCursor = 2;
    bEnter = 0;
  }
  else if(bEnter == 1){
    menuLoc[1] = menuCursor;
    menuCursor = 0;
    bEnter = 0;
  }
}

void restoreConfig(){
  package.restoreConfig = 1;
}

void modulesMenu(){
  lcd.setCursor(CurserX, menuCursor%2);
  lcd.write(char(60));
  lcd.setCursor(0, 0);
  lcd.print(menuModules[int(menuCursor/2)*2]);
  lcd.setCursor(0, 1);
  lcd.print(menuModules[int(menuCursor/2)*2+1]);

  if(bEnter == 1 and menuCursor == 0){
    menuLoc[0] = 0;
    menuLoc[1] = 0;
    menuLoc[2] = 0;
    menuCursor = 1;
    bEnter = 0;
  }
  else if(bEnter == 1){
    menuLoc[1] = menuCursor;
    menuCursor = 0;
    bEnter = 0;
  }
}

void lidarMenu(){
  lcd.setCursor(CurserX, menuCursor%2);
  lcd.write(char(60));
  lcd.setCursor(0, 0);
  lcd.print(menuLidar[int(menuCursor/2)*2]);
  lcd.setCursor(0, 1);
  lcd.print(menuLidar[int(menuCursor/2)*2+1]);

  if(bEnter == 1 and menuCursor == 0){
    menuLoc[1] = 0;
    menuLoc[2] = 0;
    menuCursor = 1;
    bEnter = 0;
  }
  else if(bEnter and menuCursor == 1 and !doOnes[1]){
    doOnes[1] = 1;
    if(!package.Lidar){
      menuLidar[1] = "Turn On ";
      package.Lidar = 1;
    }
    else{
      menuLidar[1] = "Turn Off";
      package.Lidar = 0;
    }
  }
  
  else if(bEnter and menuCursor == 2 and !doOnes[1]){
    doOnes[1] = 1;
    if(package.lidarSwitch == 0){
      menuLidar[2] = "Switch 3   ";
      package.lidarSwitch = 3;
    }
    else if(package.lidarSwitch == 3){
      menuLidar[2] = "Switch 4   ";
      package.lidarSwitch = 4;
    }
    else{
      menuLidar[2] = "Switch None";
      package.lidarSwitch = 0;
    }
  }
  else if(!bEnter and doOnes[1]){
    doOnes[1] = 0;
  }
}

void droneMenu(){
  lcd.setCursor(CurserX, menuCursor%2);
  lcd.write(char(60));
  lcd.setCursor(0, 0);
  lcd.print(menuDrone[int(menuCursor/2)*2]);
  lcd.setCursor(0, 1);
  lcd.print(menuDrone[int(menuCursor/2)*2+1]);

  if(bEnter == 1 and menuCursor == 0){
    menuLoc[1] = 0;
    menuLoc[2] = 0;
    menuCursor = 2;
    bEnter = 0;
  }
  else if(bEnter and menuCursor == 1 and !doOnes[1]){
    doOnes[1] = 1;
    if(!package.Drone){
      menuDrone[1] = "Turn On ";
      package.Drone = 1;
    }
    else{
      menuDrone[1] = "Turn Off";
      package.Drone = 0;
    }
  }
  
  else if(bEnter and menuCursor == 2 and !doOnes[1]){
    doOnes[1] = 1;
    if(package.droneSwitch == 0){
      menuDrone[2] = "Switch 3   ";
      package.droneSwitch = 3;
    }
    else if(package.droneSwitch == 3){
      menuDrone[2] = "Switch 4   ";
      package.droneSwitch = 4;
    }
    else{
      menuDrone[2] = "Switch None";
      package.droneSwitch = 0;
    }
  }
  else if(!bEnter and doOnes[1]){
    doOnes[1] = 0;
  }
}

void gasMenu(){
  lcd.setCursor(CurserX, menuCursor%2);
  lcd.write(char(60));
  lcd.setCursor(0, 0);
  lcd.print(menuGas[int(menuCursor/2)*2]);
  lcd.setCursor(0, 1);
  lcd.print(menuGas[int(menuCursor/2)*2+1]);

  if(bEnter == 1 and menuCursor == 0){
    menuLoc[1] = 0;
    menuLoc[2] = 0;
    menuCursor = 3;
    bEnter = 0;
  }
  else if(bEnter and menuCursor == 1 and !doOnes[1]){
    doOnes[1] = 1;
    if(!package.Gas){
      menuGas[1] = "Alert On ";
      package.Gas = 1;
    }
    else{
      menuGas[1] = "Alert Off";
      package.Gas = 0;
    }
  }
  else if(bEnter and menuCursor == 2 and !doOnes[1]){
    doOnes[1] = 1;
    menuLoc[2] = 1;
    menuCursor = 0;
    bEnter = 0;
  }
  else if(!bEnter and doOnes[1]){
    doOnes[1] = 0;
  }
}

void gasVarMenu(){
  delay(1);
  if(menuCursor == 0){
    lcd.setCursor(7,0);
    lcd.write("       ");
    lcd.setCursor(0,0);
    lcd.write("H2 ");
    lcd.print(gasPackage.H2);
    lcd.setCursor(7,1);
    lcd.write("       ");
    lcd.setCursor(0,1);
    lcd.write("LPG ");
    lcd.print(gasPackage.LPG);
  }
  if(menuCursor == 1){
    lcd.setCursor(7,0);
    lcd.write("       ");
    lcd.setCursor(0,0);
    lcd.write("CH4 ");
    lcd.print(gasPackage.CH4);
    lcd.setCursor(7,1);
    lcd.write("       ");
    lcd.setCursor(0,1);
    lcd.write("CO ");
    lcd.print(gasPackage.CO);
  }
  if(menuCursor == 2){
    lcd.setCursor(7,0);
    lcd.write("       ");
    lcd.setCursor(0,0);
    lcd.write("Alchol ");
    lcd.setCursor(0,1);
    lcd.write("              ");
    lcd.setCursor(0,1);
    lcd.print(gasPackage.Alchol);
  }
  
  if(bEnter == 1){
    menuLoc[2] = 0;
    menuCursor = 2;
    bEnter = 0;
  }
}

void RestoreConfig(){
  package.restoreConfig = 1;
}

void setConfig(){
  package.joystickSmoothness = config.joystickSmoothness;
  package.path = config.path;
  package.stepH = config.stepH;
  package.gyroAssist = config.gyroAssist;
  package.forceAssist = config.forceAssist;
  package.lidarSwitch = config.lidarSwitch;
  package.droneSwitch = config.droneSwitch;
}

void gasAlert(){
  bool BCH4 = gasPackage.CH4 > 10000;
  bool BCO = gasPackage.CO > 50;
  if(BCO or BCH4){
    lcd.clear();
    delay(1);
    if(BCO){
      lcd.setCursor(0,0);
      lcd.print("CO Danger!!");
    }
    else if(BCH4){
      lcd.setCursor(0,0);
      lcd.print("CH4 Danger!!");
    }
    
  }
}

void setConfigDefault(){
  package.joystickSmoothness = 4;
  package.path = 45;
  package.stepH = 75;
  package.gyroAssist = 1;
  package.forceAssist = 1;
  package.lidarSwitch = 0;
  package.droneSwitch = 0;
}
