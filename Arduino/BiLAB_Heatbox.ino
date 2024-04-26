#include <dimmable_light.h>
#include "DHT.h"
#include <PID_v1.h>
#define zcPIN 2
#define psmPIN 3
#define DHT22_1_PIN 12
#define DHT22_2_PIN 10
#define DHT22_3_PIN 11
#define DHT22_4_PIN 8
#define fanPIN_1 4
#define fanPIN_2 7


DHT dht22_1(DHT22_1_PIN,DHT22);
DHT dht22_2(DHT22_2_PIN,DHT22);
DHT dht22_3(DHT22_3_PIN,DHT22);
DHT dht22_4(DHT22_4_PIN,DHT22);
DimmableLight light(psmPIN);

//# constants
const double tempThresh = 38; 
const double tempMargin = 0.5;
const unsigned long refreshTime = 2000; //milliseconds
const int dPower_LOW = 50;  //0-255 value
const int dPower_MIDLOW = 100;
const int dPower_MIDHIGH = 150;
const int dPower_HIGH = 200;
const int dPower_MaxP = 160;
const double AMBIENT_TEMP = 24;
double startDiffTemp = 2.5;
const int fansOutput = HIGH;

//#  changing variables
int r = LOW;
uint8_t count = 0;
double dht22_1_t = 0;
double dht22_2_t = 0;
double dht22_3_t = 0;
double dht22_4_t = 0;
unsigned long time = 0;
unsigned long _lastreadtime = 0;
int dPower = 0;
double dPowerdb = 0; //for PID function, needs double type
bool start = true; //usually "true" for max power at start
bool stabilizeP = false;
double stabilizePtemp = tempThresh - startDiffTemp;
int FANS = LOW;
int chosenSensor = 1; // [1 = DHT22_1, 2 = DHT22_2]
double TempSensor = dht22_1_t; //needs to be changed to specified sensor
bool readSuccess_dht22_1_t = true;
bool readSuccess_dht22_2_t = true;
bool readSuccess_dht22_3_t = true;
bool readSuccess_dht22_4_t = true;
// bool *readSuccess = &readSuccess_dht22_1_t; //pointer to chosen sensor readsuccess value


int succCnt = 0; //when it reaches 5 all above values are non-zero and derivates can be accurately calculated
const int stabilizeSlopeTime = 32; //seconds between slope maximum same values
const int stabLoops = stabilizeSlopeTime*1000/refreshTime;
double pTempArray[stabLoops];
double prevStabRefTemp;

//PID parameters
double Kp = 55.0;
double Ki = 0.2;
double Kd = 29.1;
PID myPID(&TempSensor, &dPowerdb, &tempThresh, Kp, Ki, Kd, DIRECT);

void setup(){

  Serial.begin(115200);
  Serial.println();
  Serial.print("Initializing DimmableLight library... ");
  DimmableLight::setSyncPin(zcPIN);
  DimmableLight::begin();
  Serial.println("Done!");
  Serial.println("Initialize temperature sensors...");
  dht22_1.begin();
  dht22_2.begin();
  dht22_3.begin();
  dht22_4.begin();
  Serial.println("Done!");
  Serial.print("DHT22 Heat Dim Test: Temperature setpoint at\t");
  Serial.print(tempThresh);
  Serial.print("\t");
  Serial.print("degrees");
  Serial.println();
  Serial.print("pid values:\t");
  Serial.print(Kp);
  Serial.print("\t");
  Serial.print(Ki);
  Serial.print("\t");
  Serial.print(Kd);
  Serial.print("\t");
  Serial.println();
  Serial.print("Values for python script. Ambient Temperature:\t");
  Serial.print(AMBIENT_TEMP);
  Serial.print("\t");
  Serial.print("Stabilize Temperature Difference:");
  Serial.print("\t");
  Serial.print(startDiffTemp);
  Serial.println();
  Serial.println("Wait a few seconds for text to start updating...");
  
  pinMode(fanPIN_1,OUTPUT);
  pinMode(fanPIN_2,OUTPUT);
  digitalWrite(fanPIN_1,fansOutput);
  digitalWrite(fanPIN_2,fansOutput);
  myPID.SetSampleTime(refreshTime);
  myPID.SetOutputLimits(0, dPower_MaxP);

  delay(refreshTime);
  readTempData(); // call once to avoid first scan error in loop
  //turn the PID on
  //myPID.SetMode(MANUAL);
  myPID.SetMode(AUTOMATIC);
  
}

void loop()
{
  //Limit refresh rate, 2 sec from DHT22 datasheet
  if (millis() - _lastreadtime >= refreshTime){
    readTempData();

    if(stabilizeP){
      dPowerdb = 0;
      if (pTempArray[stabLoops-1] <= pTempArray[0]){
        stabilizeP = false;
      }
    }
    
    if(start){
      if(tempThresh - TempSensor > startDiffTemp){
        startingPhase();
      }
      else{
        start = false;
        stabilizeP = true;
      }
    }

    if(!start && !stabilizeP){
      myPID.Compute();
    }
  
    dPower = dPowerdb;
    light.setBrightness(dPower);
    printText();
  }
}

void startingPhase(){
  dPowerdb = dPower_MaxP;
  FANS = fansOutput;
}

void dPowerLogic(){
  if(TempSensor >= tempThresh + tempMargin){
   dPower = 0;
   FANS = HIGH;
  }
  else if(TempSensor <= tempThresh - tempMargin){
   dPower = dPower_MIDHIGH;
   FANS = HIGH;
  }
  else{
   dPower = dPower_LOW;
   FANS = LOW;
  }
}

void setFans(){
  digitalWrite(fanPIN_1, FANS);
  digitalWrite(fanPIN_2, FANS);
}

void readTempData() {
  
      //DHT22_1
     dht22_1_t = dht22_1.readTemperature();
     if (isnan(dht22_1_t)) {
        readSuccess_dht22_1_t = false;
        Serial.println(F("Failed to read from DHT22_1 sensor!!"));
      }

      //DHT22_2
     dht22_2_t = dht22_2.readTemperature();
     if (isnan(dht22_2_t)) {
        readSuccess_dht22_2_t = false;
        Serial.println(F("Failed to read from DHT22_2 sensor!!"));
      }

      //DHT22_3
     dht22_3_t = dht22_3.readTemperature();
     if (isnan(dht22_3_t)) {
        readSuccess_dht22_3_t = false;
        Serial.println(F("Failed to read from DHT22_3 sensor!!"));
      }

      //DHT22_4
     dht22_4_t = dht22_4.readTemperature();
     if (isnan(dht22_4_t)) {
        readSuccess_dht22_4_t = false;
        Serial.println(F("Failed to read from DHT22_4 sensor!!"));
      }

    TempSensor = dht22_1_t; //needs to be changed for specified sensor
     if (readSuccess_dht22_1_t){ //needs to be changed for specified sensor
      for (int i = 0; i < stabLoops-1; i++){
        pTempArray[i] = pTempArray[i+1];
       }
       pTempArray[stabLoops-1] = TempSensor;
    }
      readSuccess_dht22_1_t = true;
      readSuccess_dht22_2_t = true;
      readSuccess_dht22_3_t = true;
      readSuccess_dht22_4_t = true;


      //time 
      time = millis();
      _lastreadtime = time; 
}

void printText(){

    //Display values
    Serial.print("Dim Power:\t");
    Serial.print(dPower);
    Serial.print("\t");
    Serial.print("DHT22_1:\t");
    Serial.print(dht22_1_t, 1);
    Serial.print("\t");
    Serial.print("DHT22_2:\t");
    Serial.print(dht22_2_t, 1);
    Serial.print("\t");
    Serial.print("DHT22_3:\t");
    Serial.print(dht22_3_t, 1);
    Serial.print("\t");
    Serial.print("DHT22_4:\t");
    Serial.print(dht22_4_t, 1);
    Serial.print("\t");
    Serial.print("Time (ms):\t");
    Serial.print(time);
    Serial.print("\t");
    Serial.print(tempThresh);
    Serial.print("\t");
    Serial.print("Chosen sensor:\t"); 
    Serial.print(chosenSensor); 
    Serial.print("\t");
    Serial.print(stabilizeP);
    Serial.println();
  }