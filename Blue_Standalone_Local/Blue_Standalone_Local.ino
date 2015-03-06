#include <Servo.h>
#include <TimerOne.h>
#include <BluetoothClient.h>
#include <BluetoothMaster.h>
#include <ReactorProtocol.h>
#include <stdio.h>

#define radID 0x03
#define statusID 0x06
#define heartbeatID 0x07
#define fieldID 0x00
#define teamID 0x0A
#define newRodID 0x2C
#define spentRodID 0xFF

#define moveStatus_stopped 0x01
#define moveStatus_moving 0x03

#define gripStatus_noRod 0x01
#define gripStatus_hasRod 0x02

#define opStatus_gripAttempt 0x01
#define opStatus_gripRelease 0x02
#define opStatus_toReactor 0x03
#define opStatus_toStorage 0x04
#define opStatus_toSupply 0x05
#define opStatus_idle 0x06

#define buttonIn 2
#define readyLight 3
/*
Storage and Supply LED pin lines need to be wired up to Arduino
*/
#define STORAGE_LED_0 4
#define STORAGE_LED_1 5
#define STORAGE_LED_2 6
#define STORAGE_LED_3 7
#define SUPPLY_LED_0 8
#define SUPPLY_LED_1 9
#define SUPPLY_LED_2 10
#define SUPPLY_LED_3 11

byte myData[10];
byte data[3];
byte packet[10];
byte type;
byte storageTubes;
byte supplyTubes;
byte robotStatus[3];
int heartbeatCounter = 0;
int radFlag = 0;
bool beginOperations = false;
int testingTimer = 0;
int demoTimer = 0;
bool _doHeartbeat = false;  //disable heartbeat on field for now


ReactorProtocol myRobot(0x0A);
BluetoothClient myClient;
BluetoothMaster myMaster;
char message[10];

enum masterState {
  testingState,
  demoState
};

masterState _masterState;

void setup() {
  Serial1.begin(115200);
  Serial.begin(9600);
  Timer1.initialize(20000);
  Timer1.attachInterrupt(hundredMsISR);
  _masterState = testingState; //Set the master mode to testing for now
  pinMode(buttonIn, INPUT);
  pinMode(readyLight, OUTPUT);
  pinMode(STORAGE_LED_0, OUTPUT);
  pinMode(STORAGE_LED_1, OUTPUT);
  pinMode(STORAGE_LED_2, OUTPUT);
  pinMode(STORAGE_LED_3, OUTPUT);
  pinMode(SUPPLY_LED_0, OUTPUT);
  pinMode(SUPPLY_LED_1, OUTPUT);
  pinMode(SUPPLY_LED_2, OUTPUT);
  pinMode(SUPPLY_LED_3, OUTPUT);
  bool _doHeartbeat = false;
  while(!beginOperations) {
    if(digitalRead(buttonIn) == HIGH) {
      beginOperations = true;
    }
  }
}

void loop() {
  switch(_masterState) {
    case testingState:
      testingLoop();
      break;
    case demoState:
      demoLoop();
      break;
  }
}
void hundredMsISR() {
  switch(_masterState) {
    case testingState:
      testingISR();
      break;
    case demoState:
      demoISR();
      break;

  }
}

void testingLoop() {
  if (myMaster.readPacket(myData) && (testingTimer >= 6)) {
    testingTimer = 0;
    if (myRobot.getData(myData, data, type)) {
      parseData(data, type);
    }
    Serial.println("-----------NEW PACKET-----------");
  }
  else {
    Serial.println("-------NO NEW PACKET-------");
  }
  Serial.println("---------DATA CONTENTS--------");
  for (int i = 0; i < 4; i++) {
    Serial.print(data[i]);
    Serial.print(", ");
  }
  Serial.println("-------BITMASK CONTENTS----------");
  Serial.println("SUPPLY BITMASK: ");
  Serial.print(String(supplyTubes, BIN));
  Serial.println("STORAGE BITMASK: ");
  Serial.print(String(storageTubes, BIN));
  Serial.println("-------------------------------");
  if((heartbeatCounter >= 5) && _doHeartbeat) {
    doHeartbeat();
    heartbeatCounter = 0;
  }
}
void demoLoop() {
  demoTimer++;
}
void testingISR() {
  testingTimer++;
  heartbeatCounter++;
}
void demoISR() {
  
}
void doHeartbeat() {
  byte packet[10];
  int sz;
  byte data1[3];
  myRobot.setDst(0x00);
  sz = myRobot.createPkt(heartbeatID, data1, packet);
  myMaster.sendPkt(packet, sz);
}
void parseData(byte *data, byte type) {
  switch (type) {
    case 1:
      storageTubes = data[0];
      break;
    case 2:
      supplyTubes = data[0];
      break;
    case 3:
      break;
    case 4:
      stopRobot();
      break;
    case 5:
      resumeRobot();
      break;
    case 6:
      break;
    case 7:
      break;
  }
}
void sendRadAlert(byte *alertType) {
  byte packet[10];
  int sz;
  myRobot.setDst(0x00);
  sz = myRobot.createPkt(radID, alertType, packet);
  myMaster.sendPkt(packet, sz);
}
void sendStatus(byte movementStatus, byte gripperStatus, byte operationStatus) {
  byte packet[10];
  byte data[3];
  data[0] = movementStatus;
  data[1] = gripperStatus;
  data[2] = operationStatus;
  int sz;
  myRobot.setDst(0x00);
  sz = myRobot.createPkt(statusID, data, packet);
  myMaster.sendPkt(packet, sz);
}
void stopRobot() {
  //HOW STOP ROBIT?
}
void resumeRobot() {
  //IS ALSO NOT KNOW START ROBIT
}
