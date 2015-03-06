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
#define buttonIn 2

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

ReactorProtocol myRobot(0x0A);
BluetoothClient myClient;
BluetoothMaster myMaster;
char message[10];

void setup() {
  Serial1.begin(115200);
  Serial.begin(9600);
  Timer1.initialize(20000);
  Timer1.attachInterrupt(hundredMsISR);
  pinMode(buttonIn, INPUT);
}
void loop() {
  if(digitalRead(buttonIn) == HIGH) {
    beginOperations = true;
  }
  if (myMaster.readPacket(myData)) {
    if (myRobot.getData(myData, data, type)) {
      parseData(data, type);
    }
    Serial.println("-----------NEW PACKET-----------");
  }
    for (int i = 0; i < 4; i++) {
      Serial.println(data[i]);
    }
}
void hundredMsISR() {
  heartbeatCounter++;
  byte *radData;
  if (heartbeatCounter >= 25 && (beginOperations == true)) {
    doHeartbeat();
    if (radFlag != 0) {
      if (radFlag == 1) {
        *radData = 0x2C;
      }
      else if (radFlag == 2) {
        *radData = 0xFF;
      }
      sendRadAlert(radData);
      sendStatus(robotStatus[0], robotStatus[1], robotStatus[2]);
      for(int i=0;i<3;i++) {
        Serial.println(data[i]);
      }
    }
    heartbeatCounter = 0;
  }
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
