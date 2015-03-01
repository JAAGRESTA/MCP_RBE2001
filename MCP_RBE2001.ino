#include <Servo.h>
#include <TimerOne.h>
#include <BluetoothClient.h>
#include <BluetoothMaster.h>
//all pins are temporary and need changing
#define leftDrivePin 26
#define rightDrivePin 27
#define fourBarPin 25
#define rackMotorPin 26 //not a thing at the moment
#define grabberServoPin 27
#define limitPin A0
#define lineSenseRight A1
#define lineSenseLeft A2
#define lineSenseCenter A3
#define lineSenseFarLeft A4
#define potPin A5
#define encoderRight A6
#define encoderLeft A7 //not installed yet
#define buttonIntPin 2

#define stopSpeed 90
#define potRange 180
#define leftFWD 180
#define rightFWD 0
#define leftBWD 0
#define rightBWD 180
#define baseSpeedLeft 130
#define baseSpeedRight 50
#define grabberClosed 180
#define grabberOpen 0
#define rackMoveTime 50
#define upPosition 90 
#define downPosition 0

#define lengthPos 1
#define typePos 2
#define dataPos 5
#define delimiterPos 0
#define msgSourcePos 3
#define msgDestPos 4
#define checkSumOffset 1

#define storageID 0x01
#define supplyID 0x02
#define radID 0x03
#define stopID 0x04
#define resumeID 0x05
#define statusID 0x06
#define heartBeatID 0x07

char reactor; //reactor type
int lineCount; 
int lineFlag; //flag when line detected
int lightThreshold = 600; //threshold for if a line sensor is on light or dark, above threshold = dark
int measError; //difference in line tracker sensor values
int error;
float leftSpeed, rightSpeed, speedGain = 0.55; 
int heartBeatCounter = 0;
int potAngle;
int angleError = 0, prevAngleError = 0, deltaAngleError = 0, sumAngleError =0, slowTime = 0;
float adjustedSpeed, pGain, iGain, dGain;
float slowTimeGain = 0.75;
int XYcoords[2] = {0,1};
int currentXYcoords[2] = {0,1};


byte myData;
byte myDataArray[10];
byte data[3];
int blueToothCounter = 0;
bool receivingData = false;
bool supplyTubes[4];
bool storageTubes[4];

Servo leftDrive;
Servo rightDrive;
Servo fourBarMotor;
Servo rackMotor;
Servo grabberServo;

enum armState{ //needs use
	UP,
	DOWN
};

enum State{
	start,
	TESTING,
	findStart,
	findReactorB,
	grabSpent,
	findDisposal,
	placeSpent,
	findSupply,
	grabSupply,
	returnToReactorB,
	placeSupply, 
	findReactorA,
	returnToReactorA,
	idle
};

enum blueState{
	storageTube,
	supplyTube,
	radAlert,
	stopMovement,
	startMovement,
	robotStatus,
	robotHeartbeat
};
armState armStatus;
blueState _blueState;
State state;

void setup(){
	leftDrive.attach(leftDrivePin,1000,2000);
	rightDrive.attach(rightDrivePin,1000,2000);
	fourBarMotor.attach(fourBarPin,1000,2000);
	rackMotor.attach(rackMotorPin,1000,2000);
	grabberServo.attach(grabberServoPin);

	Serial1.begin(115200);
	pinMode(buttonIntPin, INPUT_PULLUP);
	attachInterrupt(0, resetISR, CHANGE);
	pinMode(lineSenseRight, INPUT);
	pinMode(lineSenseLeft, INPUT);
	pinMode(lineSenseCenter, INPUT);
	pinMode(lineSenseFarLeft, INPUT);
	pinMode(potPin, INPUT);
	pinMode(limitPin, INPUT);
	state = start;
	armStatus = DOWN;

	reactor = 'A'; //reactor type
	lineCount = 0; //counts the amount of lines
	lineFlag = 0; //flag for when line detected 

	Timer1.initialize(100000);
	Timer1.attachInterrupt(HundredMsISR);
	BluetoothClient myClient = new BluetoothClient();
	BluetoothMaster myMaster = new BluetoothMaster();
}

//method to test stand-alone modules of code for individual testing
void runTest()
{
	//insert test code here
}

//when the button is pushed, stop or resume robot operation
void resetISR()
{
	if(state == start)
	{
		state = TESTING;
	} 
	else
	{
		state = idle;
	}
}

void loop(){
  	if(myClient.receive(&myData)) {
		if(receivingData == false && myData == delimiter) {
			receivingData = true;
			blueToothCounter = 0;
		}
		if(receivingData == true) {
			parseNewByte(myData);
		}
	}
 	stateMachine();

}
//master state machine
 void stateMachine(){
// while(state != idle)
// {
// 	switch (state) {
// 		case TESTING:
// 			runTest();
// 		case findStart: 
// 			followLine();
// 			turnRight90(); //make this
// 			followLine();
// 			approachReactor();
// 			state = grabSpent;
// 	    case findReactorA: 
// 	      if(reactor == 'A') //for reactor A
// 	      { 
// 	      	goXlines(1); //depends where we start	
// 	      	approachReactor(); //works because while loop wont end till limit hit
// 	      	state = grabSpent;	
// 	      }
// 	      if(reactor == 'B') //for reactor B
// 	      { 
// 	      	goXlines(4); //depends where we start	
// 	      	approachReactor(); //works because while loop wont end till limit hit
// 	      	state = grabSpent;	
// 	      }

// 	      break;
// 	    case grabSpent:
// 	   	  rackReverse();
// 	      releaseGrab();
// 	      setArmAngle(downPosition);
// 	      rackForward();
// 	      grab();
// 	      rackReverse();
// 	      state = findDisposal;
// 	      break;
// 	    case findDisposal:
// 	   	  	//stuff
// 	   		break;
// 	    case placeSpent:
// 	    	rackReverse();
// 	    	setArmAngle(upPosition);
// 	    	rackForward();
// 	    	releaseGrab();
// 	    	rackReverse();
// 	    	setArmAngle(downPosition);
// 	    	state = findSupply;
// 	    	break;
// 	    case findSupply:
// 	    	//stuff
// 	    	break;
// 	    case grabSupply:
// 	    	rackReverse();
// 	    	setArmAngle(upPosition);
// 	    	releaseGrab();
// 	    	rackForward();
// 	    	grab();
// 	    	rackReverse();
// 	    	break;
// 	    case returnToReactor:
// 	    	//stuff
// 	    	break;
// 	    case placeSupply:
// 	    	rackReverse();
// 	    	setArmAngle(downPosition);
// 	    	rackForward();
// 	    	releaseGrab();
// 	    	rackReverse();
// 	    	break;
// 	    case idle:
// 	    	stop();
// 	    	break;
// 	}
// }
 }

//returns boolean if a certain amount of lines have been hit
boolean lineHit(int x){
	if(lineCount == x)
	{
		lineCount = 0;
		return true;
	}else
	{
		lineCount = 0;
		return false;
	}
}

//drive forward
void forward()
{
	leftDrive.write(leftFWD);
	rightDrive.write(rightFWD);
}

//stop both motors
void stop()
{
	leftDrive.write(stopSpeed);
	rightDrive.write(stopSpeed);
}

 void HundredMsISR() 
 {
 	heartBeatCounter++;
 	if(heartBeatCounter == 5) //every .5 seconds
 	{
 		sendHeartBeat();
 		heartBeatCounter = 0;
 	}
 }
 void parseNewByte(byte data) {
	myDataArray[blueToothCounter] = data;
	if(blueToothCounter < 10) {
		blueToothCounter++;
	}
	else {
		parseNewPacket();
	}
}
void parseNewPacket() {
	int dataLength = myDataArray[lengthPos];
	for(i=0; i < dataLength; i++) {
		data[i] = myDataArray[dataPos + i];
	}
	bool validPacket = checkDelimiter(myDataArray[delimiterPos]);
	bool validSource = checkSource(myDataArray[msgSourcePos]);
	bool validDest = checkDest(myDataArray[msgDestPos]);
	bool validCheckSum = checkSum(myDataArray[checkSumPos + dataLength], dataLength);
	String dataType = checkType(myDataArray[typePos]);
	if(validPacket && validSource && validDest && validCheckSum) {
		executeData(dataType, dataLength);
	}
}
bool checkDelimiter(byte delimiterByte) {
	return delimiterByte = delimiterID;
}
bool checkSource(byte sourceByte) {
	return sourceByte = fieldID;
}
bool checkDest(byte destByte) {
	return destByte = teamID;
}
bool checkSum(byte checkSumByte) {
	return true; //how do you checksum?
}
String checkType(byte typeByte) {
	String type;
	if(typeByte == storageID) {
		type = "storage";
	}
	else if(typeByte == supplyID) {
		type = "supply";
	}
	else if(typeByte == radID) {
		type = "rad";
	}
	else if(typeByte == stopID) {
		type = "stop";
	}
	else if(typeByte == resumeID) {
		type = "resume";
	}
	else if(typeByte == statusID) {
		type = "status";
	}
	else if(typeByte == heartBeatID) {
		type = "heartbeat";
	}
}
void executeData(String dataType, int dataLength) {
	byte data[dataLength];
	if(dataType == "storage") {
		parseStorage(myDataArray[dataPos]);
	}
	else if(dataType == "supply") {
		parseSupply(myDataArrat[dataPos]);
	}
	else if(dataType == "stop") {
		stopMovement();
	}
	else if(dataType == "resume") {
		resumeMovement();
	}
}
void sendHeartBeat() {
	sendMessage(heartBeat, heartBeatID);
}
void sendRadAlert(bool alertType) {
	sendMessage(radAlert, radAlertID);
}
void sendStatus(byte movementStatus, byte gripperStatus, byte operationStatus) {
	byte dataFinal[9];
	dataFinal[0] = delimiter;
	dataFinal[1] = 0x09;
	dataFinal[2] = 0x06;
	dataFinal[3] = teamID;
	dataFinal[4] = fieldID;
	dataFinal[5] = movementStatus;
	dataFinal[6] = gripperStatus;
	dataFinal[7] = operationStatus;
	for(i=0; i<9; i++) {
		myMaster.send(dataFinal[i]);
	}
}
void sendMessage(byte data, byte dataType) {
	byte dataFinal[6];
	dataFinal[0] = delimiter;
	dataFinal[1] = 0x06;
	dataFinal[2] = dataType;
	dataFinal[3] = teamID;
	dataFinal[4] = fieldID;
	dataFinal[5] = data;
	dataFinal[6] = getCheckSum(data);
	for(i=0; i<6; i++) {
		myMaster.send(dataFinal[i]);
	}
}
void parseSupply(byte bitMask) {
	compareByte = 0x01;
	for(i=0; i<4; i++) {
		supplyTubes[i] = ((bitMask & compareByte) == 0xFF);
		compareByte << 1;
	}
}
void parseStorage(byte bitMask) {
	compareByte = 0x01;
	for(i=0; i<4; i++) {
		storageTubes[i] = ((bitMask & compareByte) == 0xFF);
		compareByte << 1;
	}
}
// //method to do line tracking until the robot drives over a line
void followLine() 
{			
	while(crossHit() != true)
	{
		getError();
		leftSpeed = baseSpeedLeft + ((float) error*speedGain);
		rightSpeed = baseSpeedRight + ((float) error*speedGain);
		leftDrive.write(leftSpeed);
		rightDrive.write(leftSpeed);
		if(armStatus == DOWN)
		{
			setArmAngle(downPosition);
		} 
		else if(armStatus == UP)
		{
			setArmAngle(upPosition);
		}
	}
}

void goXlines(int lineNum)
{
	while(lineCount != lineNum)
	{
		followLine();
	}
	lineCount = 0;
}

//finds difference in line sensor values, sets that value to a useable motor speed
void getError()
{
 	measError = analogRead(lineSenseRight) - analogRead(lineSenseLeft);
	error = map(measError, -1023, 1023, -90, 90);
}

//method to see if the far line sensor and at least one other line sensor on on a line or not indicating a cross
//returns true if a cross is hit, false otherwise 
boolean crossHit() 
{
	if(overLine(lineSenseFarLeft) && (overLine(lineSenseRight) || overLine(lineSenseLeft) || overLine(lineSenseCenter))) 
  	{
  		lineCount++;
  		return true;
  	} 
  	else 
  	{
  		
  		return false;
  	}
}

//method to check if a line sensor is on the line or not
//returns true if it's on dark, false otherwise
boolean overLine(int lineSensorPin) 
{
	if (analogRead(lineSensorPin) >= lightThreshold)
	{
		return true;
	}
	else
	{
		return false;
	}
}

//method to check if limit switch is pressed
//returns true if the switch is pressed, false otherwise
//assumes active low
boolean reactorHit()
{
	if(digitalRead(limitPin) == LOW)
	{
		return true;
	}
	else
	{
		return false;
	}
}

//checks the potentiometer position and converts it to an angle
//returns the potentiometer angle value in degrees
int getPotAngle()
{
	potAngle = map(analogRead(potPin), 0, 1023, 0, potRange);
	return potAngle;
}

//sets four-bar to a given desired angle with PID control
void setArmAngle(int desiredAngle)
{
		prevAngleError = angleError; 
		angleError = desiredAngle - getPotAngle();
		deltaAngleError = prevAngleError - angleError;
		sumAngleError = angleError + prevAngleError;
		adjustedSpeed = angleError*pGain + deltaAngleError*dGain + sumAngleError*iGain;
		fourBarMotor.write(90 + adjustedSpeed);
}

//closes claw
void grab()
{
	grabberServo.write(grabberClosed);
}

//opens claw
void releaseGrab()
{
	grabberServo.write(grabberOpen);
}

//moves rack and pinion system to forward position, stops
void rackForward()
{
	rackMotor.write(150);
	delay(rackMoveTime);
	rackMotor.write(90);
}

//moves rack and pinion system to backward position, stops
void rackReverse()
{
	rackMotor.write(30);
	delay(rackMoveTime);
	rackMotor.write(90);
}

//allows us to approach the reactor until limit switch hit
//written the same as followLine method
//optional slowing approach commented out
void approachReactor()
{
	while(!reactorHit())
	{
	getError();
	leftSpeed = baseSpeedLeft + ((float) error*speedGain); // - (slowTime * slowTimeGain);
	rightSpeed = baseSpeedRight + ((float) error*speedGain); // -(slowTime * slowTimeGain);
	leftDrive.write(leftSpeed);
	rightDrive.write(leftSpeed);
	//slowTime++;
	}
}
//navigates to a desired reactor from any point on the course
//make sure when starting robot to be in initialized XYcoords position
void navigateToReactorCoord()
{
	int x;
	int y;
	y = XYcoords[1] - currentXYcoords[1];
	if(y != 0)
	{
		turn180(); //not written
		goXlines(1);
	}
	x = XYcoords[0] - currentXYcoords[0];
	if((x != 0) && (y != 0))
	{
		if( (x > 0) && (y < 0))
		{
			turnRight90(); //not written
			goXlines(abs(x-1));
			approachReactor();

		}else if((x > 0) && (y > 0))
		{
			turnLeft90(); //not written
			goXlines(abs(x-1));
			approachReactor();
		}else if((x < 0) && (y < 0))
		{
			turnLeft90(); //not written
			goXlines(abs(x-1));
			approachReactor();
		}else if((x < 0) && (y > 0))
		{
			turnRight90(); //not written
			goXlines(abs(x-1));
			approachReactor();
		}

	}else if((x != 0) && (y = 0)) //assumes we are facing the right direction, which should be the case in this case
	{
		goXlines(abs(x-1));
		approachReactor();
	}

}
//navigates to closest disposal based off of which reactor it is starting from
void navigateToDisposal()
{
	int i;
	turn180();
	if(reactor == 'A')
	{
		for(i=1; i<5; i++)
		{
			if(disposalEmpty(i))
			{
				goXlines(i);
				turnLeft90();
				approachReactor();
				currentXYcoords[0] = i;
				currentXYcoords[1] = 2;
				break;

			}
		}
	}
	else if(reactor == 'B')
	{
		for(i=4; i>0; i--)
		{
			if(disposalEmpty(i))
			{
				goXlines((5-i));
				turnRight90();
				approachReactor();
				currentXYcoords[0] = i;
				currentXYcoords[1] = 2;
				break;
			}
		}
	}
}
// uses Bluetooth signal to check if disposal x is available returns true if disposal is empty
// param: int x where x is the station number
boolean disposalEmpty(int x)
{

}
// navigates to Supply based off availablity of supply and closest coordinates
void navigateToSupply()
{
	int i;
	int x;
	x = currentXYcoords[0];
	turn180();

	if(reactor == 'A')
	{
		for(i=1; i<5; i++)
			{
				if(supplyFull(i))
				{
					XYcoords[0] = i; 
					XYcoords[1] = 0;
					break;
				}
				 
			}
		if(x == XYcoords[0])
		{

			goXlines(1);
			approachReactor();
			currentXYcoords[0] = XYcoords[0];
			currentXYcoords[1] = 0;

		}
		else if(x > XYcoords[0])
		{
			int j;
			j = x - XYcoords[0];
		    goXlines(1);
		    turnRight90();
		    goXlines(j);
		    turnLeft90();
		    approachReactor();
		    currentXYcoords[0] = XYcoords[0];
			currentXYcoords[1] = 0;
		}
		else
		{
			int j;
			j = XYcoords[0] - x;
			goXlines(1);
			turnLeft90();
			goXlines(j);
			turnRight90();
			approachReactor();
			currentXYcoords[0] = XYcoords[0];
			currentXYcoords[1] = 0;
		}

	}
	else if(reactor == 'B')
	{
		for(i=4; i>0; i--)
			{
				if(supplyFull(i))
				{
					XYcoords[0] = i; 
					XYcoords[1] = 0;
					break;
				}
				 
			}
		if(x == XYcoords[0])
		{

			goXlines(1);
			approachReactor();
			currentXYcoords[0] = XYcoords[0];
			currentXYcoords[1] = 0;

		}
		else if(x > XYcoords[0])
		{
			int j;
			j = x - XYcoords[0];
		    goXlines(1);
		    turnRight90();
		    goXlines(j);
		    turnLeft90();
		    approachReactor();
		    currentXYcoords[0] = XYcoords[0];
			currentXYcoords[1] = 0;
		}
		else
		{
			int j;
			j = XYcoords[0] - x;
			goXlines(1);
			turnLeft90();
			goXlines(j);
			turnRight90();
			approachReactor();
			currentXYcoords[0] = XYcoords[0];
			currentXYcoords[1] = 0;
		}

	}
}
// uses Bluetooth to check availablity of a given supply station and returns true if station is full
// param: int x where x is the given station number
boolean supplyFull(int x)
{

}
void turnLeft90()
{

}
void turnRight90()
{

}
void turn180()
{

}