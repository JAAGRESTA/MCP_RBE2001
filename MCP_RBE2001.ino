#include <Servo.h>
#include <TimerOne.h>
#include <BluetoothClient.h>
#include <BluetoothMaster.h>
//all pins are temporary and need changing
#define leftDrivePin 24
#define rightDrivePin 23
#define fourBarPin 25
#define rackMotorPin 26
#define grabberServoPin 27
#define limitPin A0
#define lineSensePin1 A1
#define lineSensePin2 A2
#define lineSensePin3 A3
#define lineSensePin4 A4
#define potPin A5

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

#define delimiter_pos 5
#define length_pos 4
#define type_pos 3
#define source_pos 2
#define dest_pos 1
#define checksum_pos 0

#define storageTube_length 7
#define supplyTube_length 7
#define radAlert_length 7
#define stop_length 6
#define start_length 6
#define status_length 9
#define heartBeat_length 6

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
int XYcoords[] = {0,1};
int currentXYcoords[] = {0,1};
int armStatus = DOWN; 
long long myData;

Servo leftDrive;
Servo rightDrive;
Servo fourBarMotor;
Servo rackMotor;
Servo grabberServo;

enum armState{
	UP,
	DOWN
};

enum State{
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
blueState _blueState;
State state;

void setup(){
	leftDrive.attach(leftDrivePin,1000,2000);
	rightDrive.attach(rightDrivePin,1000,2000);
	fourBarMotor.attach(fourBarPin,1000,2000);
	rackMotor.attach(rackMotorPin,1000,2000);
	grabberServo.attach(grabberServoPin);
	Serial1.begin(115200)
	pinMode(lineSensePin1, INPUT);
	pinMode(lineSensePin2, INPUT);
	pinMode(lineSensePin3, INPUT);
	pinMode(lineSensePin4, INPUT);
	pinMode(potPin, INPUT);
	pinMode(limitPin, INPUT);
	state = findReactor;

	reactor = 'A'; //reactor type
	lineCount = 0; //counts the amount of lines
	lineFlag = 0; //flag for when line detected 

	Timer1.initialize(100000);
	Timer1.attachInterrupt(HundredMsISR);
	BluetoothClient blueClient = new BluetoothClient();
	BluetoothMaster blueMaster = new BluetoothMaster();
}

void loop(){
  
 	stateMachine();

}
//master state machine
void stateMachine(){
	switch (state) {
		case findStart: 
			followLine();
			turnRight90(); //make this
			followLine();
			approachReactor();
			state = grabSpent;
	    case findReactorA: 
	      if(reactor == 'A') //for reactor A
	      { 
	      	goXlines(1); //depends where we start	
	      	approachReactor(); //works because while loop wont end till limit hit
	      	state = grabSpent;	
	      }
	      if(reactor == 'B') //for reactor B
	      { 
	      	goXlines(4); //depends where we start	
	      	approachReactor(); //works because while loop wont end till limit hit
	      	state = grabSpent;	
	      }

	      break;
	    case grabSpent:
	   	  rackReverse();
	      releaseGrab();
	      setArmAngle(downPosition);
	      rackForward();
	      grab();
	      rackReverse();
	      state = findDisposal;
	      break;
	    case findDisposal:
	   	  	//stuff
	   		break;
	    case placeSpent:
	    	rackReverse();
	    	setArmAngle(upPosition);
	    	rackForward();
	    	releaseGrab();
	    	rackReverse();
	    	setArmAngle(downPosition);
	    	state = findSupply;
	    	break;
	    case findSupply:
	    	//stuff
	    	break;
	    case grabSupply:
	    	rackReverse();
	    	setArmAngle(upPosition);
	    	releaseGrab();
	    	rackForward();
	    	grab();
	    	rackReverse();
	    	break;
	    case returnToReactor:
	    	//stuff
	    	break;
	    case placeSupply:
	    	rackReverse();
	    	setArmAngle(downPosition);
	    	rackForward();
	    	releaseGrab();
	    	rackReverse();
	    	break;
	}

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
	fetchBluetooth();
	extractBluetooth();
	heartBeatCounter++;
	if(heartBeatCounter == 20)
	{
		doHeartBeat();
		heartBeatCounter = 0;
	}
}
void fetchBluetooth() {
	fetchBluetooth(&myData);
}
void extractBluetooth()
{
	noInterrupts();
	long long myDataTemp = myData;
	interrupts();
	if(checkValidity(myDataTemp)) {
		extractData();
	}

}
bool checkValidity(long long data) {
	bool isValid = checkByte(data, delimiter_pos, 0x5F) && checkByte(data, source_pos, 0x00) && checkByte(data, dest_pos, 0x0A) && checkLengthType(data) && checkCheckSum(data);
	return isValid;
}
bool checkLengthType(long long data) {
	bool result;
	int counter = 0;
	int lengthBits;
	for(i = data; i != 0; i >> 1) {
		counter++;
	}
	checkType(data);
	switch(_blueState) {
		case storageTube:
			result = (counter == storageTube_length*8);
			break;
		case supplyTube:
			result = (counter == supplyTube_length*8);
			break;
		case radAlert:
			result = (counter == radAlert_length*8);
			break;
		case stopMovement:
			result = (counter == stop_length*8);
			break;
		case startMovement:
			result = (counter == start_length*8);
			break;
		case robotStatus:
			result = (counter == status_length*8);
			break;
		case robotHeartbeat:
			result = (counter == heartBeat_length*8);
			break;
		case default:
			break;
	}
	return result;
}
void checkType(long long data) {
	char msgType = byteShift(data, 2);
	if(checkMsgType(msgType, 0x01)) {
		_blueState = storageTube;
	}
	else if(checkMsgType(msgType, 0x02)) {
		_blueState = supplyTube;
	}
	else if(checkMsgType(msgType, 0x03)) {
		_blueState = radAlert;
	}
	else if(checkMsgType(msgType, 0x04)) {
		_blueState = stopMovement;
	}
	else if(checkMsgType(msgType, 0x05)) {
		_blueState = startMovement;
	}
	else if(checkMsgType(msgType, 0x06)) {
		_blueState = robotStatus;
	}
	else if(checkMsgType(msgType, 0x06)) {
		_blueState = robotHeartbeat;
	}
}
bool checkMsgType(char msgType, char checkVal) {
	return checkByte((long long)msgType, 0, checkVal);
}
bool checkCheckSum(long long data) {
	//HOW DO CHECKSUM HALP
}
bool checkByte(long long data, int pos, char checkVal) {
	char dataByte = byteShift(data, pos);
	if((dataByte & checkVal) == 0xFF) {
		return true;
	}
	else {
		return false;
	}
}
long long extractData(long long data) { //still need to do this
	switch(_blueState) {
		case storageTube:
			break;
		case supplyTube:
			break;
		case radAlert:
			break;
		case stopMovement:
			break;
		case startMovement:
			break;
		case robotStatus:
			break;
		case robotHeartbeat:
			break;
		case default:
	}
}
char byteShift(long long inBytes, int amt) {
	amtFinal = amt*8;
	char outBit = (inBytes >> amtFinal) & 0x0F;
	return outBit;
}
//method to do line tracking until the robot drives over a line
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
 	measError = analogRead(lineSensePin1) - analogRead(lineSensePin3);
	error = map(measError, -1023, 1023, -90, 90);
}

//method to see if the far line sensor and at least one other line sensor on on a line or not indicating a cross
//returns true if a cross is hit, false otherwise 
boolean crossHit(int lines) 
{
	if(overLine(lineSensePin4) && (overLine(lineSensePin1) || overLine(lineSensePin2) || overLine(lineSensePin3))) 
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
				goXlines(i);
				turnRight90();
				approachReactor();
				break;
			}
		}
	}
}
// uses Bluetooth signal to check if disposal x is available
void disposalEmpty(int x)
{

}