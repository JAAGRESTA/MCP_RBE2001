#include <Servo.h>
#include <TimerOne.h>
#include <BluetoothClient.h>
#include <BluetoothMaster.h>
//#include "ReactorProtocol.h"

//all pins are temporary and need changing
#define leftDrivePin 10
#define rightDrivePin 11
#define fourBarPin 4
#define rackMotorPin 26 //not a thing at the moment
#define grabberServoPin 22
#define limitPin A0
#define lineSenseRight A2
#define lineSenseLeft A1
#define lineSenseCenter A3
#define lineSenseFarLeft A4
#define potPin A5
#define encoderRight 19
#define encoderLeft 3 
#define buttonIntPin 2
#define flipperPin 5
#define radLED 27

#define stopSpeed 90
#define potRange 180
#define leftFWD 70
#define rightFWD 110
#define leftBWD 110
#define rightBWD 70
#define baseSpeedLeft 70
#define baseSpeedRight 110
#define grabberClosed 180
#define grabberOpen 0
#define rackMoveTime 50
#define upPosition 150 
#define downPosition 140

#define lengthPos 1
#define typePos 2
#define dataPos 5
#define delimiterPos 0
#define msgSourcePos 3
#define msgDestPos 4
#define checkSumPos 1

#define storageID 0x01
#define supplyID 0x02
#define radID 0x03
#define stopID 0x04
#define resumeID 0x05
#define statusID 0x06
#define heartbeatID 0x07
#define fieldID 0x00
#define teamID 0x0A
#define newRodID 0x2C
#define spentRodID 0xFF

#define delimiter 0x2C

char reactor; //reactor type
int lineCount; 
int lineFlag; //flag when line detected
int lightThreshold = 600; //threshold for if a line sensor is on light or dark, above threshold = dark
int measError; //difference in line tracker sensor values
int error;
float leftSpeed, rightSpeed, speedGain = 0.55; 
unsigned long int heartBeatCounter = 0;
int potAngle;
int angleError = 0, prevAngleError = 0, deltaAngleError = 0, sumAngleError =0, slowTime = 0;
float adjustedSpeed = 90, pGain= 4, iGain=0, dGain=0;
float slowTimeGain = 0.75;
int XYcoords[2] = {5,1};
int currentXYcoords[2] = {3,1};
int encoderLeftCount = 0, encoderRightCount = 0, encoderLeftCurrentCount, encoderRightCurrentCount;
float turn90Threshold = 95, turn180Threshold = 200, forwardThreshold = 70.35, backwardThreshold = 70.35; //turn needs tuning or calculating
int hundredMsFlag = 0, twentyMsCounter = 0;
int radFlag = 0;

bool isReceivingData = false;
byte *myData[10];
byte *data[3];
byte type;
bool storageTubes[4];
bool supplyTubes[4];

// ReactorProtocol myRobot(0x0A);
// BluetoothClient myClient;
// BluetoothMaster myMaster;

Servo leftDrive;
Servo rightDrive;
Servo fourBarMotor;
Servo flipperMotor;
Servo grabberServo;

enum armState{ //needs use
	UP,
	DOWN
};

enum State{
	Start,
	TESTING,
	findStart,
	findReactor,
	grabSpent,
	findDisposal,
	placeSpent,
	findSupply,
	grabSupply,
	returnToReactor,
	placeSupply, 
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
	flipperMotor.attach(flipperPin);
	grabberServo.attach(grabberServoPin);

	Serial1.begin(115200);
	Serial.begin(9600);
	pinMode(buttonIntPin, INPUT_PULLUP);
	attachInterrupt(0, resetISR, RISING);
	pinMode(potPin, INPUT);
	pinMode(radLED, OUTPUT);
	pinMode(limitPin, INPUT);
	pinMode(encoderLeft, INPUT);
	attachInterrupt(1, encoderLeftISR, RISING);
	pinMode(encoderRight, INPUT);
	attachInterrupt(4, encoderRightISR, RISING);

	state = Start;
	armStatus = DOWN;

	reactor = 'A'; //reactor type
	lineCount = 0; //counts the amount of lines
	lineFlag = 0; //flag for when line detected 

	Timer1.initialize(20000);
	Timer1.attachInterrupt(TwentyMsISR);
}

//method to test stand-alone modules of code for individual testing
void runTest()
{
	int x,y;
	x = analogRead(lineSenseLeft);
	y = analogRead(lineSenseRight);


	Serial.println(y);
	//Serial.println(y);
	
	//setArmAngle(80);
	//setArmAngle(downPosition);
	//insert test code here
}

//when the button is pushed, stop or resume robot operation
void resetISR()
{
	if(state == Start)
	{
		state = TESTING;
	} 
	else
	{
		state = Start;//state = idle;
	}
}

void loop(){
	if(radFlag != 0)
	{
		digitalWrite(radLED, HIGH);
	}
	stateMachine();
	// if(myMaster.readPacket(myData[0])) {
	// 	if(myRobot.getData(myData[0], data[0], type)) {
	// 		parseData(data[0], type);
	// 	}
	// }
 	

}
//master state machine
 void stateMachine(){
	
	switch (state) {
		case Start:
			stop(); 
			break;
		case TESTING:
			runTest();
			break;
		case findStart: 
			followLine();
			turnRight90(); //make this
			followLine();
			approachReactor();
			state = grabSpent;
	    case findReactor: 
	      if(reactor == 'A') //for reactor A
	      { 
	      	navigateToReactorCoord();
	      	state = grabSpent;	
	      }
	      if(reactor == 'B') //for reactor B
	      { 
	      	XYcoords[0] = 0;
	      	XYcoords[1] = 1;
	      	navigateToReactorCoord();
	      	state = grabSpent;	
	      }

	      break;
	    case grabSpent:
	   	  flipMeDown();
	      releaseGrab();
	      setArmAngle(downPosition);
	      grab();
	      flipMeUp();
	      setArmAngle(upPosition);
	      radFlag = 1; //carrying spent rod => spents radiation
	      state = findDisposal;
	      break;
	    case findDisposal:
	   	  	navigateToDisposal();
	   	  	state = placeSpent;
	   		break;
	    case placeSpent:
	    	releaseGrab();
	    	radFlag = 0; // no rod => no radiation
	    	state = findSupply;
	    	break;
	    case findSupply:
	    	navigateToSupply();
	    	state = grabSupply;
	    	break;
	    case grabSupply:
	    	grab();
	    	radFlag = 2; //carrying supply rod => full radiation
	    	state = returnToReactor;
	    	break;
	    case returnToReactor:
	    	navigateToReactorCoord();
	    	if(reactor = 'A')
	    	{
	    		reactor = 'B';
	    	}else
	    	{
	    		reactor = 'A';
	    	}
	    	state = placeSupply;
	    	break;
	    case placeSupply:
	    	flipMeDown();
	    	setArmAngle(downPosition);
	    	releaseGrab();
	    	radFlag = 0; //no rod => no radiation
	    	state = findReactor;
	    	break;
	    case idle:
	    	stop();
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
void backward()
{
	leftDrive.write(leftBWD);
	rightDrive.write(rightBWD);
}

//stop both motors
void stop()
{
	leftDrive.write(stopSpeed);
	rightDrive.write(stopSpeed);
}

 void TwentyMsISR() 
 {
 	// heartBeatCounter++;
 	// if((heartBeatCounter % 5) == 0) //every .5 seconds
 	// {
 	// 	sendHeartBeat();
 	// }
 	twentyMsCounter++;
 	if(twentyMsCounter == 25)
 	{
 		hundredMsFlag = 1;
 		twentyMsCounter = 0;
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
	//potAngle = analogRead(potPin);
	return potAngle;
}

//sets four-bar to a given desired angle with PID control 
//pot is BACKWARD!
void setArmAngle(int desiredAngle)
{
		angleError = desiredAngle - getPotAngle();
		adjustedSpeed = angleError*pGain + deltaAngleError*dGain + sumAngleError*iGain;
		Serial.print("Motor Speed :");
		Serial.println(90 + adjustedSpeed);
		fourBarMotor.write(90 + adjustedSpeed);
		prevAngleError = angleError;
		deltaAngleError = prevAngleError - angleError;
		sumAngleError = angleError + prevAngleError;
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

// //moves rack and pinion system to forward position, stops
// void rackForward()
// {
// 	rackMotor.write(150);
// 	delay(rackMoveTime);
// 	rackMotor.write(90);
// }

// //moves rack and pinion system to backward position, stops
// void rackReverse()
// {
// 	rackMotor.write(30);
// 	delay(rackMoveTime);
// 	rackMotor.write(90);
// }

//allows us to approach the reactor until limit switch hit
//written the same as followLine method
//optional slowing approach commented out
void approachReactor()
{
	hundredMsFlag = 0;
	while(hundredMsFlag != 1)
	{
		getError();
		leftSpeed = baseSpeedLeft + ((float) error*speedGain); // - (slowTime * slowTimeGain);
		rightSpeed = baseSpeedRight + ((float) error*speedGain); // -(slowTime * slowTimeGain);
		leftDrive.write(leftSpeed);
		rightDrive.write(leftSpeed);
	}
	
	while( (encoderLeftCount < forwardThreshold) && (encoderRightCount < forwardThreshold))
	{
		if(encoderLeftCount >= forwardThreshold)
		{
			leftDrive.write(stopSpeed);
		}
		else
		{
			leftDrive.write(leftFWD);
		}
		if(encoderRightCount >= forwardThreshold)
		{
			rightDrive.write(stopSpeed);
		}
		else
		{
			rightDrive.write(rightFWD);
		}
	}


	//slowTime++;
	
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
	int i;
	i = x-1;
	byte mask = 0x01;
	mask << i;
	return (mask & storageTubes[i] != mask);
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
	int i;
	i = x-1;
	byte mask = 0x01;
	mask << i;
	return (mask & supplyTubes[i] == mask);
}
void turnLeft90()
{
	
	forwardToThreshold();
	turnLeftToThreshold();
	// forward();
	// if(encoderLeftCount >= forwardThreshold)
	// {
	// 	leftDrive.write(stopSpeed);
	// 	encoderLeftCount = 0;
	// }
	// if(encoderRightCount >= forwardThreshold)
	// {
	// 	rightDrive.write(stopSpeed);
	// 	encoderRightCount = 0;
	// }
	// turnLeft();

	// if(encoderLeftCount >= turn90Threshold)
	// {
	// 	leftDrive.write(stopSpeed);
	// 	encoderLeftCount = 0;
	// }
	// if(encoderRightCount >= turn90Threshold)
	// {
	// 	rightDrive.write(stopSpeed);
	// 	encoderRightCount = 0;
	// }
}
void turnRight90()
{
	forwardToThreshold();
	turnRightToThreshold();
}
void turn180()
{

	backwardToThreshold();
	while((encoderLeftCount < turn180Threshold) && (encoderRightCount < turn180Threshold))
	{
		if(encoderLeftCount >= turn180Threshold)
		{
			leftDrive.write(stopSpeed);
		}
		else
		{
			leftDrive.write(leftBWD);
		}
		if(encoderRightCount >= turn180Threshold)
		{
			rightDrive.write(stopSpeed);
		}
		else
		{
			rightDrive.write(rightFWD);
		}
	}
	encoderRightCount = 0;
	encoderLeftCount = 0;
}
void encoderLeftISR()
{
	encoderLeftCount++;
}
void encoderRightISR()
{
	encoderRightCount++;
}
//pls work
void turnLeftToThreshold()
{
	while((encoderLeftCount < turn90Threshold) && (encoderRightCount < turn90Threshold))
	{
		if(encoderLeftCount >= turn90Threshold)
		{
			leftDrive.write(stopSpeed);
		}
		else
		{
			leftDrive.write(leftBWD);
		}
		if(encoderRightCount >= turn90Threshold)
		{
			rightDrive.write(stopSpeed);
		}
		else
		{
			rightDrive.write(rightFWD);
		}
	}
	encoderRightCount = 0;
	encoderLeftCount = 0;
}
void turnRightToThreshold()
{
	while((encoderLeftCount < turn90Threshold) && (encoderRightCount < turn90Threshold))
	{
		if(encoderLeftCount >= turn90Threshold)
		{
			leftDrive.write(stopSpeed);
		}
		else
		{
			leftDrive.write(leftFWD);
		}
		if(encoderRightCount >= turn90Threshold)
		{
			rightDrive.write(stopSpeed);
		}
		else
		{
			rightDrive.write(rightBWD);
		}
	}
	encoderRightCount = 0;
	encoderLeftCount = 0;
}
//pretty pls
void forwardToThreshold()
{
	while( (encoderLeftCount < forwardThreshold) && (encoderRightCount < forwardThreshold))
	{
		if(encoderLeftCount >= forwardThreshold)
		{
			leftDrive.write(stopSpeed);
		}
		else
		{
			leftDrive.write(leftFWD);
		}
		if(encoderRightCount >= forwardThreshold)
		{
			rightDrive.write(stopSpeed);
		}
		else
		{
			rightDrive.write(rightFWD);
		}
	}
	encoderRightCount = 0;
	encoderLeftCount = 0;
	
}
void backwardToThreshold()
{
	while( (encoderLeftCount < backwardThreshold) && (encoderRightCount < backwardThreshold))
	{
		if(encoderLeftCount >= forwardThreshold)
		{
			leftDrive.write(stopSpeed);
		}
		else
		{
			leftDrive.write(leftBWD);
		}
		if(encoderRightCount >= forwardThreshold)
		{
			rightDrive.write(stopSpeed);
		}
		else
		{
			rightDrive.write(rightBWD);
		}
	}
	encoderRightCount = 0;
	encoderLeftCount = 0;
}
void flipMeUp()
{
	flipperMotor.write(180);
}
void flipMeDown()
{
	flipperMotor.write(0);
}