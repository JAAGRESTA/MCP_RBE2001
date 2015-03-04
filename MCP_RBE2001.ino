#include <Servo.h>
#include <TimerOne.h>
#include <BluetoothClient.h>
#include <BluetoothMaster.h>
//#include <ReactorProtocol.h>

//i/o pins
#define leftDrivePin 10
#define rightDrivePin 11
#define fourBarPin 4
#define grabberServoPin 6
#define limitPin A0
#define lineSenseRight A2
#define lineSenseLeft A1
#define lineSenseFarLeft A4
#define potPin A5
#define encoderRight 19
#define encoderLeft 3 
#define buttonIntPin 2
#define flipperPin 7
#define radLED 27

//constants for motor speeds and servo positions
#define stopSpeed 90
#define potRange 180
#define leftFWD 70
#define rightFWD 110
#define leftBWD 110
#define rightBWD 70
#define grabberClosed 180
#define grabberOpen 0
#define rackMoveTime 50
#define upPosition 454  
#define downPosition 400
#define flipperUp 0
#define flipperDown 180

//bluetooth constants
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
int lightThreshold = 600; //threshold for if a line sensor is on light or dark, above threshold = light
float error;
float leftSpeed = 70, rightSpeed = 110, speedGain = 0.01; 
unsigned long int heartBeatCounter = 0;
float potVal;
float angleError = 0, prevAngleError = 0, deltaAngleError = 0, sumAngleError =0, slowTime = 0;
float adjustedSpeed = 90, pGain= 450, iGain=50, dGain=100;
int XYcoords[2] = {5,1};
int currentXYcoords[2] = {3,1};
int encoderLeftCount = 0, encoderRightCount = 0, encoderLeftCurrentCount, encoderRightCurrentCount;
float turn90Threshold = 95, turn180Threshold = 200, forwardThreshold = 70.35, backwardThreshold = 70.35; //turn needs tuning or calculating
int hundredMsFlag = 0, twentyMsCounter = 0;
int radFlag = 0;
int baseSpeedLeft = 70;
int baseSpeedRight = 110;

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
Servo flipperServo;
Servo grabberServo;

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

blueState _blueState;
State state;

void setup(){
	leftDrive.attach(leftDrivePin,1000,2000);
	rightDrive.attach(rightDrivePin,1000,2000);
	fourBarMotor.attach(fourBarPin,1000,2000);
	flipperServo.attach(flipperPin);
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

	reactor = 'A'; //reactor type
	lineCount = 0; //counts the amount of lines
	lineFlag = 0; //flag for when line detected 

	Timer1.initialize(20000);
	Timer1.attachInterrupt(TwentyMsISR);
}

//method to test stand-alone modules of code for individual testing
void runTest()
{
	// int x,y;
	// x = analogRead(lineSenseLeft);
	// y = analogRead(lineSenseRight);

	followLine();
	//releaseGrab();
	//grab();
	// flipMeDown();
	// delay(1000);
	// flipMeUp();
	// delay(1000);

	// Serial.print(analogRead(lineSenseLeft));
	// Serial.print(", ");
	// Serial.print(analogRead(lineSenseRight));
	// Serial.print(", ");
	// getError();
	// Serial.println(error);

	// Serial.println(y);
	//Serial.println(analogRead(potPin));
	
	//Serial.println(error);
	//Serial.println(analogRead(lineSenseLeft));
	//Serial.println(getPotVal());
	//setArmAngle(downPosition);
	//insert test code here
	//goXlines(5);
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
			goXlines(3);
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
	      setArmAngle(upPosition);
	      flipMeUp();
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
	if(overLine(lineSenseFarLeft) && (!lineFlag))
	{
		
		lineFlag = 1;
	
	}else if(!(overLine(lineSenseFarLeft)))
	{
		lineCount++;
		lineFlag = 0;
	}
	if(lineCount == x)
	{
		return true;
	}else
	{
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

//uses timer one to count 20ms intervals, useful for encoders
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

// //method to do proportional line tracking 
void followLine() 
{			
		getError();
		leftSpeed = baseSpeedLeft + ((float) error*speedGain);
		rightSpeed = baseSpeedRight + ((float) error*speedGain);
		leftDrive.write(leftSpeed);
		rightDrive.write(rightSpeed);
}

//follow a line until a given number of crosses or intersections are hit
void goXlines(int lineNum)
{
	while(!lineHit(lineNum))
	{
		followLine();
	}
	lineCount = 0;
}

//finds difference in line sensor values, sets that value to a useable motor speed
void getError()
{
 	error = analogRead(lineSenseLeft) - analogRead(lineSenseRight);
}

//method to see if the far line sensor and at least one other line sensor on on a line or not indicating a cross
//returns true if a cross is hit, false otherwise 
boolean crossHit() 
{
	if(overLine(lineSenseFarLeft) && (overLine(lineSenseRight) || overLine(lineSenseLeft))) 
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
	if (analogRead(lineSensorPin) <= lightThreshold)
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

//checks the potentiometer position 
//returns the potentiometer value 
float getPotVal()
{
	potVal = analogRead(potPin);
	return potVal;
}

//sets four-bar to a given desired setpoint with PID control 
void setArmAngle(float desiredPosition)
{
		angleError = (desiredPosition/1023) - (getPotVal()/1023);
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

//allows us to approach the reactor until limit switch hit
//written the same as followLine method
//optional slowing approach commented out
void approachReactor()
{
	hundredMsFlag = 0;
	while(hundredMsFlag != 1)
	{
		getError();
		leftSpeed = baseSpeedLeft - ((float) error*speedGain); // - (slowTime * slowTimeGain);
		rightSpeed = baseSpeedRight - ((float) error*speedGain); // -(slowTime * slowTimeGain);
		leftDrive.write(leftSpeed);
		rightDrive.write(rightSpeed);
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

//drives forward slightly and turns 90 degrees using encoders
void turnRight90()
{
	forwardToThreshold();
	turnRightToThreshold();
}

//backs up slightly and turns 180 degrees using encoders
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

//increments encoder count each rising edge
void encoderLeftISR()
{
	encoderLeftCount++;
}
void encoderRightISR()
{
	encoderRightCount++;
}

//turns left 90 degrees using encoders
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

//turns right 90 degrees using encoders
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

//moves straight forward using encoders
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

//moves straight backwards using encoders
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

//moves string tensioner to up position
void flipMeUp()
{
	flipperServo.write(flipperUp);
}

//moves string tensioner to down position
void flipMeDown()
{
	flipperServo.write(flipperDown);
}