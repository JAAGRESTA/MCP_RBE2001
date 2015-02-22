#include <Servo.h>
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

#define leftFWD 180
#define rightFWD 0
#define leftBWD 0
#define rightBWD 180

char reactor; //reactor type
int lineCount; //counts amount of lines
int lineFlag; //flage when line detected

#include <TimerOne.h>
#include <BluetoothClient.h>
#include <BluetoothMaster.h>

Servo leftDrive;
Servo rightDrive;
Servo fourBarMotor;
Servo rackMotor;
Servo grabberServo;

enum State{
	findReactor,
	grabSpent,
	findDisposal,
	placeSpent,
	findSupply,
	grabSupply,
	returnToReactor,
	placeSupply
};

State state;

void setup(){
	leftDrive.attach(leftDrivePin,1000,2000);
	rightDrive.attach(rightDrivePin,1000,2000);
	fourBarMotor.attach(fourBarPin,1000,2000);
	rackMotor.attach(rackMotorPin,1000,2000);
	grabberServo.attach(grabberServoPin);
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
	Timer1.attachInterrupt(100msISR);
	Timer2.initialize(2000000);
	Timer2.attachInterrupt(hearbeatISR);
}

void loop(){
  
 	stateMachine();

}
//state machine that runs between states of 
void stateMachine(){
	switch (state) {
	    case findReactor: //finds a reactor
	      if(reactor == 'A'){ //for reactor A
	      	forward();	
	      	if(lineHit(1)){ //changes depends on where we start
	      		if(analogRead(limitPin) == LOW){
	      		state = grabSpent;
	      		}
	      	}
	      }
	      if(reactor == 'B'){ //for reactor B
	      	if(lineHit(4)){ //might need to change depending on where it turns around
	      		if(analogRead(limitPin) == LOW){
	      		state = grabSpent;
	      		}
	      	}
	      }
	      break;
	    case grabSpent:
	      // do something
	      break;
	    case findDisposal:
	    	//stuff
	    	break;
	    case placeSpent:
	    	//stuff
	    	break;
	    case findSupply:
	    	//stuff
	    	break;
	    case grabSupply:
	    	//stuff
	    	break;
	    case returnToReactor:
	    	//stuff
	    	break;
	    case placeSupply:
	    	//stuff
	    	break;
	}

}
//returns boolean if a certain amount of lines have been hit
boolean lineHit(int x){
	if((lineFlag == 0) && (analogRead(lineSensePin4 == HIGH))){
		lineCount++;
		lineFlag = 1; 
	}else if((lineFlag == 1) && (analogRead(lineSensePin4 == LOW))){
		lineFlag = 0;			
	}				
	if(lineCount == x){
		return true;
	}else{
		return false;
	}
}

void forward(){
	leftDrive.write(leftFWD);
	rightDrive.write(rightFWD);
=======
void 100msISR() {
	fetchBluetooth();
}
void heartbeatISR() {

}