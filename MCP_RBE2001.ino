#include <Servo.h>
#include <TimerOne.h>
#include <BluetoothClient.h>
#include <BluetoothMaster.h>
#define leftDrivePin;
#define rightDrivePin;
#define fourBarPin;
#define rackMotorPin;
#define grabberServoPin;
#define limitPin;
#define lineSensePin1;
#define lineSensePin2;
#define lineSensePin3;
#define lineSensePin4;
#define potPin;

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
	leftDrive.attach(leftDrivePin, 1000, 2000);
	rightDrive.attach(rightDrivePin, 1000, 2000);
	fourBarMotor.attach(fourBarPin, 1000, 2000);
	rackMotor.attach(rackMotorPin, 1000, 2000);
	grabberServo.attach(grabberServoPin);

	state = findReactor;
	Timer1.initialize(100000);
	Timer1.attachInterrupt(100msISR);
	Timer2.initialize(2000000);
	Timer2.attachInterrupt(hearbeatISR);
}

void loop(){
  
 	main();

}

void main(){
	switch (state) {
	    case findReactor:
	      // do something
	      break;
	    case grabSpent:
	      // do something
	      break;
	    default:
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
void 100msISR() {
	fetchBluetooth();
}
void heartbeatISR() {

}