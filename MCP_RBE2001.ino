#include <Servo.h>

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
}

void loop(){
  
 	main();

}

void main(){
	switch (state) {
	    case :
	      // do something
	      break;
	    case :
	      // do something
	      break;
	    default:
	      // do something
	}

}