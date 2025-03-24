#include "MotorInterface.h"

MotorInterface::MotorInterface():myServo(&Serial0, 8, 1) { //myServo(&Serial, enable pin, Tx Level)
	//Constructor
}

void MotorInterface::setup() {
	//Initialize the PA12 bus
	// Baudrate -> 32: 57600 (hardcoded in PA12.cpp)
	myServo.begin();
}


/*
Uses the generated command vector from InvKinematics
vector is a list of arrays ... {step1, step2, ...}
each step is a list of goal positions for actuator IDs in order {ID#1 goalpos, ID#2 goalpos, ... , ID#6 goalpos}
*/

void MotorInterface::actuate(const std::vector<std::array<double, 6>> cmdArray) {
	setup();

	Serial.println("Setting Speeds");
	for (int i = 0; i < 6; i++) {
		//myServo.movingSpeed(i + 1, 0); //moving speed of 0 is max speed
		//delay(10);
		// set any other parameters here
	}

	Serial.println("Executing Commands");
	for (int i = 0; i < 6; i++) {
		myServo.forceEnable(i + 1, 0x01); //Enable active force on motors.
		delay(10);
	}
	for (const auto& arr : cmdArray) {
		int num_elements = sizeof(arr) / sizeof(arr[0]);
		int writeArray[2*num_elements];
		
		//std::array<int, 2 * num_elements> writeArray;
		for (int i = 0; i < num_elements; i++) {
			double converted = a1 + b1 * 1000.0 * arr[i] + c1 * std::pow((1000.0 * arr[i]), 2) ;
			writeArray[2*i] = i+1;
			writeArray[(2 * i) + 1] = (int)converted;
			Serial.print(writeArray[2 * i]);
			Serial.print(" : ");
			Serial.print(writeArray[(2 * i) + 1]);
			Serial.print(", ");
		}
		Serial.print("N: ");
		Serial.print(cmdArray.size());
		Serial.print(", ");
		move(writeArray);
	}

	for (int i = 0; i < 6; i++) {
		myServo.forceEnable(i + 1, 0); //Disable active force on motors, use passive friction to hold pos until next movement.
		//The servo being actively driven to hold a position for more than 30sec may cause
		//an overload error?
		delay(10);
	}
}

void MotorInterface::move(int* moveArr) {
	Serial.println("syncWrite");
	myServo.syncWrite(GOAL_POS, moveArr, 12);
	delay(200);		//Need to experiment with delay or use the myServo.moving command
}

/*
reads the current positions of each of the actuators and returns an array of the positions in order of ID#
*/
std::array<double, 6> MotorInterface::readPos() {
	std::array<double, 6> positions = { 0.0 };

	for (int i = 0; i < 6; i++) {
		double pos = myServo.presentPosition(i + 1);
		positions[i] = (a2 + b2 * pos + (c2 * std::pow(pos, 2)))/1000.0;
		delay(200);
	}

	return positions;
}

void MotorInterface::testSmth(int test) {
	if (test == 1) {
		Serial.println("Test 1: SyncWrite 1200");
		for (int i = 0; i < 6; i++) {
			myServo.forceEnable(i + 1, 0x01); //Enable active force on motors.
			delay(10);
		}
		int moveArr[12] = { 1, 1200, 2, 1200, 3, 1200, 4, 1200, 5, 1200, 6, 1200 };
		myServo.syncWrite(GOAL_POS, moveArr, 12);
		delay(200);
		for (int i = 0; i < 6; i++) {
			myServo.forceEnable(i + 1, 0); //Enable active force on motors.
			delay(10);
		}
	}
	else {
		
	}
}