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

	for (int i = 0; i < 6; i++) {
		myServo.movingSpeed(i + 1, 0); //moving speed of 0 is max speed
		delay(10);
		// set any other parameters here
	}

	for (const auto& arr : cmdArray) {
		int num_elements = sizeof(arr) / sizeof(arr[0]);
		int writeArray[2*num_elements];
		
		//std::array<int, 2 * num_elements> writeArray;
		for (int i = 0; i < num_elements; i++) {
			double converted = a1 + b1 * 1000.0 * arr[i] + c1 * std::pow((1000.0 * arr[i]), 2) ;
			writeArray[2*i] = i+1;
			writeArray[(2 * i) + 1] = (int)converted;
		}
		move(writeArray);
	}
}

void MotorInterface::move(int* moveArr) {
	myServo.syncWrite(GOAL_POS, moveArr, 12);
	delay(200);		//Need to experiment with delay or use the myServo.moving command
}

/*
reads the current positions of each of the actuators and returns an array of the positions in order of ID#
*/
std::array<double, 6> MotorInterface::readPos() {
	std::array<double, 6> positions;

	for (int i = 0; i < 6; i++) {
		double pos = myServo.presentPosition(i + 1);
		positions[i] = a2 + b2 * (pos/1000.0) + (c2 * std::pow((pos/1000.0), 2));
		delay(200);
	}

	return positions;
}


