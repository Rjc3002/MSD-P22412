#include "MotorInterface.h"

MotorInterface::MotorInterface():myServo(&Serial0, 8, 1) { //myServo(&Serial, enable pin, Tx Level)
	//Constructor
}

void MotorInterface::setup() {
	//Initialize the PA12 bus
	// Baudrate = 57600 (hardcoded in PA12.cpp)
	Serial.println("Setting up Motor Interface");
	myServo.begin();

	for (int i = 0; i < 6; i++) {
		//myServo.complianceMargin(i+1, Short, 1);
		//myServo.complianceMargin(i+1, Long, 1);
		myServo.movingSpeed(i + 1, 1023); //Max speed
		myServo.BaudRate(i + 1, 32);
		//delay(10);
		ntDelay(10);
	}
	
}


/*
Uses the generated command vector from InvKinematics
vector is a list of arrays ... {step1, step2, ...}
each step is a list of goal positions for actuator IDs in order {ID#1 goalpos, ID#2 goalpos, ... , ID#6 goalpos}
*/

void MotorInterface::actuate(const std::vector<std::array<double, 6>> cmdArray) {
	Serial.println("Executing Commands");
	for (int i = 0; i < 6; i++) {
		int temp = myServo.movingSpeed(i + 1);
		Serial.print("Motor ");
		Serial.print(i + 1);
		Serial.print(" speed: ");
		Serial.println(temp);
		myServo.forceEnable(i + 1, 0x01); //Enable active force on motors.
		//delay(10);
		ntDelay(10);
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
		//delay(10);
		ntDelay(10);
	}
}

void MotorInterface::move(int* moveArr) {
	Serial.println("GoalWrite");

	for (int i = 0; i < 6; i++) {
		myServo.goalPosition(i + 1, moveArr[2 * i + 1]);
		//delay(10);
		ntDelay(10);
	}

	//delay(50);		//Need to experiment with delay or use the myServo.moving command
	ntDelay(50);
}

/*
reads the current positions of each of the actuators and returns an array of the positions in order of ID#
*/
std::array<double, 6> MotorInterface::readPos() {
	std::array<double, 6> positions = { 0.0 };
	Serial.print("Reading Positions: ");
	for (int i = 0; i < 6; i++) {
		double pos = myServo.quick_presentPosition(i + 1);
		//delay(20);
		ntDelay(20);
		pos = myServo.quick_presentPosition(i + 1);
		Serial.print(pos);
		Serial.print(", ");
		positions[i] = (a2 + b2 * pos + (c2 * std::pow(pos, 2)))/1000.0;
		//delay(200);
		ntDelay(200);
	}
	Serial.println();

	return positions;
}

void MotorInterface::testSmth(int test, int param) {
	if (test == 1) {
		Serial.println("Test 1 not implemented");
	}
	else if (test == 2) {
		Serial.println("Test 2 not implemented");
	}
	else {
		Serial.println("Unknown Test number. Restart.");
	}
}