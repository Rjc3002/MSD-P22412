#include "MotorInterface.h"

int GOAL_POS = 0x86;

//polynomial mapping values [-221 + 98.9x + 1.43x^2] meters -> actuation value
double a1 = 14.5;
double b1 = 105;
double c1 = 1.43;

//actuation value -> meters
double a2 = -0.0366;
double b2 = 0.00879;
double c2 = -0.000000475;

PA12 myServo(&Serial0, 8, 1); //(&Serial, enable pin, Tx Level)

void setup() {
	//Initialize the PA12 bus
	// Baudrate -> 128: 9600, 32: 57600, 16: 115200 
	myServo.begin(32);
}


/*
Uses the generated command vector from InvKinematics
vector is a list of arrays ... {step1, step2, ...}
each step is a list of goal positions for actuator IDs in order {ID#1 goalpos, ID#2 goalpos, ... , ID#6 goalpos}
*/

void actuate(const std::vector<std::array<double, 6>> cmdArray) {
	setup();

	for (int i = 0; i < 6; i++) {
		myServo.movingSpeed(i + 1, 0); //moving speed of 0 is max speed
		delay(10);
		// set any other parameters here
	}

	for (const auto& arr : cmdArray) {
		int num_elements = sizeof(*arr) / sizeof(*arr[0]);
		std::array<int, 2 * num_elements> writeArray;
		for (int i = 0; i < num_elements; i++) {
			double converted = a + b * 1000.0 * *arr[i] + c * std::pow((1000.0 * *arr[i]), 2) ;
			writeArray[2*i] = i+1;
			writeArray[(2 * i) + 1] = (int)converted;
		}
		move(writeArray);
	}
}

void move(std::array<int, 12> moveArr) {
	syncWrite(moveArr);
	delay(200);		//Need to experiment with delay or use the myServo.moving command
}

/*
reads the current positions of each of the actuators and returns an array of the positions in order of ID#
*/
std::array<double, 6> readPos() {
	std::array<double, 6> positions;

	for (int i = 0; i < 6; i++) {
		double pos = myServo.presentPosition(i + 1);
		positions[i] = a2 + b2*(pos/1000.0) + (c2*std::pow((pos/1000.0)^2), 2);
		delay(200);
	}

	return positions;
}


