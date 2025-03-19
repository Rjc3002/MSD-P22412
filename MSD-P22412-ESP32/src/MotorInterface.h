#pragma once
#ifndef MOTORINTERFACE_H_
#define MOTORINTERFACE_H_

#include <vector>
#include <cmath>
#include <array>
#include <Arduino.h>  // Required for Serial functions
#include "PA12_Arduino/PA12.h"
#include <algorithm>

class MotorInterface {
private:
	PA12 myServo;

	int GOAL_POS = 0x86;

	//polynomial mapping values [-221 + 98.9x + 1.43x^2] meters -> actuation value
	double a1 = 14.5;
	double b1 = 105;
	double c1 = 1.43;

	//actuation value -> meters
	double a2 = -0.0366;
	double b2 = 0.00879;
	double c2 = -0.000000475;

public:
	MotorInterface();
	void setup();
	void actuate(const std::vector<std::array<double, 6>> cmdArray);
	void move(int* moveArr);
	std::array<double, 6> readPos();
	void testSmth(int test);

};

#endif /* MOTORINTERFACE_H_ */