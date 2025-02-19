#pragma once
#ifndef STEWARTPLATFORM_H_
#define STEWARTPLATFORM_H_


#include <iostream>
#include <vector>
#include <cmath>
#include <array>
#include "InvKinematics.h"
#include "MotorInterface.h"
#include "InputParser.h"
#include "PA12_Arduino/PA12.h"

class StewartPlatform {
private:
	double RB, RP, GB, GP;			 //Platform variables
	double Actuator_Neutral, Actuator_Min, Actuator_Max, H; //Actuator variables
	double dX, dY, dZ, dRoll, dPitch, dYaw; //Operation variables
	std::array<std::array<double, 6>, 3> B, P, L;
public:
	StewartPlatform(double radious_base, double radious_platform, double gamma_base, double gamma_platform,
		double h, double actuator_min = 0, double actuator_nominal = 0.006, double actuator_max = 0.012,
		double deltaX = 0, double deltaY = 0, double deltaZ = 0,
		double deltaRoll = 0, double deltaPitch = 0, double deltaYaw = 0);
	int run(); //Setup and Start State Machine
	int stateMachine();//State Machine Fxn -> Which uses the below functions
	int actuate();//Move Motors Fxn (Blue Box) -> Uses stuff from motorinterface.h
	int readData();//Parse Data Fxn (Green Box) -> Uses stuff from inputparser.h
	int solveKinematics();//Calculate Kinematics Fxn (Purple Box) -> Uses stuff from invkinematics.h
};

#endif /* STEWARTPLATFORM_H_ */