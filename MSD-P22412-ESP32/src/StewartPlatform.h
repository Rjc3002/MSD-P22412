#pragma once
#ifndef STEWARTPLATFORM_H_
#define STEWARTPLATFORM_H_


#include <iostream>
#include <vector>
#include <cmath>
#include <array>
#include <string>
#include <memory>
#include <mutex>
#include <Arduino.h>  // Required for Serial functions
#include "InvKinematics.h"
#include "MotorInterface.h"
#include "InputParser.h"
#include "PA12_Arduino/PA12.h"
#include "timerISR.h"
#include "Delay.h"

class StewartPlatform {
private:
	static std::unique_ptr<StewartPlatform> instance;
	static std::mutex mutex;

	double RB, RP, GB, GP;			 //Platform variables
	double Actuator_Neutral, Actuator_Min, Actuator_Max; //Actuator variables
	double dX, dY, dZ, dRoll, dPitch, dYaw; //Operation variables
	std::array<std::array<double, 6>, 3> B, P, L;
	InvKinematics clf;
	MotorInterface motors;

	bool running = true;

	static constexpr int UNINITIALIZED = -9999;
	static constexpr double originPathThreshold = 2.0; //(deg)
	static constexpr double motorDisplacementThreshold = 0.00007; //(m) 0.0007
	static constexpr double angularResoltion = 0.05; //(deg)

	double lastThetaR = UNINITIALIZED;
	double lastThetaP = UNINITIALIZED;

	// Private constructor to prevent external instantiation
	StewartPlatform(double radious_base, double radious_platform, double gamma_base, double gamma_platform,
		double actuator_min = 0, double actuator_nominal = 0.006, double actuator_max = 0.012,
		double deltaX = 0, double deltaY = 0, double deltaZ = 0,
		double deltaRoll = 0, double deltaPitch = 0, double deltaYaw = 0);

	// Delete copy constructor and assignment operator to prevent copying
	StewartPlatform(const StewartPlatform&) = delete;
	StewartPlatform& operator = (const StewartPlatform&) = delete;

public:
	// Static method to initialize or return the instance
	static StewartPlatform& initialize(double radious_base, double radious_platform, double gamma_base, double gamma_platform,
		double actuator_min = 0, double actuator_nominal = 0.006, double actuator_max = 0.012,
		double deltaX = 0, double deltaY = 0, double deltaZ = 0,
		double deltaRoll = 0, double deltaPitch = 0, double deltaYaw = 0);

	bool startStop(bool start = true); //Access the running state machine variable
	int run(double goalPitch, double goalRoll); //Setup and Start State Machine
	std::array<double, 6> getRotationLengths(double dr, double dp, bool solve = false, double dyaw = 0, double dx = 0, double dy = 0, double dz = 0); //Set translation and rotation frames, then optionally solve for leg lengths
	int stateMachine();//State Machine Fxn -> Which uses the below functions -> Maybe just use run() for this
	std::vector<std::array<double, 3>> readData();//Parse Data Fxn (Green Box) -> Uses stuff from inputparser.h
	std::vector<std::array<double, 6>> solveKinematics(double thetaR, double thetaP);//Calculate Kinematics Fxn (Purple Box) -> Uses stuff from invkinematics.h and motorinterface.h
	std::vector<std::array<double, 6>> home();
};

#endif /* STEWARTPLATFORM_H_ */