/*
 Name:		MSD_P22412_ESP32.ino
 Created:	2/10/2025 9:29:49 PM
 Author:	rylan
*/


#include "src\StewartPlatform.h"

double H = 0.1;     // meters, height of the platform(z - axis) relative to base, unused when AdvancedMode = True

// Define Linear Actuator Specifications
double Actuator_Min = 0.0762; //What we're going with (m)
double Actuator_Neutral = 0.0822;
double Actuator_Max = 0.0882;
double RB = 0.050;
double RP = 0.020;
double GB = 30 / 2.0; //(deg)
double GP = 60 / 2.0;

// Define the operation to perform -> Unused
//   Linear Translation of Platform (m)
double dX = 0;
double dY = 0;
double dZ = 0;

//   Rotation of Platform (deg) -> Not used in current function calls
double dRoll = 0.01;
double dPitch = 0;
double dYaw = 0; // Twist, unused

//Create Stewart Platform object
StewartPlatform splat(RB, RP, GB, GP, H, Actuator_Min, Actuator_Neutral, Actuator_Max);

// the setup function runs once when you press reset or power the board
void setup() {
	Serial.begin(115200);
	//while (!Serial);  // Wait for Serial Monitor to open
	Serial.println("Starting.");
}

// the loop function runs over and over again until power down or reset
void loop() {
	splat.run(); //Start State Machine

	exit; //Should never reach this
}
