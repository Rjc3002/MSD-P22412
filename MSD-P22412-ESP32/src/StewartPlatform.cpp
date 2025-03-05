#include "StewartPlatform.h"


/*
3DOF Stewart Platform RPR configuration analysis:
    - We take 6DOF Stewart Platform as a reference.
    - We fix the platform in the home position.
    - The platform can only rotate in the x, y, z axis roll, pitch, yaw.
*/

StewartPlatform::StewartPlatform(double radious_base, double radious_platform, double gamma_base, double gamma_platform,
    double actuator_min, double actuator_nominal, double actuator_max, double deltaX, double deltaY, double deltaZ,
    double deltaRoll, double deltaPitch, double deltaYaw)
    : RB(radious_base), RP(radious_platform), Actuator_Neutral(actuator_nominal), Actuator_Min(actuator_min), Actuator_Max(actuator_max),
    dX(deltaX), dY(deltaY), dZ(deltaZ), dRoll(deltaRoll), dPitch(deltaPitch), dYaw(deltaYaw), 
    clf(radious_base, radious_platform, gamma_base* M_PI / 180.0, gamma_platform* M_PI / 180.0, actuator_min, actuator_nominal, actuator_max) {
    GB = gamma_base * M_PI / 180.0;
    GP = gamma_platform * M_PI / 180.0;
}

bool StewartPlatform::startStop(bool start) {
    running = start;
    return running;
}

int StewartPlatform::run() {
    startStop(true); //Start state machine
    while (running) { //State Machine begins here
        readData();

        //temporary demo code UI:
        double targetRoll, targetPitch = 0;
        bool inputSuccess = false;
        Serial.println("Enter the desired goal Roll angle (degs.) and desired goal Pitch angle (degs.), then press enter: ");
        Serial.println("For example:  5 15");
        while (!Serial.available()) {} //Wait for numbers to be entered
        targetRoll = Serial.parseFloat();
        targetPitch = Serial.parseFloat();
        //if (targetRoll != 0 && targetPitch != 0) { // Check if the input was successful
            Serial.print("You entered: Roll=");
            Serial.print(targetRoll, 2);
            Serial.print(" deg, Pitch=");
            Serial.print(targetPitch, 2);
            Serial.println(" deg");
            inputSuccess = true;
            dRoll = targetRoll;
            dPitch = targetPitch;
       // }
       // else {
            //Serial.println("Issue parsing input");
        //}
        //
        //Set motion and calculate leg lengths
        std::array<double, 6> new_l = getRotationLengths(dRoll, dPitch, true);
        //if (inputSuccess && std::all_of(new_l.begin(), new_l.end(), [](double val) { return val != 0;})) { //Make sure new_l is not {0}
            Serial.print("New Leg Lengths (m): ");
            for (size_t i = 0; i < 6; ++i) {
                Serial.print(new_l[i],12);
                Serial.print(", ");
            }
            Serial.println();

            if (solveKinematics(dRoll, dPitch)) {
                actuate();
            }

        //}



    }
    return 0;
}

std::array<double, 6> StewartPlatform::getRotationLengths(double dr, double dp, bool solve, double dyaw, double dx, double dy, double dz ) {
    // Create translation
    //Serial.println("In getRotationLengths()");
    std::array<double, 3> trans = { dx, dy, dz };

    // Create Rotation
    std::array<double, 3> rot1 = { dr, dp, dyaw };

    if (solve) {
        // Calculate Actuator lengths
        std::array<double, 6> l = clf.solve(trans, rot1);
        return l;
    }
    return { 0 };
}

int StewartPlatform::stateMachine() { return 0; }
int StewartPlatform::actuate() { return 0; }
int StewartPlatform::readData() { return 0; }


int StewartPlatform::solveKinematics(double thetaR, double thetaP) 
{ 
    int error = 0;
    
    bool throughOrigin = false;
    bool motorDispOutOfBounds = false;

	if (lastThetaR == UNINITIALIZED || lastThetaP == UNINITIALIZED) {
        error = -2;
        return 0;
	}

    double initialThetaR = lastThetaR;
    double initialThetaP = lastThetaP;
    double goalThetaR, goalThetaP;

    do {
        if (throughOrigin) { //On second path
            throughOrigin = false;
            initialThetaP = 0.0;
            initialThetaR = 0.0;
            goalThetaP = thetaP;
            goalThetaR = thetaR;
        }
        else { //First path, is a second path needed?
            //check and set throughOrigin
			if ((abs(thetaP-initialThetaP) < originPathThreshold || abs(thetaR-initialThetaR) < originPathThreshold)
                && ((thetaP * initialThetaP) < 0.0 || (thetaR * initialThetaR) < 0.0)) {
				throughOrigin = true;
			}
            if (throughOrigin) {
                goalThetaP = 0.0;
                goalThetaR = 0.0;
            }
            else {
                goalThetaP = thetaP;
                goalThetaR = thetaR;
            }
        }
        //Calculate initial N
        double N = max(abs(goalThetaR - initialThetaR) / angularResoltion, abs(goalThetaP - initialThetaP) / angularResoltion);

        bool solvingN = true;
        do {
            //For each step
            for (size_t i = 0; i < N; ++i) {
                //Solve inverse kinematics for point on path
				double thetaI_P = clf.calculatePointOnPath(N, initialThetaP, goalThetaP, i);
				double thetaI_R = clf.calculatePointOnPath(N, initialThetaR, goalThetaR, i);

                double lastStepThetaR = 0.0;
                double lastStepThetaP = 0.0;

				if (i == 0) {
					lastStepThetaR = initialThetaR;
					lastStepThetaP = initialThetaP;
                }
                else {
					lastStepThetaR = clf.calculatePointOnPath(N, initialThetaR, goalThetaR, i - 1);
					lastStepThetaP = clf.calculatePointOnPath(N, initialThetaP, goalThetaP, i - 1);
                }

				std::array<double, 6> legLengths = getRotationLengths(thetaI_R, thetaI_P, true);
                std::array<double, 6> lastLengths = getRotationLengths(lastStepThetaR, lastStepThetaP, true);

                //Check if actuator lengths are valid
                if (clf.checkLengths(legLengths)) {
                    solvingN = false;
                    throughOrigin = false;
                    error = -1;
                    break;
                }
                //Calculate and check change in actuator lengths
				for (size_t j = 0; j < 6; ++j) {
					if (abs(legLengths[j] - lastLengths[j]) > motorDisplacementThreshold) { //Check that actuator lengths are less than the threshold displacement
						motorDispOutOfBounds = true;
						break;
					}
				}
                
                if (motorDispOutOfBounds) { //If not, increase N and jump to top of loop
                    N++;
					motorDispOutOfBounds = false;
                    //clear and resize cmd array for this path only

                    break;
                }

                //Store actuator lengths in command array for this path


                if (i == N - 1) {
                    solvingN = false; //End of path
                }
            }
        } while (solvingN);
    } while (throughOrigin); //Do the second path if neccessary

    //Error handling

    return 0; //Return full command array (both cmd arrays appended)
}

int StewartPlatform::home() { 
    //Measure current actuator lengths

	//Compare to home lengths using inv kinematics

    //Calculate optimal N

    //Linear stepping of motors


    //lastTheta values should only be updated in the motor moving cmds

	return 0; //return command array
}