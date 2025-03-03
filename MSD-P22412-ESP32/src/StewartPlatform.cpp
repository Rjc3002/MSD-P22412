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

            if (solveKinematics()) {
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
int StewartPlatform::solveKinematics() { return 0; }
