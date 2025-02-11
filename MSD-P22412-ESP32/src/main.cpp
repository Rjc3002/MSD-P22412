#include "main.h"


/*
3DOF Stewart Platform RPR configuration analysis:
    - We take 6DOF Stewart Platform as a reference.
    - We fix the platform in the home position.
    - The platform can only rotate in the x, y, z axis roll, pitch, yaw.
*/

int main() {
    // Set Hexapod Constants
    std::array<double, 9> design_variables = {
        MSD_Hexapod_Simplified::RB, MSD_Hexapod_Simplified::RP, MSD_Hexapod_Simplified::GB,
        MSD_Hexapod_Simplified::GP, MSD_Hexapod_Simplified::AdvancedMode, MSD_Hexapod_Simplified::H,
        MSD_Hexapod_Simplified::Actuator_Min, MSD_Hexapod_Simplified::Actuator_Neutral, MSD_Hexapod_Simplified::Actuator_Max
    };

    // Create translation
    std::array<double, 3> trans = { MSD_Hexapod_Simplified::dX, MSD_Hexapod_Simplified::dY, MSD_Hexapod_Simplified::dZ };

    // Create Rotation
    std::array<double, 3> rot1 = { MSD_Hexapod_Simplified::dRoll, MSD_Hexapod_Simplified::dPitch, MSD_Hexapod_Simplified::dYaw };

    // Define the desired end effector position
    std::vector<std::pair<std::array<double, 3>, std::array<double, 3>>> data = { {trans, rot1} };

    // Create the Stewart platform object
    StewartPlatform clf(design_variables);

    // Calculate Actuator lengths
    clf.startSimulation(data, MSD_Hexapod_Simplified::SearchMode);

    return 0;
}
