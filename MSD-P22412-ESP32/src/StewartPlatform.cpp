#include "StewartPlatform.h"


/*
3DOF Stewart Platform RPR configuration analysis:
    - We take 6DOF Stewart Platform as a reference.
    - We fix the platform in the home position.
    - The platform can only rotate in the x, y, z axis roll, pitch, yaw.
*/

StewartPlatform::StewartPlatform(double radious_base, double radious_platform, double gamma_base, double gamma_platform,
    double h, double actuator_min, double actuator_nominal, double actuator_max, double deltaX, double deltaY, double deltaZ,
    double deltaRoll, double deltaPitch, double deltaYaw)
    : RB(radious_base), RP(radious_platform), Actuator_Neutral(actuator_nominal), Actuator_Min(actuator_min), Actuator_Max(actuator_max),
    H(h), dX(deltaX), dY(deltaY), dZ(deltaZ), dRoll(deltaRoll), dPitch(deltaPitch), dYaw(deltaYaw) {
    GB = gamma_base * M_PI / 180.0;
    GP = gamma_platform * M_PI / 180.0;
}
int StewartPlatform::run() {
    // Set Hexapod Constants

    InvKinematics clf(RB,RP,GB,GP,H,Actuator_Min,Actuator_Neutral,Actuator_Max);
    std::array<double, 6> l;

    // Create translation
    std::array<double, 3> trans = { dX, dY, dZ };

    // Create Rotation
    std::array<double, 3> rot1 = { dRoll, dPitch, dYaw };

    // Calculate Actuator lengths
     std::array<double, 6> new_l = clf.solve(trans, rot1);

    std::cout << "Leg actuation distance (m): ";
    for (size_t i = 0; i < 6; ++i) {
        std::cout << (new_l[i] - l[i]) << " ";
    }
    std::cout << std::endl;

    return 0;
}
