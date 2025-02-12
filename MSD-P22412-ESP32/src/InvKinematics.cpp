#include "InvKinematics.h"


InvKinematics::InvKinematics(double radious_base, double radious_platform, double gamma_base, double gamma_platform,
        double h, double actuator_min, double actuator_nominal, double actuator_max)
        : rb(radious_base), rp(radious_platform), aNom(actuator_nominal), aMin(actuator_min), aMax(actuator_max),
        height(h), gamma_B(gamma_base), gamma_P(gamma_platform){}

    //Frame() Function Defines geometry of Stewart Platform
    std::tuple<std::array<std::array<double, 6>, 3>, std::array<std::array<double, 6>, 3>, double> InvKinematics::frame() {
        //Base angles for each motor
        std::array<double, 6> phi_B = { 7 * M_PI / 6 + gamma_B, 7 * M_PI / 6 - gamma_B,
                                       M_PI / 2 + gamma_B, M_PI / 2 - gamma_B,
                                       11 * M_PI / 6 + gamma_B, 11 * M_PI / 6 - gamma_B };
        //Platform angles for each motor
        std::array<double, 6> phi_P = { 3 * M_PI / 2 - gamma_P, 5 * M_PI / 6 + gamma_P,
                                       5 * M_PI / 6 - gamma_P, M_PI / 6 + gamma_P,
                                       M_PI / 6 - gamma_P, 3 * M_PI / 2 + gamma_P };
        
        //Coordinates of the points where servo arms are attached
        for (int i = 0; i < 6; ++i) {
            B[0][i] = rb * cos(phi_B[i]);
            B[1][i] = rb * sin(phi_B[i]);
            B[2][i] = 0;
            P[0][i] = rp * cos(phi_P[i]);
            P[1][i] = rp * sin(phi_P[i]);
            P[2][i] = 0;
        }

        double Bx = B[0][0], By = B[1][0], Px = P[0][0], Py = P[1][0];
        //Height of platform
        double H = sqrt(aNom * aNom - ((Px - Bx) * (Px - Bx) + (Py - By) * (Py - By)));
        //Should check if H is -1 here
        return { B, P, H };
    }

    //Rotation matrices for X, Y, Z axis
    std::array<std::array<double, 3>, 3> InvKinematics::rotX(double theta) {
        theta = theta * M_PI / 180.0;
        return { {{1, 0, 0}, {0, cos(theta), -sin(theta)}, {0, sin(theta), cos(theta)}} };
    }

    std::array<std::array<double, 3>, 3> InvKinematics::rotY(double theta) {
        theta = theta * M_PI / 180.0;
        return { {{cos(theta), 0, sin(theta)}, {0, 1, 0}, {-sin(theta), 0, cos(theta)}} };
    }

    std::array<std::array<double, 3>, 3> InvKinematics::rotZ(double theta) {
        theta = theta * M_PI / 180.0;
        return { {{cos(theta), -sin(theta), 0}, {sin(theta), cos(theta), 0}, {0, 0, 1}} };
    }


std::array<double, 6> InvKinematics::solve(std::array<double, 3> translation, std::array<double, 3> rotation) {
    InvKinematics ik(0.1, 0.05, 30, 30);
    std::array<double, 6> l;
    l = { 0.0,0.0,0.0,0.0,0.0,0.0 };
    auto [B, P, H] = ik.frame();

    //Calculate rotation matrix: matrix multiply rotX(thetax) * ...y * ...z

    //Calculate leg length by applying transformations (see notes)

    std::cout << "Calculated height: " << H << std::endl;
    return l;
}
