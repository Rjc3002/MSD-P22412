#include "InvKinematics.h"

class InvKinematics {
private:
    double rb, rp, gamma_B, gamma_P;
    double aNom, aMin, aMax, height;
    bool advanced;
    std::array<std::array<double, 6>, 3> B, P, L;

public:
    InvKinematics(double radious_base, double radious_platform, double gamma_base, double gamma_platform,
        bool adv = false, double h = 0.1, double actuator_min = 0, double actuator_nominal = 0, double actuator_max = 0)
        : rb(radious_base), rp(radious_platform), aNom(actuator_nominal), aMin(actuator_min), aMax(actuator_max),
        height(h), advanced(adv) {
        gamma_B = gamma_base * M_PI / 180.0;
        gamma_P = gamma_platform * M_PI / 180.0;
    }

    std::tuple<std::array<std::array<double, 6>, 3>, std::array<std::array<double, 6>, 3>, double> frame() {
        std::array<double, 6> phi_B = { 7 * M_PI / 6 + gamma_B, 7 * M_PI / 6 - gamma_B,
                                       M_PI / 2 + gamma_B, M_PI / 2 - gamma_B,
                                       11 * M_PI / 6 + gamma_B, 11 * M_PI / 6 - gamma_B };

        std::array<double, 6> phi_P = { 3 * M_PI / 2 - gamma_P, 5 * M_PI / 6 + gamma_P,
                                       5 * M_PI / 6 - gamma_P, M_PI / 6 + gamma_P,
                                       M_PI / 6 - gamma_P, 3 * M_PI / 2 + gamma_P };

        for (int i = 0; i < 6; ++i) {
            B[0][i] = rb * cos(phi_B[i]);
            B[1][i] = rb * sin(phi_B[i]);
            B[2][i] = 0;
            P[0][i] = rp * cos(phi_P[i]);
            P[1][i] = rp * sin(phi_P[i]);
            P[2][i] = 0;
        }

        double Bx = B[0][0], By = B[1][0], Px = P[0][0], Py = P[1][0];
        double H = sqrt(aNom * aNom - ((Px - Bx) * (Px - Bx) + (Py - By) * (Py - By)));

        return { B, P, H };
    }

    std::array<std::array<double, 3>, 3> rotX(double theta) {
        theta = theta * M_PI / 180.0;
        return { {{1, 0, 0}, {0, cos(theta), -sin(theta)}, {0, sin(theta), cos(theta)}} };
    }

    std::array<std::array<double, 3>, 3> rotY(double theta) {
        theta = theta * M_PI / 180.0;
        return { {{cos(theta), 0, sin(theta)}, {0, 1, 0}, {-sin(theta), 0, cos(theta)}} };
    }

    std::array<std::array<double, 3>, 3> rotZ(double theta) {
        theta = theta * M_PI / 180.0;
        return { {{cos(theta), -sin(theta), 0}, {sin(theta), cos(theta), 0}, {0, 0, 1}} };
    }
};

int main() {
    InvKinematics ik(0.1, 0.05, 30, 30);
    auto [B, P, H] = ik.frame();
    std::cout << "Calculated height: " << H << std::endl;
    return 0;
}
