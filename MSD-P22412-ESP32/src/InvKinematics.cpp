#include "InvKinematics.h"


InvKinematics::InvKinematics(double radious_base, double radious_platform, double gamma_base, double gamma_platform,
        double actuator_min, double actuator_nominal, double actuator_max)
        : rb(radious_base), rp(radious_platform), aNom(actuator_nominal), aMin(actuator_min), aMax(actuator_max),
        gamma_B(gamma_base), gamma_P(gamma_platform){}

//Matrix multiplication helper function for NxM times MxP Matrixes
template <size_t N, size_t M, size_t Q>
std::array<std::array<double, Q>, N> InvKinematics::MatMul(const std::array<std::array<double, M>, N>& X, const std::array<std::array<double, Q>, M>& Y){
    std::array<std::array<double, Q>, N> result = {};

    for (size_t i = 0; i < N; ++i) {
        for (size_t j = 0; j < Q; ++j) {
            double sum = 0.0;
            for (size_t k = 0; k < M; ++k) {
                sum += X[i][k] * Y[k][j];
            }
            result[i][j] = sum;
        }
    }
    return result;
}

// Helper function to find the Euclidean norm of a 3x6 matrix along axis 0
std::array<double, 6> InvKinematics::Norm(const std::array<std::array<double, 6>, 3>& matrix) {
    std::array<double, 6> norm = {};
    for (size_t j = 0; j < 6; ++j) {
        norm[j] = std::sqrt(matrix[0][j] * matrix[0][j] +
            matrix[1][j] * matrix[1][j] +
            matrix[2][j] * matrix[2][j]);
    }
    return norm;
}

    //Frame() Function Defines geometry of Stewart Platform
    std::tuple<std::array<std::array<double, 6>, 3>, std::array<std::array<double, 6>, 3>, double> InvKinematics::frame() {
        //Base angles for each motor
        std::array<double, 6> phi_B = { 7.0 * M_PI / 6.0 + gamma_B, 7.0 * M_PI / 6.0 - gamma_B,
                                       M_PI / 2.0 + gamma_B, M_PI / 2.0 - gamma_B,
                                       11.0 * M_PI / 6.0 + gamma_B, 11.0 * M_PI / 6.0 - gamma_B };
        //Platform angles for each motor
        std::array<double, 6> phi_P = { 3.0 * M_PI / 2.0 - gamma_P, 5.0 * M_PI / 6.0 + gamma_P,
                                       5.0 * M_PI / 6.0 - gamma_P, M_PI / 6.0 + gamma_P,
                                       M_PI / 6.0 - gamma_P, 3.0 * M_PI / 2.0 + gamma_P };
        
		//Frames of the points where servo arms are attached on Base [B] and Platform [P]
        for (int i = 0; i < 6; ++i) {
            B[0][i] = rb * cos(phi_B[i]);
            B[1][i] = rb * sin(phi_B[i]);
            B[2][i] = 0.0;
            P[0][i] = rp * cos(phi_P[i]);
            P[1][i] = rp * sin(phi_P[i]);
            P[2][i] = 0.0;
        }

		//Take one frame from the base and platform to calculate the height of the platform
        double Bx = B[0][0], By = B[1][0], Px = P[0][0], Py = P[1][0];
        double H = sqrt(aNom * aNom - ((Px - Bx) * (Px - Bx) + (Py - By) * (Py - By)));
        //Serial.print("Height: ");
        //Serial.println(H, 7);
   
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

	//Check if the lengths of the legs are within the range of the actuators
	bool InvKinematics::checkLengths(std::array<double, 6> lengths) {
		for (size_t i = 0; i < 6; ++i) {
			if (lengths[i] < aMin || lengths[i] > aMax) {
				return false;
			}
		}
		return true;
	}


    double InvKinematics::calculatePointOnPath(int steps, double thetaInitial, double thetaFinal, int point) {
        double theta = 0.0;
        double A, B = 0.0;

        A = -2 * (thetaFinal - thetaInitial) / double(steps * steps * steps);
        B = 3 * (thetaFinal - thetaInitial) / double(steps * steps);

		theta = A * point * point * point + B * point * point + thetaInitial;

        return theta;
    }

std::array<double, 6> InvKinematics::solve(std::array<double, 3> translation, std::array<double, 3> rotation) {
	//Get base mounting frames, platform mounting frames, and height of platform
    auto [B, P, H] = this->frame();

    //Calculate total rotation matrix: matrix multiply rotX(thetax) * ...y * ...z
    std::array<std::array<double, 3>, 3> rotMatrix = this->MatMul(this->MatMul(this->rotX(rotation[0]), this->rotY(rotation[1])), this->rotZ(rotation[2]));
    
    std::array<std::array<double, 6>, 3> l = {};
    std::array<double, 3> home = { 0.0, 0.0, H };

    //To find l values (x,y,z) for each leg: l = T + H + (R*P) - B
    //T+H
    for (size_t i = 0; i < 3; ++i) {
        for (size_t j = 0; j < 6; ++j) {
            l[i][j] = translation[i] + home[i];
        }
    }

    //R*P
    std::array<std::array<double, 6>, 3> RP = MatMul(rotMatrix, P);

    //(R*P) - B
    for (size_t i = 0; i < 3; ++i) {
        for (size_t j = 0; j < 6; ++j) {
            l[i][j] += RP[i][j] - B[i][j];
        }
    }

    // magnitude of l  = sqrt(lx^2 + ly^2 + lz^2)
    std::array<double, 6> lll = Norm(l);

    return lll;
}
