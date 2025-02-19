#pragma once
#ifndef INVKINEMATICS_H_
#define INVKINEMATICS_H_

#include <iostream>
#include <vector>
#include <cmath>
#include <array>
#include <tuple>

class InvKinematics {
private:
	double rb, rp, gamma_B, gamma_P; //Platform variables
	double aNom, aMin, aMax, height; //Actuator variables
	std::array<std::array<double, 6>, 3> B, P, L;
public:
	InvKinematics(double radious_base, double radious_platform, double gamma_base, double gamma_platform,
		double h = 0.1, double actuator_min = 0, double actuator_nominal = 0, double actuator_max = 0);

	template <size_t N, size_t M, size_t Q>
	std::array<std::array<double, Q>, N> MatMul(const std::array<std::array<double, M>, N>& X, const std::array<std::array<double, Q>, M>& Y);
	std::array<double, 6> Norm(const std::array<std::array<double, 6>, 3>& matrix);
	std::tuple<std::array<std::array<double, 6>, 3>, std::array<std::array<double, 6>, 3>, double> frame();
	std::array<std::array<double, 3>, 3> rotX(double theta);
	std::array<std::array<double, 3>, 3> rotY(double theta);
	std::array<std::array<double, 3>, 3> rotZ(double theta);
	std::array<double, 6> solve(std::array<double, 3> translation = { 0, 0, 0 }, std::array<double, 3> rotation = { 0, 0, 0 });
};




#endif /* INVKINEMATICS_H_ */