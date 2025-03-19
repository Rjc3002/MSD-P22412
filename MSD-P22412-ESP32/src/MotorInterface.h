#pragma once
#ifndef MOTORINTERFACE_H_
#define MOTORINTERFACE_H_

#include <vector>
#include <cmath>
#include <array>
#include <Arduino.h>  // Required for Serial functions
#include "InvKinematics.h"
#include "InputParser.h"
#include "PA12_Arduino/PA12.h"
#include "StewartPlatform.h"
#include <algorithm>

extern PA12 myServo;

void setup();
void actuate(const std::vector<std::array<double, 6>>& cmdArray);
void move(std::array<int, 12> moveArr);
std::array<double, 6> readPos();

#endif /* MOTORINTERFACE_H_ */