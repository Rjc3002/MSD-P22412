#pragma once
#ifndef TIMERISR_H_
#define TIMERISR_H_

#include <Arduino.h>

extern hw_timer_t* timer;

extern volatile SemaphoreHandle_t timerSemaphore;

void ARDUINO_ISR_ATTR onTimer();

void setupISRFlag();

void setupTimer();

void stopTimer();

void startTimer(double delay);

#endif /* TIMERISR_H_ */
