#pragma once
#ifndef TIMERISR_H_
#define TIMERISR_H_

#include <Arduino.h>

hw_timer_t* timer = NULL;
// Create semaphore to inform us when the timer has fired
volatile SemaphoreHandle_t timerSemaphore;

void ARDUINO_ISR_ATTR onTimer() {
	// Give a semaphore that we can check in the state machine
	xSemaphoreGiveFromISR(timerSemaphore, NULL);
}

void setupTimer() {
	timerSemaphore = xSemaphoreCreateBinary();
	timer = timerBegin(0, 80, true);
	timerAttachInterrupt(timer, &onTimer, true);
}

#endif /* TIMERISR_H_ */
