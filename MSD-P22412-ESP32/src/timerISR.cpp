#include "timerISR.h"

hw_timer_t* timer = NULL;
// Create semaphore to inform us when the timer has fired
volatile SemaphoreHandle_t timerSemaphore;

void ARDUINO_ISR_ATTR onTimer() {
	// Give a semaphore that we can check in the state machine
	Serial.println("In ISR");
	xSemaphoreGiveFromISR(timerSemaphore, NULL);
}

void setupISRFlag() {
	timerSemaphore = xSemaphoreCreateBinary();
}

void setupTimer() {
	timer = timerBegin(2, 80, true);
	timerAttachInterrupt(timer, &onTimer, true);
}

void stopTimer() {
	//Serial.println("Stopping timer");
	timerAlarmDisable(timer);
	timerStop(timer);
	timerWrite(timer, 0);
}

void startTimer(double delay) {
	Serial.println("Starting timer");
	timerAlarmWrite(timer, delay * 1000, false);
	timerAlarmEnable(timer);
	timerStart(timer);
}
