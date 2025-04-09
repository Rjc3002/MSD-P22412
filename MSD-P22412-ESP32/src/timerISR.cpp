#include "timerISR.h"

hw_timer_t* timer = NULL;
// Create semaphore to inform us when the timer has fired
volatile SemaphoreHandle_t timerSemaphore;

void ARDUINO_ISR_ATTR onTimer() {
	// Give a semaphore that we can check in the state machine
	Serial.println("In ISR");
	xSemaphoreGiveFromISR(timerSemaphore, NULL);
}

void setupTimer() {
	timerSemaphore = xSemaphoreCreateBinary();
	timer = timerBegin(0, 80, true);
	timerAttachInterrupt(timer, &onTimer, true);
}
