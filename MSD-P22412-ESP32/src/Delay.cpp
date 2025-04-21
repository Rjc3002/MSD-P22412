#include "Delay.h"

void ntDelay(byte t) {    // non timer delay in milliseconds
    for (byte i = 0; i < t; i++) {
        delayMicroseconds(1000);
    }
}