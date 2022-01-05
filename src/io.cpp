#include <Arduino.h>
#include "io.h"
extern void initIO(void) {
	pinMode(IN2, OUTPUT);
	pinMode(IN1, OUTPUT);
	pinMode(throttlePin, OUTPUT);
	pinMode(statusLedFront, OUTPUT);
	pinMode(statusLedRear, OUTPUT);
	pinMode(detectorPin, INPUT_PULLUP);
	pinMode(leftServoPin, OUTPUT);
	pinMode(rightServoPin, OUTPUT);
	pinMode(JP1, INPUT_PULLUP);
	pinMode(JP2, INPUT_PULLUP);
	pinMode(JP3, INPUT_PULLUP);
	pinMode(relayPin, OUTPUT);
	pinMode(shortCircuitPin, INPUT);
	pinMode(auto_manualPin, INPUT_PULLUP);
	pinMode(frontSwPin, INPUT_PULLUP);
	pinMode(rearSwPin, INPUT_PULLUP);
	pinMode(speedPin, INPUT);
	pinMode(frontBrakeSpeed, INPUT);
	pinMode(rearBrakeSpeed, INPUT);
}