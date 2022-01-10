#include "src/io.h"
#include "tramControl.h"
#include "src/date.h"

void setup()
{
	initIO();
	Serial.begin(115200);
	Serial.println(date);
	tramControlInit();

	// digitalWrite( IN1, HIGH ) ;
	// digitalWrite( IN2,  LOW ) ;
	// digitalWrite(throttlePin, HIGH) ;
	// delay( 100000 ) ;
}

void loop()
{
	tramControl();
}