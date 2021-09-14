#include "src/basics/io.h"
#include "tramControl.h"

void setup()
{
	initIO();
	Serial.begin(115200);
	tramControlInit();
}

void loop()
{
	tramControl();
}