
#include <Arduino.h>

#ifndef LED_BUILTIN
#define LED_BUILTIN 13
#endif


void setup() {
	pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
	static bool s;
	s = !s;
	digitalWrite(LED_BUILTIN, s);
	delay(1000/2/2); // 2 Hz blink
}