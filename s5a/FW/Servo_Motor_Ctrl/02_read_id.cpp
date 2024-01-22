
///////////////////////////////////////////////////////////////////////////////

#include <Arduino.h>

#include "type_shorts.h"

#ifndef LED_BUILTIN
#define LED_BUILTIN 13
#endif

//////////////////

void setup() {
	// WARNING: Do not print anything if not asked for by protocol.
	Serial.begin(115200);
	
	// Error is indicated by this LED off.
	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, 1);

	// Find out ID.
	pinMode(A5, OUTPUT);
	digitalWrite(A5, 0);
	for(i8 pin = A4; pin >= A0; pin--){
		pinMode(pin, INPUT_PULLUP);
	}
	
}

void loop() {
	u8 id = 0;
	for(i8 pin = A4; pin >= A0; pin--){
		id <<= 1;
		id |= !digitalRead(pin);
	}
	Serial.println((int)id);
}
