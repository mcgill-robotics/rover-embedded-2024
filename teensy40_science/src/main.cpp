#include <Arduino.h>

// put function declarations here:
int myFunction(int, int);
#define STEP_DIR 7
#define STEP 8
#define M0 10
#define M1 9
#define STEP_DELAY 2000
#define STEP_DELAY 1000
#define STEP_PER_REV 200
void setup()
{
	// Declare pins as Outputs
	pinMode(STEP, OUTPUT);
	pinMode(STEP_DIR, OUTPUT);
  pinMode(M0, OUTPUT);
  pinMode(M1, OUTPUT);
  delay(1000); // Wait a second
}
void loop()
{
	// Set motor direction clockwise
	digitalWrite(STEP_DIR, HIGH);

	// Spin motor slowly
	for(int x = 0; x < STEP_PER_REV; x++)
	{
		digitalWrite(STEP, HIGH);
		delayMicroseconds(STEP_DELAY);
		digitalWrite(STEP, LOW);
		delayMicroseconds(STEP_DELAY);
	}
	delay(1000); // Wait a second
	
	// Set motor direction counterclockwise
	digitalWrite(STEP_DIR, LOW);

	// Spin motor quickly
	for(int x = 0; x < STEP_PER_REV; x++)
	{
		digitalWrite(STEP, HIGH);
		delayMicroseconds(STEP_DELAY);
		digitalWrite(STEP, LOW);
		delayMicroseconds(STEP_DELAY);
	}
	delay(1000); // Wait a second
}
