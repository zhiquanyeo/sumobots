#include <Wire.h>
#include <Zumo32U4.h>

Zumo32U4LCD lcd;
Zumo32U4Motors motors;
Zumo32U4LineSensors lineSensors;
Zumo32U4ProximitySensors proxSensors;
Zumo32U4Encoders encoders;
Zumo32U4ButtonA buttonA;
Zumo32U4ButtonB buttonB;
Zumo32U4ButtonC buttonC;

uint16_t lineSensorValues[3] = {0, 0, 0};

//Constants
//Above this value is empty space
#define LINE_THRESHOLD		500 //This might need to be tweaked to account for lighting conditions

//For speeds, 0 is stopped, 400 is full speed


//States
#define STATE_HUNT		0
#define STATE_TARGET	1
#define STATE_ATTACK	2
#define STATE_SURVIVE	3

uint16_t state = STATE_HUNT; //Always start off in the hunting state

bool searchClockwise = false;

//Utility functions
bool overLeft() {
	return lineSensorValues[0] < LINE_THRESHOLD;
}

bool overRight() {
	return lineSensorValues[2] < LINE_THRESHOLD;
}

//=== Movement Functions

/*
 * Move forward, in an arc
 * Positive factor = arc right, negative factor = arc left
 */
void arc(int speed, int factor) {
	motors.setSpeeds(speed + factor, speed - factor);
}

/*
 * Spin in place at a certain speed
 * Positive speed = spin clockwise, negative = spin counterclockwise
 */
void spin(int speed) {
	motors.setSpeeds(speed, -speed);
}

/*
 * Move forward at a certain speed
 */
void forward(int speed) {
	motors.setSpeeds(speed, speed);
}

/*
 * Turn at a predefined turn speed, for a certain number of degrees
 * Positive degrees = clockwise, negative = counterclockwise
 */
void turn(int degrees) {
	//TODO: Implement after hooking up to Gyro
}
void setup() {
	//Initialize proximity and line sensors
	proxSensors.initThreeSensors();
	lineSensors.initThreeSensors();
}

void loop() {
	//Get the readings
	proxSensors.read();
	lineSensors.read(lineSensorValues);
}
