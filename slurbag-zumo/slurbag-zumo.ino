#include <Wire.h>
#include <Zumo32U4.h>

Zumo32U4LCD lcd;
Zumo32U4Motors motors;
Zumo32U4LineSensors lineSensors;
Zumo32U4ProximitySensors proxSensors;
Zumo32U4Encoders encoders;
L3G gyro;
Zumo32U4ButtonA buttonA;
Zumo32U4ButtonB buttonB;
Zumo32U4ButtonC buttonC;

uint16_t lineSensorValues[3] = {0, 0, 0};

//Constants
#define GYRO_MULTIPLIER		8.75


//Above this value is empty space
#define LINE_THRESHOLD		500 //This might need to be tweaked to account for lighting conditions
#define TARGET_THRESHOLD		4 //Adjust accordingly
#define ATTACK_THRESHOLD		5

//For speeds, 0 is stopped, 400 is full speed
#define ROTATE_SPEED	100
#define HUNT_SPEED		110
#define HUNT_ARC_FACTOR	38
#define TARGET_SPEED	200
#define SPIN_SPEED		50
#define ATTACK_SPEED	400

//States
#define STATE_HUNT		0
#define STATE_TARGET	1
#define STATE_ATTACK	2
#define STATE_SURVIVE	3

uint16_t state = STATE_HUNT; //Always start off in the hunting state

bool searchClockwise = false;

//Utility functions
bool overLeft() {
	return lineSensorValues[0] > LINE_THRESHOLD;
}

bool overRight() {
	return lineSensorValues[2] > LINE_THRESHOLD;
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
void rotate(int degrees) {
	//TODO Ugh oh god please rewrite this
	double degreesTraveled = 0.0;
	long prevTime, currTime;
	
	if (degrees > 0) {
		motors.setSpeeds(ROTATE_SPEED, -ROTATE_SPEED);
	}
	else {
		motors.setSpeeds(-ROTATE_SPEED, ROTATE_SPEED);
	}

	prevTime = millis();
	while (1) {
		gyro.read();
		currTime = millis();

		//angularVelocity in deg/s
		double angularVelocity = gyro.g.z * GYRO_MULTIPLIER / 1000.0;
		double actualAngle = angularVelocity * ((currTime - prevTime) / 1000.0);

		actualAngle *= -1;
		degreesTraveled += actualAngle;

		if (degrees > 0 && degreesTraveled > (double)degrees) {
			break;
		}
		else if (degrees <= 0 && degreesTraveled < (double)degrees) {
			break;
		}
		
		prevTime = currTime;
	}

}
void setup() {
	Wire.begin();

	//Init the gyro
	if (!gyro.init()) {
		//TODO Output to the LCD
		//busy loop since we can't continue
		while(1) {}
	}
	gyro.enableDefault();
	
	//Initialize proximity and line sensors
	proxSensors.initThreeSensors();
	lineSensors.initThreeSensors();

	lineSensors.read(lineSensorValues);
	searchClockwise = ((lineSensorValues[0] & 0x1) == 0);

	lcd.clear();
	lcd.print(F("Press A"));

	buttonA.waitForButton();

	for (int i = 5; i >= 1; i--) {
		lcd.clear();
		lcd.print(i);
		delay(1000);
	}
	
	lcd.clear();
	lcd.print(F("HUNT"));
}

void loop() {
	//Get the readings
	proxSensors.read();
	lineSensors.read(lineSensorValues);

	bool isOverLeft = overLeft();
	bool isOverRight = overRight();
	uint8_t leftValue = proxSensors.countsFrontWithLeftLeds();
	uint8_t rightValue = proxSensors.countsFrontWithRightLeds();
	uint8_t rangeAverage = (leftValue + rightValue) / 2;

	lcd.clear();
	lcd.gotoXY(0,1);
	lcd.print(leftValue);
	lcd.print(' ');
	lcd.print(rightValue);
	
	
	bool targetSeen = leftValue > TARGET_THRESHOLD || rightValue > TARGET_THRESHOLD;

	if (isOverLeft || isOverRight) {
		lcd.gotoXY(0,0);
		lcd.print(F("SURVIVE"));
		state = STATE_SURVIVE;
	}

	switch (state) {
		case STATE_SURVIVE: {
			if (isOverLeft) {
				rotate(135);
				searchClockwise = true;
			}
			else if (isOverRight) {
				rotate(-135);
				searchClockwise = false;
			}
			lcd.gotoXY(0,0);
			lcd.print(F("HUNT"));
			state = STATE_HUNT;
		} break;

		case STATE_HUNT: {
			if (rangeAverage > TARGET_THRESHOLD) {
				lcd.gotoXY(0,0);
				lcd.print(F("TARGET"));
				state = STATE_TARGET;
			}
			else if (searchClockwise) {
				arc(HUNT_SPEED, HUNT_ARC_FACTOR);
			}
			else {
				arc(HUNT_SPEED, -HUNT_ARC_FACTOR);
			}
		} break;

		case STATE_TARGET: {
			if (leftValue < rightValue) {
				//Right value is greater, turn to the right
				spin(SPIN_SPEED);
			}
			else if (leftValue > rightValue) {
				spin(-SPIN_SPEED);
			}
			else if (rangeAverage > ATTACK_THRESHOLD) {
				lcd.gotoXY(0,0);
				lcd.print(F("ATTACK"));
				state = STATE_ATTACK;
			}
			else if (rangeAverage > TARGET_THRESHOLD) {
				forward(TARGET_SPEED);
			}
			else {
				lcd.gotoXY(0,0);
				lcd.print(F("HUNT"));
				state = STATE_HUNT;
			}
		} break;

		case STATE_ATTACK: {
			forward(ATTACK_SPEED);
		} break;
	}
}
