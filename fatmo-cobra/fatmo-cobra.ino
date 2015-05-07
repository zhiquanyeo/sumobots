#include <Adafruit_NECremote.h>
#include <SharpIR.h>

#define LINE_SNSR_F_L 	0
#define LINE_SNSR_F_R 	1
#define LINE_SNSR_R		2

#define LINE_SNSR_F_L_PIN	A0
#define LINE_SNSR_F_R_PIN	A1
#define LINE_SNSR_R_PIN		A2

#define DIST_SNSR_L_PIN		A3
#define DIST_SNSR_R_PIN		A4

#define LEFT_MOTOR_DIR 8
#define RIGHT_MOTOR_DIR 10
#define LEFT_MOTOR_SPEED 9
#define RIGHT_MOTOR_SPEED 11

#define IR_PIN 4

#define IR_CODE_START	9 //The Enter/Save button

//Anything less than this is WHITE, anything more is BLACK
#define DEFAULT_SENSOR_THRESHOLD 1000

//Movement Constants
#define ROTATE_SPEED	64
#define ROTATE_FACTOR	8  //ms/deg
#define HUNT_SPEED		70
#define HUNT_ARC_FACTOR	24
#define TARGET_SPEED	128
#define SPIN_SPEED		32
#define ATTACK_SPEED	255

//Attacking/Tracking Constants
#define TARGET_THRESHOLD	20 //in cm
#define ATTACK_THRESHOLD	6 //in cm

//State Machine
#define STATE_SURVIVE		0
#define STATE_HUNT			1
#define STATE_TARGET		2
#define STATE_ATTACK		3

int state = STATE_HUNT;
bool searchClockwise;

//Stores the line sensor readings
int line_sensor_readings[3] = {0,0,0};

//Sensor Classes
SharpIR leftDistSensor(DIST_SNSR_L_PIN, 25, 93, 1080);
SharpIR rightDistSensor(DIST_SNSR_R_PIN, 25, 93, 1080);
//Distance is in CM

//IR Module
Adafruit_NECremote remote(IR_PIN);

//======== Sensor Methods ==========

/**
 * This function obtains a reading from the specified
 * QTI sensor.
 * 
 * The lower the value, the darker the object
 */
long RCtime(int pin) {
	long result = 0;
	pinMode(pin, OUTPUT);
	digitalWrite(pin, HIGH); //Send a high signal to the pin
	delay(1);
	pinMode(pin, INPUT);
	digitalWrite(pin, LOW); //Turn pullups off

	while (digitalRead(pin)) { //wait for the pin to go low
		result++;
	}

	return result;
}

void get_line_sensor_readings() {
	line_sensor_readings[LINE_SNSR_F_L] = RCtime(LINE_SNSR_F_L_PIN);
	line_sensor_readings[LINE_SNSR_F_R] = RCtime(LINE_SNSR_F_R_PIN);
	line_sensor_readings[LINE_SNSR_R] = RCtime(LINE_SNSR_R_PIN);
}



bool overFrontLeft() {
	return line_sensor_readings[LINE_SNSR_F_L] >= DEFAULT_SENSOR_THRESHOLD;
}

bool overFrontRight() {
	return line_sensor_readings[LINE_SNSR_F_R] >= DEFAULT_SENSOR_THRESHOLD;
}

bool overRear() {
	return line_sensor_readings[LINE_SNSR_R] >= DEFAULT_SENSOR_THRESHOLD;
}


void setLeftMotorSpeed(int speed) {
	if (speed < 0) {
		digitalWrite(LEFT_MOTOR_DIR, LOW);
		speed *= -1;
	}
	else {
		digitalWrite(LEFT_MOTOR_DIR, HIGH);
	}
	analogWrite(LEFT_MOTOR_SPEED, speed);
}

void setRightMotorSpeed(int speed) {
	if (speed < 0) {
		digitalWrite(RIGHT_MOTOR_DIR, LOW);
		speed *= -1;
	}
	else {
		digitalWrite(RIGHT_MOTOR_DIR, HIGH);
	}
	analogWrite(RIGHT_MOTOR_SPEED, speed);
}

//TODO Switch this function to use a switch whenever we wire it up
/*
void waitForStart() {
	Serial.println("Hello. Send 'A' to Go");

	while (Serial.read() != 'A') {
		delay(10);
	}

	Serial.println("Starting in");
	for (int i = 5; i >= 1; i--) {
		Serial.println(i);
		delay(1000);
	}
	Serial.println("GO!");
}
*/

void waitForStart() {
	Serial.println("Hello. Hit 'Enter/Save' on remote to Go");

	while (1) {
		int c = remote.listen();
		if (c == IR_CODE_START) {
			break;
		}
	}

	Serial.println("Starting in");
	for (int i = 5; i >= 1; i--) {
		Serial.println(i);
		delay(1000);
	}
	Serial.println("GO!");
}

//===== Movement functions =====
void arc(int speed, int turnFactor) {
	setLeftMotorSpeed(speed + turnFactor);
	setRightMotorSpeed(speed - turnFactor);
}

void rotate(int degrees) {
	Serial.print("Rotating ");
	Serial.println(degrees);
	
	if (degrees > 0) {
		setLeftMotorSpeed(ROTATE_SPEED);
		setRightMotorSpeed(-ROTATE_SPEED);
	}
	else {
		setLeftMotorSpeed(-ROTATE_SPEED);
		setRightMotorSpeed(ROTATE_SPEED);
		degrees *= -1;
	}
	
	delay(degrees * ROTATE_FACTOR);
}

void spin(int speed) {
	setLeftMotorSpeed(speed);
	setRightMotorSpeed(-speed);
}

void forward(int speed) {
	setLeftMotorSpeed(speed);
	setRightMotorSpeed(speed);
}


void setup() {
	//We need to hook up a button, and on press, wait 5 seconds, and then go
	Serial.begin(57600);

	pinMode(LEFT_MOTOR_DIR, OUTPUT);
	pinMode(RIGHT_MOTOR_DIR, OUTPUT);

	get_line_sensor_readings();
	searchClockwise = ((overFrontLeft() & 0x1) == 0);
	waitForStart();
	
}

void printSensors() {
	Serial.print("FL: ");
	Serial.print(line_sensor_readings[0]);
	Serial.print(", FR: ");
	Serial.print(line_sensor_readings[1]);
	Serial.print(", R: ");
	Serial.println(line_sensor_readings[2]);
}

int sanitizeRange(int range) {
	if (range >= 0 && range <= 80) {
		return range;
	}
	else {
		return 80; //range max
	}
}

void loop() {
	get_line_sensor_readings();

	bool overLeft = overFrontLeft();
	bool overRight = overFrontRight();
	int leftRange = sanitizeRange(leftDistSensor.distance());
	int rightRange = sanitizeRange(rightDistSensor.distance());
	int rangeDifference = leftRange - rightRange;
	int rangeAverage = (leftRange + rightRange) / 2;
	/*
	Serial.print("rD: ");
	Serial.print(rangeDifference);
	Serial.print(" rA: ");
	Serial.println(rangeAverage);
	*/
	if (overLeft || overRight) {
		Serial.println("Entering State SURVIVE");
		state = STATE_SURVIVE;
	}

	switch (state) {
		case STATE_SURVIVE: {
			if (overLeft) {
				rotate(135);
				searchClockwise = true;
			}
			else if (overRight) {
				rotate(-135);
				searchClockwise = false;
			}
			Serial.println("Entering State HUNT");
			state = STATE_HUNT;
		} break;

		case STATE_HUNT: {
			if (rangeAverage < TARGET_THRESHOLD) {
				Serial.println("Target is within target threshold distance");
				Serial.println("Entering State TARGET");
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
			//Turn to face the opponent
			//Move toward opponent if straight ahead
			//go into attack mode if opponent is close enough
			if (rangeDifference > TARGET_THRESHOLD) {
				spin(-SPIN_SPEED);
			}
			else if (-rangeDifference > TARGET_THRESHOLD) {
				spin(SPIN_SPEED);
			}
			else if (rangeAverage < ATTACK_THRESHOLD) {
				Serial.println("Target is within attack threshold distance");
				Serial.println("Entering State ATTACK");
				state = STATE_ATTACK;
			}
			else if (rangeAverage < TARGET_THRESHOLD) {
				Serial.println("Moving forward...");
				forward(TARGET_SPEED);
			}
			else {
				Serial.println("*** Lost target ***");
				Serial.println("Entering State HUNT");
				state = STATE_HUNT;
			}
		} break;

		case STATE_ATTACK: {
			forward(ATTACK_SPEED);
		} break;
	}

	
}
