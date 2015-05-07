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

uint16_t lineSensorValues[5] = {0, 0, 0, 0, 0};

//Constants

void setup() {
	//Initialize proximity and line sensors
	proxSensors.initThreeSensors();
	lineSensors.initThreeSensors();
}

void loop() {
	//Get the readings
	proxSensors.read();
	lineSensors.readCalibrated(lineSensorValues);
}
