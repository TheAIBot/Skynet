#ifndef IRSENSOR_H_
#define IRSENSOR_H_

#define IR_SENSOR_COUNT 5
#define numberRequiredForPillarDetected 5

enum IRSensor{
	ir_left = 0, ir_front_left, ir_front_middle, ir_front_right, ir_right
};

typedef struct
{
	double Ka;
	double Kb;
} irSensorCalibrationData;

int loadIRCalibrationData(const char* fileLoc);
double irDistance(enum IRSensor sensor);
void testIRDistance();

#endif
