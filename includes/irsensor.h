#ifndef IRSENSOR_H_
#define IRSENSOR_H_

#include <stdio.h>
#include "includes/robotconnector.h"

#define IR_SENSOR_COUNT 5

enum IRSensor
{
	ir_left = 0, ir_front_left = 1, ir_front_middle = 2, ir_front_right = 3, ir_right = 4
};

typedef struct{
	double Ka;
	double Kb;
}irSensorConstant;

double irDistance(enum IRSensor sensor);
void testIRDistance();

#endif
