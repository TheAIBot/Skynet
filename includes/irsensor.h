#ifndef IRSENSOR_H_
#define IRSENSOR_H_

#include "robotconnector.h"

enum IRSensor
{
	left = 0, front_left = 1,front_middle = 2, front_right = 3, right = 4
};

typedef struct{
	double Ka;
	double Kb;
}irSensorConstant;

double irDistance(IRSensor sensor, double Ka, double Kb);

#endif
