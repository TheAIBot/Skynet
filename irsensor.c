#include "includes/irsensor.h"

double irDistance(IRSensor sensor, double Ka, double Kb){
	int sensorIntensity = irsensor->data[sensor];
	double distance = Ka/(sensorIntensity - Kb);
	return distance;
}

