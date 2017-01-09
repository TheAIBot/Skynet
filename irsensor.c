#include "includes/irsensor.h"


//Constants used by the different IR sensors.
static double[IR_SENSOR_COUNT] írSensorConstants; //Temp name

double irDistance(IRSensor sensor){
	int sensorIntensity = irsensor->data[sensor];
	double distance = 16/(sensorIntensity - 10);
	//double distance = írSensorConstants[sensor].Ka/(sensorIntensity - írSensorConstants[sensor].Kb);
	return distance;
}

void testIrDistance(){
	do
	{
		printf("%f, %f, %f, %f, %f \n", irDistance(IRSensor.left), irDistance(IRSensor.front_left),irDistance(IRSensor.front_middle), 
			                            irDistance(IRSensor.front_right), irDistance(IRSensor.right));
	} while (1);
}