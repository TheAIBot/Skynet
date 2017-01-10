#include <stdio.h>
#include "includes/robotconnector.h"
#include "includes/irsensor.h"


/*Constants used by the different IR sensors.*/
static irSensorCalibrationData irSensorCalibData[IR_SENSOR_COUNT]; //Temp name

double irDistance(enum IRSensor sensor){
	int sensorIntensity = irsensor->data[sensor];
	double distance = 16/(sensorIntensity - 10);
	//double distance = írSensorConstants[sensor].Ka/(sensorIntensity - írSensorConstants[sensor].Kb);
	return distance;
}

void testIRDistance(){
	do
	{
		printf("%f, %f, %f, %f, %f \n", irDistance(ir_left), irDistance(ir_front_left),irDistance(ir_front_middle), 
			                            irDistance(ir_front_right), irDistance(ir_right));
	} while (1);
}
