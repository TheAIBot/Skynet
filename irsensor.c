#include <stdio.h>
#include "includes/irsensor.h"
#include <sys/time.h>
#include <stdint.h>
#include "includes/robotconnector.h"

/*Constants used by the different IR sensors.*/
static irSensorCalibrationData irSensorCalibData[IR_SENSOR_COUNT]; //Temp name

int loadIRCalibrationData(const char* fileLoc){
	FILE* file = fopen(fileLoc, "r");
	if (file == NULL) {// Check if the give file is found		
		// If not display an error and return an error value
		printf("%s NOT FOUND!\n", fileLoc);
		return 0;
	}
	//Error the data value pair for each sensor
	int i;
	for (i = 0; i < IR_SENSOR_COUNT; i++)	{
		double Ka, Kb;
		const int scanStatus = fscanf(file, "%lf %lf\n", &Ka, &Kb);
		if (scanStatus != 2) { //Check if the correct number of items was read
			printf("Error occured when reading linesensor calibration file. %d numbers expected, but %d was found.", 2, scanStatus);
			return 0;
		}
		irSensorCalibData[i].Ka = Ka;
		irSensorCalibData[i].Kb = Kb;
	}
	fclose(file);
	return 1;
}

double irDistance(enum IRSensor sensor){
	int sensorIntensity = irsensor->data[sensor];
	double distance = irSensorCalibData[sensor].Ka/(sensorIntensity - irSensorCalibData[sensor].Kb);
	return distance;
}

void testIRDistance(){
	do {
		//printf("Sensor 0 = %d, Sensor 1 = %d, Sensor 2 = %d, Sensor 3 = %d\n",irsensor->data[0],irsensor->data[1],irsensor->data[2],irsensor->data[3]);
		printf("%f, %f, %f, %f, %f \n", irDistance(ir_left), irDistance(ir_front_left),irDistance(ir_front_middle), 
			                            irDistance(ir_front_right), irDistance(ir_right));
		rhdSync();
	} while (1);
}

void measureDistance(odotype *odo){
	int i;
	double sum=0;
	for(i=0;i<100;i++){
		syncAndUpdateOdo(odo);
		sum+=irDistance(ir_front_left)+irDistance(ir_front_middle)+irDistance(ir_front_right);
	}
	 printf("%f \n", sum/300);
}
