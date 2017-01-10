#include <stdio.h>
#include <sys/time.h>
#include <stdint.h>
#include "includes/irsensor.h"
#include "includes/robotconnector.h"

/*Constants used by the different IR sensors.*/
static irSensorCalibrationData irSensorCalibData[IR_SENSOR_COUNT];

int loadIRCalibrationData(const char* fileLoc)
{
	FILE* file = fopen(fileLoc, "r");

	if (file == NULL)
	{
		printf("%s NOT FOUND!\n", fileLoc);
		return 0;
	}

	int i;
	for (i = 0; i < IR_SENSOR_COUNT; i++)
	{
		double Ka;
		double Kb;
		const int scanStatus = fscanf(file, "%lf %lf\n", &Ka, &Kb);
		if (scanStatus != 2) //Check if the correct number of items was read
		{
			printf("Error occured when reading linesensor calibration file. %d numbers expected, but %d was found.", 2, scanStatus);
			return 0;
		}
		irSensorCalibData[i].Ka = Ka;
		irSensorCalibData[i].Kb = Kb;
	}

	fclose(file);
	return 1;
}

double irDistance(enum IRSensor sensor)
{
	int sensorIntensity = irsensor->data[sensor];
	return irSensorCalibData[sensor].Ka / (sensorIntensity - irSensorCalibData[sensor].Kb);
}
