#include <stdio.h>
#include <sys/time.h>
#include <stdint.h>
#include "includes/irsensor.h"
#include "includes/robotconnector.h"

static irSensorCalibrationData irSensorCalibData[IR_SENSOR_COUNT];

/*
 * Loads calibration data for ir sensor from fileLoc
 */
bool loadIRCalibrationData(const char* fileLoc)
{
	FILE* file = fopen(fileLoc, "r");
	if (file == NULL)
	{
		printf("%s NOT FOUND!\n", fileLoc);
		return false;
	}
	//Error the data value pair for each sensor
	for (int i = 0; i < IR_SENSOR_COUNT; i++)
	{
		double Ka, Kb;
		const int scanStatus = fscanf(file, "%lf %lf\n", &Ka, &Kb);
		if (scanStatus != 2) //Check if the correct number of items was read
		{
			printf("Error occured when reading linesensor calibration file. %d numbers expected, but %d was found.", 2, scanStatus);
			return false;
		}
		irSensorCalibData[i].Ka = Ka;
		irSensorCalibData[i].Kb = Kb;
	}
	fclose(file);
	return true;
}

/*
 * Returns calibrated distance from an ir sensor
 */
double irDistance(enum IRSensor sensor)
{
	const int sensorIntensity = irsensor->data[sensor];
	//calibrate raw sensor value and return
	return irSensorCalibData[sensor].Ka / (sensorIntensity - irSensorCalibData[sensor].Kb);
}
