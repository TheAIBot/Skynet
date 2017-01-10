#include <stdio.h>
#include "includes/linesensor.h"

static lineSensorCalibratedData lineSensorCalibData[LINE_SENSORS_COUNT];

int readLineSensorValues(const char* fileLoc){

	FILE* file = fopen(fileLoc, "r");

	if (file == NULL) // Check if the give file is found
	{
		// If not display an error and return an error value
		printf("%s NOT FOUND!\n", fileLoc);
		return 0;
	}

	//Error the data value pair for each sensor
	int i;
	for (i = 0; i < LINE_SENSORS_COUNT; i++)
	{
		double a;
		double b;
		const int scanStatus = fscanf(file, "%lf %lf\n", &a, &b);
		if (scanStatus != 2) //Check if the correct number of items was read
		{
			printf("Error occured when reading linesensor calibration file. %d numbers expected, but %d was found.", 2, scanStatus);
			return 0;
		}
		lineSensorCalibData[i].a = a;
		lineSensorCalibData[i].b = b;
	}

	fclose(file);
	return 1;
}

double calibrateLineSensorValue(const double sensorValue, const int sensorID)
{
	const double a = lineSensorCalibData[sensorID].a;
	const double b = lineSensorCalibData[sensorID].b;

	double calibValue = a * sensorValue + b;
	if (calibValue == 0 || calibValue > 1)
	{
		printf("Incorrect line sensor callibration. Value = %f", calibValue);
	}
	return calibValue;
}

double getLineCenteringOffset(enum lineCentering centering)
{
	static double centers[3] = { ((double)LINE_SENSOR_WIDTH / 3) * 1, (double)LINE_SENSOR_WIDTH / 2, ((double)LINE_SENSOR_WIDTH / 3) * 2 };
	return centers[centering];
}

double getLineOffSetDistance(enum lineCentering centering){
	double sum_m = 0, sum_i = 0;
	int i;
	for (i = 0; i < LINE_SENSORS_COUNT; i++)
	{
		const double calibValue = calibrateLineSensorValue(linesensor->data[i], i);
		sum_m += (1 - calibValue) * i;
		sum_i += (1 - calibValue);
	}
	const double c_m = sum_m / sum_i;
	return ((double) LINE_SENSOR_WIDTH / (LINE_SENSORS_COUNT - 1)) * c_m - getLineCenteringOffset(centering);
}

