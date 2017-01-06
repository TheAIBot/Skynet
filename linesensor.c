#include "includes/linesensor.h"
#include <stdio.h>

linesensorCalibratedData linesensorCalibData[LINE_SENSORS_COUNT];

int readLineSensorValues(char* fileLoc)
{

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
		int scanStatus = fscanf(file, "%lf %lf\n", &a, &b);
		if (scanStatus != 2) //Check if the correct number of items was read
		{
			printf("Error occured when reading linesensor calibration file. %d numbers expected, but %d was found.", 2, scanStatus);
			return 0;
		}
		linesensorCalibData[i].a = a;
		linesensorCalibData[i].b = b;
	}

	fclose(file);
	return 1;
}

double calibrateLineSensorValue(double sensorValue, int sensorID)
{
	double a = linesensorCalibData[sensorID].a;
	double b = linesensorCalibData[sensorID].b;

	double calibValue = a * sensorValue + b;
	if (calibValue == 0 || calibValue > 1)
	{
			printf("Incorrect line sensor callibration. Value = %f", calibValue);
	}
	return calibValue;
}
