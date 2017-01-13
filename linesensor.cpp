#include <stdio.h>
#include <cstdlib>
#include "includes/linesensor.h"
#include "includes/odometry.h"
#include "includes/robotconnector.h"

#define MAX_VALUE_FOR_BLACK 0.25
#define MIN_VALUE_FOR_WHITE 0.80

static lineSensorCalibratedData lineSensorCalibData[LINE_SENSORS_COUNT];

int readLineSensorCalibrationData(const char* fileLoc)
{

	FILE* file = fopen(fileLoc, "r");

	if (file == NULL)
	{
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

static double calibrateLineSensorValue(const double sensorValue, const int sensorID)
{
	const double a = lineSensorCalibData[sensorID].a;
	const double b = lineSensorCalibData[sensorID].b;

	double calibValue = a * sensorValue + b;
	if (calibValue == 0 || calibValue > 1){
		printf("Incorrect line sensor callibration. Value = %f\n", calibValue);
	}
	return calibValue;
}


double correctCalibratedValue(enum LineColor color, const double value){
	double correctedValue;
	if (color == LineColor::black){
		correctedValue = (1 - value);
	} else {
		correctedValue = value;
	}
	/*
	if (correctedValue < 0.50) {
		correctedValue = 0.5;
	}
	*/
	return correctedValue;
}

double getLineOffSetDistance(enum LineCentering centering, enum LineColor color)
{
	double sum_m = 0;
	double sum_i = 0;
	int i;
	double min = 2, max=-1;
	double a,b;

	for (i = 0; i < LINE_SENSORS_COUNT; ++i)	{
		double calibValue = calibrateLineSensorValue(linesensor->data[i], i);
		if (calibValue > 0.5) calibValue = 0.5;
		if (calibValue > max) max = calibValue;
		else if (calibValue < min) min = calibValue;
	}
	printf("min = %f, max = %f\n", min, max);
	a=-1/(min-max);
	b=min/(min-max);
	printf("a = %f, b = %f\n",a,b);
	static const LineCentering lineC[8] = { right, right, right, right, left, left, left, left };
	for (i = 0; i < LINE_SENSORS_COUNT; ++i)	{
		double trueCalib = calibrateLineSensorValue(linesensor->data[i], i);
		if (trueCalib > 0.5) trueCalib = 0.5;
		double calibValue = a*trueCalib+b;
		const double weight = (centering == lineC[i]) ? 2: 1;
		printf("sensor %d, calibValue = %f, true calibValue = %f, correctedValue = %f. \n", i, calibValue, trueCalib, correctCalibratedValue(color, calibValue));
		sum_m += correctCalibratedValue(color, calibValue) * i * weight;
		sum_i += correctCalibratedValue(color, calibValue) * weight;
	} printf("\n\n");

	const double c_m = sum_m / sum_i;
	const double offsetDistance = ((double) LINE_SENSOR_WIDTH / (LINE_SENSORS_COUNT -1)) * c_m - (LINE_SENSOR_WIDTH / 2);
	printf("offset Distance = %f\n", offsetDistance);
	return offsetDistance;
}

int crossingLine(enum LineColor color, int konf)
{
	int count = 0;
	int i;
	if (color == LineColor::black)
	{
		for (i = 0; i < LINE_SENSORS_COUNT; i++)
		{
			const double calibvalue = calibrateLineSensorValue(linesensor->data[i], i);
			if (calibvalue <= MAX_VALUE_FOR_BLACK)
			{
				count++;
			}
		}
	}
	else if (color == LineColor::white)
	{
		for (i = 0; i < LINE_SENSORS_COUNT; i++)
		{
			const double calibvalue = calibrateLineSensorValue(linesensor->data[i], i);
			if (calibvalue >= MIN_VALUE_FOR_WHITE)
			{
				count++;
			}
		}
	}
	//printf("%d\n", count);
	return count >= konf;
}

int parallelLine(enum LineColor color)
{
	if (color == LineColor::black)	{
		return (calibrateLineSensorValue(linesensor->data[3], 3) < MAX_VALUE_FOR_BLACK || calibrateLineSensorValue(linesensor->data[4], 4) < MAX_VALUE_FOR_BLACK);
	} else{
		return (calibrateLineSensorValue(linesensor->data[3], 3) > MIN_VALUE_FOR_WHITE || calibrateLineSensorValue(linesensor->data[4], 4) > MIN_VALUE_FOR_WHITE);
	}
}
