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
	if (calibValue == 0 || calibValue > 1)
	{
		printf("Incorrect line sensor callibration. Value = %f", calibValue);
	}
	return calibValue;
}

double fRand(double fMin, double fMax)
{
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
}

double correctCalibratedValue(enum LineColor color, const double value)
{
	double correctedValue;
	if (color == LineColor::black)
	{
		correctedValue = (1 - value);
	}
	else
	{
		correctedValue = value;
	}
	if (correctedValue < 0.85)
	{
		correctedValue = 0.6 + fRand(0.00, 0.05);
	}
	return correctedValue;
}

double getLineOffSetDistance(enum LineCentering centering, enum LineColor color)
{
	double sum_m = 0;
	double sum_i = 0;
	int i;
	static LineCentering lineC[8] = { right, right, right, right, left, left, left, left };
	double maxValue = 0;
	double average = 0;
	for (i = 0; i < LINE_SENSORS_COUNT; ++i)
	{
		const double calibValue = calibrateLineSensorValue(linesensor->data[i], i);
		const double correctedValue = correctCalibratedValue(color, calibValue);
		if (correctedValue > maxValue)
		{
			maxValue = correctedValue;
		}
		average += correctedValue;
	}
	average /= LINE_SENSORS_COUNT;

	const double a = maxValue - average;
	const double b = average;
	printf("a %f b %f\n", a, b);

	for (i = 0; i < LINE_SENSORS_COUNT; ++i)
	{
		const double calibValue = calibrateLineSensorValue(linesensor->data[i], i);
		const double weight = (centering == lineC[i]) ? 3 : 1;
		//printf("%f ", weight);
		sum_m += a * (correctCalibratedValue(color, calibValue) * i * weight) + b;
		sum_i += a * (correctCalibratedValue(color, calibValue) * weight) + b;
	}
	//printf("\n");
	const double c_m = sum_m / sum_i;
	//printf("%f\n", c_m);
	const double offsetDistance = ((double) LINE_SENSOR_WIDTH / (LINE_SENSORS_COUNT - 1)) * c_m - (LINE_SENSOR_WIDTH / 2);
	//printf("%f\n", offsetDistance);
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
	if (color == LineColor::black)
	{
		return (calibrateLineSensorValue(linesensor->data[3], 3) < MAX_VALUE_FOR_BLACK && calibrateLineSensorValue(linesensor->data[4], 4) < MAX_VALUE_FOR_BLACK);
	}
	else
	{
		return (calibrateLineSensorValue(linesensor->data[3], 3) > MIN_VALUE_FOR_WHITE && calibrateLineSensorValue(linesensor->data[4], 4) > MIN_VALUE_FOR_WHITE);
	}
}
