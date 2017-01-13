#include <stdio.h>
#include <cstdlib>
#include "includes/linesensor.h"
#include "includes/odometry.h"
#include "includes/robotconnector.h"

#define MAX_VALUE_FOR_BLACK 0.25
#define MIN_VALUE_FOR_WHITE 0.80

bool simulateFloor = false;

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
		printf("Incorrect line sensor callibration. Value = %f\n", calibValue);
	}

	if (simulateFloor && calibValue < 0.70)
	{
		calibValue = 0.6 + floatRandom(-0.05, 0.05);
	}
	return calibValue;
}

double floatRandom(const double min, const double max)
{
	const double f = (double) rand() / RAND_MAX;
	return f * min + (max - min);
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

	return correctedValue;
}

double getLineOffSetDistance(enum LineCentering centering, enum LineColor color)
{
	double sum_m = 0;
	double sum_i = 0;
	double min = 2;
	double max = -1;

	for (int i = 0; i < LINE_SENSORS_COUNT; ++i)
	{
		const double calibValue = calibrateLineSensorValue(linesensor->data[i], i);
		max = (calibValue > max) ? calibValue : max;
		min = (calibValue < min) ? calibValue : min;
	}
	//printf("min = %f, max = %f\n", min, max);
	const double a = -1 / (min - max);
	const double b = min / (min - max);
	//printf("a = %f, b = %f\n", a, b);
	static const LineCentering lineC[8] = { right, right, right, right, left, left, left, left };
	for (int i = 0; i < LINE_SENSORS_COUNT; ++i)
	{
		const double trueCalib = calibrateLineSensorValue(linesensor->data[i], i);
		const double calibValue = a * trueCalib + b;
		const double correctedValue = correctCalibratedValue(color, calibValue);
		const double weight = (centering == lineC[i]) ? 2 : 1;
		sum_m += correctedValue * i * weight;
		sum_i += correctedValue * weight;
		//printf("sensor %d, calibValue = %f, true calibValue = %f, correctedValue = %f. \n", i, calibValue, trueCalib, correctCalibratedValue(color, calibValue));
	}
	//printf("\n\n");

	const double c_m = sum_m / sum_i;
	const double offsetDistance = ((double) LINE_SENSOR_WIDTH / (LINE_SENSORS_COUNT - 1)) * c_m - (LINE_SENSOR_WIDTH / 2);
	//printf("offset Distance = %f\n", offsetDistance);
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
		return (calibrateLineSensorValue(linesensor->data[3], 3) < MAX_VALUE_FOR_BLACK || calibrateLineSensorValue(linesensor->data[4], 4) < MAX_VALUE_FOR_BLACK);
	}
	else
	{
		return (calibrateLineSensorValue(linesensor->data[3], 3) > MIN_VALUE_FOR_WHITE || calibrateLineSensorValue(linesensor->data[4], 4) > MIN_VALUE_FOR_WHITE);
	}
}
