#include <stdio.h>
#include <cstdlib>
#include "includes/linesensor.h"
#include "includes/odometry.h"
#include "includes/robotconnector.h"
#include "includes/commands.h"
#define MAX_VALUE_FOR_BLACK 0.25
#define MIN_VALUE_FOR_WHITE 0.80

bool simulateFloor = false;

static lineSensorCalibratedData lineSensorCalibData[LINE_SENSORS_COUNT];

/*
 * Loads calibration data for live sensor from fileLoc
 */
bool loadLineSensorCalibrationData(const char* fileLoc)
{
	FILE* file = fopen(fileLoc, "r");

	if (file == NULL)
	{
		printf("%s NOT FOUND!\n", fileLoc);
		return false;
	}

	//Error the data value pair for each sensor
	for (int i = 0; i < LINE_SENSORS_COUNT; i++)
	{
		double a;
		double b;
		const int scanStatus = fscanf(file, "%lf %lf\n", &a, &b);
		if (scanStatus != 2) //Check if the correct number of items was read
		{
			printf("Error occured when reading linesensor calibration file. %d numbers expected, but %d was found.", 2, scanStatus);
			return false;
		}
		lineSensorCalibData[i].a = a;
		lineSensorCalibData[i].b = b;
	}

	fclose(file);
	return true;
}

/*
 * Returns a random double between min and max
 */
static double floatRandom(const double min, const double max)
{
	const double f = (double) rand() / RAND_MAX;
	return f * min + (max - min);
}

/*
 * Returns a calibrated value of sensorValue that is calibrated with sensorID calibration
 * data
 */
static double calibrateLineSensorValue(const double sensorValue, const int sensorID)
{
	const double a = lineSensorCalibData[sensorID].a;
	const double b = lineSensorCalibData[sensorID].b;

	const double calibValue = a * sensorValue + b;
	//if true then the calibration of the sensor is incorrect
	//as the calibrated value should be a value between 0 and 1
	if (calibValue < -0.1 || calibValue > 1.1)
	{
		printf("Incorrect line sensor callibration. Value = %f\n", calibValue);
	}
	return calibValue;
}

/*
 * Converts value to a number between 0 and 1 where 1 means that the value
 * Looks exactly like the color color
 */
double correctCalibratedValue(enum LineColor color, const double value)
{
	double correctedValue;
	//black is a 0 and white is 1 so when color is black
	//switch it around so black is 1 and white is 0
	if (color == LineColor::black)
	{
		correctedValue = (1 - value);
	}
	else
	{
		correctedValue = value;
	}
	//if simulate floor then take all values below 0.7 and give it a
	//random value around 0.6 as that should simulate a wooden floor
	if (simulateFloor && correctedValue < 0.70)
	{
		correctedValue = 0.6 + floatRandom(-0.1, 0.1);
	}
	return correctedValue;
}

/*
 * Returns a value indicating how far off the center of the line sensor the color line is
 */
double getLineOffsetDistance(enum LineCentering centering, enum LineColor color)
{
	double min = 2;
	double max = -1;
	//get the highest and lowest corrected calibrated value
	for (int i = 0; i < LINE_SENSORS_COUNT; ++i)
	{
		const double calibValue = calibrateLineSensorValue(linesensor->data[i], i);
		const double correctedValue = correctCalibratedValue(color, calibValue);
		max = (correctedValue > max) ? correctedValue : max;
		min = (correctedValue < min) ? correctedValue : min;
	}
	//use linear transformation to make corrected calibrated min 0 and max 1
	//as opposed to min ~0.6 and max ~0.95
	//This is done to remove the weight the floor has on the
	//center of mass function
	const double a = -1 / (min - max);
	const double b = min / (min - max);
	double sum_m = 0;
	double sum_i = 0;
	static const LineCentering lineC[LINE_SENSORS_COUNT] = { right, right, right, right, left, left, left, left };
	//center of mass sum
	for (int i = 0; i < LINE_SENSORS_COUNT; ++i)
	{
		const double trueCalib = calibrateLineSensorValue(linesensor->data[i], i);
		const double correctedValue = correctCalibratedValue(color, trueCalib);
		//do linear transformation
		const double calibValue = a * correctedValue + b;
		//add a weight to the sensor values if either right or left lineCentering is chosen
		//which makes the robot favor a certain direction if the line splits up into two lines
		const double weight = (centering == lineC[i]) ? 2 : 1;
		sum_m += calibValue * i * weight;
		sum_i += calibValue * weight;
	}
	//calucate center of mass where the center is 3.5
	const double c_m = sum_m / sum_i;
	//recalculate the center so the, line sensor center offset, is a value between -6.5 and 6.5 and the center is at 0
	return ((double) LINE_SENSOR_WIDTH / (LINE_SENSORS_COUNT - 1)) * c_m - (LINE_SENSOR_WIDTH / 2);
}

/*
 * Returns wether the robot is crossing a line with color color and
 * only if konf sensors can see the line
 */
bool crossingLine(enum LineColor color, int konf)
{
	int count = 0;
	if (color == LineColor::black)
	{
		for (int i = 0; i < LINE_SENSORS_COUNT; i++)
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
		for (int i = 0; i < LINE_SENSORS_COUNT; i++)
		{
			const double calibvalue = calibrateLineSensorValue(linesensor->data[i], i);
			if (calibvalue >= MIN_VALUE_FOR_WHITE)
			{

				count++;
			}
		}
	}
	return count >= konf;
}

/*
 * Returns wether there is a  parallel line of color color
 * in the middle of the robots line sensor
 */
bool parallelLine(enum LineColor color)
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
