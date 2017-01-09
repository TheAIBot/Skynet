#ifndef LINESENSOR_H_
#define LINESENSOR_H

//Line sensor information
#define LINE_SENSOR_WIDTH 13
#define LINE_SENSORS_COUNT 8

enum lineCentering
{
	right = 0, center, left
};

enum lineColor
{
	white, black
};

typedef struct
{
	double a;
	double b;
} lineSensorCalibratedData;

//Read calibration values from the calibation file and inserts the data in the given array
int readLineSensorValues(const char* fileLoc);

double calibrateLineSensorValue(const double sensorValue, const int sensorID);

inline double getLineCenteringOffset(enum lineCentering centering);

#endif
