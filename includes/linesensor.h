#ifndef LINESENSOR_H_
#define LINESENSOR_H

//Line sensor information
#define LINE_SENSOR_WIDTH 13
#define LINE_SENSORS_COUNT 8

typedef struct
{
	double a;
	double b;
}
linesensorCalibratedData;


//Read calibration values from the calibation file and inserts the data in the given array
int readLineSensorValues(const char* fileLoc);

double calibrateLineSensorValue(const double sensorValue, const int sensorID);

#endif
