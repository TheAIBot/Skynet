#ifndef LINESENSOR_H_
#define LINESENSOR_H_

//Line sensor information
#define LINE_SENSOR_WIDTH 13
#define LINE_SENSORS_COUNT 8
#define WHEEL_CENTER_TO_LINE_SENSOR_DISTANCE 22

extern bool simulateFloor;

enum LineCentering{
	left = 0, center, right
};

enum LineColor{
	white, black
};

typedef struct
{
	double a;
	double b;
} lineSensorCalibratedData;

//Read calibration values from the calibation file and inserts the data in the given array
int readLineSensorCalibrationData(const char* fileLoc);

double getLineCenteringOffset(enum LineCentering centering);

double getLineOffsetDistance(enum LineCentering centering, enum LineColor color);

int crossingLine(enum LineColor color, int konf);

int parallelLine(enum LineColor color);

#endif
