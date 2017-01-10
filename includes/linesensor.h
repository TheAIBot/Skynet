#ifndef LINESENSOR_H_
#define LINESENSOR_H_

//Line sensor information
#define LINE_SENSOR_WIDTH 13
#define LINE_SENSORS_COUNT 8
#define WHEEL_CENTER_TO_LINE_SENSOR_DISTANCE 22


enum lineCentering {
	left = 0, center, right
};

enum lineColor {
	white, black
};

typedef struct {
	double a;
	double b;
} lineSensorCalibratedData;

//Read calibration values from the calibation file and inserts the data in the given array
int readLineSensorCalibrationData(const char* fileLoc);

double getLineOffSetDistance(enum lineCentering centering);

double getLineCenteringOffset(enum lineCentering centering);

double getLineOffSetDistance(enum lineCentering centering);

int crossingLine(enum lineColor color, int konf);

#endif
