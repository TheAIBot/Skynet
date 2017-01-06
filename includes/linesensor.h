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
linesensor_data_pair;

extern linesensor_data_pair linesensor_calib_data[LINE_SENSORS_COUNT];

//Read calibration values from the calibation file and inserts the data in the given array
int readLineSensorValues(char* fileLoc);


#endif