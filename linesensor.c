#include "includes/linesensor.h"
#include <stdio.h>

linesensor_data_pair linesensor_calib_data[LINE_SENSORS_COUNT];


int readLineSensorValues(char* fileLoc)
{

	FILE* file;
	file = fopen(fileLoc,"r");

	if (file == NULL) // Check if the give file is found
	{
		// If not display an error and return an error value
		printf("%s NOT FOUND!\n",fileLoc);
		return 0;
	}

	//Error the data value pair for each sensor
	int i;
	for(i=0; i < LINE_SENSORS_COUNT; i++)
	{
		double a;
		double b;
		int scanStatus = fscanf(file,"%lf %lf\n",&a,&b);
		if(scanStatus != 2) //Check if the correct number of items was read
		{
			printf("Error occured when reading linesensor calibration file. %d numbers expected, but %d was found.", 2, scanStatus);
			return 0;
		}
		linesensor_calib_data[i].a = a;
		linesensor_calib_data[i].b = b;
	}

	fclose(file);
	return 1;
}