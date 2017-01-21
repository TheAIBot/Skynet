#include <stdio.h>
#include "includes/odometry.h"

#define MAX_LOGS 20000

static odotype logs[MAX_LOGS];
static int logCount = 0;

/*
 * Saves a copy of odo to logs
 */
void logOdo(const odotype* const odo)
{
	logs[logCount] = *odo;
	//override old logs if length of array is reached
	logCount = (logCount + 1) % MAX_LOGS;
}

/*
 * Writes logs to odo
 */
void writeLogs(const char* const filename)
{
	FILE* writeFile = fopen(filename, "w");
	for (int x = 0; x < logCount; ++x)
	{
		fprintf(writeFile, "%f %f %f\n", logs[x].xpos, logs[x].ypos, logs[x].angle);
	}
	fclose(writeFile);
}
