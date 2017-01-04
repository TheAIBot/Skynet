#include <stdio.h>
#include "includes/odometry.h"

#define MAX_LOGS 20000

static odotype logs[MAX_LOGS];
static int logCount = 0;


void logOdo(odotype * odo)
{
	logs[logCount] = *odo;
	logCount = (logCount + 1) % MAX_LOGS;
}

void writeLogs(char* filename)
{
	FILE* writeFile = fopen(filename, "w");
	int x;
	for (x = 0; x < logCount; ++x) {
		fprintf(writeFile, "%f %f %f\n", logs[x].xpos, logs[x].ypos, logs[x].angle);
	}
	fclose(writeFile);
}
