/*
 * stopconditions.h
 *
 *  Created on: Jan 10, 2017
 *      Author: smr
 */

#ifndef STOPCONDITIONS_H_
#define STOPCONDITIONS_H_

#include <math.h>
#include "odometry.h"
#include "linesensor.h"
#include "irsensor.h"

#define ANGLE(x) ((double)x / 180.0 * M_PI)
static enum IRSensor currentIRSensor;
static double distancePillarDetection = 50.0;

int noStopCondition(odotype *odo)
{
	return 0;
}

int stopAtBlackLine(odotype *odo)
{
	return crossingLine(black, 3);
}

int stopAtNeg85Deg(odotype *odo)
{
	return odo->angle < ANGLE(-85);
}

int stopAt85Deg(odotype *odo)
{
	return odo->angle < ANGLE(85);
}

void setIRDetectionSensor(enum IRSensor sensor){
	currentIRSensor = sensor;
}

void setIRDetectionDistance(double distance){
	distancePillarDetection = distance;
}

int stopAtDetectedPillar(odotype *odo){
	static int countWithinDistance = 0;
	if (irDistance(currentIRSensor) < distancePillarDetection) 
		countWithinDistance++;
	else countWithinDistance = 0;
	//printf("Current Ir sensor = %d, distance from this sensor is = %f\n", currentIRSensor, irDistance(currentIRSensor));
	return (countWithinDistance >= numberRequiredForPillarDetected);
}

#endif /* STOPCONDITIONS_H_ */
