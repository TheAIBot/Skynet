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
#include "robotconnector.h"
#include "lasersensor.h"

#define ANGLE(x) ((double)(x) / 180.0 * M_PI)

bool noStopCondition(odotype*const odo)
{
	return false;
}

template<enum LineColor color, int conf>
bool stopAtLine(odotype* const odo)
{
	return crossingLine(color, conf);
}

template<enum LineColor color>
bool stopAtParallelLine(odotype* const odo)
{
	return parallelLine(color);
}

template<int angle, int deviation>
bool stopAtDeg(odotype* const odo)
{
	return odo->angle <= ANGLE(angle + deviation) && odo->angle >= ANGLE(angle - deviation);
}

template<enum IRSensor sensor, int distance>
bool stopAtDetectedPillar(odotype* const odo)
{
	static int countWithinDistance = 0;
	countWithinDistance = (irDistance(sensor) < distance) ? countWithinDistance + 1 : 0;
	if (countWithinDistance >= numberRequiredForPillarDetected)
	{
		countWithinDistance = 0;
		return true;
	}
	else
	{
		return false;
	}
}

template<enum LaserDistance laser, int distance>
bool stopAtBlankSpace(odotype* const odo)
{
	return (getLaserDistance(laser) > ((double)distance / 100) + 0.2 || getLaserDistance(laser) < 0.005);
}

bool stopAtBlockedForwardPath(odotype* const odo)
{
	return (irDistance(ir_front_left) < 20 && irDistance(ir_front_center) < 20 && irDistance(ir_front_right) < 20);
}

bool stopAtFreeRightIR(odotype* const odo)
{
	static int countWithinDistance = 0;
	if ((ir_front_right) > 50)
	{
		countWithinDistance++;
	}
	else
	{
		countWithinDistance = 0;
	}
	if (countWithinDistance >= numberRequiredForPillarDetected)
	{
		countWithinDistance = 0;
		return true;
	}
	return false;
}

template<int startAngle, int endAngle, int distance>
bool stopAtLaserDetectedPillar(odotype* const odo)
{
	const int startIndex = ANGLE_TO_INDEX(-endAngle);
	const int endIndex = ANGLE_TO_INDEX(-startAngle);
	//printf("%d %d\n", startIndex, endIndex);

	double lowest = 1000;
	for (int i = startIndex; i < endIndex; ++i)
	{
		if (laserpar[i] > MIN_LASER_DISTANCE && laserpar[i] < lowest)
		{
			lowest = laserpar[i];
		}
		//printf("%d %f\n", i, laserpar[i]);
		if (laserpar[i] > MIN_LASER_DISTANCE && laserpar[i] < ((double) distance) / 100)
		{
			//printf("%d %f\n", i, laserpar[i]);
			return true;
		}
	}
	//printf("%f\n", lowest);
	return false;
}

#endif /* STOPCONDITIONS_H_ */
