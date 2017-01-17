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

#define ANGLE(x) ((double)(x) / 180.0 * M_PI)

bool noStopCondition(odotype *odo)
{
	return false;
}

template<enum LineColor color, int conf>
bool stopAtLine(odotype *odo)
{
	return crossingLine(color, conf);
}

template<enum LineColor color>
bool stopAtParallelLine(odotype *odo)
{
	return parallelLine(color);
}

template<int angle, int deviation>
bool stopAtDeg(odotype *odo)
{
	return odo->angle <= ANGLE(angle + deviation) && odo->angle >= ANGLE(angle - deviation);
}

template<enum IRSensor sensor, int distance>
bool stopAtDetectedPillar(odotype *odo)
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

template<enum IRSensor sensor, int distance>
bool stopAtBlankSpace(odotype *odo)
{
	return (irDistance(sensor) > distance);
}

bool stopAtBlockedForwardPath(odotype *odo)
{
	return (irDistance(ir_front_left) < 20 && irDistance(ir_front_center) < 20 && irDistance(ir_front_right) < 20);
}

bool stopAtFreeRightIR(odotype *odo)
{
	static int countWithinDistance = 0;
	if (irDistance(ir_front_right) > 50)
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
bool stopAtLaserDetectedPillar(odotype *odo)
{
	const int startIndex = (startAngle + (LASER_SEARCH_ANGLE / 2)) / ((double) LASER_SEARCH_ANGLE / MAX_LASER_COUNT);
	const int endIndex = (endAngle + (LASER_SEARCH_ANGLE / 2)) / ((double) LASER_SEARCH_ANGLE / MAX_LASER_COUNT);
	printf("%d %d\n", startIndex, endIndex);

	for (int i = startIndex; i < endIndex; ++i)
	{
		if (laserpar[i] > 0.01 && laserpar[i] < (double)distance / 100)
		{
			return true;
		}
	}
	return false;
}

#endif /* STOPCONDITIONS_H_ */
