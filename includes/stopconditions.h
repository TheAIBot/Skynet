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

#define ANGLE(x) ((double)(x) / 180.0 * M_PI)

int noStopCondition(odotype *odo)
{
	return 0;
}

template<enum LineColor color, int conf>
int stopAtLine(odotype *odo)
{
	return crossingLine(color, conf);
}

template<enum LineColor color>
int stopAtParallelLine(odotype *odo)
{
	return parallelLine(color);
}

template<int angle, int deviation>
int stopAtDeg(odotype *odo)
{
	return odo->angle <= ANGLE(angle + deviation) && odo->angle >= ANGLE(angle - deviation);
}

template<enum IRSensor sensor, int distance>
int stopAtDetectedPillar(odotype *odo)
{
	static int countWithinDistance = 0;
	countWithinDistance = (irDistance(sensor) < distance) ? countWithinDistance + 1 : 0;
	if (countWithinDistance >= numberRequiredForPillarDetected)
	{
		countWithinDistance = 0;
		return 1;
	}
	else
	{
		return 0;
	}
}

template<enum IRSensor sensor, int distance>
int stopAtBlankSpace(odotype *odo)
{
	return (irDistance(sensor) > distance);
}

int stopAtBlockedForwardPath(odotype *odo)
{
	return (irDistance(ir_front_left) < 20 && irDistance(ir_front_center) < 20 && irDistance(ir_front_right) < 20);
}

int stopAtFreeRightIR(odotype *odo)
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
		return 1;
	}
	return 0;
}

#endif /* STOPCONDITIONS_H_ */
