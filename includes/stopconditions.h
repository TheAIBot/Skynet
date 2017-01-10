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

#define ANGLE(x) ((double)x / 180.0 * M_PI)

int noStopCondition(odotype *odo)
{
	return 0;
}

int stopAtBlackLine(odotype *odo)
{
	return crossingLine(black, 3);
}

int stopAtNeg80Deg(odotype *odo)
{
	return odo->angle < ANGLE(-85);
}

#endif /* STOPCONDITIONS_H_ */
