#include <math.h>
#include <stdio.h>
#include "includes/odometry.h"
#include "includes/log.h"
#include <stdio.h>
#include <math.h>

/*
 * Returns a truncated delta between -2^16 and 2^16
 */
static int preventOverflow(int delta)
{
	if (delta > 0x8000)
	{
		delta -= 0x10000;
	}
	else if (delta < -0x8000)
	{
		delta += 0x10000;
	}
	return delta;
}

/*
 * Converts motor ticks to meters
 */
double getDistanceFromTicks(odotype* const p, double ticks)
{
	return ticks * p->metersPerEncoderTick;
}

/*
 * Updates the total distance the right wheel has traveled and returns
 * the distance the wheel traveled in this tick in meters
 */
static double updateRightEncPos(odotype* const p)
{
	double delta = p->rightWheelEncoderTicks - p->oldRightWheelEncoderTicks;
	delta = preventOverflow(delta);
	p->oldRightWheelEncoderTicks = p->rightWheelEncoderTicks;
	const double traveledDistance = getDistanceFromTicks(p, delta);
	p->rightWheelPos += traveledDistance;
	return traveledDistance;
}

/*
 * Updates the total distance the left wheel has traveled and returns
 * the distance the wheel traveled in this tick in meters
 */
static double updateLeftEncPos(odotype* const p)
{
	double delta = p->leftWheelEncoderTicks - p->oldLeftWheelEncoderTicks;
	delta = preventOverflow(delta);
	p->oldLeftWheelEncoderTicks = p->leftWheelEncoderTicks;
	const double traveledDistance = getDistanceFromTicks(p, delta);
	p->leftWheelPos += traveledDistance;
	return traveledDistance;
}

/*
 * Updates odomety with new data
 */
void updateOdo(odotype* const p)
{
	const double incR = updateRightEncPos(p);
	const double incL = updateLeftEncPos(p);

	//add distance treveled to tota ldistance
	p->totalDistance += fabs(incR + incL) / 2;
	//add changed angle to angle
	p->angle += (incR - incL) / p->wheelSeparation; // deltaTheta

	//update robot position
	const double deltaU = (incR + incL) / 2;
	p->xpos += deltaU * cos(p->angle);
	p->ypos += deltaU * sin(p->angle);
	//printf("%f %f %f\n", p->xpos, p->ypos, p->angle);
	logOdo(p);
}
