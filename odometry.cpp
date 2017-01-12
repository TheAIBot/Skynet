#include <math.h>
#include <stdio.h>
#include "includes/odometry.h"
#include "includes/log.h"
#include <stdio.h>
#include <math.h>

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

double getDistanceFromTicks(odotype *p, double ticks)
{
	return ticks * p->metersPerEncoderTick;
}

static double updateRightEncPos(odotype* p)
{
	double delta = p->rightWheelEncoderTicks - p->oldRightWheelEncoderTicks;
	delta = preventOverflow(delta);
	p->oldRightWheelEncoderTicks = p->rightWheelEncoderTicks;
	const double traveledDistance = getDistanceFromTicks(p, delta);
	p->rightWheelPos += traveledDistance;
	return traveledDistance;
}

static double updateLeftEncPos(odotype* p)
{
	double delta = p->leftWheelEncoderTicks - p->oldLeftWheelEncoderTicks;
	delta = preventOverflow(delta);
	p->oldLeftWheelEncoderTicks = p->leftWheelEncoderTicks;
	const double traveledDistance = getDistanceFromTicks(p, delta);
	p->leftWheelPos += traveledDistance;
	return traveledDistance;
}

void updateOdo(odotype *p)
{
	const double incR = updateRightEncPos(p);
	const double incL = updateLeftEncPos(p);

	p->totalDistance += fabs(incR + incL) / 2;
	p->angle += (incR - incL) / p->wheelSeparation; // deltaTheta

	const double deltaU = (incR + incL) / 2;
	p->xpos += deltaU * cos(p->angle);
	p->ypos += deltaU * sin(p->angle);
	//printf("%f %f %f\n", p->xpos, p->ypos, p->angle);
	logOdo(p);
}
