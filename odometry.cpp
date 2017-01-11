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

static double updateRightencPos(odotype* p)
{
	double delta = p->rightWheelEncoderTicks - p->oldRightWheelEncoderTicks;
	delta = preventOverflow(delta);
	p->oldRightWheelEncoderTicks = p->rightWheelEncoderTicks;
	p->rightWheelPos += delta * p->metersPerEncoderTick;
	return delta * p->metersPerEncoderTick;
}

static double updateLeftEncPos(odotype* p)
{
	double delta = p->leftWheelEncoderTicks - p->oldLeftWheelEncoderTicks;
	delta = preventOverflow(delta);
	p->oldLeftWheelEncoderTicks = p->leftWheelEncoderTicks;
	p->leftWheelPos += delta * p->metersPerEncoderTick;
	return delta * p->metersPerEncoderTick;
}

void updateOdo(odotype *p)
{
	const double incR = updateRightencPos(p);
	const double incL = updateLeftEncPos(p);

	p->totalDistance += fabs(incR + incL) / 2;
	p->angle += (incR - incL) / p->wheelSeparation; // deltaTheta

	const double deltaU = (incR + incL) / 2;
	p->xpos += deltaU * cos(p->angle);
	p->ypos += deltaU * sin(p->angle);
	//printf("%f %f %f\n", p->xpos, p->ypos, p->angle);
	logOdo(p);
}
