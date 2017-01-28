#include <math.h>
#include <stdio.h>
#include "includes/odometry.h"
#include "includes/log.h"
#include <stdio.h>
#include <math.h>

/*
 * Returns a truncated delta between -2^16 and 2^16
 * But i don't understand why this is nessesary
 */
static wheels<int> preventOverflow(wheels<int> delta)
{
	if (delta.left > 0x8000)
	{
		printf("hitl+\n");
		printf("%d %d\n", delta.left, delta.right);
		delta.left -= 0x10000;
	}
	else if (delta.left < -0x8000)
	{
		printf("hitl-\n");
		printf("%d %d\n", delta.left, delta.right);
		delta.left += 0x10000;
	}

	if (delta.right > 0x8000)
	{
		printf("hitr+\n");
		printf("%d %d\n", delta.left, delta.right);
		delta.right -= 0x10000;
	}
	else if (delta.right < -0x8000)
	{
		printf("hitr-\n");
		printf("%d %d\n", delta.left, delta.right);
		delta.right += 0x10000;
	}
	return delta;
}

/*
 * Converts motor ticks to meters
 */
inline wheels<double> getDistanceFromTicks(odotype* const p, wheels<int> ticks)
{
	return ticks * p->metersPerEncoderTick;
}

/*
 * Updates the total distance the wheels have traveled and returns
 * the distance the wheels traveled in this tick in meters
 */
static wheels<double> updateEncodersPositions(odotype* const p)
{
	const wheels<int> delta = preventOverflow(p->wheelsEncoderTicks - p->oldWheelsEncoderTicks);
	p->oldWheelsEncoderTicks = p->wheelsEncoderTicks;
	const wheels<double> traveledDistance = getDistanceFromTicks(p, delta);
	p->wheelsDrivenDistance += traveledDistance;
	//printf("%d %d\n", p->wheelsEncoderTicks.left, p->wheelsEncoderTicks.right);
	return traveledDistance;
}

/*
 * Updates odomety with new data
 */
void updateOdo(odotype* const p)
{
	const wheels<double> movedDist = updateEncodersPositions(p);

	//add distance traveled to total distance
	p->totalDistance += fabs(movedDist.left + movedDist.right) / 2;
	//add changed angle to angle
	p->angle += (movedDist.left - movedDist.right) / p->wheelSeparation; // deltaTheta

	//update robot position
	const double deltaU = (movedDist.left + movedDist.right) / 2;
	p->robotPosition.x += deltaU * cos(p->angle);
	p->robotPosition.y += deltaU * sin(p->angle);
	//printf("%f %f %f\n", p->xpos, p->ypos, p->angle);
	logOdo(p);
}
