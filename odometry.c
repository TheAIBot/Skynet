#include <math.h>
#include <stdio.h>
#include "includes/odometry.h"
#include "includes/log.h"
#include <stdio.h>
#include <math.h>

void reset_odo(odotype * p)
{
	p->right_pos = 0;
	p->left_pos = 0;
	p->right_enc_old = p->right_enc;
	p->left_enc_old = p->left_enc;
	p->xpos = 0;
	p->ypos = 0;
	p->angle = 0;
}

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
	double delta = p->right_enc - p->right_enc_old;
	delta = preventOverflow(delta);
	p->right_enc_old = p->right_enc;
	p->right_pos += delta * p->cr;
	return delta * p->cr;
}

static double updateLeftEncPos(odotype* p)
{
	double delta = p->left_enc - p->left_enc_old;
	delta = preventOverflow(delta);
	p->left_enc_old = p->left_enc;
	p->left_pos += delta * p->cl;
	return delta * p->cl;
}

void update_odo(odotype *p)
{
	double incR = updateRightencPos(p);
	double incL = updateLeftEncPos(p);

	p->totalDistance += fabs(incR + incL) / 2;
	p->angle += (incR - incL) / p->w; // deltaTheta

	double deltaU = (incR + incL) / 2;
	p->xpos += deltaU * cos(p->angle);
	p->ypos += deltaU * sin(p->angle);
	//printf("%f %f %f\n", p->xpos, p->ypos, p->angle);
	logOdo(p);
}
