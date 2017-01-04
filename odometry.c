#include <math.h>
#include <stdio.h>
#include "includes/odometry.h"
#include "includes/log.h"

void reset_odo(odotype * p)
{
	p->right_pos = p->left_pos = 0.0;
	p->right_enc_old = p->right_enc;
	p->left_enc_old = p->left_enc;
	p->xpos = 0;
	p->ypos = 0;
	p->angle = 0;
}

int preventOverflow(int delta)
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

void update_odo(odotype *p)
{
	int delta;

	delta = p->right_enc - p->right_enc_old;
	delta = preventOverflow(delta);

	p->right_enc_old = p->right_enc;
	p->right_pos += delta * p->cr;
	double incR = delta * p->cr;

	delta = p->left_enc - p->left_enc_old;
	delta = preventOverflow(delta);

	p->left_enc_old = p->left_enc;
	p->left_pos += delta * p->cl;
	double incL = delta * p->cl;

	double deltaU = (incR + incL) / 2;
	double deltaTheta = (incR - incL) / p->w;

	p->angle += deltaTheta;
	//while (p->angle > 2 * M_PI)
	//	p->angle -= 2 * M_PI;
	p->xpos += deltaU * cos(p->angle);
	p->ypos += deltaU * sin(p->angle);
	printf("%f %f %f\n", p->xpos, p->ypos, p->angle);
	logOdo(p);
}
