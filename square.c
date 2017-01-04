#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include "includes/robotconnector.h"

FILE* writeFile;

/*****************************************
 * odometry
 */
#define WHEEL_DIAMETER   0.067	/* m */
#define WHEEL_SEPARATION 0.256	/* m */
#define DELTA_M (M_PI * WHEEL_DIAMETER / 2000)

typedef struct
{ //input signals
	int left_enc, right_enc; // encoderticks
	// parameters
	double w;	// wheel separation
	double cr, cl;   // meters per encodertick
	//output signals
	double right_pos, left_pos;
	// internal variables
	int left_enc_old, right_enc_old;
	double xpos, ypos, angle;
} odotype;

/********************************************
 * Motion control
 */

typedef struct
{	        //input
	int cmd;
	int curcmd;
	double speedcmd;
	double dist;
	double angle;
	double left_pos, right_pos;
	// parameters
	double w;
	//output
	double motorspeed_l, motorspeed_r;
	int finished;
	// internal variables
	double startpos;
} motiontype;

enum
{
	mot_stop = 1, mot_move, mot_turn
};

typedef struct
{
	int state, oldstate;
	int time;
} smtype;

odotype odo;
smtype mission;
motiontype mot;

enum
{
	ms_init, ms_fwd, ms_turn, ms_end
};


/*
 * Routines to convert encoder values to positions.
 * Encoder steps have to be converted to meters, and
 * roll-over has to be detected and corrected.
 */

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
	fprintf(writeFile, "%f %f %f\n", p->xpos, p->ypos, p->angle);
}

#define MIN(X, Y) (((X) < (Y)) ? (X) : (Y))

void update_motcon(motiontype *p)
{

	if (p->cmd != 0)
	{

		p->finished = 0;
		switch (p->cmd) {
		case mot_stop:
			p->curcmd = mot_stop;
			break;
		case mot_move:
			p->startpos = (p->left_pos + p->right_pos) / 2;
			p->curcmd = mot_move;
			break;

		case mot_turn:
			p->startpos = (p->angle > 0) ? p->right_pos :  p->left_pos;
			p->curcmd = mot_turn;
			break;

		}

		p->cmd = 0;
	}

	double movedDist = (p->right_pos + p->left_pos) / 2 - p->startpos;
	switch (p->curcmd) {
	case mot_stop:
		p->motorspeed_l = 0;
		p->motorspeed_r = 0;
		break;
	case mot_move:
		if (movedDist > p->dist)
		{
			p->finished = 1;
			p->motorspeed_l = 0;
			p->motorspeed_r = 0;
		}
		else
		{
			p->motorspeed_l = p->speedcmd;
			p->motorspeed_r = p->speedcmd;

			//p->motorspeed_l = MIN(p->speedcmd, sqrt(2 * 0.5 * movedDist));
			//p->motorspeed_r = MIN(p->speedcmd, sqrt(2 * 0.5 * movedDist));

			//p->motorspeed_l  =  MIN(MIN(p->speedcmd, sqrt(2*0.5*movedDist)),  sqrt(2*0.5*(p->dist - movedDist)));
			//p->motorspeed_r  =  MIN(MIN(p->speedcmd, sqrt(2*0.5*movedDist)),  sqrt(2*0.5*(p->dist - movedDist)));
		}
		break;

	case mot_turn:
		if (p->angle > 0)
		{
			if (p->right_pos - p->startpos < (p->angle * p->w) / 2)
			{
				p->motorspeed_r = p->speedcmd / 8;
				p->motorspeed_l = -p->speedcmd / 8;
			}
			else
			{
				p->motorspeed_r = 0;
				p->motorspeed_l = 0;
				p->finished = 1;
			}
		}
		else
		{
			if (p->left_pos - p->startpos < (fabs(p->angle) * p->w) / 2)
			{
				p->motorspeed_r = -p->speedcmd / 8;
				p->motorspeed_l = p->speedcmd / 8;
			}
			else
			{
				p->motorspeed_l = 0;
				p->finished = 1;
			}
		}

		break;
	}
}

int fwd(double dist, double speed, int time)
{
	if (time == 0)
	{
		mot.cmd = mot_move;
		mot.speedcmd = speed;
		mot.dist = dist;
		return 0;
	}
	else
		return mot.finished;
}

int turn(double angle, double speed, int time)
{
	if (time == 0)
	{
		mot.cmd = mot_turn;
		mot.speedcmd = speed;
		mot.angle = angle;
		return 0;
	}
	else
		return mot.finished;
}

void sm_update(smtype *p)
{
	if (p->state != p->oldstate)
	{
		p->time = 0;
		p->oldstate = p->state;
	}
	else
	{
		p->time++;
	}
}

int main()
{
	writeFile = fopen("logging.txt", "w");

	int running;
	int n = 0;
	int arg;
	int time = 0;
	double dist = 0;
	double angle = 0;

	if (!connectRobot())
	{
		exit(EXIT_FAILURE);
	}

	/* Read sensors and zero our position.
	 */
	rhdSync();

	odo.w = 0.256;
	odo.cr = DELTA_M;
	odo.cl = odo.cr;
	odo.left_enc = lenc->data[0];
	odo.right_enc = renc->data[0];
	reset_odo(&odo);
	printf("position: %f, %f\n", odo.left_pos, odo.right_pos);
	mot.w = odo.w;
	running = 1;
	mission.state = ms_init;
	mission.oldstate = -1;
	while (running)
	{
		if (lmssrv.config && lmssrv.status && lmssrv.connected)
		{
			while ((xml_in_fd(xmllaser, lmssrv.sockfd) > 0))
				xml_proca(xmllaser);
		}

		if (camsrv.config && camsrv.status && camsrv.connected)
		{
			while ((xml_in_fd(xmldata, camsrv.sockfd) > 0))
				xml_proc(xmldata);
		}

		rhdSync();
		odo.left_enc = lenc->data[0];
		odo.right_enc = renc->data[0];
		update_odo(&odo);

		/****************************************
		 / mission statemachine
		 */
		sm_update(&mission);
		switch (mission.state) {
		case ms_init:
			n = 4;
			dist = 1;
			angle = 90.0 / 180 * M_PI;
			mission.state = ms_fwd;
			break;

		case ms_fwd:
			if (fwd(dist, 0.3, mission.time))
				mission.state = ms_turn;
			break;

		case ms_turn:
			if (turn(angle, 0.3, mission.time))
			{
				n--;
				mission.state = (n == 0) ? ms_end : ms_fwd;
			}
			break;

		case ms_end:
			mot.cmd = mot_stop;
			running = 0;
			break;
		}
		/*  end of mission  */

		mot.left_pos = odo.left_pos;
		mot.right_pos = odo.right_pos;
		update_motcon(&mot);
		speedl->data[0] = 100 * mot.motorspeed_l;
		speedl->updated = 1;
		speedr->data[0] = 100 * mot.motorspeed_r;
		speedr->updated = 1;
		if (time % 100 == 0)
		{
			//    printf(" laser %f \n",laserpar[3]);
			time++;
		}
		/* stop if keyboard is activated
		 *
		 */
		ioctl(0, FIONREAD, &arg);
		if (arg != 0)
		{
			running = 0;
		}

	}/* end of main control loop */
	speedl->data[0] = 0;
	speedl->updated = 1;
	speedr->data[0] = 0;
	speedr->updated = 1;
	rhdSync();
	rhdDisconnect();
	fclose(writeFile);
	exit(0);
}

