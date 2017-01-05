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
#include "includes/odometry.h"
#include "includes/log.h"

/*****************************************
 * odometry
 */
#define WHEEL_DIAMETER   0.067	/* m */
#define WHEEL_SEPARATION 0.256	/* m */
#define DELTA_M (M_PI * WHEEL_DIAMETER / 2000)
#define MAX_ACCELERATION 0.5
#define MIN_SPEED 0.01

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

enum
{
	ms_init, ms_fwd, ms_turn, ms_end
};

#define MIN(X, Y) (((X) < (Y)) ? (X) : (Y))
#define MAX(X, Y) (((X) > (Y)) ? (X) : (Y))

double getAcceleratedSpeed(double stdSpeed, double distanceLeft, int tickTime)
{
	double speedFunc = sqrt(2 * (MAX_ACCELERATION) * distanceLeft);
	double accFunc = (MAX_ACCELERATION / 100) * tickTime;
	double speed = MIN(MIN(stdSpeed, speedFunc), accFunc);
	//printf("%f %f %f %d %f\n", stdSpeed, speedFunc, accFunc, tickTime, speed);
	return speed;
}

void update_motcon(motiontype *p, int tickTime)
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
			p->startpos = (p->angle > 0) ? p->right_pos : p->left_pos;
			p->curcmd = mot_turn;
			break;
		}
		p->cmd = 0;
	}

	double distLeft = p->dist - (((p->right_pos + p->left_pos) / 2) - p->startpos);
	switch (p->curcmd) {
	case mot_stop:
		p->motorspeed_l = 0;
		p->motorspeed_r = 0;
		break;
	case mot_move:
		if (distLeft <= 0)
		{
			p->finished = 1;
			p->motorspeed_l = 0;
			p->motorspeed_r = 0;
		}
		else
		{
			p->motorspeed_l = MAX(getAcceleratedSpeed(p->speedcmd, distLeft, tickTime), MIN_SPEED);
			p->motorspeed_r = p->motorspeed_l;
		}
		break;

	case mot_turn:
		if (p->angle > 0)
		{
			if (p->right_pos - p->startpos < (p->angle * p->w) / 2)
			{
				distLeft = (p->angle * p->w) / 2 - (p->right_pos - p->startpos);
				p->motorspeed_r = MAX(getAcceleratedSpeed(p->speedcmd, distLeft, tickTime) / 2, MIN_SPEED);
				p->motorspeed_l = -p->motorspeed_r;
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
				distLeft = (fabs(p->angle) * p->w) / 2 - (p->left_pos - p->startpos);
				p->motorspeed_l = MAX(getAcceleratedSpeed(p->speedcmd, distLeft, tickTime) / 2, MIN_SPEED);
				p->motorspeed_r = -p->motorspeed_l;
			}
			else
			{
				p->motorspeed_r = 0;
				p->motorspeed_l = 0;
				p->finished = 1;
			}
		}
		break;
	}
}

int fwd(motiontype *mot, double dist, double speed, int time)
{
	if (time == 0)
	{
		mot->cmd = mot_move;
		mot->speedcmd = speed;
		mot->dist = dist;
		return 0;
	}
	else
	{
		return mot->finished;
	}
}

int turn(motiontype *mot, double angle, double speed, int time)
{
	if (time == 0)
	{
		mot->cmd = mot_turn;
		mot->speedcmd = speed;
		mot->angle = angle;
		return 0;
	}
	else
	{
		return mot->finished;
	}
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
	int running;
	int n = 0;
	int arg;
	int time = 0;
	double dist = 0;
	double angle = 0;

	odotype odo;
	smtype mission;
	motiontype mot;

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
			if (fwd(&mot, dist, 0.6, mission.time))
				mission.state = ms_turn;
			break;
		case ms_turn:
			if (turn(&mot, angle, 0.3, mission.time))
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
		update_motcon(&mot, mission.time);
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
	writeLogs("logging.txt");
	exit(0);
}

