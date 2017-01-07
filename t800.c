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
#define K_MOVE_TURN 0.2
#define MAX_ACCELERATION 0.5
#define MIN_SPEED 0.01
#define TICKS_PER_SECOND 100

#define ANGLE(x) ((double)x / 180.0 * M_PI)

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
	double left_pos;
	double right_pos;
	// parameters
	double w;
	//output
	double motorspeed_l, motorspeed_r;
	int finished;
	// internal variables
	double startpos;
	double angleRef;
} motiontype;

enum
{
	mot_stop = 1, mot_move, mot_turn, mot_follow_line, mot_move_turn
};

typedef struct
{
	int state;
	int oldstate;
	int time;
} smtype;

enum
{
	ms_init, ms_fwd, ms_turn, ms_end, ms_follow_line, ms_move_turn
};

static inline double min(double x, double y)
{
	return ((x) < (y)) ? (x) : (y);
}

static inline double max(double x, double y)
{
	return ((x) > (y)) ? (x) : (y);
}

static double getAcceleratedSpeed(double stdSpeed, double distanceLeft, int tickTime)
{
	double speedFunc = sqrt(2 * (MAX_ACCELERATION) * distanceLeft);
	double accFunc = (MAX_ACCELERATION / TICKS_PER_SECOND) * tickTime;
	double speed = min(min(stdSpeed, speedFunc), accFunc);
	printf("%f %f %f %d %f\n", stdSpeed, speedFunc, accFunc, tickTime, speed);
	return speed;
}

static void update_motcon(motiontype *p, odotype *odo, int tickTime)
{
	double distLeft = 0;
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
		case mot_move_turn:
			p->startpos = (p->left_pos + p->right_pos) / 2;
			p->curcmd = mot_move_turn;
			break;
		case mot_turn:
			p->startpos = (p->angle > 0) ? p->right_pos : p->left_pos;
			p->curcmd = mot_turn;
			break;
		case mot_follow_line:
			p->startpos = odo->totalDistance;
			p->curcmd = mot_follow_line;
		}
		p->cmd = 0;
	}

	switch (p->curcmd) {
	case mot_stop:
		p->motorspeed_l = 0;
		p->motorspeed_r = 0;
		break;
	case mot_move:
	{
		distLeft = p->dist - (((p->right_pos + p->left_pos) / 2) - p->startpos);
		if (distLeft <= 0)
		{
			p->finished = 1;
			p->motorspeed_l = 0;
			p->motorspeed_r = 0;
		}
		else
		{
			p->motorspeed_l = max(getAcceleratedSpeed(p->speedcmd, distLeft, tickTime), MIN_SPEED);
			p->motorspeed_r = p->motorspeed_l;
		}
		break;
	}
	case mot_move_turn:
	{
		printf("Difference angle %f, accepted = %f\n",(p->angleRef-odo->angle),ANGLE(0.5));
		//distLeft = p->dist - (((p->right_pos + p->left_pos) / 2) - p->startpos);
		if (p->angleRef-odo->angle <= ANGLE(0.5) || p->angleRef-odo->angle <= -ANGLE(0.5))
		{
			printf("Finished at angel differnce : %f\n", (p->angleRef-odo->angle));//
			p->finished = 1;
			p->motorspeed_l = 0;
			p->motorspeed_r = 0;
		}
		else
		{
			double deltaV = K_MOVE_TURN*(p->angleRef-odo->angle);
			printf("detlaV = %f\n",deltaV);
			p->motorspeed_l = max(getAcceleratedSpeed(p->speedcmd, 10, tickTime), MIN_SPEED) - deltaV/2;
			p->motorspeed_r = p->motorspeed_l + deltaV; //gives -deltaV/2
		}
		break;
	}
	case mot_turn:
	{
		distLeft = (fabs(p->angle) * p->w) / 2 - (((p->angle > 0) ? p->right_pos : p->left_pos) - p->startpos);
		if (distLeft <= 0)
		{
			p->finished = 1;
			p->motorspeed_r = 0;
			p->motorspeed_l = 0;
		}
		else
		{
			double speed = max(getAcceleratedSpeed(p->speedcmd, distLeft, tickTime) / 2, MIN_SPEED);
			if (p->angle > 0)
			{
				p->motorspeed_r = speed;
				p->motorspeed_l = -speed;
			}
			else
			{
				p->motorspeed_r = -speed;
				p->motorspeed_l = speed;
			}
		}
		break;
	}
	case mot_follow_line:
		break;
	}
}

static int fwd(motiontype *mot, double dist, double speed, int time)
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


static int fwdturn(motiontype *mot, double angleTurn, double speed, int time)
{
	if (time == 0)
	{
		mot->angleRef = angleTurn;
		mot->cmd = mot_move_turn;
		mot->speedcmd = speed;
		return 0;
	}
	else
	{
		return mot->finished;
	}
}


static int turn(motiontype *mot, double angle, double speed, int time)
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

static int follow_line(motiontype *mot, double dist, double speed, int time)
{
	if (time == 0)
	{
		mot->cmd = mot_follow_line;
		mot->speedcmd = speed;
		mot->dist = dist;
		return 0;
	}
	else
	{
		return mot->finished;
	}
}

static void sm_update(smtype *p)
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

static void setMotorSpeeds(double leftSpeed, double rightSpeed)
{
	speedl->data[0] = 100 * leftSpeed;
	speedl->updated = 1;
	speedr->data[0] = 100 * rightSpeed;
	speedr->updated = 1;
}

int main()
{
	int running;
	int n = 0;
	int arg;
	int time = 0;
	double dist = 0;
	double angle = 0;

	odotype odo = { 0 };
	smtype mission = { 0 };
	motiontype mot = { 0 };

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
			angle = ANGLE(90);
			//mission.state = ms_fwd;
			mission.state = ms_move_turn;
			break;
		case ms_fwd:
			if (fwd(&mot, dist, 0.6, mission.time))
			{
				mission.state = ms_turn;
			}
			break;
		case ms_turn:
			if (turn(&mot, angle, 0.3, mission.time))
			{
				n--;
				mission.state = (n == 0) ? ms_end : ms_fwd;
			}
			break;
		case ms_follow_line:
			if (follow_line(&mot, dist, 0.3, mission.time))
			{
				mission.state = ms_end;
			}
		case ms_move_turn:
			if(fwdturn(&mot,angle + odo.angle ,0.3,mission.time)){
				mission.state = ms_end;
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
		update_motcon(&mot,&odo, mission.time);
		setMotorSpeeds(mot.motorspeed_l, mot.motorspeed_r);
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
	setMotorSpeeds(0, 0);
	rhdSync();
	rhdDisconnect();
	writeLogs("logging.txt");
	exit(0);
}

