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
#include "includes/linesensor.h"

/*****************************************
 * odometry
 */
#define WHEEL_DIAMETER   0.067	/* m */
#define WHEEL_SEPARATION 0.256	/* m */
#define DELTA_M (M_PI * WHEEL_DIAMETER / 2000)
#define MAX_ACCELERATION 0.5
#define MIN_SPEED 0.01
#define TICKS_PER_SECOND 100
#define MIN_ACCELERATION (MAX_ACCELERATION / TICKS_PER_SECOND)
#define WHEEL_CENTER_TO_LINE_SENSOR_DISTANCE 22

#define ANGLE(x) ((double)x / 180.0 * M_PI)

static inline double min(const double x, const double y)
{
	return ((x) < (y)) ? (x) : (y);
}

static inline double max(const double x, const double y)
{
	return ((x) > (y)) ? (x) : (y);
}

static double getAcceleratedSpeed(const double stdSpeed, const double distanceLeft, const int tickTime)
{
	const double speedFunc = sqrt(2 * (MAX_ACCELERATION) * distanceLeft);
	const double accFunc = (MAX_ACCELERATION / TICKS_PER_SECOND) * tickTime;
	const double speed = (stdSpeed > 0) ? min(min(stdSpeed, speedFunc), accFunc) : max(max(stdSpeed, speedFunc), accFunc);
	//printf("%f %f %f %d %f\n", stdSpeed, speedFunc, accFunc, tickTime, speed);
	return speed;
}

static void syncAndUpdateOdo(odotype *odo)
{
	if (lmssrv.config && lmssrv.status && lmssrv.connected)
	{
		while ((xml_in_fd(xmllaser, lmssrv.sockfd) > 0))
		{
			xml_proca(xmllaser);
		}
	}

	if (camsrv.config && camsrv.status && camsrv.connected)
	{
		while ((xml_in_fd(xmldata, camsrv.sockfd) > 0))
		{
			xml_proc(xmldata);
		}
	}

	rhdSync();
	odo->leftWheelEncoderTicks = lenc->data[0];
	odo->rightWheelEncoderTicks = renc->data[0];
	updateOdo(odo);
}

static void exitOnButtonPress()
{
	int arg;
	ioctl(0, FIONREAD, &arg);
	if (arg != 0)
	{
		rhdSync();
		rhdDisconnect();
		exit(0);
	}
}

static void setMotorSpeeds(const double leftSpeed, const double rightSpeed)
{
	printf("%f %f\n", leftSpeed, rightSpeed);

	speedl->data[0] = 100 * leftSpeed;
	speedl->updated = 1;
	speedr->data[0] = 100 * rightSpeed;
	speedr->updated = 1;
}

static void fwd(odotype *odo, const double dist, const double speed, int (*stopCondition)(odotype*))
{
	const double startpos = (odo->rightWheelPos + odo->leftWheelPos) / 2;
	int time = 0;

	double distLeft;
	do
	{
		syncAndUpdateOdo(odo);

		distLeft = dist - (((odo->rightWheelPos + odo->leftWheelPos) / 2) - startpos);

		const double motorSpeed = max(getAcceleratedSpeed(speed, distLeft, time), MIN_SPEED);

		setMotorSpeeds(motorSpeed, motorSpeed);

		time++;

		exitOnButtonPress();

		if ((*stopCondition)(odo))
		{
			break;
		}

	} while (distLeft > 0);

	setMotorSpeeds(0, 0);
}

static void fwdTurn(odotype *odo, const double angle, const double speed)
{
	int time = 0;
	//angle %= 2*M_PI; //Setting it in the range of 0 to 2 Pi.
	//printf("Starting with angle = %f, odo angle = %f\n", angle, odo->angle);
	double angleDifference;
	do
	{
		//printf("%f, %f\n", angleDifference, ANGLE(0.5));
		syncAndUpdateOdo(odo);
		angleDifference = angle - odo->angle;
#define K_MOVE_TURN 0.2
		double deltaV = max(K_MOVE_TURN * (angleDifference), MIN_SPEED); //Check this for general case.(*)
		//printf("deltaV = %f\n", deltaV);
		const double motorSpeed = max(getAcceleratedSpeed(speed, deltaV / 4, time) / 2, MIN_SPEED); //Modify to use this (*)
		setMotorSpeeds(motorSpeed - deltaV / 2, motorSpeed + deltaV / 2);
		time++;
		exitOnButtonPress();
	} while (fabs(angleDifference) > ANGLE(0.1));
	//printf("%f\n", angleDifference);

	setMotorSpeeds(0, 0);
}

static void turn(odotype *odo, const double angle, const double speed, int (*stopCondition)(odotype*))
{
	const double startpos = (angle > 0) ? odo->rightWheelPos : odo->leftWheelPos;
	int time = 0;

	double distLeft;
	do
	{
		syncAndUpdateOdo(odo);

		distLeft = (fabs(angle) * odo->wheelSeparation) / 2 - (((angle > 0) ? odo->rightWheelPos : odo->leftWheelPos) - startpos);

		const double motorSpeed = max(getAcceleratedSpeed(speed, distLeft, time) / 2, MIN_SPEED);
		if (angle > 0)
		{
			setMotorSpeeds(-motorSpeed, motorSpeed);
		}
		else
		{
			setMotorSpeeds(motorSpeed, -motorSpeed);
		}

		time++;

		exitOnButtonPress();

		if ((*stopCondition)(odo))
		{
			break;
		}

	} while (distLeft > 0);

	setMotorSpeeds(0, 0);
}

static void followLine(odotype *odo, const double dist, const double speed, const enum lineCentering centering, int (*stopCondition)(odotype*))
{
	const double endPosition = odo->totalDistance + dist;
	int time = 0;

	double distLeft;
	do
	{
		syncAndUpdateOdo(odo);

		distLeft = endPosition - odo->totalDistance;

		const double motorSpeed = max(getAcceleratedSpeed(speed, distLeft, time), MIN_SPEED);
		const double lineOffDist = getLineOffSetDistance(centering);
		const double thetaRef = atan(lineOffDist / WHEEL_CENTER_TO_LINE_SENSOR_DISTANCE) + odo->angle;
		const double K = 2;
		const double speedDiffPerMotor = (K * (thetaRef - odo->angle)) / 2;

		setMotorSpeeds(motorSpeed - speedDiffPerMotor, motorSpeed + speedDiffPerMotor);

		time++;
		exitOnButtonPress();

		if ((*stopCondition)(odo))
		{
			break;
		}

	} while (distLeft > 0);

	setMotorSpeeds(0, 0);
}

static int noStopCondition(odotype *odo)
{
	return 0;
}

int main()
{
	odotype odo = { 0 };

	printf("Started");

	if (!readLineSensorValues("linesensor_calib_script/linesensor_calib.txt"))
	{
		//exit(EXIT_FAILURE);
	}

	if (!connectRobot())
	{
		exit(EXIT_FAILURE);
	}

	/* Read sensors and zero our position.
	 */
	rhdSync();

	odo.wheelSeparation = 0.256;
	odo.metersPerEncoderTick = DELTA_M;
	odo.leftWheelEncoderTicks = lenc->data[0];
	odo.rightWheelEncoderTicks = renc->data[0];
	odo.oldLeftWheelEncoderTicks = odo.leftWheelEncoderTicks;
	odo.oldRightWheelEncoderTicks = odo.rightWheelEncoderTicks;
	printf("position: %f, %f\n", odo.leftWheelPos, odo.rightWheelPos);
	/*
	 fwd(&odo, 1, 0.6, &noStopCondition);
	 turn(&odo, ANGLE(90), 0.3, &noStopCondition);

	 fwd(&odo, 1, 0.6, &noStopCondition);
	 turn(&odo, ANGLE(90), 0.3, &noStopCondition);

	 fwd(&odo, 1, 0.6, &noStopCondition);
	 turn(&odo, ANGLE(90), 0.3, &noStopCondition);
s
	 fwd(&odo, 1, 0.6, &noStopCondition);
	 turn(&odo, ANGLE(90), 0.3, &noStopCondition);
	 */
	turn(&odo, ANGLE(180), 0.3, &noStopCondition);
	followLine(&odo, 3000, -0.2, center, &noStopCondition);

	setMotorSpeeds(0, 0);
	rhdSync();
	rhdDisconnect();
	writeLogs("logging.txt");
	exit(0);
}

