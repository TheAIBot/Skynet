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
#define MAX_ACCELERATION 1.0
#define MIN_SPEED 0.01
#define TICKS_PER_SECOND 100
#define MIN_ACCELERATION (MAX_ACCELERATION / TICKS_PER_SECOND)

#define ANGLE(x) ((double)x / 180.0 * M_PI)

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
	//printf("%f %f %f %d %f\n", stdSpeed, speedFunc, accFunc, tickTime, speed);
	return speed;
}

static double getLineOffSetDistance()
{
	double sum_m = 0;
	double sum_i = 0;
	int i;
	for (i = 0; i < LINE_SENSORS_COUNT; i++)
	{
		int sensorValue = linesensor->data[i];
		printf("%d\n", sensorValue);
		double calibValue = calibrateLineSensorValue(sensorValue, i);
		printf("%f\n", calibValue);
		sum_m += (1 - calibValue) * i;
		sum_i += (1 - calibValue);
	}
	double c_m = sum_m / sum_i;
	printf("%f\n", c_m);
	return (( LINE_SENSOR_WIDTH / LINE_SENSORS_COUNT) * c_m - 6.5);
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
	odo->left_enc = lenc->data[0];
	odo->right_enc = renc->data[0];
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

static void setMotorSpeeds(double leftSpeed, double rightSpeed)
{
	static double currentSpeedLeft = 0;
	static double currentSpeedRight = 0;

	double diffLeft = leftSpeed - currentSpeedLeft;
	double correctSpeedLeft;
	if (diffLeft != 0)
	{
		correctSpeedLeft = (diffLeft > 0) ? min(leftSpeed, currentSpeedLeft + MIN_ACCELERATION) : max(leftSpeed, currentSpeedLeft - MIN_ACCELERATION);
	}
	else
	{
		correctSpeedLeft = leftSpeed;
	}

	double diffRight = rightSpeed - currentSpeedRight;
	double correctSpeedRight;
	if (diffRight != 0)
	{
		correctSpeedRight = (diffRight > 0) ? min(rightSpeed, currentSpeedRight + MIN_ACCELERATION) : max(rightSpeed, currentSpeedRight - MIN_ACCELERATION);
	}
	else
	{
		correctSpeedRight = rightSpeed;
	}

	currentSpeedLeft = correctSpeedLeft;
	currentSpeedRight = correctSpeedRight;

	//printf("%f %f\n", currentSpeedLeft, currentSpeedRight);

	speedl->data[0] = 100 * correctSpeedLeft;
	speedl->updated = 1;
	speedr->data[0] = 100 * correctSpeedRight;
	speedr->updated = 1;
}

static void fwd(odotype *odo, double dist, double speed)
{
	double startpos = (odo->right_pos + odo->left_pos) / 2;
	int time = 0;

	double distLeft;
	do
	{
		syncAndUpdateOdo(odo);

		distLeft = dist - (((odo->right_pos + odo->left_pos) / 2) - startpos);

		double motorSpeed = max(getAcceleratedSpeed(speed, distLeft, time), MIN_SPEED);

		setMotorSpeeds(motorSpeed, motorSpeed);

		time++;

		exitOnButtonPress();

	} while (distLeft > 0);

	setMotorSpeeds(0, 0);
}

static void turn(odotype *odo, double angle, double speed)
{
	double startpos = (angle > 0) ? odo->right_pos : odo->left_pos;
	int time = 0;

	double distLeft;
	do
	{
		syncAndUpdateOdo(odo);

		distLeft = (fabs(angle) * odo->w) / 2 - (((angle > 0) ? odo->right_pos : odo->left_pos) - startpos);

		double motorSpeed = max(getAcceleratedSpeed(speed, distLeft, time) / 2, MIN_SPEED);
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

	} while (distLeft > 0);

	setMotorSpeeds(0, 0);
}

static void follow_line(odotype *odo, double dist, double speed)
{
	double startpos = odo->totalDistance + dist;
	int time = 0;

	double distLeft;
	do
	{
		syncAndUpdateOdo(odo);

		distLeft = startpos - odo->totalDistance;

		double motorSpeed = max(getAcceleratedSpeed(speed, distLeft, time), MIN_SPEED);
		//double motorSpeedr =max(getAcceleratedSpeed(speed, distLeft, time), MIN_SPEED);
		double lineOffDist = getLineOffSetDistance();
		const double CENTER_TO_LINE_SENSOR_DISTANCE = 22;
		const double K = 4;
		double thetaRef = atan(lineOffDist / CENTER_TO_LINE_SENSOR_DISTANCE) + odo->angle;
		double speedDiffPerMotor = (K * (thetaRef - odo->angle)) / 2;

		setMotorSpeeds(motorSpeed - speedDiffPerMotor, motorSpeed + speedDiffPerMotor);

		time++;
		exitOnButtonPress();

	} while (distLeft > 0);

	setMotorSpeeds(0, 0);
}

int main()
{
	odotype odo = { 0 };

	if (!readLineSensorValues("calibvalues.txt"))
	{
		exit(EXIT_FAILURE);
	}

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
	resetOdo(&odo);
	printf("position: %f, %f\n", odo.left_pos, odo.right_pos);

	/*
	 fwd(&odo, 1, 0.6);
	 turn(&odo, ANGLE(90), 0.3);

	 fwd(&odo, 1, 0.6);
	 turn(&odo, ANGLE(90), 0.3);

	 fwd(&odo, 1, 0.6);
	 turn(&odo, ANGLE(90), 0.3);

	 fwd(&odo, 1, 0.6);
	 turn(&odo, ANGLE(90), 0.3);
	 */

	follow_line(&odo, 3000, 0.6);

	setMotorSpeeds(0, 0);
	rhdSync();
	rhdDisconnect();
	writeLogs("logging.txt");
	exit(0);
}
