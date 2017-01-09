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
	const double speed = min(min(stdSpeed, speedFunc), accFunc);
	//printf("%f %f %f %d %f\n", stdSpeed, speedFunc, accFunc, tickTime, speed);
	return speed;
}

static double getLineOffSetDistance(enum lineCentering centering)
{
	double sum_m = 0;
	double sum_i = 0;
	int i;
	for (i = 0; i < LINE_SENSORS_COUNT; i++)
	{
		const double calibValue = calibrateLineSensorValue(linesensor->data[i], i);
		sum_m += (1 - calibValue) * i;
		sum_i += (1 - calibValue);
	}
	const double c_m = sum_m / sum_i;
	return ((double) LINE_SENSOR_WIDTH / (LINE_SENSORS_COUNT - 1)) * c_m - getLineCenteringOffset(centering);
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
	//printf("%f %f\n", leftSpeed, rightSpeed);

	speedl->data[0] = 100 * leftSpeed;
	speedl->updated = 1;
	speedr->data[0] = 100 * rightSpeed;
	speedr->updated = 1;
}

static void fwd(odotype *odo, const double dist, const double speed)
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

	} while (distLeft > 0);

	setMotorSpeeds(0, 0);
}

static void turn(odotype *odo, const double angle, const double speed)
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

	} while (distLeft > 0);

	setMotorSpeeds(0, 0);
}

static void followLine(odotype *odo, const double dist, const double speed, const enum lineCentering centering)
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

	} while (distLeft > 0);

	setMotorSpeeds(0, 0);
}

int main()
{
	odotype odo = { 0 };

	if (!readLineSensorValues("linesensor_calib_script/linesensor_calib.txt"))
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

	odo.wheelSeparation = 0.256;
	odo.metersPerEncoderTick = DELTA_M;
	odo.leftWheelEncoderTicks = lenc->data[0];
	odo.rightWheelEncoderTicks = renc->data[0];
	odo.oldLeftWheelEncoderTicks = odo.leftWheelEncoderTicks;
	odo.oldRightWheelEncoderTicks = odo.rightWheelEncoderTicks;
	printf("position: %f, %f\n", odo.leftWheelPos, odo.rightWheelPos);

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

	followLine(&odo, 3000, 0.6, left);

	setMotorSpeeds(0, 0);
	rhdSync();
	rhdDisconnect();
	writeLogs("logging.txt");
	exit(0);
}

