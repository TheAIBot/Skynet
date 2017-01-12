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
#include "includes/log.h"
#include "includes/irsensor.h"
#include "includes/commands.h"
#include "includes/serverif.h"

#define WHEEL_DIAMETER   0.067	/* m */
#define WHEEL_SEPARATION 0.256	/* m */
#define DELTA_M (M_PI * WHEEL_DIAMETER / 2000)
#define MAX_ACCELERATION 0.5
#define MIN_SPEED 0.01
#define TICKS_PER_SECOND 100
#define MAX_ACCELERATION_PER_TICK (MAX_ACCELERATION / TICKS_PER_SECOND)

#define ANGLE(x) ((double)x / 180.0 * M_PI)

inline double min(const double x, const double y)
{
	return ((x) < (y)) ? (x) : (y);
}

inline double max(const double x, const double y)
{
	return ((x) > (y)) ? (x) : (y);
}

double getAcceleratedSpeed(const double stdSpeed, const double distanceLeft, const int tickTime)
{
	const double speedFunc = sqrt(2 * (MAX_ACCELERATION) * fabs(distanceLeft));
	const double accFunc = (MAX_ACCELERATION / TICKS_PER_SECOND) * tickTime;
	const double speed = (stdSpeed >= 0) ? min(min(stdSpeed, speedFunc), accFunc) : max(max(stdSpeed, -speedFunc), -accFunc);
	//printf("%f, %f, %f %d %f\n", stdSpeed, speedFunc, accFunc, tickTime, speed);
	return speed;
}

void syncAndUpdateOdo(odotype *odo)
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

void forceSetMotorSpeeds(const double leftSpeed, const double rightSpeed)
{
	//printf("%f %f\n", currentSpeedLeft, currentSpeedRight);

	speedl->data[0] = 100 * leftSpeed;
	speedl->updated = 1;
	speedr->data[0] = 100 * rightSpeed;
	speedr->updated = 1;
}

void exitOnButtonPress()
{
	int arg;
	ioctl(0, FIONREAD, &arg);
	if (arg != 0)
	{
		forceSetMotorSpeeds(0, 0);
		rhdSync();
		rhdDisconnect();
		exit(0);
	}
}

void setMotorSpeeds(const double leftSpeed, const double rightSpeed)
{
	static double currentSpeedLeft = 0;
	static double currentSpeedRight = 0;

	double diffLeft = leftSpeed - currentSpeedLeft;
	double correctSpeedLeft;
	if (diffLeft != 0)
	{
		correctSpeedLeft = (diffLeft > 0) ? min(leftSpeed, currentSpeedLeft + MAX_ACCELERATION_PER_TICK) : max(leftSpeed, currentSpeedLeft - MAX_ACCELERATION_PER_TICK);
	}
	else
	{
		correctSpeedLeft = leftSpeed;
	}

	double diffRight = rightSpeed - currentSpeedRight;
	double correctSpeedRight;
	if (diffRight != 0)
	{
		correctSpeedRight = (diffRight > 0) ? min(rightSpeed, currentSpeedRight + MAX_ACCELERATION_PER_TICK) : max(rightSpeed, currentSpeedRight - MAX_ACCELERATION_PER_TICK);
	}
	else
	{
		correctSpeedRight = rightSpeed;
	}

	currentSpeedLeft = correctSpeedLeft;
	currentSpeedRight = correctSpeedRight;

	//printf("%f %f\n", currentSpeedLeft, currentSpeedRight);

	speedl->data[0] = 100 * currentSpeedLeft;
	speedl->updated = 1;
	speedr->data[0] = 100 * currentSpeedRight;
	speedr->updated = 1;
}

void waitForCompleteStopAndCorrectPosition(odotype* odo)
{
	const double startRightWheelPos = odo->rightWheelPos;
	const double startLeftWheelPos = odo->leftWheelPos;
	int previousRightWheelEncoderTicks;
	int previousLeftWheelEncoderTicks;
	do
	{
		previousRightWheelEncoderTicks = odo->rightWheelEncoderTicks;
		previousLeftWheelEncoderTicks = odo->leftWheelEncoderTicks;

		syncAndUpdateOdo(odo);
		setMotorSpeeds(0, 0);
		exitOnButtonPress();

	} while (previousRightWheelEncoderTicks != odo->rightWheelEncoderTicks || previousLeftWheelEncoderTicks != odo->leftWheelEncoderTicks);
	//robot has now come to a complete stop. Now need to correct the robots position

	const double distanceForRightWheel = startRightWheelPos - odo->rightWheelPos;
	const double distanceForLeftWheel = startLeftWheelPos - odo->leftWheelPos;
	const double oldRightWheelPos = odo->rightWheelPos;
	const double oldLeftWheelPos = odo->leftWheelPos;

	int time = 0;

	//this will get it close but not all the way there.
	double distanceLeftForRightWheel;
	double distanceLeftForLeftWheel;
	do
	{
		syncAndUpdateOdo(odo);

		distanceLeftForRightWheel = distanceForRightWheel - (odo->rightWheelPos - oldRightWheelPos);
		distanceLeftForLeftWheel = distanceForLeftWheel - (odo->leftWheelPos - oldLeftWheelPos);

		//printf("%f, %f\n", distanceLeftForRightWheel, distanceLeftForLeftWheel);

#define ACCEPTABLE_DISTANCE_ERROR 0.005
		double motorSpeedRight;
		if (fabs(distanceLeftForRightWheel) < ACCEPTABLE_DISTANCE_ERROR)
		{
			motorSpeedRight = 0;
		}
		else
		{
			motorSpeedRight = (distanceLeftForRightWheel >= 0) ? max(getAcceleratedSpeed(0.05, distanceLeftForRightWheel, time), MIN_SPEED) : min(getAcceleratedSpeed(-0.05, distanceLeftForRightWheel, time), -MIN_SPEED);
		}

		double motorSpeedLeft;
		if (fabs(distanceLeftForLeftWheel) < ACCEPTABLE_DISTANCE_ERROR)
		{
			motorSpeedLeft = 0;
		}
		else
		{
			motorSpeedLeft = (distanceLeftForLeftWheel >= 0) ? max(getAcceleratedSpeed(0.05, distanceLeftForLeftWheel, time), MIN_SPEED) : min(getAcceleratedSpeed(-0.05, distanceLeftForLeftWheel, time), -MIN_SPEED);
		}
		//printf("%f, %f\n", motorSpeedRight, motorSpeedLeft);

		setMotorSpeeds(motorSpeedLeft, motorSpeedRight);

		time++;

		exitOnButtonPress();

	} while (fabs(distanceLeftForRightWheel) > ACCEPTABLE_DISTANCE_ERROR || fabs(distanceLeftForLeftWheel) > ACCEPTABLE_DISTANCE_ERROR);

	setMotorSpeeds(0, 0);
}

void fwd(odotype *odo, const double dist, const double speed, int (*stopCondition)(odotype*))
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

	} while (distLeft > 0 && !(*stopCondition)(odo));

	waitForCompleteStopAndCorrectPosition(odo);
}

void fwdTurn(odotype *odo, const double angle, const double speed, int (*stopCondition)(odotype*))
{
#define K_MOVE_TURN 0.2
	int time = 0;

	double angleDifference;
	do
	{
		syncAndUpdateOdo(odo);
		angleDifference = angle - odo->angle;
		double deltaV = max(K_MOVE_TURN * (angleDifference), MIN_SPEED);
		const double motorSpeed = max(getAcceleratedSpeed(speed, deltaV / 4, time) / 2, MIN_SPEED);
		setMotorSpeeds(motorSpeed - deltaV / 2, motorSpeed + deltaV / 2);
		time++;
		exitOnButtonPress();
	} while (fabs(angleDifference) > ANGLE(0.1) && !(*stopCondition)(odo));

	waitForCompleteStopAndCorrectPosition(odo);
}

void turn(odotype *odo, const double angle, const double speed, int (*stopCondition)(odotype*))
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

	} while (distLeft > 0 && !(*stopCondition)(odo));

	waitForCompleteStopAndCorrectPosition(odo);
}

void followLine(odotype *odo, const double dist, const double speed, enum LineCentering centering, enum LineColor color, int (*stopCondition)(odotype*))
{

	const double endPosition = odo->totalDistance + dist;
	int time = 0;

	double distLeft;
	do
	{
		syncAndUpdateOdo(odo);

		distLeft = endPosition - odo->totalDistance;

		//tried to make it go backwards
		const double motorSpeed = (speed >= 0) ? max(getAcceleratedSpeed(speed, distLeft, time), MIN_SPEED) : min(getAcceleratedSpeed(speed, distLeft, time), -MIN_SPEED);
		const double lineOffDist = getLineOffSetDistance(centering, color);
		const double thetaRef = atan(lineOffDist / WHEEL_CENTER_TO_LINE_SENSOR_DISTANCE) + odo->angle;
		const double K = 2;
		const double speedDiffPerMotor = (K * (thetaRef - odo->angle)) / 2;

		if (speed >= 0)
		{
			setMotorSpeeds(motorSpeed - speedDiffPerMotor, motorSpeed + speedDiffPerMotor);
		}
		else
		{
			setMotorSpeeds(motorSpeed + speedDiffPerMotor, motorSpeed - speedDiffPerMotor);
		}

		time++;
		exitOnButtonPress();

	} while (distLeft > 0 && !(*stopCondition)(odo));

	waitForCompleteStopAndCorrectPosition(odo);
}

void followWall(odotype *odo, const double dist, const double speed, int (*stopCondition)(odotype*))
{
	const double startpos = (odo->rightWheelPos + odo->leftWheelPos) / 2;
	int time = 0;

	double distLeft;
	do
	{
		syncAndUpdateOdo(odo);

		distLeft = dist - (((odo->rightWheelPos + odo->leftWheelPos) / 2) - startpos);
		const double motorSpeed = max(getAcceleratedSpeed(speed, distLeft, time), MIN_SPEED);
		const double K = 0.05;
		const double medTerm = -(20 - irDistance(ir_left)); //A distance of 20 centimeters is optimal
		const double speedDiffPerMotor = (K * medTerm) / 2;
		printf("IR distance = %f, speedDiff = %f, motorSpeed = %f\n", irDistance(ir_left), speedDiffPerMotor, motorSpeed); //

		if (speed >= 0)
		{
			setMotorSpeeds(motorSpeed - speedDiffPerMotor, motorSpeed + speedDiffPerMotor);
		}
		else
		{
			setMotorSpeeds(motorSpeed + speedDiffPerMotor, motorSpeed - speedDiffPerMotor);
		}

		time++;
		exitOnButtonPress();

	} while (distLeft > 0 && !(*stopCondition)(odo));

	waitForCompleteStopAndCorrectPosition(odo);
}

double measureDistance(odotype *odo)
{
	int i;
	double sum = 0;
	for (i = 0; i < 100; i++)
	{
		syncAndUpdateOdo(odo);
		sum += irDistance(ir_front_left) + irDistance(ir_front_center) + irDistance(ir_front_right);
	}
	return sum / 100;
}
