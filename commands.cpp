#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <math.h>
#include <unistd.h>
#include <stdlib.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <iostream>
#include "includes/robotconnector.h"
#include "includes/log.h"
#include "includes/irsensor.h"
#include "includes/commands.h"
#include "includes/serverif.h"
#include "includes/lasersensor.h"

#define WHEEL_DIAMETER   0.067	/* m */
#define WHEEL_SEPARATION 0.256	/* m */
#define DELTA_M (M_PI * WHEEL_DIAMETER / 2000) /* rad */
#define MAX_ACCELERATION 0.5 /* m/s^2 */
#define MIN_SPEED 0.01 /* m/s */
#define TICKS_PER_SECOND 100
#define MAX_ACCELERATION_PER_TICK (MAX_ACCELERATION / TICKS_PER_SECOND) /* m/s^2 */

//converts an angle in deg to rad
#define ANGLE(x) ((double)x / 180.0 * M_PI)

/*
 * Returns the minimum of x and y
 */
inline double min(const double x, const double y)
{
	return ((x) < (y)) ? (x) : (y);
}

/*
 * Returns the maximum og x and y
 */
inline double max(const double x, const double y)
{
	return ((x) > (y)) ? (x) : (y);
}

/*
 * Returns a speed that takes acceleration and deacceleration into account where stdSpeed is max speed,
 * distanceLeft is the distance the robot has left to go and tickTime is the amount of ticks
 * since the robot began accelerating
 */
double getAcceleratedSpeed(const double stdSpeed, const double distanceLeft, const int tickTime)
{
	const double speedFunc = sqrt(2 * (MAX_ACCELERATION) * fabs(distanceLeft));
	const double accFunc = (MAX_ACCELERATION / TICKS_PER_SECOND) * tickTime;
	//to take negative speeds into account the max has to be taken if stdSpeed is negative
	return (stdSpeed >= 0) ? min(min(stdSpeed, speedFunc), accFunc) : max(max(stdSpeed, -speedFunc), -accFunc);
}

/*
 * Updates odo, laser and camera values if they are available
 */
void syncAndUpdateOdo(odotype* const odo)
{
	//static clock_t startTime = clock();

	//sync with robot
	rhdSync();

	updateCameraData();
	updateLaserData();

	//update odo
	odo->wheelsEncoderTicks.left = lenc->data[0];
	odo->wheelsEncoderTicks.right = renc->data[0];
	updateOdo(odo);

	//printf("%fms\n", ((double)(clock() - startTime) / CLOCKS_PER_SEC) * 1000);
	//startTime = clock();
}

/*
 * Set right and left motor speed without taking accelerating into account
 */
void forceSetMotorSpeeds(const double leftSpeed, const double rightSpeed)
{
	speedl->data[0] = 100 * leftSpeed;
	speedl->updated = 1;
	speedr->data[0] = 100 * rightSpeed;
	speedr->updated = 1;
}

/*
 * Stops program if any key other than p was pressed.
 * If p was pressed then pause the program and resume when
 * p is pressed again
 */
void exitOnButtonPress()
{
	int arg;
	//wait for any character
	ioctl(0, FIONREAD, &arg);
	if (arg != 0)
	{
		forceSetMotorSpeeds(0, 0);
		rhdSync();
		rhdDisconnect();
		exit(0);
	}
}

/*
 * Tries to set motor speeds to leftSpeed and rightSpeed but takes acceleration into account
 * which makes sure that the robot can't accelerate too fast
 */
static void setMotorSpeeds(const double leftSpeed, const double rightSpeed)
{
	//these two variables contains the current speed of the robot
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

/*
 * Runs until the robot has come to a complete stop
 */
static void waitForCompleteStop(odotype* const odo)
{
	wheels<int> previousWheelsEncoderTicks;
	do
	{
		previousWheelsEncoderTicks = odo->wheelsEncoderTicks;

		syncAndUpdateOdo(odo);
		setMotorSpeeds(0, 0);
		exitOnButtonPress();

		//Only stop when there is no difference in ticks for both wheels since last sync
	} while (previousWheelsEncoderTicks != odo->wheelsEncoderTicks);
}

/*
 * Makes the robot go forward
 */
void fwd(odotype* const odo, const double dist, const double speed, bool (*stopCondition)(odotype*))
{
	const double startpos = odo->totalDistance;
	int time = 0;

	double distLeft;
	do
	{
		syncAndUpdateOdo(odo);
		distLeft = dist - (odo->totalDistance - startpos);
		const double motorSpeed = max(getAcceleratedSpeed(speed, distLeft, time), MIN_SPEED);
		setMotorSpeeds(motorSpeed, motorSpeed);
		time++;
		exitOnButtonPress();

	} while (distLeft > 0 && !(*stopCondition)(odo));
	waitForCompleteStop(odo);
}

/*
 * Turns the robot up into an angle
 */
void fwdTurn(odotype* const odo, const double angle, const double speed, bool (*stopCondition)(odotype*))
{
	//remember forward regulated
	const double K_MOVE_TURN = 0.2;
	int time = 0;

	double angleDifference;
	do
	{
		syncAndUpdateOdo(odo);
		angleDifference = angle - odo->angle;
		const double deltaV = max(K_MOVE_TURN * (angleDifference), MIN_SPEED);
		const double motorSpeed = max(getAcceleratedSpeed(speed, deltaV / 4, time) / 2, MIN_SPEED);
		setMotorSpeeds(motorSpeed - deltaV / 2, motorSpeed + deltaV / 2);

		time++;
		exitOnButtonPress();
	} while (fabs(angleDifference) > ANGLE(0.1) && !(*stopCondition)(odo));
	odo->supposedAngle += angle;
	waitForCompleteStop(odo);
}

void fwdRegulated(odotype* const odo, const double dist, const double speed, bool (*stopCondition)(odotype*))
{
	const double K_MOVE_TURN = 0.5;
	const double startpos = odo->totalDistance;
	int time = 0;
	//const double startAngleDifference = odo->supposedAngle - odo->angle;
	double angleDifference, distLeftTurn, distLeftForward, distLeft;

	do
	{
		syncAndUpdateOdo(odo);
		//Finds the difference in the angle wanted and the one currently had. 
		angleDifference = odo->supposedAngle - odo->angle;
		printf("angleDifference = %f, supposedAngle = %f, angle = %f\n", angleDifference, odo->supposedAngle, odo->angle);
		// A combination of the distLeft formula for fwd and turn:
		distLeftForward = dist - (odo->totalDistance - startpos);
		//distLeftTurn = (fabs(startAngleDifference) * odo->wheelSeparation) / 2 - (((startAngleDifference > 0) ?	odo->rightWheelPos : odo->leftWheelPos) - startpos);
		distLeftTurn = 0;
		distLeft = distLeftForward + distLeftTurn; //Rewrite and recalculate (*).
		printf("distLeft = %f\n", distLeft);
		const double deltaV = (angleDifference > 0) ? max(min((K_MOVE_TURN * (angleDifference)), speed / 2), MIN_SPEED) : min(max((K_MOVE_TURN * (angleDifference)), -speed / 2), MIN_SPEED);
		printf("deltaV = %f\n", deltaV);
		// speed - fabs(deltaV), so that the collective speed at most can reach speed:
		const double motorSpeed = max(getAcceleratedSpeed(speed - fabs(deltaV), distLeft, time), MIN_SPEED);
		setMotorSpeeds(motorSpeed - deltaV / 2, motorSpeed + deltaV / 2);
		time++;
		exitOnButtonPress();
		printf("\n");
	} while (distLeft > 0 && !(*stopCondition)(odo));
	printf("distLeft = %f, and stop = %d", distLeft, (distLeft <= 0 && !(*stopCondition)(odo)));

	waitForCompleteStop(odo);
}

/*
 * Turns the robot angle rads
 */
void turn(odotype* const odo, const double angle, const double speed, bool (*stopCondition)(odotype*))
{

	//doesn't matter which wheel is used to measure the distance turned as they should turn the same amount
	const double startpos = odo->wheelsDrivenDistance.right;
	int time = 0;

	double distLeft;
	do
	{
		syncAndUpdateOdo(odo);
		//the distance the left and right wheel has left to move for the angle to be correct
		distLeft = ((fabs(angle) * odo->wheelSeparation) / 2) - fabs(odo->wheelsDrivenDistance.right - startpos);
		//printf("%f\n", odo->wheelsDrivenDistance.right);
		const double motorSpeed = max(getAcceleratedSpeed(speed, distLeft, time) / 2, MIN_SPEED);
		//allow for the robot to turn cw and ccw
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
	odo->supposedAngle += angle;
	waitForCompleteStop(odo);
}

/*
 * Makes the robot follow a line
 */
void followLine(odotype* const odo, const double dist, const double speed, enum LineCentering centering, enum LineColor color, bool (*stopCondition)(odotype*))
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
		const double lineOffDist = getLineOffsetDistance(centering, color);

		//calcuate how much the robot has to turn to keep the line in the middle of the robot
		const double maxDiff = atan(((double) LINE_SENSOR_WIDTH / 2) / (double) WHEEL_CENTER_TO_LINE_SENSOR_DISTANCE);
		const double thetaRef = atan(lineOffDist / WHEEL_CENTER_TO_LINE_SENSOR_DISTANCE);
		const double percentOff = (sin(thetaRef) / sin(maxDiff));

		setMotorSpeeds(motorSpeed - motorSpeed * percentOff, motorSpeed + motorSpeed * percentOff);

		time++;
		exitOnButtonPress();

	} while (distLeft > 0 && !(*stopCondition)(odo));
	odo->supposedAngle = odo->angle; //Reset relative angle, as it is impossible to know what angle one is supposed to be at here.
	waitForCompleteStop(odo);
}

/*
 * Make the robot follow a wall
 */
void followWall(odotype* const odo, const double dist, const double distanceFromWall, const double speed, bool (*stopCondition)(odotype*))
{
	const double startpos = odo->totalDistance;
	int time = 0;

	double distLeft;
	do
	{
		syncAndUpdateOdo(odo);

		distLeft = dist - (odo->totalDistance - startpos);
		const double motorSpeed = max(getAcceleratedSpeed(speed, distLeft, time), MIN_SPEED);
		//get distance from wall and calculate speed of wheel to keep dist distance from wall
		const double K = 0.5;
		//printf("%f\n", getLaserDistance(LaserDistance::laser_left));
		const double medTerm = -(distanceFromWall + 0.2 - getLaserDistance(LaserDistance::laser_left));
		//printf("%f\n", medTerm);
		const double speedDiffPerMotor = (K * medTerm) / 2;

		setMotorSpeeds(motorSpeed - speedDiffPerMotor, motorSpeed + speedDiffPerMotor);
		//forceSetMotorSpeeds(0,0);

		time++;
		exitOnButtonPress();

	} while (distLeft > 0 && !(*stopCondition)(odo));

	odo->supposedAngle = odo->angle; //Reset relative angle, as it is impossible to know what angle one is supposed to be at here.
	waitForCompleteStop(odo);
}

/*
 * Make the robot go through a gate
 */
void throughGate(odotype* const odo, const double dist, const double speed, bool (*stopCondition)(odotype*))
{
	const double endPosition = odo->totalDistance + dist;
	int time = 0;

	double distLeft;
	bool goneThroughGate = false;
	do
	{
		syncAndUpdateOdo(odo);

		double minLeftSide = 1000;
		double minRightSide = 1000;
		int minLeftSideIndex = -1;
		int minRightSideIndex = -1;

		//while the robot can see the gate the robot
		//should correct itself. When it can't see the gate anymore
		//it should go straight forward
		if (!goneThroughGate)
		{
			//this assumes that the gates pillars are the two closest things to the robot
			//Find the fist closest thing to the robot
			for (int i = 0; i < MAX_LASER_COUNT; ++i)
			{
				if (laserpar[i] > 0.01)
				{
					if (laserpar[i] < minLeftSide)
					{
						minLeftSide = laserpar[i];
						minLeftSideIndex = i;
					}
				}
			}

			//find the second closest thing the the robot that isn't close to the
			//first closest thing
			const double LASER_SPACEING = 40;
			for (int i = 0; i < MAX_LASER_COUNT; ++i)
			{
				if (laserpar[i] > 0.01 && (minLeftSideIndex - LASER_SPACEING > i || i > minLeftSideIndex + LASER_SPACEING))
				{
					if (laserpar[i] < minRightSide)
					{
						minRightSide = laserpar[i];
						minRightSideIndex = i;
					}
				}
			}

			//make the thing that was furthest to the left be set to minLeftSide
			if (minLeftSideIndex > minRightSideIndex)
			{
				const double temp = minLeftSide;
				minLeftSide = minRightSide;
				minRightSide = temp;

				const int tempIndex = minLeftSideIndex;
				minLeftSideIndex = minRightSideIndex;
				minRightSideIndex = tempIndex;
			}
			//printf("%f %f\n", minLeftSide, minRightSide);
			//printf("%d %d\n", minLeftSideIndex, minRightSideIndex);
		}
		else {
			minLeftSide = 1;
			minRightSide = 1;
		}

		distLeft = endPosition - odo->totalDistance;
		//now use the gates two pillars to calculate how the robot should turn to keep the distances equal
		const double motorSpeed = max(getAcceleratedSpeed(speed, distLeft, time), MIN_SPEED);
		const double K = 1;
		const double speedDiffPerMotor = (minLeftSide - minRightSide) * K;
		//printf("%f %f %f\n", speedDiffPerMotor, minLeftSide, minRightSide);

		//setMotorSpeeds(0, 0);

		setMotorSpeeds(motorSpeed + speedDiffPerMotor, motorSpeed - speedDiffPerMotor);

		time++;
		exitOnButtonPress();

		//if both pillars are close to the edge of the robots sight then mark the robot as if
		//it has gone through the gate already
		if (minLeftSideIndex < 30 && minRightSideIndex > MAX_LASER_COUNT - 30)
		{
			goneThroughGate = true;
		}

	} while (distLeft > 0 && !(*stopCondition)(odo));
	odo->supposedAngle = odo->angle; //Reset relative angle, as it is impossible to know what angle one is supposed to be at here.
	waitForCompleteStop(odo);
}
