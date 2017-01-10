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
#include "includes/commands.h"
#include "includes/stopconditions.h"
#include "includes/irsensor.h"

#define WHEEL_DIAMETER   0.067	// m
#define WHEEL_SEPARATION 0.256	// m
#define DELTA_M (M_PI * WHEEL_DIAMETER / 2000)
#define MAX_ACCELERATION 0.5
#define MIN_SPEED 0.01
#define TICKS_PER_SECOND 100
#define MIN_ACCELERATION (MAX_ACCELERATION / TICKS_PER_SECOND)
#define WHEEL_CENTER_TO_LINE_SENSOR_DISTANCE 22
#define STD_SPEED 0.2

int main()
{
	odotype odo = { 0 };

	//need calib file for the problem to work
	if (!readLineSensorCalibrationData("sensor_calib_scripts/linesensor_calib.txt"))
	{
		exit(EXIT_FAILURE);
	}

	//need calib file for the problem to work
	if (!loadIRCalibrationData("sensor_calib_scripts/irSensorCalib.txt"))
	{
		exit(EXIT_FAILURE);
	}

	//can't run program if can't connect to robot
	if (!connectRobot())
	{
		exit(EXIT_FAILURE);
	}

	/* Read sensors and zero our position.
	 */
	rhdSync();

	odo.wheelSeparation = WHEEL_SEPARATION;
	odo.metersPerEncoderTick = DELTA_M;
	odo.leftWheelEncoderTicks = lenc->data[0];
	odo.rightWheelEncoderTicks = renc->data[0];
	odo.oldLeftWheelEncoderTicks = odo.leftWheelEncoderTicks;
	odo.oldRightWheelEncoderTicks = odo.rightWheelEncoderTicks;

	//go to box
	followLine(&odo, 100, STD_SPEED, LineCentering::right, LineColor::black, &stopAtDeg<-90, 5>);
	followLine(&odo, 100, STD_SPEED, LineCentering::center, LineColor::black, &stopAtLine<LineColor::black, 4>);

	//go to line towards box to move
	turn(&odo, ANGLE(90), STD_SPEED, &noStopCondition);
	fwd(&odo, 100, STD_SPEED, &stopAtLine<LineColor::black, 4>);
	fwd(&odo, 0.1, STD_SPEED, &noStopCondition);
	fwd(&odo, 100, STD_SPEED, &stopAtLine<LineColor::black, 4>);
	fwd(&odo, 0.3, STD_SPEED, &noStopCondition);
	turn(&odo, ANGLE(-90), STD_SPEED, &noStopCondition);

	//push box and go through gate
	followLine(&odo, 100, STD_SPEED, LineCentering::center, LineColor::black, &stopAtLine<LineColor::black, 4>);
	fwd(&odo, 0.135, STD_SPEED, &noStopCondition);
	turn(&odo, ANGLE(90), STD_SPEED, &noStopCondition);
	followLine(&odo, 1, STD_SPEED, LineCentering::right, LineColor::black, &stopAtLine<LineColor::black, 4>);
	followLine(&odo, 1, STD_SPEED, LineCentering::center, LineColor::black, &stopAtLine<LineColor::black, 4>);

	//Around the gates and to the wall.
	followLine(&odo, 100, STD_SPEED, LineCentering::center, LineColor::black, &stopAtDetectedPillar<IRSensor::ir_left, 50>);
	followLine(&odo, 0.4, STD_SPEED, LineCentering::center, LineColor::black, &noStopCondition); // change this to detect the wall?
	turn(&odo, ANGLE(90), STD_SPEED, &noStopCondition);
	fwd(&odo, 1, STD_SPEED, &noStopCondition);
	turn(&odo, ANGLE(90), STD_SPEED, &noStopCondition);
	fwd(&odo, 0.6, STD_SPEED, &noStopCondition);
	turn(&odo, ANGLE(90), STD_SPEED, &noStopCondition);
	fwd(&odo, 100, STD_SPEED, &stopAtLine<LineColor::black, 4>);
	fwd(&odo, 0.3, STD_SPEED, &noStopCondition);
	turn(&odo, ANGLE(90), STD_SPEED, &noStopCondition);


	followLine(&odo, 100, STD_SPEED, LineCentering::center, LineColor::black, &stopAtLine<LineColor::black, 4>);
	fwd(&odo, 0.3, STD_SPEED, &noStopCondition);
	turn(&odo, ANGLE(90), STD_SPEED, &noStopCondition);
	fwd(&odo, 0.1, STD_SPEED, &noStopCondition);

	followLine(&odo, 100, STD_SPEED, center, black, &stopAtDetectedPillar<ir_right , 20>);
	fwd(&odo, 0.5, STD_SPEED, &noStopCondition);
	turn(&odo, ANGLE(90), STD_SPEED, &noStopCondition);

	/*
	turn(&odo, ANGLE(90), 0.3, &noStopCondition);
	followLine(&odo, 100, 0.2, center, black, &stopAtBlockedForwardPath);
	double distance = measureDistance(&odo);
	turn(&odo, ANGLE(90), 0.3, &noStopCondition);
	followWall(&odo, distance, 0.2, &stopAtFreeRightIR);
	*/


	setMotorSpeeds(0.0, 0.0);
	rhdSync();
	rhdDisconnect();
	writeLogs("logging.txt");
	exit(0);
}

