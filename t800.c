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

#define WHEEL_DIAMETER   0.067	// m
#define WHEEL_SEPARATION 0.256	// m
#define DELTA_M (M_PI * WHEEL_DIAMETER / 2000)
#define MAX_ACCELERATION 0.5
#define MIN_SPEED 0.01
#define TICKS_PER_SECOND 100
#define MIN_ACCELERATION (MAX_ACCELERATION / TICKS_PER_SECOND)
#define WHEEL_CENTER_TO_LINE_SENSOR_DISTANCE 22

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

	odo.wheelSeparation = 0.256;
	odo.metersPerEncoderTick = DELTA_M;
	odo.leftWheelEncoderTicks = lenc->data[0];
	odo.rightWheelEncoderTicks = renc->data[0];
	odo.oldLeftWheelEncoderTicks = odo.leftWheelEncoderTicks;
	odo.oldRightWheelEncoderTicks = odo.rightWheelEncoderTicks;

	followLine(&odo, 100, 0.2, right, &stopAtNeg85Deg);
	followLine(&odo, 100, 0.2, center, &stopAtBlackLine);
	turn(&odo, ANGLE(90), 0.3, &noStopCondition);
	fwd(&odo, 100, 0.2, &stopAtBlackLine);
	fwd(&odo, 0.1, 0.2, &noStopCondition);
	fwd(&odo, 100, 0.2, &stopAtBlackLine);
	fwd(&odo, 0.3, 0.2, &noStopCondition);
	turn(&odo, ANGLE(-90), 0.3, &noStopCondition);
	followLine(&odo, 100, 0.2, center, &stopAtBlackLine);
	fwd(&odo, 0.135, 0.2, &noStopCondition);
	turn(&odo, ANGLE(90), 0.3, &noStopCondition);
	followLine(&odo, 100, 0.2, right, &stopAtBlackLine);

	setMotorSpeeds(0.0, 0.0);
	rhdSync();
	rhdDisconnect();
	writeLogs("logging.txt");
	exit(0);
}

