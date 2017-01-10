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
#include "includes/irsensor.h"
#include "includes/commands.h"

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

#define ANGLE(x) ((double)(x) / 180.0 * M_PI)

int main()
{
	odotype odo = { 0 };

	printf("Started");

	if 	   (!readLineSensorValues("sensor_calib_scripts/linesensor_calib.txt") ||
			!loadIRCalibrationData("sensor_calib_scripts/irSensorCalib.txt")   ||
			!connectRobot()){
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
	//printf("position: %f, %f\n", odo.leftWheelPos, odo.rightWheelPos);
	
	 fwd(&odo, 1, 0.6);
	 turn(&odo, ANGLE(90), 0.3);

	 fwd(&odo, 1, 0.6);
	 turn(&odo, ANGLE(90), 0.3);

	 fwd(&odo, 1, 0.6);
	 turn(&odo, ANGLE(90), 0.3);

	 fwd(&odo, 1, 0.6);
	 turn(&odo, ANGLE(90), 0.3);
	 //follow_line(&odo, 3000, 0.6);
	 

	//followLine(&odo, 3000, 0.6, left);	

	//fwd(&odo, 2.5, 0.6);
	//testIRDistance();

	setMotorSpeeds(0.0, 0.0);
	rhdSync();
	rhdDisconnect();
	writeLogs("logging.txt");
	exit(0);
}

