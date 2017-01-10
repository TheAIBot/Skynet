#ifndef COMMANDS_H_
#define COMMANDS_H_


//Probably to many includes...
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
#include "robotconnector.h"
#include "odometry.h"
#include "log.h"
#include "linesensor.h"
#include "irsensor.h"
#include "odometry.h"


#define K_MOVE_TURN 0.2
#define WHEEL_DIAMETER   0.067	/* m */
#define WHEEL_SEPARATION 0.256	/* m */
#define DELTA_M (M_PI * WHEEL_DIAMETER / 2000)
#define MAX_ACCELERATION 0.5
#define MIN_SPEED 0.01
#define TICKS_PER_SECOND 100
#define MIN_ACCELERATION (MAX_ACCELERATION / TICKS_PER_SECOND)
#define WHEEL_CENTER_TO_LINE_SENSOR_DISTANCE 22

#define ANGLE(x) ((double)(x) / 180.0 * M_PI)



void setMotorSpeeds(const double leftSpeed, const double rightSpeed);

void fwd(odotype *odo, const double dist, const double speed);

void fwdTurn(odotype *odo, const double angle, const double speed);

void turn(odotype *odo, const double angle, const double speed);

void followLine(odotype *odo, const double dist, const double speed, const enum lineCentering centering);


#endif