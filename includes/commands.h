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

void fwd(odotype *odo, const double dist, const double speed, int (*stopCondition)(odotype*));

void fwdTurn(odotype *odo, const double angle, const double speed, int (*stopCondition)(odotype*));

void turn(odotype *odo, const double angle, const double speed, int (*stopCondition)(odotype*));

void followLine(odotype *odo, const double dist, const double speed, const enum lineCentering centering, int (*stopCondition)(odotype*));



#endif
