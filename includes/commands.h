#ifndef COMMANDS_H_
#define COMMANDS_H_

#include "odometry.h"
#include "linesensor.h"

void setMotorSpeeds(const double leftSpeed, const double rightSpeed);

void fwd(odotype *odo, const double dist, const double speed, int (*stopCondition)(odotype*));

void fwdTurn(odotype *odo, const double angle, const double speed, int (*stopCondition)(odotype*));

void turn(odotype *odo, const double angle, const double speed, int (*stopCondition)(odotype*));

void followLine(odotype *odo, const double dist, const double speed, const enum lineCentering centering, int (*stopCondition)(odotype*));



#endif
