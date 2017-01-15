#ifndef COMMANDS_H_
#define COMMANDS_H_

#include "odometry.h"
#include "linesensor.h"

void syncAndUpdateOdo(odotype *odo);

void forceSetMotorSpeeds(const double leftSpeed, const double rightSpeed);

void setMotorSpeeds(const double leftSpeed, const double rightSpeed);

void fwd(odotype *odo, const double dist, const double speed, bool (*stopCondition)(odotype*));

void fwdTurn(odotype *odo, const double angle, const double speed, bool (*stopCondition)(odotype*));

void fwdRegulated(odotype *odo, const double dist, const double speed, bool (*stopCondition)(odotype*));

void turn(odotype *odo, const double angle, const double speed, bool (*stopCondition)(odotype*));

void followLine(odotype *odo, const double dist, const double speed, enum LineCentering centering, enum LineColor color, bool (*stopCondition)(odotype*));

void followWall(odotype *odo, const double dist, const double distanceFromWall, const double speed, bool (*stopCondition)(odotype*));

void throughGate(odotype *odo, const double dist, const double speed, bool (*stopCondition)(odotype*));

double measureDistance(odotype *odo);

#endif
