/*
 * log.h
 *
 *  Created on: Jan 4, 2017
 *      Author: smr
 */

#ifndef LOG_H_
#define LOG_H_

#include "odometry.h"

void logOdo(const odotype * odo);

void writeLogs(const char* fileName);

#endif /* LOG_H_ */
