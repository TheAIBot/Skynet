/*
 * odometry.h
 *
 *  Created on: Jan 4, 2017
 *      Author: smr
 */

#ifndef ODOMETRY_H_
#define ODOMETRY_H_

typedef struct
{
	// parameters
	double wheelSeparation;
	double metersPerEncoderTick;
	//output signals
	double rightWheelPos;
	double leftWheelPos;

	double xpos;
	double ypos;
	double angle;
	double totalDistance;
	//input signals
	int leftWheelEncoderTicks;
	int rightWheelEncoderTicks; // encoderticks
	// internal variables
	int oldLeftWheelEncoderTicks;
	int oldRightWheelEncoderTicks;
	//For forward regulated:
	double supposedAngle;
} odotype;

void updateOdo(odotype *p);

double getDistanceFromTicks(odotype *p, double ticks);

#endif /* ODOMETRY_H_ */
