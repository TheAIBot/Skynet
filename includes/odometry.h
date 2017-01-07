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
} odotype;

void updateOdo(odotype *p);

#endif /* ODOMETRY_H_ */
