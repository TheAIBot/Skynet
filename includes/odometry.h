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
	//For forward regulated:
	double supposedAngle;
	//input signals
	int leftWheelEncoderTicks;
	int rightWheelEncoderTicks;
	// internal variables
	int oldLeftWheelEncoderTicks;
	int oldRightWheelEncoderTicks;

} odotype;

void updateOdo(odotype* const p);

double getDistanceFromTicks(odotype* const p, double ticks);

#endif /* ODOMETRY_H_ */
