/*
 * odometry.h
 *
 *  Created on: Jan 4, 2017
 *      Author: smr
 */

#ifndef ODOMETRY_H_
#define ODOMETRY_H_

typedef struct
{ //input signals
	int left_enc;
	int right_enc; // encoderticks
	// parameters
	double w;	// wheel separation
	double cr;
	double cl;   // meters per encodertick
	//output signals
	double right_pos;
	double left_pos;
	// internal variables
	int left_enc_old;
	int right_enc_old;
	double xpos;
	double ypos;
	double angle;
	double totalDistance;
} odotype;

void resetOdo(odotype * p);

void updateOdo(odotype *p);



#endif /* ODOMETRY_H_ */
