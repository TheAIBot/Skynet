/*
 * odometry.h
 *
 *  Created on: Jan 4, 2017
 *      Author: smr
 */

#ifndef ODOMETRY_H_
#define ODOMETRY_H_

#include "point.h"

template<typename T>
class wheels
{
public:
	T left;
	T right;

	wheels<T> operator+(const wheels<T>& p)
	{
		wheels<T> temp;
		temp.left = left + p.left;
		temp.right = right + p.right;
		return temp;
	}
	wheels<T> operator-(const wheels<T>& p)
	{
		wheels<T> temp;
		temp.left = left - p.left;
		temp.right = right - p.right;
		return temp;
	}
	wheels<T> operator*(const wheels<T>& p)
	{
		wheels<T> temp;
		temp.left = left * p.left;
		temp.right = right * p.right;
		return temp;
	}
	wheels<double> operator*(const double& s)
	{
		wheels<double> temp;
		temp.left = s * left;
		temp.right = s * right;
		return temp;
	}
	wheels<T> operator/(const wheels<T>& p)
	{
		wheels<T> temp;
		temp.left = left / p.left;
		temp.right = right / p.right;
		return temp;
	}
	void operator+=(const wheels<T>& p)
	{
		left += p.left;
		right += p.right;
	}
	bool operator!=(const wheels<T>& p)
	{
		return left != p.left || right != p.right;
	}
};

typedef struct
{
	// parameters
	double wheelSeparation;
	double metersPerEncoderTick;
	//output signals
	wheels<double> wheelsDrivenDistance;

	point<double> robotPosition;

	double angle;
	double totalDistance;
	//For forward regulated:
	double supposedAngle;
	//input signals
	wheels<int> wheelsEncoderTicks;
	// internal variables
	wheels<int> oldWheelsEncoderTicks;

} odotype;

void updateOdo(odotype* const p);

double getDistanceFromTicks(odotype* const p, double ticks);

#endif /* ODOMETRY_H_ */
