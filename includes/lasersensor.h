/*
 * lasersensor.h
 *
 *  Created on: Jan 17, 2017
 *      Author: smr
 */

#ifndef LASERSENSOR_H_
#define LASERSENSOR_H_

#include <vector>

enum LaserDistance
{
	laser_left = 499, laser_center = 250, laser_right = 0
};

typedef struct spoint
{
	double x;
	double y;

	spoint operator+(const spoint& p)
	{
		spoint sum;
		sum.x = x + p.x;
		sum.y = y + p.y;
		return sum;
	}

	spoint operator/(const spoint& p)
	{
		spoint sum;
		sum.x = x / p.x;
		sum.y = y / p.y;
		return sum;
	}
	spoint operator/(const int& s)
		{
			spoint sum;
			sum.x = x / s;
			sum.y = y / s;
			return sum;
		}
} point;

typedef struct despillar
{
	point pos;
	point nearestPos;
	point *points;
	int pointsCount;

	~despillar()
	{
		delete[] points;
	}

} pillar;

typedef struct deswall
{
	point startPos;
	point endPos;
	point nearestPos;
	point *points;
	int pointsCount;

	~deswall()
	{
		delete[] points;
	}
} wall;

typedef struct deslaserObjects
{
	std::vector<pillar*> pillars;
	std::vector<wall*> walls;

	~deslaserObjects()
	{
		for (unsigned int i = 0; i < pillars.size(); ++i)
		{
			delete pillars[i];
		}
		for (unsigned int i = 0; i < walls.size(); ++i)
		{
			delete walls[i];
		}
	}
} laserObjects;

laserObjects* getLaserObjects(const int startAngle, const int searchAngle);

double getLaserDistance(enum LaserDistance l);

#endif /* LASERSENSOR_H_ */
