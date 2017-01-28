#ifndef LASERSENSOR_H_
#define LASERSENSOR_H_

#include <vector>
#include "point.h"

#define ANGLE_TO_INDEX(x) ((x + (LASER_SEARCH_ANGLE / 2)) / ((double) LASER_SEARCH_ANGLE / MAX_LASER_COUNT))
#define INDEX_TO_ANGLE(x) (((double)LASER_SEARCH_ANGLE / MAX_LASER_COUNT) * x)

enum LaserDistance
{
	laser_left = 499, laser_center = 250, laser_right = 0
};

typedef struct despillar
{
	point<double> pos;
	point<double> nearestPos;
	point<double> *points;
	int pointsCount;

	static bool sortByLength(const despillar* a, const despillar* b)
	{
		return a->nearestPos.length() < b->nearestPos.length();
	}

	~despillar()
	{
		delete[] points;
	}

} pillar;

typedef struct deswall
{
	point<double> startPos;
	point<double> endPos;
	point<double> nearestPos;
	point<double> *points;
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
