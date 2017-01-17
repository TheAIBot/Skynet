#include <stdio.h>
#include <math.h>
#include "includes/lasersensor.h"
#include "includes/robotconnector.h"

#define MAX_DISTANCE_FOR_CONNECTED_POINTS 0.07
#define MAX_PILLAR_SIZE 0.1

static inline point getPointFromLaser(double dist, double angle)
{
	point newPoint;
	newPoint.x = dist * cos(angle);
	newPoint.y = dist * sin(angle);
	return newPoint;
}

static double getLength(const point p)
{
	return sqrt(pow(p.x, 2) + pow(p.y, 2));
}


static double distanceBetweenPoints(point a, point b)
{
	return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

static point getPointFromLaserIndex(int index)
{
	const double laserAngle = ((double) LASER_SEARCH_ANGLE / laserZoneCount) * index;
	return getPointFromLaser(laserpar[index], laserAngle);
}

static std::vector<std::vector<point>*>* getUnknownLaserObjects(const int startIndex, const int endIndex)
{
	const point dd250 = getPointFromLaserIndex(250);
	const point dd260 = getPointFromLaserIndex(251);
	printf("%f %f\n", dd250.x, dd250.y);
	printf("%f %f\n", dd260.x, dd260.y);
	printf("%f\n", MAX_DISTANCE_FOR_CONNECTED_POINTS * getLength(dd250));
	printf("%f\n", distanceBetweenPoints(dd250, dd260));
	std::vector<std::vector<point>*> *unknownObjects = new std::vector<std::vector<point>*>;
	bool isLaserUsed[MAX_LASER_COUNT] = { 0 };
	for (int i = startIndex; i < endIndex; ++i)
	{
		if (laserpar[i] > MIN_LASER_DISTANCE && !isLaserUsed[i])
		{
			std::vector<point> *objectPositions = new std::vector<point>;

			objectPositions->push_back(getPointFromLaserIndex(i));
			isLaserUsed[i] = true;

			for (int z = i + 1; z < endIndex; ++z)
			{
				if (laserpar[z] > MIN_LASER_DISTANCE)
				{
					const point obstaclePos = getPointFromLaserIndex(z);
					//printf("%f\n", distanceBetweenPoints(objectPositions->back(), obstaclePos));
					if (distanceBetweenPoints(objectPositions->back(), obstaclePos) <= MAX_DISTANCE_FOR_CONNECTED_POINTS * getLength(objectPositions->back()))
					{
						//printf("true\n");
						objectPositions->push_back(obstaclePos);
						isLaserUsed[z] = true;
					}
				}
			}
			if (unknownObjects->size() > 1)
			{
				unknownObjects->push_back(objectPositions);
			}
			else
			{
				delete objectPositions;
			}

		}
	}
	printf("%d\n", unknownObjects->size());
	return unknownObjects;
}

static laserObjects* getCategorizedLaserObject(std::vector<std::vector<point>*>& unknownObjects)
{
	laserObjects* objects = new laserObjects;
	for (unsigned int unknownObjectIndex = 0; unknownObjectIndex < unknownObjects.size(); ++unknownObjectIndex)
	{
		std::vector<point> unknownObject = *unknownObjects[unknownObjectIndex];
		const point firstPoint = unknownObject.front();
		bool isPillar = true;

		for (unsigned int i = 0; i < unknownObject.size(); ++i)
		{
			if (distanceBetweenPoints(firstPoint, unknownObject[i]) > MAX_PILLAR_SIZE)
			{
				isPillar = false;
				break;
			}
		}

		if (isPillar)
		{
			pillar* newPillar = new pillar;

			point pointsSum = { 0 };
			for (unsigned int i = 0; i < unknownObject.size(); ++i)
			{
				pointsSum.x += unknownObject[i].x;
				pointsSum.y += unknownObject[i].y;
			}
			pointsSum.x /= unknownObject.size();
			pointsSum.y /= unknownObject.size();
			newPillar->pos = pointsSum;

			point nearestPos = unknownObject[0];
			for (unsigned int i = 0; i < unknownObject.size(); ++i)
			{
				if (getLength(nearestPos) > getLength(unknownObject[i]))
				{
					nearestPos = unknownObject[i];
				}
			}
			newPillar->nearestPos = nearestPos;

			newPillar->points = new point[unknownObject.size()];
			for (unsigned int i = 0; i < unknownObject.size(); ++i)
			{
				newPillar->points[i] = unknownObject[i];
			}

			newPillar->pointsCount = unknownObject.size();

			objects->pillars.push_back(newPillar);
		}
		else
		{
			wall* newWall = new wall;

			newWall->startPos = unknownObject.front();
			newWall->endPos = unknownObject.back();

			point nearestPos = unknownObject[0];
			for (unsigned int i = 1; i < unknownObject.size(); ++i)
			{
				if (getLength(nearestPos) > getLength(unknownObject[i]))
				{
					nearestPos = unknownObject[i];
				}
			}
			newWall->nearestPos = nearestPos;

			newWall->points = new point[unknownObject.size()];
			for (unsigned int i = 0; i < unknownObject.size(); ++i)
			{
				newWall->points[i] = unknownObject[i];
			}

			newWall->pointsCount = unknownObject.size();

			objects->walls.push_back(newWall);
		}
	}
	return objects;
}

laserObjects* getLaserObjects(const int startAngle, const int searchAngle)
{
	const int startIndex = 0;
	const int endIndex = MAX_LASER_COUNT;

	std::vector<std::vector<point>*>* unknownObjects = getUnknownLaserObjects(startIndex, endIndex);

	return getCategorizedLaserObject(*unknownObjects);
}
