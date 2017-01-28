#include <stdio.h>
#include <math.h>
#include "includes/lasersensor.h"
#include "includes/robotconnector.h"

#define MAX_DISTANCE_FOR_CONNECTED_POINTS 0.1
#define MAX_PILLAR_SIZE 0.1

#define ANGLE(x) ((double)x / 180.0 * M_PI)

static point<double> getPointFromLaserIndex(const int index)
{
	const double laserAngle = ((double) LASER_SEARCH_ANGLE / MAX_LASER_COUNT) * index;
	return point<double>::createPoint(laserpar[index], ANGLE(laserAngle));
}

static std::vector<std::vector<point<double>>*>* getUnknownLaserObjects(const int startIndex, const int endIndex)
{
	std::vector<std::vector<point<double>>*>* unknownObjects = new std::vector<std::vector<point<double>>*>;
	for (int i = startIndex; i < endIndex; ++i)
	{
		if (laserpar[i] > MIN_LASER_DISTANCE)
		{
			std::vector<point<double>> *objectPositions = new std::vector<point<double>>;

			objectPositions->push_back(getPointFromLaserIndex(i));

			int z;
			for (z = i + 1; z < endIndex; ++z)
			{
				if (laserpar[z] > MIN_LASER_DISTANCE)
				{
					const point<double> obstaclePos = getPointFromLaserIndex(z);
					if (objectPositions->back().distanceTo(obstaclePos) <= MAX_DISTANCE_FOR_CONNECTED_POINTS * objectPositions->back().length())
					{
						objectPositions->push_back(obstaclePos);
					}
					else
					{
						break;
					}
				}
			}
			i = z - 1;
			if (objectPositions->size() > 1)
			{
				unknownObjects->push_back(objectPositions);
			}
			else
			{
				delete objectPositions;
			}

		}
	}
	return unknownObjects;
}

static laserObjects* categorizeUnknownLaserObjects(const std::vector<std::vector<point<double>>*>& unknownObjects)
{
	laserObjects* objects = new laserObjects;
	for (unsigned int unknownObjectIndex = 0; unknownObjectIndex < unknownObjects.size(); ++unknownObjectIndex)
	{
		const std::vector<point<double>> unknownObject = *unknownObjects[unknownObjectIndex];

		//shitty solution but it works for straight walls
		const bool isPillar = unknownObject.front().distanceTo(unknownObject.back()) <= MAX_PILLAR_SIZE;

		if (isPillar)
		{
			pillar* newPillar = new pillar;

			point<double> pointsSum = { 0 };
			for (unsigned int i = 0; i < unknownObject.size(); ++i)
			{
				pointsSum += unknownObject[i];
			}
			newPillar->pos = pointsSum / unknownObject.size();

			point<double> nearestPos = unknownObject[0];
			for (unsigned int i = 1; i < unknownObject.size(); ++i)
			{
				if (nearestPos.length() > unknownObject[i].length())
				{
					nearestPos = unknownObject[i];
				}
			}
			newPillar->nearestPos = nearestPos;

			newPillar->points = new point<double> [unknownObject.size()];
			std::copy(unknownObject.begin(), unknownObject.end(), newPillar->points);
			newPillar->pointsCount = unknownObject.size();

			objects->pillars.push_back(newPillar);
		}
		else
		{
			wall* newWall = new wall;

			newWall->startPos = unknownObject.front();
			newWall->endPos = unknownObject.back();

			point<double> nearestPos = unknownObject[0];
			for (unsigned int i = 1; i < unknownObject.size(); ++i)
			{
				if (nearestPos.length() > unknownObject[i].length())
				{
					nearestPos = unknownObject[i];
				}
			}
			newWall->nearestPos = nearestPos;

			newWall->points = new point<double> [unknownObject.size()];
			std::copy(unknownObject.begin(), unknownObject.end(), newWall->points);
			newWall->pointsCount = unknownObject.size();

			objects->walls.push_back(newWall);
		}
		delete unknownObjects[unknownObjectIndex];
	}
	return objects;
}

laserObjects* getLaserObjects(const int startAngle, const int searchAngle)
{
	const int startIndex = 0;
	const int endIndex = MAX_LASER_COUNT;

	const std::vector<std::vector<point<double>>*>* unknownObjects = getUnknownLaserObjects(startIndex, endIndex);

	laserObjects* categorizedObjects = categorizeUnknownLaserObjects(*unknownObjects);
	delete unknownObjects;
	return categorizedObjects;
}

double getLaserDistance(enum LaserDistance l)
{
	return laserpar[l];
}
