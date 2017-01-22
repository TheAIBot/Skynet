#ifndef ROBOTCONNECTOR_H_
#define ROBOTCONNECTOR_H_

#include <sys/time.h>

#ifdef __cplusplus
extern "C" {
#endif

#include "rhd.h"
#include "componentserver.h"
#include "xmlio.h"

#ifdef __cplusplus
}
#endif

#define MAX_LASER_COUNT 500
#define LASER_SEARCH_ANGLE 180
#define MIN_LASER_DISTANCE 0.02

typedef struct
{
	double x;
	double y;
	double z;
	double omega;
	double phi;
	double kappa;
	double code;
	double id;
	double crc;
} laserData;

extern laserData gmk;


extern double visionpar[10];
extern double laserpar[MAX_LASER_COUNT];

extern symTableElement *inputtable;
extern symTableElement *outputtable;
extern symTableElement *lenc;
extern symTableElement *renc;
extern symTableElement *linesensor;
extern symTableElement *irsensor;
extern symTableElement *speedl;
extern symTableElement *speedr;
extern symTableElement *resetmotorr;
extern symTableElement *resetmotorl;

void setLaserZoneCount(const int zoneCount);
bool connectRobot(void);
void updateCameraData();
void updateLaserData();

#endif /* ROBOTCONNECTOR_H_ */
