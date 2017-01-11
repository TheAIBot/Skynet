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

extern struct xml_in *xmldata;
extern struct xml_in *xmllaser;

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
} roboConnectData;

extern roboConnectData gmk;


extern double visionpar[10];
extern double laserpar[10];

extern componentservertype lmssrv;
extern componentservertype camsrv;

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

int connectRobot(void);

#endif /* ROBOTCONNECTOR_H_ */
