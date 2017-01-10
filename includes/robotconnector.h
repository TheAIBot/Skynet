#ifndef ROBOTCONNECTOR_H_
#define ROBOTCONNECTOR_H_

#include <sys/time.h>

#ifdef __cplusplus
extern "C" {
#endif

#include "rhd.h"
#include "componentserver.h"
#include "xmlio.h"

void xml_proc(struct xml_in *x);
void xml_proca(struct xml_in *x);
void serverconnect(componentservertype *s);

#ifdef __cplusplus
}
#endif

struct xml_in *xmldata;
struct xml_in *xmllaser;
struct
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
} gmk;

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
