/*
 * robotconnector.h
 *
 *  Created on: Jan 4, 2017
 *      Author: smr
 */

#ifndef ROBOTCONNECTOR_H_
#define ROBOTCONNECTOR_H_

#include <sys/time.h>
#include "rhd.h"
#include "componentserver.h"
#include "xmlio.h"

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

void xml_proc(struct xml_in *x);
void xml_proca(struct xml_in *x);

int connectRobot(void);

#endif /* ROBOTCONNECTOR_H_ */
