/*
 * serverif.h
 *
 *  Created on: Jan 10, 2017
 *      Author: smr
 */

#ifndef SERVERIF_H_
#define SERVERIF_H_

#include "robotconnector.h"

void serverconnect(componentservertype *s);
void xml_proc(struct xml_in *x);
void xml_proca(struct xml_in *x);

#endif /* SERVERIF_H_ */