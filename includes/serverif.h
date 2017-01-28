/*
 * serverif.h
 *
 *  Created on: Jan 10, 2017
 *      Author: smr, lord of all
 */

#ifndef SERVERIF_H_
#define SERVERIF_H_

#include "robotconnector.h"

void serverconnect(componentservertype* const s);
void decodeCameraXML(struct xml_in* const x);
void decodeLaserXML(struct xml_in* const x);


#endif /* SERVERIF_H_ */
