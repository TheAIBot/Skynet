#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include "includes/robotconnector.h"
#include "includes/serverif.h"

#define ROBOTPORT 24902

int laserZoneCount = 2;

struct xml_in *xmldata;
struct xml_in *xmllaser;

laserData gmk;

double visionpar[10];
double laserpar[MAX_LASER_COUNT];

componentservertype lmssrv;
componentservertype camsrv;

symTableElement *inputtable;
symTableElement *outputtable;
symTableElement *lenc;
symTableElement *renc;
symTableElement *linesensor;
symTableElement *irsensor;
symTableElement *speedl;
symTableElement *speedr;
symTableElement *resetmotorr;
symTableElement *resetmotorl;

symTableElement * getinputref(const char *sym_name, symTableElement * tab)
{
	int i;
	for (i = 0; i < getSymbolTableSize('r'); i++)
		if (strcmp(tab[i].name, sym_name) == 0)
			return &tab[i];
	return 0;
}

symTableElement * getoutputref(const char *sym_name, symTableElement * tab)
{
	int i;
	for (i = 0; i < getSymbolTableSize('w'); i++)
	{
		if (strcmp(tab[i].name, sym_name) == 0)
		{
			return &tab[i];
		}
	}
	return 0;
}

static void connectToCamera()
{
	camsrv.port = 24920;
	strcpy(camsrv.host, "127.0.0.1");
	strcpy(camsrv.name, "cameraserver");
	camsrv.status = 1;
	camsrv.config = 1;

	if (camsrv.config)
	{
		int errno = 0;
		camsrv.sockfd = socket(AF_INET, SOCK_STREAM, 0);
		if (camsrv.sockfd < 0)
		{
			perror(strerror(errno));
			fprintf(stderr, " Can not make  socket\n");
			return;
		}
		serverconnect(&camsrv);

		xmldata = xml_in_init(4096, 32);
		printf(" camera server xml initialized \n");
	}
}

void setLaserZoneCount(const int zoneCount)
{
	if (lmssrv.connected)
	{
		laserZoneCount = zoneCount;
		char buf[256];
		const int len = sprintf(buf, "scanpush cmd='scanget interval=%d codec=TAG'\n", (MAX_LASER_COUNT / laserZoneCount));
		send(lmssrv.sockfd, buf, len, 0);
	}
	else
	{
		printf("Failed to set laser zone count\n");
	}
}

static void connectToLaser()
{
	lmssrv.port = 24919;
	strcpy(lmssrv.host, "127.0.0.1");
	strcpy(lmssrv.name, "laserserver");
	lmssrv.status = 1;
	lmssrv.config = 1;

	if (lmssrv.config)
	{
		int errno = 0;
		lmssrv.sockfd = socket(AF_INET, SOCK_STREAM, 0);
		if (lmssrv.sockfd < 0)
		{
			perror(strerror(errno));
			fprintf(stderr, " Can not make  socket\n");
			return;
		}

		serverconnect(&lmssrv);
		if (lmssrv.connected)
		{
			xmllaser = xml_in_init(4096, 32);
			printf(" laserserver xml initialized \n");
			setLaserZoneCount(laserZoneCount);
		}
	}
}

bool connectRobot()
{
	/* Establish connection to robot sensors and actuators.
	 */
	if (rhdConnect('w', "localhost", ROBOTPORT) != 'w')
	{
		printf("Can't connect to rhd \n");
		return false;
	}

	printf("connected to robot \n");
	if ((inputtable = getSymbolTable('r')) == NULL)
	{
		printf("Can't connect to rhd \n");
		return false;
	}
	if ((outputtable = getSymbolTable('w')) == NULL)
	{
		printf("Can't connect to rhd \n");
		return false;
	}
	// connect to robot I/O variables
	lenc = getinputref("encl", inputtable);
	renc = getinputref("encr", inputtable);
	linesensor = getinputref("linesensor", inputtable);
	irsensor = getinputref("irsensor", inputtable);

	speedl = getoutputref("speedl", outputtable);
	speedr = getoutputref("speedr", outputtable);
	resetmotorr = getoutputref("resetmotorr", outputtable);
	resetmotorl = getoutputref("resetmotorl", outputtable);

	connectToCamera();
	connectToLaser();

	return 1;
}
