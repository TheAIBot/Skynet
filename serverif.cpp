#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <math.h>
#include "includes/serverif.h"
#include "includes/robotconnector.h"
#include "includes/lasersensor.h"

void serverconnect(componentservertype *s)
{
	char buf[256];
	int len;
	s->serv_adr.sin_family = AF_INET;
	s->serv_adr.sin_port = htons(s->port);
	s->serv_adr.sin_addr.s_addr = inet_addr(s->host);
	printf("port %d host %s \n", s->port, s->host);
	if ((s->connected = (connect(s->sockfd, (struct sockaddr *) &s->serv_adr, sizeof(s->serv_adr))) > -1))
	{
		printf(" connected to %s  \n", s->name);
		len = sprintf(buf, "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
		send(s->sockfd, buf, len, 0);
		len = sprintf(buf, "mrc version=\"1.00\" >\n");
		send(s->sockfd, buf, len, 0);
		if (fcntl(s->sockfd, F_SETFL, O_NONBLOCK) == -1)
		{
			fprintf(stderr, "startserver: Unable to set flag O_NONBLOCK on %s fd \n", s->name);
		}
	}
	else
	{
		printf("Not connected to %s  %d \n", s->name, s->connected);
	}
}

void xml_proc(struct xml_in *x)
{
	while (1)
	{
		switch (xml_in_nibble(x)) {
		case XML_IN_NONE:
			return;
		case XML_IN_TAG_START:
#if (0)
			{
				int i;
				printf("start tag: %s, %d attributes\n", x->a, x->n);
				for (i = 0; i < x->n; i++)
				{
					printf("  %s    %s  \n", x->attr[i].name, x->attr[i].value);
				}
			}
#endif
			if (strcmp("gmk", x->a) == 0)
			{
				printf("  %s    %s  \n", x->attr[0].name, x->attr[0].value);
				double a;
				if (getdouble(&a, "id", x))
				{
					gmk.id = a;
					printf("id= %f\n", gmk.id);
				}
				if (getdouble(&a, "crcOK", x))
				{
					gmk.crc = a;
					printf("crc= %f\n", gmk.crc);
				}
			}
			else if (strcmp("pos3d", x->a) == 0)
			{
				getdouble(&gmk.x, "x", x);
				getdouble(&gmk.y, "y", x);
				getdouble(&gmk.z, "z", x);
			}
			else if (strcmp("rot3d", x->a) == 0)
			{
				getdouble(&gmk.omega, "Omega", x);
				getdouble(&gmk.phi, "Phi", x);
				getdouble(&gmk.kappa, "Kappa", x);
			}
			else if (strcmp("vision", x->a) == 0)
			{
				for (int i = 0; i < x->n; i++)
				{
					const int ix = atoi(x->attr[i].name + 3);
					if (ix > -1 && ix < 10)
					{
						visionpar[ix] = atof(x->attr[i].value);
					}
				}
			}

			break;
		case XML_IN_TAG_END:
			//printf("end tag: %s\n", x->a);
			break;
		case XML_IN_TEXT:
			//printf("text: %d bytes\n  \"", x->n);
			//fwrite(x->a, 1, x->n, stdout);
			//printf("\"\n");
			break;
		}
	}
}

void xml_proca(struct xml_in *x)
{
	while (1)
	{
		switch (xml_in_nibble(x)) {
		case XML_IN_NONE:
			return;
		case XML_IN_TAG_START:
#if (0)
			{
				printf("start tag: %s, %d attributes\n", x->a, x->n);
				for (int i = 0; i < x->n; i++)
				{
					printf("  %s    %s  \n", x->attr[i].name, x->attr[i].value);
				}
			}
#endif
			//check if xml name is the correct one
			if (strcmp("lval", x->a) == 0)
			{
				const int ANGLE_INDEX = 1;
				const int DISTANCE_INDEX = 2;
				//double check if the xml is correct by checking if one element has
				//the correct name
				if (strcmp(x->attr[DISTANCE_INDEX].name, "dist") == 0)
				{
					const double angle = atof(x->attr[ANGLE_INDEX].value);
					//calculate where the index the laser value should be saved at by converting the angle to an index
					const int laserIndex = (int) round(ANGLE_TO_INDEX(angle));
					//printf("%d\n", laserIndex);
					laserpar[laserIndex] = atof(x->attr[DISTANCE_INDEX].value);
					//printf("%f\n", laserpar[laserIndex]);
				}
			}
			break;
		case XML_IN_TAG_END:
			//printf("end tag: %s\n", x->a);
			break;
		case XML_IN_TEXT:
			//printf("text: %d bytes\n  \"", x->n);
			//fwrite(x->a, 1, x->n, stdout);
			//printf("\"\n");
			break;
		}
	}
}
