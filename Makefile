#
# The Compiler
#
CC      = g++
LD      = ${CC}
SMR     = /usr/local/smr
CFLAGS  = -Wall -O2 -I${SMR}/include
LDFLAGS = -L${SMR}/lib 

#
# Our program files
#
PROG   = t800
HDRS   =
OBJS   = t800.o serverif.o robotconnector.o odometry.o log.o linesensor.o irsensor.o commands.o
LIBS   = -lm librhd.a -lrobot

all:	${PROG}

%.o: %.c
	gcc ${CFLAGS} -o $@ -c $<
	
%.o: %.cpp
	g++ ${CFLAGS} -o $@ -c $<
	

#robotconnector.o: robotconnector.c
#	gcc ${CFLAGS} -o robotconnector.o -c robotconnector.c
#t800.o: t800.c
#	gcc ${CFLAGS} -o t800.o -c t800.c
#serverif.o: serverif.c
#	gcc ${CFLAGS} -o serverif.o -c serverif.c
#odometry.o: odometry.c
#	gcc ${CFLAGS} -o odometry.o -c odometry.c
#log.o: log.c
#	gcc ${CFLAGS} -o log.o -c log.c
#linesensor.o: linesensor.c
#	gcc ${CFLAGS} -o linesensor.o -c linesensor.c
#irsensor.o: irsensor.c
#	gcc ${CFLAGS} -o irsensor.o -c irsensor.c
#commands.o: commands.c
#	gcc ${CFLAGS} -o commands.o -c commands.c

${PROG}: ${OBJS}
	${LD} -o ${@} ${LDFLAGS} ${OBJS} ${LIBS}

clean:
	rm -f ${OBJS}

${OBJS}: ${HDRS} Makefile
