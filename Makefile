#
# The Compiler
#
CC      = g++
LD      = ${CC}
SMR     = /usr/local/smr
CFLAGS  = -Wall -O2 -I${SMR}/include -Wno-write-strings
LDFLAGS = -L${SMR}/lib 

#
# Our program files
#
PROG   = t800
HDRS   =
OBJS   = t800.o serverif.o robotconnector.o odometry.o log.o linesensor.o irsensor.o commands.o
LIBS   = -lm librhd.a -lrobot

all:	${PROG}
	
%.o: %.cpp
	g++ ${CFLAGS} -c $<

${PROG}: ${OBJS}
	${LD} -o ${@} ${LDFLAGS} ${OBJS} ${LIBS}

clean:
	rm -f ${OBJS}

${OBJS}: ${HDRS} Makefile
