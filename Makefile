#
# The Compiler
#
CC      = gcc
LD      = ${CC}
SMR     = /usr/local/smr
CFLAGS  = -Wall -O2 -I${SMR}/include
LDFLAGS = -L${SMR}/lib 

#
# Our program files
#
PROG   = square
HDRS   =
OBJS   = square.o serverif.o robotconnector.o odometry.o log.o
LIBS   = -lm librhd.a -lrobot

all:	${PROG}

#${PROG}: ${OBJS}
#	${LD} -o ${@} ${LDFLAGS} ${OBJS} ${LIBS}

#gcc ${CFLAFS} -c -std=c99 -o square.o square.c


#gcc -c -std=c99 -o file1.o file1.c
#g++ -c -std=c++0x -o file2.o file2.cpp
#g++ -o myapp file1.o file2.o

clean:
	rm -f ${OBJS}

${OBJS}: ${HDRS} Makefile
