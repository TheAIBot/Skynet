#
# The Compiler
#
CC      = g++
LD      = ${CC}
SMR     = /usr/local/smr
CFLAGS  = -Wall -O2 -I${SMR}/include -Wno-write-strings -ffast-math
LDFLAGS = -L${SMR}/lib 

#
# Our program files
#
PROG   = t800
HDRS   =
OBJS   = t800.o serverif.o robotconnector.o odometry.o log.o linesensor.o irsensor.o commands.o lasersensor.o
LIBS   = -lm librhd.a -lrobot

all:	${PROG}
	
%.o: %.cpp
	g++ -std=c++11 -flto ${CFLAGS} -c $<

${PROG}: ${OBJS}
	${LD} -std=c++11 -o ${@} ${LDFLAGS} ${OBJS} ${LIBS} -flto -ffunction-sections -Wl,--gc-sections -fno-asynchronous-unwind-tables -Wl,--strip-all

clean:
	rm -f ${OBJS}

${OBJS}: ${HDRS} Makefile
