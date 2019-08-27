INCLUDE_DIRS = 
LIB_DIRS = 
CC=g++

CDEFS=
CFLAGS= -ggdb -O0 $(INCLUDE_DIRS) $(CDEFS)
LIBS= 

HFILES= 
CFILES= project.cpp 

SRCS= ${HFILES} ${CFILES}
OBJS= ${CFILES:.c=.o}

all:	project 

clean:
	-rm -f *.o *.d
	-rm -f project

project: project.cpp 
	g++ -g project.cpp -o project -lpthread -lrt `pkg-config --cflags --libs opencv`

clean_images:
	 rm -f transformed\ frames/*.pgm
	rm -f grabbed\ frames/*.ppm

#project: project.o
#	$(CC) $(LDFLAGS) $(CFLAGS) -o $@ $@.o  -lpthread -lrt `pkg-config --cflags --libs opencv`
depend:

.c.o:
	$(CC) $(CFLAGS) -c $<
