CC=gcc

all: deadlock deadlock_fix 

deadlock: deadlock.c 
	$(CC) -pthread -o deadlock deadlock.c

deadlock_fix: deadlock_fix.c
	$(CC)  -pthread -o deadlock_fix deadlock_fix.c

clean:
	rm -f deadlock
	rm -f deadlock_fix
