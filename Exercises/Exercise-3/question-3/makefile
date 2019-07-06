CC=gcc

all:  deadlock_fix 

deadlock_fix: deadlock_fix.c
	$(CC)  -pthread -o deadlock_fix deadlock_fix.c

clean:
	rm -f deadlock_fix
