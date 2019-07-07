CC=gcc

all:  deadlock_fix pthread3

deadlock_fix: deadlock_fix.c
	$(CC)  -pthread -o deadlock_fix deadlock_fix.c

pthread3: pthread3.c
	$(CC)  -pthread -o pthread3 pthread3.c

clean:
	rm -f deadlock_fix


