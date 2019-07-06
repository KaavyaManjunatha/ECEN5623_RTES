

all: posix_message_queue heap_message_queue

posix_message_queue:posix_message_queue.c
	gcc -pthread -o posix_message_queue posix_message_queue.c -lrt

heap_message_queue: heap_message_queue.c
	gcc -pthread -o heap_message_queue heap_message_queue.c -lrt
	
