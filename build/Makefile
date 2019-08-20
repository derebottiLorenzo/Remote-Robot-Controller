PREFIX=../
CC=gcc
INCLUDE_DIRS=-I$(PREFIX)/src/common -I$(PREFIX)/src/orazio_host/ 
CC_OPTS=-Wall -Ofast -std=gnu99 $(INCLUDE_DIRS)

LIBS=-lpthread -lreadline -lwebsockets

LOBJS = packet_handler.o\
		deferred_packet_handler.o\
		orazio_client.o\
		orazio_print_packet.o\
		serial_linux.o\
		capture_camera_mod.o\
		
OBJS = rrc_ws.o\

HEADERS = packet_header.h\
		packet_operations.h\
		packet_handler.h\
		deferred_packet_handler.h\
		orazio_packets.h\
	  	orazio_print_packet.h\

BINS = rrc_client\
		rrc_host


.phony:	clean all

all:	$(BINS) 

#common objects
%.o:	$(PREFIX)/src/common/%.c 
	$(CC) $(CC_OPTS) -c  $<

#rrc 
%.o:	$(PREFIX)/src/orazio_rrc/%.c 
	$(CC) $(CC_OPTS) -c  $<

#host 
%.o:	$(PREFIX)/src/orazio_host/%.c 
	$(CC) $(CC_OPTS) -c  $<

rrc_client: rrc_client.o $(LOBJS)
	$(CC) $(CC_OPTS) -o $@ $^ $(LIBS) `pkg-config --cflags --libs opencv` 

rrc_host:  rrc_host.o $(LOBJS) $(OBJS)
	$(CC) $(CC_OPTS) -o $@ $^ $(LIBS) `pkg-config --cflags --libs opencv`

clean:
	rm -rf $(OBJS) $(BINS) *~ *.d *.o buf write