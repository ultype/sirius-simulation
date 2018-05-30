BIN_IMAGE1 = simgen_udp_server
BIN_IMAGE2 = simgen_tcp_server
###### C flags #####
CC = gcc
CFLAGS = -Wall -g


##### C Source #####


##### OBJECTS #####

all: $(BIN_IMAGE1) $(BIN_IMAGE2)

deps := $(OBJECTS:%.o=%.o.d)

$(BIN_IMAGE1): simgen_udp_server.c
	$(CC) -Wall -g $^ -o $@ $(CFLAGS)
$(BIN_IMAGE2): simgen_tcp_server.c
	$(CC) -Wall -g $^ -o $@ $(CFLAGS)
.PHONY : clean
clean:
	rm -f $(BIN_IMAGE1) $(BIN_IMAGE2)
	find . -name "*.o" -type f -delete
	find . -name "*.d" -type f -delete