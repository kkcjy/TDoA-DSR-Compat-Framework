CC = gcc
CFLAGS = -Wall -g -I../../../../../vendor/libdw3000/include -I../../../../../vendor/FreeRTOS/include -I../Inc -I../Src -DSIMULATION_ENABLE

FRAME_SRC = frame.h
CENTER_SRC = center.c
DRONE_SRC = drone.c
RANGING_SRC = ../Src/adhocuwb_dynamic_swarm_ranging.c

CENTER_OUT = center
DRONE_OUT = drone

all: $(CENTER_OUT) $(DRONE_OUT)

$(CENTER_OUT): $(CENTER_SRC) $(FRAME_SRC)
	$(CC) $(CFLAGS) -o $@ $(CENTER_SRC)

$(DRONE_OUT): $(DRONE_SRC) $(FRAME_SRC) $(RANGING_SRC)
	$(CC) $(CFLAGS) -o $@ $(DRONE_SRC) $(RANGING_SRC)

clean:
	rm -f $(CENTER_OUT) $(DRONE_OUT)