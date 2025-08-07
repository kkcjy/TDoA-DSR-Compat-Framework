CC = gcc

SR_CFLAGS  = -Wall -g -I../../../../../vendor/libdw3000/include -I../../../../../vendor/FreeRTOS/include -I../Inc -DSIMULATION_ENABLE
DSR_CFLAGS = -Wall -g -I../../../../../vendor/libdw3000/include -I../../../../../vendor/FreeRTOS/include -I../Inc -DSIMULATION_ENABLE

FRAME_SRC = frame.h
CENTER_SRC = center.c
DRONE_SRC = drone.c

SR_SRC = ../Src/adhocuwb_swarm_ranging.c
DSR_SRC = ../Src/adhocuwb_dynamic_swarm_ranging.c

CENTER_OUT = center
DRONE_OUT = drone

all: $(CENTER_OUT) $(DRONE_OUT)

SWARM_MODE_DEFINED = $(shell grep 'define SWARM_RANGING_MODE' frame.h | grep '//' -v -c) 
DYNAMIC_MODE_DEFINED = $(shell grep 'define DYNAMIC_SWARM_RANGING_MODE' frame.h | grep '//' -v -c) 

ifeq ("$(SWARM_MODE_DEFINED)""$(DYNAMIC_MODE_DEFINED)","1 1 ")
$(error "Error: Both modes are defined! Please define only one mode")
else ifeq ("$(SWARM_MODE_DEFINED)","1 ")
$(info 'else ifeq ("(SWARM_MODE_DEFINED)","1 ")')
$(info 'else ifeq SWARM_MODE_DEFINED')
$(CENTER_OUT): $(CENTER_SRC) $(FRAME_SRC)
	$(CC) $(SR_CFLAGS) -o $@ $(CENTER_SRC)

$(DRONE_OUT): $(DRONE_SRC) $(FRAME_SRC) $(SR_SRC)
	$(CC) $(SR_CFLAGS) -o $@ $(DRONE_SRC) $(SR_SRC)
else ifeq ("$(DYNAMIC_MODE_DEFINED)","1 ")
$(info 'else ifeq DYNAMIC_MODE_DEFINED')
$(CENTER_OUT): $(CENTER_SRC) $(FRAME_SRC)
	$(CC) $(DSR_CFLAGS) -o $@ $(CENTER_SRC)

$(DRONE_OUT): $(DRONE_SRC) $(FRAME_SRC) $(DSR_SRC)
	$(CC) $(DSR_CFLAGS) -o $@ $(DRONE_SRC) $(DSR_SRC)
else
$(error "Error: No valid mode defined in $(FRAME_SRC)! Please define either DYNAMIC_SWARM_RANGING_MODE or SWARM_RANGING_MODE (not commented)")
endif

mode:
ifeq ($(SWARM_MODE_DEFINED),1)
	@echo "Current mode: SWARM_RANGING_MODE"
else ifeq ($(DYNAMIC_MODE_DEFINED),1)
	@echo "Current mode: DYNAMIC_SWARM_RANGING_MODE"
endif

clean:
	rm -f $(CENTER_OUT) $(DRONE_OUT)