CC = gcc

SR_CFLAGS  = -Wall -g -I../../../../../vendor/libdw3000/include -I../../../../../vendor/FreeRTOS/include -I../Inc -I../platform/Inc -DSIMULATION_COMPILE
DSR_CFLAGS = -Wall -g -I../Inc -I../../../../../vendor/libdw3000/include -DSIMULATION_COMPILE -Wno-format -Wno-unused-variable -I...

FRAME_SRC = frame.h
CENTER_SRC = center.c
DRONE_SRC = drone.c

SR_SRC = ../Src/adhocuwb_swarm_ranging.c
DSR_SRC = ../Src/adhocuwb_dynamic_swarm_ranging.c support.c

CENTER_OUT = center
DRONE_OUT = drone

all: $(CENTER_OUT) $(DRONE_OUT)

SWARM_MODE_DEFINED = $(shell grep -v '^[[:space:]]*//' $(FRAME_SRC) | grep -v '^[[:space:]]*$$' | grep -q '^[[:space:]]*#define[[:space:]]*SWARM_RANGING_MODE[[:space:]]*$$' && echo 1)
DYNAMIC_MODE_DEFINED = $(shell grep -v '^[[:space:]]*//' $(FRAME_SRC) | grep -v '^[[:space:]]*$$' | grep -q '^[[:space:]]*#define[[:space:]]*DYNAMIC_SWARM_RANGING_MODE[[:space:]]*$$' && echo 1)

ifeq ($(SWARM_MODE_DEFINED)$(DYNAMIC_MODE_DEFINED),11)
$(error "Error: Both modes are defined! Please define only one mode")
else ifneq ($(SWARM_MODE_DEFINED),)
$(CENTER_OUT): $(CENTER_SRC) $(FRAME_SRC)
	$(CC) $(SR_CFLAGS) -o $@ $(CENTER_SRC)

$(DRONE_OUT): $(DRONE_SRC) $(FRAME_SRC) $(SR_SRC)
	$(CC) $(SR_CFLAGS) -o $@ $(DRONE_SRC) $(SR_SRC)
else ifneq ($(DYNAMIC_MODE_DEFINED),)
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