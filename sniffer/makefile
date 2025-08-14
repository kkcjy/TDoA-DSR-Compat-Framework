CC = gcc

# for linux
CFLAGS = -I/usr/include/libusb-1.0 -I../../../../../vendor/libdw3000/include -I../Inc -DSNIFFER_COMPILE
# for macOS
# CFLAGS = -I/opt/homebrew/include -I../../../../../vendor/libdw3000/include -I../Inc -DSNIFFER_COMPILE
# for windows


# for linux
LDFLAGS = -lusb-1.0
# for macOS
# LDFLAGS = -L/opt/homebrew/lib -lusb-1.0
# for windows


SRC = sniffer.c
TARGET = sniffer

all: $(TARGET)

$(TARGET): $(SRC)
	$(CC) $(CFLAGS) -o $(TARGET) $(SRC) $(LDFLAGS)

clean:
	rm -f $(TARGET)
