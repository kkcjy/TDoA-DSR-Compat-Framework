# 编译器
CC = gcc

# for linux
CFLAGS = -I/usr/include/libusb-1.0 -I../../../../../vendor/libdw3000/include

# 链接库
LDFLAGS = -lusb-1.0

# 源文件和目标文件
SRC = sniffer.c
TARGET = sniffer

# 默认目标
all: $(TARGET)

# 构建目标程序
$(TARGET): $(SRC)
	$(CC) $(CFLAGS) -o $(TARGET) $(SRC) $(LDFLAGS)

# 清理生成文件
clean:
	rm -f $(TARGET)
