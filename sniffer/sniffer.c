#include <libusb-1.0/libusb.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <signal.h>
#include <time.h>

// #include "adhocuwb_swarm_ranging.h"
// #include "adhocuwb_sniffer.h"

#define VENDOR_ID  0x0483
#define PRODUCT_ID 0x5740
#define MAX_PACKET_SIZE 64
#define MAGIC_MATCH 0xBB88
#define OUTPUT_FILENAME "data/log_data.csv"

volatile sig_atomic_t keep_running = 1;

void handle_sigint(int sig) {
    keep_running = 0;
}


//swarm ranging struct
typedef struct {
    uint32_t magic;
    uint16_t sender_addr;
    uint16_t seq_num;
    uint32_t msg_len;
    uint64_t sniffer_rx_time;
    uint8_t bin_data[256]; // 可动态分配，这里示例为固定最大值
} LogEntry;


#define RANGING_MAX_Tr_UNIT 5

typedef union {
    struct {
        uint8_t rawtime[5];   // 5 bytes
        uint8_t address;      // 1 byte
        uint16_t seqNumber;   // 2 bytes
    } __attribute__((packed));
    uint8_t bytes[8]; // 8 bytes total
} Timestamp_Tuple_t_2;

typedef struct {
    uint16_t srcAddress;
    uint16_t msgSequence;
    Timestamp_Tuple_t_2 lastTxTimestamps[RANGING_MAX_Tr_UNIT]; // 8 * 5 = 40 bytes
    uint16_t msgLength;
    uint16_t filter;
    float posiX;
    float posiY;
    float posiZ;
} __attribute__((packed)) Ranging_Message_Header_t;

void fprintRangingMessageCSV(FILE* fp, uint8_t* bin_buffer, size_t length) {
    if (length < sizeof(Ranging_Message_Header_t)) {
        fprintf(fp, "ERROR,数据长度不足,%zu < %zu\n", length, sizeof(Ranging_Message_Header_t));
        return;
    }

    Ranging_Message_Header_t header;
    memcpy(&header, bin_buffer, sizeof(Ranging_Message_Header_t));

    // 写入基础字段
    fprintf(fp, "%u,%u,", header.srcAddress, header.msgSequence);



    // 写入每个时间戳字段（共 RANGING_MAX_Tr_UNIT 个）
    for (int i = 0; i < RANGING_MAX_Tr_UNIT; i++) {
        Timestamp_Tuple_t_2* ts = &header.lastTxTimestamps[i];

        // rawtime（5 字节）写成十六进制
        for (int j = 0; j < 5; j++) {
            if(ts->rawtime[j]){
                fprintf(fp, "%02X", ts->rawtime[j]);
            }
        }

        // address 和 seqNumber
        fprintf(fp, ",%u,%u,", ts->address, ts->seqNumber);
    }

    // 写入消息长度、过滤器、位置坐标
    fprintf(fp, "%u,%u,%.3f,%.3f,%.3f\n",
            header.msgLength,
            header.filter,
            header.posiX,
            header.posiY,
            header.posiZ);
}


int main() {
    libusb_device_handle *dev_handle = NULL;
    libusb_context *ctx = NULL;
    int r;
    uint8_t endpoint_in = 0x81; 
    int transferred;
    uint8_t buffer[MAX_PACKET_SIZE];
    FILE *log_file;

    signal(SIGINT, handle_sigint);

    r = libusb_init(&ctx);
    if (r < 0) {
        fprintf(stderr, "libusb init error\n");
        return 1;
    }

    dev_handle = libusb_open_device_with_vid_pid(ctx, VENDOR_ID, PRODUCT_ID);
    if (!dev_handle) {
        fprintf(stderr, "cannot find USB device\n");
        return 1;
    }

    printf("Begin trans");

    libusb_set_auto_detach_kernel_driver(dev_handle, 1);
    libusb_claim_interface(dev_handle, 0);

    log_file = fopen(OUTPUT_FILENAME, "w");
    if (!log_file) {
        perror("open log file");
        return 1;
    }

    // 写 CSV 表头
    fprintf(log_file, "magic,sender_addr,seq_num,msg_len,sniffer_rx_time,"
                      "src_addr,msg_seq,");
    for (int i = 0; i < RANGING_MAX_Tr_UNIT; i++) {
        fprintf(log_file, "ts%d_rawtime,ts%d_addr,ts%d_seq,", i, i, i);
    }
    fprintf(log_file, "msgLen,filter,posX,posY,posZ\n");

    while (keep_running) {
        r = libusb_bulk_transfer(dev_handle, endpoint_in, buffer, MAX_PACKET_SIZE, &transferred, 1000);


        if (r == 0 && transferred <= MAX_PACKET_SIZE) {
            // 解析数据
            uint32_t magic = *(uint32_t*)(buffer);
            uint16_t sender_addr = *(uint16_t*)(buffer + 4);
            uint16_t seq_num = *(uint16_t*)(buffer + 6);
            uint16_t msg_len = *(uint16_t*)(buffer + 8);
            uint64_t sniffer_rx_time = *(uint64_t*)(buffer + 10);
        
            // printf( "magic = 0x%x , msg_len = %d\n",magic ,msg_len);
            
            if (magic == MAGIC_MATCH && msg_len <= 256) {
                uint8_t bin_buffer[256] = {0};

                // 读取后续 binary 数据
                r = libusb_bulk_transfer(dev_handle, endpoint_in, bin_buffer, msg_len, &transferred, 1000);
                // printf( "r = %d,transferred =%d \n",r,transferred);
                if (r == 0 && transferred == msg_len) {
                    // 写一行到 CSV
                    fprintf(log_file, "%u,%u,%u,%u,%lu,",
                            magic, sender_addr, seq_num, msg_len, sniffer_rx_time);
                    fprintRangingMessageCSV(log_file, bin_buffer, msg_len);

                    fprintf(log_file, "\n");
                }
            }
        }
    }

    fclose(log_file);
    libusb_release_interface(dev_handle, 0);
    libusb_close(dev_handle);
    libusb_exit(ctx);

    printf("✅ Logging finished. Data saved to %s\n", OUTPUT_FILENAME);
    return 0;
}
