/*
How to use sniffer.c
in linux 
0 Insert usb and crazyflie-sniffer
1 cd ~/sniffer
2 sudo rmmod cdc-acm
3 make
4 ./sniffer
5 Ctrl+c to exit
*/


#include <libusb-1.0/libusb.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <signal.h>
#include <time.h>


// #define SWARM_RANGING_MODE
#define DYNAMIC_SWARM_RANGING_MODE

#if defined(SWARM_RANGING_MODE)
#include "../Inc/adhocuwb_swarm_ranging.h"
#elif defined(DYNAMIC_SWARM_RANGING_MODE)
#include "../Inc/adhocuwb_dynamic_swarm_ranging.h"
#endif

#define VENDOR_ID  0x0483
#define PRODUCT_ID 0x5740
#define MAX_PACKET_SIZE 64
#define MAGIC_MATCH 0xBB88
#define OUTPUT_FILENAME_BUFFER_SIZE 32


char filename[OUTPUT_FILENAME_BUFFER_SIZE];
volatile sig_atomic_t keep_running = 1;


typedef union {
  uint8_t raw[18];
  struct {
    uint32_t magic;
    uint16_t senderAddress;
    uint16_t seqNumber;
    uint16_t msgLength;
    uint64_t rxTime;
  } __attribute__((packed));
} __attribute__((packed)) Sniffer_Meta_t;


void handle_sigint(int sig) {
    keep_running = 0;
}

void fprintRangingMessageCSV(FILE* fp, uint8_t* bin_buffer, size_t length) {
    if (length < sizeof(Ranging_Message_Header_t)) {
        fprintf(fp, "ERROR,数据长度不足,%zu < %zu\n", length, sizeof(Ranging_Message_Header_t));
        return;
    }

    Ranging_Message_Header_t header;
    memcpy(&header, bin_buffer, sizeof(Ranging_Message_Header_t));

    // 写入基础字段
    fprintf(fp, "%u,%u,", header.srcAddress, header.msgSequence);
    printf("srcAddress = %u,msgSequence = %u\n", header.srcAddress, header.msgSequence);

    // 写入每个时间戳字段（共 RANGING_MAX_Tr_UNIT 个）
    #if defined(SWARM_RANGING_MODE)
    for (int i = 0; i < RANGING_MAX_Tr_UNIT; i++) {
        // avoid an unaligned pointer value
        Timestamp_Tuple_t_2 ts = header.lastTxTimestamps[i];

        // rawtime（5 字节）写成十六进制
        for (int j = 0; j < 5; j++) {
            if(ts.rawtime[j]){
                fprintf(fp, "%02X", ts.rawtime[j]);
            }
        }

        // address 和 seqNumber
        fprintf(fp, ",%u,%u,", ts.address, ts.seqNumber);
    }

    #elif defined(DYNAMIC_SWARM_RANGING_MODE)
    uint8_t address = (uint8_t)(header.srcAddress & 0xFF);
    uint16_t seqNumber = header.msgSequence;

    /* get last Txtimestamp
        WARN: the ranging frequency of each drone should remain consistent.
    */
    dwTime_t timestamp = header.Txtimestamps[0].timestamp;

    // rawtime(hex)
    for(int j = 0; j < 5; j++) {
        if(timestamp.raw[j]) {
            fprintf(fp, "%02X", timestamp.raw[j]);
        }
    }

    fprintf(fp, ",%u,%u,", address, seqNumber);
    #endif

    // 写入消息长度、过滤器、位置坐标
    fprintf(fp, "%u,%u,",
            header.msgLength,
            header.filter
            );
}

void generate_output_filename(char *buffer, size_t buffer_size) {
    time_t rawtime;
    struct tm *timeinfo;

    time(&rawtime);
    timeinfo = localtime(&rawtime);

    strftime(buffer, buffer_size, "data/%Y-%m-%d-%H-%M-%S.csv", timeinfo);
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

    libusb_set_auto_detach_kernel_driver(dev_handle, 1);
    libusb_claim_interface(dev_handle, 0);
    generate_output_filename(filename, sizeof(filename));
    log_file = fopen(filename, "w");
    if (!log_file) {
        perror("open log file");
        return 1;
    }

    // 写 CSV 表头
    fprintf(log_file, "magic,sender_addr,seq_num,msg_len,sniffer_rx_time,"
                      "src_addr,msg_seq,");
                      
    #ifdef SWARM_RANGING_MODE
    for (int i = 0; i < RANGING_MAX_Tr_UNIT; i++) {
    #elif defined(DYNAMIC_SWARM_RANGING_MODE)
    for (int i = 0; i < MESSAGE_TX_POOL_SIZE; i++) {
    #endif
        fprintf(log_file, "ts%d_rawtime,ts%d_addr,ts%d_seq,", i, i, i);
    }
    fprintf(log_file, "msgLen,filter,posX,posY,posZ\n");

    while (keep_running) {
        r = libusb_bulk_transfer(dev_handle, endpoint_in, buffer, MAX_PACKET_SIZE, &transferred, 1000);


        if (r == 0 && transferred <= MAX_PACKET_SIZE) {
            // 解析数据
            Sniffer_Meta_t *meta = (Sniffer_Meta_t *)buffer;

            // Accessing fields
            uint32_t magic = meta->magic;
            uint16_t sender_addr = meta->senderAddress;
            uint16_t seq_num = meta->seqNumber;
            uint16_t msg_len = meta->msgLength;
            uint64_t sniffer_rx_time = meta->rxTime;
        
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
                    printf("magic=%u, sender_addr=%u, seq_num=%u, msg_len=%u, sniffer_rx_time=%lu\n",
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

    printf("✅ Logging finished. Data saved to %s\n", filename);
    return 0;
}