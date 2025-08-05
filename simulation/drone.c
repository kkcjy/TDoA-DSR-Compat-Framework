#include "frame.h"


const char* localAddress;
dwTime_t TxTimestamp;                           // store timestamp from flightLog
dwTime_t RxTimestamp;                           // store timestamp from flightLog


void sendToCenter(int center_socket, const char* address, const Ranging_Message_t *ranging_msg) {
    Simu_Message_t simu_message;

    if(sizeof(Ranging_Message_t) > PAYLOAD_SIZE) {
        perror("Warning: %ld > %ld, Ranging_Message_t too large!\n", sizeof(Ranging_Message_t), PAYLOAD_SIZE);
    }

    snprintf(simu_message.srcAddress, sizeof(simu_message.srcAddress), "%s", address);
    memcpy(simu_message.payload, ranging_msg, sizeof(Ranging_Message_t));
    simu_message.size = sizeof(Ranging_Message_t);

    if (send(center_socket, &simu_message, sizeof(simu_message), 0) < 0) {
        perror("Send failed");
    }
}

void *receive_from_center(void *arg) {
    int center_socket = *(int*)arg;
    Simu_Message_t simu_message;

    while(true) {
        ssize_t bytes_received = recv(center_socket, &simu_message, sizeof(simu_message), 0);

        if(bytes_received <= 0) {
            printf("Disconnected from Control Center\n");
            break;
        }

        if (strcmp(simu_message.data, REJECT_INFO) == 0) {
            printf("Connection rejected: Maximum drones reached (%d)\n", NODES_NUM);
            exit(1);
        }

        // ignore the message from itself
        if(strcmp(simu_message.srcAddress, localAddress) != 0) {
            // handle message of flightLog
            if(simu_message.size == sizeof(Line_Message_t)) {
                Line_Message_t *line_message = (Line_Message_t*)simu_message.payload;
                if(strcmp(line_message->address, localAddress) == 0) {
                    // sender
                    if(line_message->status == TX) {
                        TxTimestamp = line_message->timestamp;
                        TxCallBack(TxTimestamp);
                        // reset TxTimestamp after callback
                        TxTimestamp.full = NULL_TIMESTAMP;
                    }
                    // receiver
                    else if(line_message->status == RX) {
                        RxTimestamp = line_message->timestamp;
                    }
                }
            }

            // handle message of rangingMessage
            else if(simu_message.size == sizeof(Ranging_Message_t)) {
                // wait for flightLog
                while(RxTimestamp == NULL_TIMESTAMP);

                Ranging_Message_t *ranging_msg = (Ranging_Message_t*)simu_message.payload;
                RxCallBack(ranging_msg, RxTimestamp);
                RxTimestamp.full = NULL_TIMESTAMP;
            }
            else {
                printf("Received unknown message size: %zu\n", simu_message.size);
                return NULL;
            }
        }
    }
    return NULL;
}

int main(int argc, char *argv[]) {
    if (argc < 3) {
        printf("Usage: %s <center_ip> <localAddress>\n", argv[0]);
        return 1;
    }

    const char *center_ip = argv[1];
    localAddress = argv[2];

    TxTimestamp.full = NULL_TIMESTAMP;
    RxTimestamp.full = NULL_TIMESTAMP;

    rangingTableSetInit();

    int center_socket = socket(AF_INET, SOCK_STREAM, 0);
    if (center_socket < 0) {
        perror("Socket creation error");
        return -1;
    }

    struct sockaddr_in serv_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(CENTER_PORT)
    };
    
    if (inet_pton(AF_INET, center_ip, &serv_addr.sin_addr) <= 0) {
        perror("Invalid address");
        close(center_socket); 
        return -1;
    }

    if (connect(center_socket, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
        perror("Connection Failed");
        close(center_socket); 
        return -1;
    }

    // Send drone ID first
    send(center_socket, localAddress, strlen(localAddress), 0);

    // Receive thread
    pthread_t receive_thread;
    if (pthread_create(&receive_thread, NULL, receive_from_center, &center_socket) != 0) {
        perror("Failed to create receive thread");
        close(center_socket);
        return -1;
    }

    printf("Node %s connected to center\n", localAddress);

    pthread_join(receive_thread, NULL);
    close(center_socket);
    return 0;
}