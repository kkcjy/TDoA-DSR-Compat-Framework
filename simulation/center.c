#include "frame.h"


Drone_Node_Set_t *droneNodeSet;
long file_pos = 0;


void droneNodeSetInit() {
    droneNodeSet = (Drone_Node_Set_t*)malloc(sizeof(Drone_Node_Set_t));
    memset(droneNodeSet->node, 0, sizeof(droneNodeSet->node));

    droneNodeSet->count = 0;

    if(pthread_mutex_init(&droneNodeSet->mutex, NULL) != 0) {
        perror("Mutex init failed");
        free(droneNodeSet);
        droneNodeSet = NULL;
        exit(1);
    }
}

void broadcastRangingMessage(Simu_Message_t *message) {
    pthread_mutex_lock(&droneNodeSet->mutex);
    for (int i = 0; i < droneNodeSet->count; i++) {
        if (send(droneNodeSet->node[i].socket, message, sizeof(Simu_Message_t), 0) < 0) {
            perror("Failed to broadcast message");
        }
    }
    pthread_mutex_unlock(&droneNodeSet->mutex);
}

void broadcastFlightLog(void *arg) {
    
    return NULL;
}

void *handleNodeConnection(void *arg) {
    int node_socket = *(int*)arg;
    free(arg);

    char node_address[ADDR_SIZE];
    ssize_t bytes_received = recv(node_socket, node_address, sizeof(node_address), 0);
    if (bytes_received <= 0) {
        close(node_socket);
        return NULL;
    }
    node_address[bytes_received] = '\0';

    pthread_mutex_lock(&droneNodeSet->mutex);
    if (droneNodeSet->count < NODES_NUM) {
        droneNodeSet->node[droneNodeSet->count].socket = node_socket;
        strncpy(droneNodeSet->node[droneNodeSet->count].address, node_address, sizeof(droneNodeSet->node[droneNodeSet->count].address));
        printf("New drone connected: %s\n", droneNodeSet->node[droneNodeSet->count].address);
        droneNodeSet->count++;
    }
    else {
        printf("Max nodes reached (%d), rejecting %s\n", NODES_NUM, node_address);
        Simu_Message_t reject_msg;
        strcpy(reject_msg.payload, REJECT_INFO);
        reject_msg.size = strlen(reject_msg.payload);
        send(node_socket, &reject_msg, sizeof(reject_msg), 0);
        close(node_socket);
        pthread_mutex_unlock(&droneNodeSet->mutex);
        return NULL;
    }
    pthread_mutex_unlock(&droneNodeSet->mutex);

    // Broadcast received message to all nodes
    Simu_Message_t message;
    while ((bytes_received = recv(node_socket, &message, sizeof(message), 0)) > 0) {
        Ranging_Message_t *ranging_msg = (Ranging_Message_t*)message;
        printf("[Broadcast]: address = %d, msgSeq = %d\n", ranging_msg->header.srcAddress, ranging_msg->header.msgSequence);
        broadcastRangingMessage(&message);
    }

    pthread_mutex_lock(&droneNodeSet->mutex);
    for (int i = 0; i < droneNodeSet->count; i++) {
        // Find the node that matches the disconnected socket
        if (droneNodeSet->node[i].socket == node_socket) {
            printf("Node %s disconnected\n", droneNodeSet->node[i].address);
            for(int j = i + 1; j < droneNodeSet->count; j++) {
                droneNodeSet->node[j - 1] = droneNodeSet->node[j];
            }
            // Clear the last node
            memset(&droneNodeSet->node[droneNodeSet->count - 1], 0, sizeof(Drone_Node_t));
            droneNodeSet->count--;
            break;
        }
    }
    pthread_mutex_unlock(&droneNodeSet->mutex);

    close(node_socket);
    return NULL;
}

int main() {
    droneNodeSetInit();

    int server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd < 0) {
        perror("socket failed");
        exit(EXIT_FAILURE);
    }

    int opt = 1;
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
        perror("setsockopt");
        exit(EXIT_FAILURE);
    }

    struct sockaddr_in address = {
        .sin_family = AF_INET,
        .sin_addr.s_addr = INADDR_ANY,
        .sin_port = htons(CENTER_PORT)
    };

    if (bind(server_fd, (struct sockaddr*)&address, sizeof(address)) < 0) {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }

    if (listen(server_fd, NODES_NUM) < 0) {
        perror("listen");
        exit(EXIT_FAILURE);
    }

    printf("Control Center started on port %d (Max drones: %d)\n", CENTER_PORT, NODES_NUM);
    printf("Waiting for drone connections...\n");

    while (1) {
        struct sockaddr_in client_addr;
        socklen_t addrlen = sizeof(client_addr);
        int *new_socket = malloc(sizeof(int));
        if (!new_socket) {
            perror("malloc failed");
            continue;
        }

        *new_socket = accept(server_fd, (struct sockaddr*)&client_addr, &addrlen);
        if (*new_socket < 0) {
            perror("accept");
            free(new_socket);
            continue;
        }

        pthread_t thread_id;
        if (pthread_create(&thread_id, NULL, handleNodeConnection, new_socket) != 0) {
            perror("pthread_create");
            close(*new_socket);
            free(new_socket);
        }
        pthread_detach(thread_id);

        // make sure all drones connected before broadcastFlightLog
        pthread_mutex_lock(&droneNodeSet->mutex);
        if (droneNodeSet->count == NODES_NUM) {
            pthread_mutex_unlock(&droneNodeSet->mutex);
            break;
        }
        pthread_mutex_unlock(&droneNodeSet->mutex);
    }

    // broadcastFlightLog
    pthread_t broadcast_thread;
    pthread_create(&broadcast_thread, NULL, broadcastFlightLog, NULL);

    pthread_detach(broadcast_thread);
    pause();
    
    close(server_fd);
    return 0;
}