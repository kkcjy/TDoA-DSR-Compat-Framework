/*
 * adhocuwb_transceive.c
 *
 *  Created on: Jun 20, 2025
 *      Author: twinhorse
 */

static QueueHandle_t rxQueue;
void exampleTaskInit() {
  // init reception queue
  rxQueue = xQueueCreate(EXAMPLE_RX_QUEUE_SIZE, EXAMPLE_RX_QUEUE_ITEM_SIZE);

  // init listener
  UWB_Message_Listener_t listener;
  listener.type = CUSTOM_MESSAGE_TYPE;
  listener.rxQueue = rxQueue;
  listener.rxCb = exampleRxCallback;
  listener.txCb = exampleTxCallback;

  // register listener
  uwbRegisterListener(&listener);

  // ........
}

static void uwbExampleRxTask(void *parameters) {
  systemWaitStart();

  UWB_Packet_t rxPacketCache;

  while (true) {
    if (uwbReceivePacketBlock(CUSTOM_MESSAGE_TYPE, &rxPacketCache)) {
      // receive and process CUSTOM_MESSAGE_TYPE message.
    }
    // ........
  }
}
