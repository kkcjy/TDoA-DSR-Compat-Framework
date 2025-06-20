/*
 * adhocuwb_transceive.c
 *
 *  Created on: Jun 20, 2025
 *      Author: twinhorse
 */

static QueueHandle_t rxQueue;

/* todo
 * function transceiveRxCallback, refer to simpleTxCallback
 * function transceiveTxCallback, refer to simpleRxCallback
 * 修改确定EXAMPLE_RX_QUEUE_SIZE, EXAMPLE_RX_QUEUE_ITEM_SIZE, 参照swarm ranging中的xQueueCreate
 *
 */

void transceiveTaskInit() {
  // init reception queue
  rxQueue = xQueueCreate(EXAMPLE_RX_QUEUE_SIZE, EXAMPLE_RX_QUEUE_ITEM_SIZE);

  // init listener
  UWB_Message_Listener_t listener;
  listener.type = UWB_TRANSCEIVE_MESSAGE;
  listener.rxQueue = rxQueue;
  listener.rxCb = transceiveRxCallback;
  listener.txCb = transceiveTxCallback;

  // register listener
  uwbRegisterListener(&listener);

  // ........
  xTaskCreate(uwbExampleRxTask, ...
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
