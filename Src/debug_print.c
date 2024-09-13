//// debug_print.c
//#include "debug_print.h"
//#include "stm32l4xx.h"
//
//void ITM_Init(void) {
//    // 启用 Debug Trace 功能
//    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
//
//    // 解锁 ITM 寄存器
//    ITM->LAR = 0xC5ACCE55;
//
//    // 配置 ITM：启用 ITM，启用时间戳，启用 SWO
//    ITM->TCR = ITM_TCR_ITMENA_Msk   |  // 启用 ITM
//               ITM_TCR_TSENA_Msk    |  // 启用时间戳
//               ITM_TCR_SWOENA_Msk;     // 启用 SWO
//
//    // 启用通道 0 以发送数据
//    ITM->TER |= (1UL << ITM_PORT);
//}
//
//static int MY_ITM_SendChar(int ch) {
//    if ((ITM->TCR & ITM_TCR_ITMENA_Msk) &&  // 检查 ITM 是否启用
//        (ITM->TER & (1UL << ITM_PORT))) {   // 检查端口 0 是否启用
//        while (ITM->PORT[ITM_PORT].u32 == 0); // 等待缓冲区空闲
//        ITM->PORT[ITM_PORT].u8 = (uint8_t) ch; // 发送字符
//    }
//    return (ch); // 返回发送的字符
//}
//
//void debugprint(const char *fmt, ...) {
//    char buffer[256];
//    va_list args;
//    va_start(args, fmt);
//    vsnprintf(buffer, sizeof(buffer), fmt, args);  // 格式化字符串
//    va_end(args);
//
//    for (char *p = buffer; *p != '\0'; p++) {
//        MY_ITM_SendChar(*p); // 通过 MY_ITM_SendChar 发送每个字符
//    }
//}
