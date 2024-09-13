/*
 * debug_print.h
 *
 *  Created on: Sep 13, 2024
 *      Author: 86152
 */

#ifndef INC_DEBUG_PRINT_H_
#define INC_DEBUG_PRINT_H_

#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>

//#define ITM_TCR_ITMENA_Msk (1UL << 0)   // ITM Enable bit in the TCR register
//#define ITM_TER_PORT0_Msk   (1UL << 0)   // Trace Enable bit for Port 0



#define ITM_PORT (0)  // ITM Port 0 用于 SWO 输出

void debugprint(const char *fmt, ...);
void ITM_Init(void);


#endif /* INC_DEBUG_PRINT_H_ */
