#ifndef __DWT_DELAY_H__
#define __DWT_DELAY_H__

#include <stdint.h>

void dwt_delay_init(void);
void dwt_delay_us(uint32_t us);

#endif // __DWT_DELAY_H__