#ifndef _USSENSOR_H_
#define _USSENSOR_H_

#include <stdint.h>

void USSensor_Init(void);
void USSensor_SendPulse(void);
uint32_t USSensor_GetDistance(void);

#endif
