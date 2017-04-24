#ifndef _USSENSOR_H_
#define _USSENSOR_H_

#include <stdint.h>

void USSensor_Init(void);

void USSensor_SendFrontPulse(void);
void USSensor_SendLeftPulse(void);
void USSensor_SendRightPulse(void);

uint32_t USSensor_GetFrontDistance(void);
uint32_t USSensor_GetLeftDistance(void);
uint32_t USSensor_GetRightDistance(void);

#endif
