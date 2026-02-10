//
// Created by Pobed on 10.02.2026.
//

#ifndef RTC_H
#define RTC_H

#include "main.h"

#include <time.h>

typedef struct tm tm;

void RTC_Initialization(tm *start_time);
void RTC_IRQHandlerMs(TIM_TypeDef *tim);
uint64_t RTC_GetUnixTimeMs();
void RTC_SetUnixTimeMs(uint64_t milliseconds);

#endif