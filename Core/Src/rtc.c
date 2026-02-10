//
// Created by Pobed on 10.02.2026.
//

#include "main.h"

#include "rtc.h"

/**
 * @param unix_time_ms time counter in milliseconds
 */
volatile uint64_t unix_time_ms = 0;

/**
 * Real-time counter initialization function
 * @param start_time the starting value of the countdown time
 */
void RTC_Initialization(tm *start_time) {
    const uint64_t milliseconds = (uint64_t)mktime(start_time) * 1000;
    RTC_SetUnixTimeMs(milliseconds);
}

/**
 * This function must be called in a hardware timer interrupt
 * that is clearly configured to be called every millisecond.
 * @param tim the selected timer for the real-time clock
 */
void RTC_IRQHandlerMs(TIM_TypeDef *tim) {
    if (tim->SR & TIM_SR_UIF) {
        tim->SR &= ~TIM_SR_UIF;
        unix_time_ms++;
    }
}

/**
 * Get now unix-time in milliseconds, an atomic operation.
 * @return now unix-time in milliseconds
 */
uint64_t RTC_GetUnixTimeMs() {
    __disable_irq();
    const uint64_t milliseconds = unix_time_ms;
    __enable_irq();
    return milliseconds;
}

/**
 * Set a new unix-time in milliseconds, an atomic operation.
 * @param milliseconds new unix-time in milliseconds
 */
void RTC_SetUnixTimeMs(uint64_t milliseconds) {
    __disable_irq();
    unix_time_ms = milliseconds;
    __enable_irq();
}