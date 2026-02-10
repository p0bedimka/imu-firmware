/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {

#endif

#include <stdio.h>

#include "stm32f1xx_hal.h"

void Error_Handler(void);

#define LED_Pin           GPIO_PIN_13
#define LED_GPIO_Port     GPIOC
#define LED               LED_GPIO_Port, LED_Pin

#ifdef __cplusplus
}
#endif

#endif
