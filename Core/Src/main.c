/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

#include "main.h"

#include <stdbool.h>

#include "mpu6050.h"

#include "rtc.h"

I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart1;

MPU6050_t mpu6050;

static void Led_Init(void);
static void SystemClock_Config(void);
static void I2C1_Init(void);
static void RTC_TIM2_Init(void);
static void USART1_Init(void);

static void loop(void);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
    HAL_Init();
    Led_Init();
    SystemClock_Config();

    I2C1_Init();
    RTC_TIM2_Init();
    USART1_Init();

    mpu6050.I2Cx = &hi2c1;
    mpu6050.addr = MPU6050_DEFAULT_ADDRESS;

    MPU6050_Reset(&mpu6050);
    HAL_Delay(50);

    MPU6050_GetDeviceID(&mpu6050);

    MPU6050_SetClockSource(&mpu6050, CLOCK_PLL_XGYRO);

    MPU6050_SetFullScaleAccelRange(&mpu6050, A16G);
    MPU6050_SetFullScaleGyroRange(&mpu6050, G1000DPS);

    MPU6050_SetSleepEnabled(&mpu6050, false);

    MPU6050_SetTempSensorEnabled(&mpu6050, true);

    MPU6050_SetDMPEnabled(&mpu6050, true);

    MPU6050_AverageCalibration(&mpu6050);

    MPU6050_SetIntDataReadyEnabled(&mpu6050, true);

    HAL_GPIO_WritePin(LED, GPIO_PIN_SET);

    loop();
}

/**
  * @brief  Function implementing the defaultTask thread.
  * @retval None
  */
static void loop(void) {
    float acc_x, acc_y, acc_z;
    float gyr_x, gyr_y, gyr_z;
    float temperature;
    uint64_t time;
    // ReSharper disable once CppDFAEndlessLoop
    while (true) {
        if (MPU6050_GetIntDataReadyStatus(&mpu6050) == true) {
            MPU6050_GetAcceleration(&mpu6050, &acc_x, &acc_y, &acc_z);
            MPU6050_GetRotation(&mpu6050, &gyr_x, &gyr_y, &gyr_z);
            temperature = MPU6050_GetTemperature(&mpu6050);
            time = RTC_GetUnixTimeMs();
            printf("time = %llu, acc x = %0.3f, y = %0.3f, z = %0.3f; gyr x = %0.3f, y = %0.3f, z = %0.3f; temperature = %0.3f\r\n",
                    time, acc_x, acc_y, acc_z, gyr_x, gyr_y, gyr_z, temperature);
        }
        HAL_Delay(200);
    }
}

/**
  * @brief GPIO Initialization Function
  * @retval None
  */
static void Led_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();

    HAL_GPIO_WritePin(LED, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin = LED_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
static void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_LSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.LSEState = RCC_LSE_ON;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
        Error_Handler();

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
        Error_Handler();

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC | RCC_PERIPHCLK_USB;
    PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
    PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
        Error_Handler();

    HAL_RCC_EnableCSS();
}

/**
  * @brief I2C1 Initialization Function
  * @retval None
  */
static void I2C1_Init(void) {
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 400000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
        Error_Handler();
}

static void RTC_TIM2_Init(void) {
    tm time = {
        45,
        30,
        12,
        15,
        10,
        2023 - 1900
    };

    RTC_Initialization(&time);

    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    // Расчет для 1 мс при 72 МГц (подстройте под вашу частоту!)
    // Prescaler = (SystemCoreClock / 1000000) - 1 = для 1 МГц
    // 72MHz / 7200 = 10 kHz (0.1 мс)
    TIM2->PSC = 7200 - 1;

    // Период для 1 мс: 10 отсчетов по 0.1 мс
    TIM2->ARR = 10 - 1;

    TIM2->CR1 = TIM_CR1_CEN | TIM_CR1_ARPE;

    TIM2->DIER |= TIM_DIER_UIE;

    NVIC_SetPriority(TIM2_IRQn, 0);
    NVIC_EnableIRQ(TIM2_IRQn);

    TIM2->CR1 |= TIM_CR1_CEN;
}

void TIM2_IRQHandler(void) {
    RTC_IRQHandlerMs(TIM2);
}

/**
  * @brief USART1 Initialization Function
  * @retval None
  */
static void USART1_Init(void) {
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK)
        Error_Handler();
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void) {
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    // ReSharper disable once CppDFAEndlessLoop
    for (uint8_t i = 0; i < 4; ++i) {
        HAL_GPIO_TogglePin(LED);
        Delay_Ms(100);
    }
    NVIC_SystemReset();
}

void Delay_Ms(uint32_t milliseconds) {
    uint32_t cycles = (SystemCoreClock / 1000) * milliseconds;

    if (CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) {
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

        uint32_t start = DWT->CYCCNT;
        while ((DWT->CYCCNT - start) < cycles);
    } else {
        for (uint32_t i = 0; i < cycles; ++i)
            __NOP();
    }
}

#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)

PUTCHAR_PROTOTYPE {
    HAL_UART_Transmit(&huart1, (uint8_t *) &ch, 1, HAL_MAX_DELAY);
    return ch;
}

int write(int file, char *ptr, int len) {
    (void) file;

    if (HAL_UART_Transmit(&huart1, (uint8_t *) (ptr), len, HAL_MAX_DELAY) == HAL_OK)
        return len;
    else
        return 0;
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line) {
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
