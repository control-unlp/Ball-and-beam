/*
 * ultrasonic.c
 *
 *  Created on: Jul 1, 2025
 *      Author: thiag
 */

#include "ultrasonic.h"

#define TRIG_PORT GPIOA
#define TRIG_PIN  GPIO_PIN_0

#define ECHO_PORT GPIOA
#define ECHO_PIN  GPIO_PIN_1

extern TIM_HandleTypeDef htim2;

void Ultrasonic_Init(void)
{
    HAL_TIM_Base_Start(&htim2);
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);
}

uint16_t Ultrasonic_Read(void)
{
    uint32_t val1 = 0;
    uint32_t val2 = 0;
    uint32_t pMillis = 0;
    uint16_t distance = 0;

    // Trigger pulse: 10us
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    while(__HAL_TIM_GET_COUNTER(&htim2) < 10);
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);

    // Wait for ECHO high
    pMillis = HAL_GetTick();
    while(HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN) == GPIO_PIN_RESET)
    {
        if ((HAL_GetTick() - pMillis) > 100) return 0; // Timeout
    }
    val1 = __HAL_TIM_GET_COUNTER(&htim2);

    // Wait for ECHO low
    pMillis = HAL_GetTick();
    while(HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN) == GPIO_PIN_SET)
    {
        if ((HAL_GetTick() - pMillis) > 100) return 0; // Timeout
    }
    val2 = __HAL_TIM_GET_COUNTER(&htim2);

    // Duration in us
    uint32_t duration = (val2 > val1) ? (val2 - val1) : (0xFFFF - val1 + val2);

    // Distance in cm: (duration x speed of sound) /2
    // Speed = 0.034 cm/us
    distance = duration * 0.034/2;

    return distance;
}

