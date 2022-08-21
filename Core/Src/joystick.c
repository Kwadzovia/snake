/**
  ******************************************************************************
  * @file    joystick.c
  * @brief   This file provides code for the configuration and use of a joystick 
  *          Module
  ******************************************************************************
  * @attention
  *
  * Made by Selikem Kwadzovia
  ******************************************************************************
  */

  /* --------------------- Includes --------------------- */
  #include "adc.h"
  #include "main.h"
  #include "stm32f4xx_hal_adc.h"
  #include "stm32f4xx_hal_def.h"
  #include <stdint.h>
  #include "joystick.h"
  /* --------------------- Functions --------------------- */
  void joystick_init(joystick * joyHandle)
  {
      joyHandle->column = 1;
      joyHandle->direction = JOY_NORTH;
  }

  void joystick_read(joystick * joyHandle)
  {
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = ADC_CHANNEL_8;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
    
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
    joyHandle->x_axis = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);

    sConfig.Channel = ADC_CHANNEL_9;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
    joyHandle->y_axis = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
  }
