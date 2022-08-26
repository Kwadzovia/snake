/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MAX7219.h"
#include "joystick.h"
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define INITIAL_LENGTH 4
#define MAX_LENGTH 64
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void updateWorld(joystick * joyHandle, MAX7219 * segmentHandle);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void updateWorld(joystick * joyHandle, MAX7219 * segmentHandle)
{
     // Update Input
      joystick_read(joyHandle);

      if(joyHandle->x_axis > 200)
      {
        joyHandle->direction = JOY_EAST;
        // if(joyHandle->column < 8)
        // {
        //   MAX7219_setFullCol(segmentHandle, joyHandle->column);
        //   joyHandle->column++;
        // }
        // else
        // {
        //   MAX7219_setFullCol(segmentHandle, MAX7219_ALL_ROWS);
        // }
      }
      else if(joyHandle->x_axis < 50)
      {
        joyHandle->direction = JOY_WEST;
        // if(joyHandle->column > 1)
        // {
        //   MAX7219_clearCol(segmentHandle, joyHandle->column);
        //   joyHandle->column--;
        // }
        // else
        // {
        //   MAX7219_clearCol(segmentHandle, 1);
        // }
      }
      else if(joyHandle->y_axis < 50) //Note: Joystick Y axis is flipped
      {
        joyHandle->direction = JOY_NORTH;
        // if(joyHandle->column < 8)
        // {
        //   MAX7219_setFullCol(segmentHandle, joyHandle->column);
        //   joyHandle->column++;
        // }
        // else
        // {
        //   MAX7219_setFullCol(segmentHandle, MAX7219_ALL_ROWS);
        // }
      }
      else if(joyHandle->y_axis > 200)
      {
        joyHandle->direction = JOY_SOUTH;
        // if(joyHandle->column > 1)
        // {
        //   MAX7219_clearCol(segmentHandle, joyHandle->column);
        //   joyHandle->column--;
        // }
        // else
        // {
        //   MAX7219_clearCol(segmentHandle, 1);
        // }
      }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  MAX7219 segHandle;
  joystick joyHandle;

  joystick_init(&joyHandle);
  MAX7219_init(&segHandle, &hspi2);

  typedef struct
  {
    uint8_t length;
    uint8_t row[64];
    uint8_t col[64];
  } snakeStruct;

  typedef struct
  {
    uint8_t prevFrame[8];
    uint8_t currFrame[8];
  } worldStruct;

  worldStruct snakeWorld = {{0},{0}};
  snakeWorld.currFrame[0] = 3 << 2;

  snakeStruct snake = {INITIAL_LENGTH,{0},{0}};
  snake.row[0] = 4;
  snake.row[1] = 4;
  snake.row[2] = 4;
  snake.col[0] = 6;
  snake.col[1] = 5;
  snake.col[2] = 4;

  uint8_t temp_counter = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      temp_counter++;
      if(temp_counter > 5)
      {
        temp_counter = 0;
        snake.length++;
      }
      // Read Input
      updateWorld(&joyHandle,&segHandle);

      // New Display Frame
      MAX7219_clearAll(&segHandle);

      for(int i = 1; i < snake.length+1; i++)
      {
        snake.row[i] = snake.row[i-1];
        snake.col[i] = snake.col[i-1];
      }

      if(joyHandle.direction == JOY_EAST)
      {
        snake.col[0] = (snake.col[0] == MAX7219_ALL_ROWS) ? 1 : snake.col[0]+1 ;
      }
      else if(joyHandle.direction == JOY_WEST)
      {
        snake.col[0] = (snake.col[0] == 1) ? MAX7219_ALL_ROWS : snake.col[0]-1 ;
      }
      else if(joyHandle.direction == JOY_NORTH)
      {
        snake.row[0] = (snake.row[0] == 1) ? MAX7219_ALL_ROWS : snake.row[0]-1 ;
        
      }
      else if(joyHandle.direction == JOY_SOUTH)
      {
        snake.row[0] = (snake.row[0] == MAX7219_ALL_ROWS) ? 1 : snake.row[0]+1 ;
      }
      
      for(int i = 0; i < snake.length; i++)
      {
        MAX7219_setSegment(&segHandle, snake.col[i],snake.row[i]);
      }


      HAL_Delay(300);





    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
