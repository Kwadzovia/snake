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
#include "stm32f4xx_hal.h"
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
#define INITIAL_LENGTH 3
#define MAX_LENGTH 16
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

  typedef struct
  {
    uint8_t length;
    uint8_t row[64];
    uint8_t col[64];
  } snakeStruct;

  uint8_t skull1[8] = { 0b00111100,
                      0b00110011,
                      0b11110011,
                      0b01001111,
                      0b11110011,
                      0b00110011,
                      0b00111100,
                      0b00000000};

  uint8_t skull2[8] = { 0b00000000,
                      0b00111100,
                      0b00110011,
                      0b11110011,
                      0b01001111,
                      0b11110011,
                      0b00110011,
                      0b00111100};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void updateInput(joystick * joyHandle, MAX7219 * segmentHandle);
uint8_t updateSnake(joystick * joyHandle, snakeStruct * snakeHandle);
void updateWorld(uint8_t * row, uint8_t * col, snakeStruct * snakeHandle, uint8_t worldFrame[8]);
void initWorld(uint8_t * row, uint8_t * col, MAX7219 * segHandle, joystick * joyHandle, snakeStruct * snakeHandle, uint8_t worldFrame[8]);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void updateInput(joystick * joyHandle, MAX7219 * segmentHandle)
{
      joyHandle->prev_direction = joyHandle->direction;
     // Update Input
      joystick_read(joyHandle);

      if(joyHandle->x_axis > 200 && joyHandle->prev_direction != JOY_WEST)
      {
        joyHandle->direction = JOY_EAST;
      }
      else if(joyHandle->x_axis < 50 && joyHandle->prev_direction != JOY_EAST)
      {
        joyHandle->direction = JOY_WEST;
      }
      else if(joyHandle->y_axis < 50 && joyHandle->prev_direction != JOY_SOUTH) //Note: Joystick Y axis is flipped
      {
        joyHandle->direction = JOY_NORTH;
      }
      else if(joyHandle->y_axis > 200 && joyHandle->prev_direction != JOY_NORTH)
      {
        joyHandle->direction = JOY_SOUTH ;
      }
}

uint8_t updateSnake(joystick * joyHandle, snakeStruct * snakeHandle)
{

      //Update snake body
      for(int i = snakeHandle->length+1; i > 0; i--)
      {
        if(snakeHandle->row[i] == snakeHandle->row[0] && snakeHandle->col[i] == snakeHandle->col[0])
        {
          return 1;
        }
        snakeHandle->row[i] = snakeHandle->row[i-1];
        snakeHandle->col[i] = snakeHandle->col[i-1];
      }

      //Update snake head
      if(joyHandle->direction == JOY_EAST)
      {
        snakeHandle->col[0] = (snakeHandle->col[0] == MAX7219_ALL_ROWS-1) ? 0 : snakeHandle->col[0]+1 ;
      }
      else if(joyHandle->direction == JOY_WEST)
      {
        snakeHandle->col[0] = (snakeHandle->col[0] == 0) ? MAX7219_ALL_ROWS-1: snakeHandle->col[0]-1 ;
      }
      else if(joyHandle->direction == JOY_NORTH)
      {
        snakeHandle->row[0] = (snakeHandle->row[0] == 0) ? MAX7219_ALL_ROWS-1 : snakeHandle->row[0]-1 ;
      }
      else if(joyHandle->direction == JOY_SOUTH)
      {
        snakeHandle->row[0] = (snakeHandle->row[0] == MAX7219_ALL_ROWS-1) ? 0 : snakeHandle->row[0]+1 ;
      }

      return 0;
}

void updateWorld(uint8_t * row, uint8_t * col, snakeStruct * snakeHandle, uint8_t worldFrame[8])
{
        //Clear Snake from World     
      for(int i = 0; i < 8; i++)
      {
        worldFrame[i] = 0;
      }

      if(snakeHandle->row[0] == *row && snakeHandle->col[0] == *col)
      {
        snakeHandle->length++;
        
        *row = (uint8_t) HAL_GetTick() >> 4;
        *row = (uint8_t) *row % 7;
        *col = (uint8_t) HAL_GetTick();
        *col = (uint8_t) *col % 7;
      }

      for(int i = 0; i < snakeHandle->length; i++)
      {
        worldFrame[snakeHandle->col[i]] |= (1 << snakeHandle->row[i]);
      }
      
      worldFrame[*col] |= 1 << *row;
}

void initWorld(uint8_t * row, uint8_t * col, MAX7219 * segHandle, joystick * joyHandle, snakeStruct * snakeHandle, uint8_t worldFrame[8])
{
  joyHandle->direction = JOY_EAST;
  for(int i = 0; i<8; i++)
  {
    worldFrame[i] = 0;
  }

  snakeHandle->length = INITIAL_LENGTH;

  snakeHandle->row[0] = 4;
  snakeHandle->row[1] = 4;
  snakeHandle->row[2] = 4;
  snakeHandle->col[0] = 6;
  snakeHandle->col[1] = 5;
  snakeHandle->col[2] = 4;

  MAX7219_blockSet(segHandle, skull1);
  HAL_Delay(100);
  MAX7219_blockSet(segHandle, skull2);
  HAL_Delay(100);
  MAX7219_blockSet(segHandle, skull1);
  HAL_Delay(100);

  *row = (uint8_t) HAL_GetTick();
  *row = (uint8_t) *row % 7;
  
  *col = (uint8_t) HAL_GetTick() >> 1;
  *col = (uint8_t) *col % 7;

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

  // ADC_ChannelConfTypeDef sConfig = {0};
  // sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;

  joystick_init(&joyHandle);
  MAX7219_init(&segHandle, &hspi2);

  uint8_t world[8];

  snakeStruct snake = {INITIAL_LENGTH,{0},{0}};
  snake.row[0] = 4;
  snake.row[1] = 4;
  snake.row[2] = 4;
  snake.col[0] = 6;
  snake.col[1] = 5;
  snake.col[2] = 4;

  joyHandle.direction = JOY_EAST;

  uint8_t rand_row;
  uint8_t rand_col;
  
  initWorld(&rand_row, &rand_col, &segHandle, &joyHandle, &snake, world);
          

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

      // Read Input
      updateInput(&joyHandle,&segHandle);

      //Update Snake Position
      if(updateSnake(&joyHandle,&snake))
      {
        //Hit Self, Restart
        initWorld(&rand_row, &rand_col, &segHandle, &joyHandle, &snake, world);
      }

      updateWorld(&rand_row,&rand_col,&snake,world);

      //Write World to Display
      MAX7219_blockSet(&segHandle, world);
      HAL_Delay(100);

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
