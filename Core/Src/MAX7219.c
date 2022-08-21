/**
  ******************************************************************************
  * @file    max7219_driver.c
  * @brief   This file provides code for the configuration an use of an 8x8 LED
  *          Matrix Module
  ******************************************************************************
  * @attention
  *
  * Made by Selikem Kwadzovia
  ******************************************************************************
  */

  /* --------------------- Includes --------------------- */
  #include "MAX7219.h"
  #include "gpio.h"
  #include "main.h"
  #include "stm32f4xx_hal_gpio.h"
  #include <stdint.h>
  /* --------------------- Functions --------------------- */

static HAL_StatusTypeDef WriteRegister(SPI_HandleTypeDef *hspi, uint8_t *pData);
static HAL_StatusTypeDef MAX7219_setIntensity(MAX7219 * dev, uint8_t intensity);
static HAL_StatusTypeDef MAX7219_setScanLimit(MAX7219 * dev, uint8_t scanLimit);
static HAL_StatusTypeDef MAX7219_shutdown(MAX7219 * dev);
static HAL_StatusTypeDef MAX7219_wake(MAX7219 * dev);
static HAL_StatusTypeDef MAX7219_enable_displayTest(MAX7219 * dev);
static HAL_StatusTypeDef MAX7219_disable_displayTest(MAX7219 * dev);

static HAL_StatusTypeDef WriteRegister(SPI_HandleTypeDef *hspi, uint8_t *pData)
{
  HAL_StatusTypeDef retVal;
  uint8_t reg1 = pData[0];
  uint8_t reg2 = pData[1];
  HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);
  retVal = HAL_SPI_Transmit(hspi, &reg1, 1, HAL_MAX_DELAY);
  retVal = HAL_SPI_Transmit(hspi, &reg2, 1, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);
  return retVal;
}

HAL_StatusTypeDef MAX7219_init(MAX7219 * dev, SPI_HandleTypeDef * spiHandle)
{
  dev->spiHandle = spiHandle;
  MAX7219_shutdown(dev);
  MAX7219_disable_displayTest(dev);
  MAX7219_setIntensity(dev,MAX7219_DEFAULT_INTENSITY);
  MAX7219_setScanLimit(dev,MAX7219_DEFAULT_SCAN_LIMIT);
  return MAX7219_wake(dev);
}

static HAL_StatusTypeDef MAX7219_setIntensity(MAX7219 * dev, uint8_t intensity)
{
  uint8_t spiMessage[2] = {MAX7219_INTENSITY,intensity};
  if(intensity >= MAX7219_FULL_INTENSITY)
  {
    spiMessage[1] = MAX7219_DEFAULT_INTENSITY;
  }
  return WriteRegister(dev->spiHandle, spiMessage);
}

static HAL_StatusTypeDef MAX7219_setScanLimit(MAX7219 * dev, uint8_t scanLimit)
{
  uint8_t spiMessage[2] = {MAX7219_SCAN_LIMIT,scanLimit};
  if(scanLimit >= MAX7219_ALL_ROWS)
  {
    spiMessage[1] = MAX7219_DEFAULT_SCAN_LIMIT;
  }
  return WriteRegister(dev->spiHandle, spiMessage);
}

static HAL_StatusTypeDef MAX7219_shutdown(MAX7219 * dev)
{
  HAL_StatusTypeDef retVal;
  uint8_t spiMessage[2] = {MAX7219_SHUTDOWN_REGISTER,0x00};
  retVal = WriteRegister(dev->spiHandle, spiMessage);
  return retVal;
}

static HAL_StatusTypeDef MAX7219_wake(MAX7219 * dev)
{
  HAL_StatusTypeDef retVal;
  uint8_t spiMessage[2] = {MAX7219_SHUTDOWN_REGISTER,0x01};
  retVal = WriteRegister(dev->spiHandle, spiMessage);
  return retVal;
}

static HAL_StatusTypeDef MAX7219_enable_displayTest(MAX7219 * dev)
{
  HAL_StatusTypeDef retVal;
  uint8_t spiMessage[2] = {MAX7219_DISPLAY_TEST,0x01};
  retVal = WriteRegister(dev->spiHandle, spiMessage);
  return retVal;
}

static HAL_StatusTypeDef MAX7219_disable_displayTest(MAX7219 * dev)
{
  HAL_StatusTypeDef retVal;
  uint8_t spiMessage[2] = {MAX7219_DISPLAY_TEST,0x00};
  retVal = WriteRegister(dev->spiHandle, spiMessage);
  return retVal;
}

HAL_StatusTypeDef MAX7219_setSegment(MAX7219 * dev, uint8_t row, uint8_t value)
{
  if(row > 0 && row <= 8)
  {
    dev->rowMatrix[row-1] |= (1 << (value-1));
    uint8_t spiMessage[2] = {row,(1 << (value-1))};
    return WriteRegister(dev->spiHandle, spiMessage);
  }
  else
  {
    return HAL_ERROR;
  }
}

HAL_StatusTypeDef MAX7219_clearRow(MAX7219 * dev, uint8_t row)
{
  if(row > 0 && row <= 8)
  {
    dev->rowMatrix[row] = row;
    uint8_t spiMessage[2] = {row,0x00};
    return WriteRegister(dev->spiHandle, spiMessage);
  }
  else
  {
    return HAL_ERROR;
  }
}

HAL_StatusTypeDef MAX7219_setRow(MAX7219 * dev, uint8_t row)
{
  if(row > 0 && row <= 8)
  {
    dev->rowMatrix[row] = row;
    uint8_t spiMessage[2] = {row,0x00};
    return WriteRegister(dev->spiHandle, spiMessage);
  }
  else
  {
    return HAL_ERROR;
  }
}