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
  /* --------------------- Functions --------------------- */


HAL_StatusTypeDef ReadRegisters(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size)
{
  return HAL_SPI_Receive(hspi, pData, Size, HAL_MAX_DELAY);
}

HAL_StatusTypeDef WriteRegister(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size)
{
  return HAL_SPI_Transmit(hspi, pData, Size, HAL_MAX_DELAY);
}

HAL_StatusTypeDef MAX7219_init(MAX7219 * dev, SPI_HandleTypeDef * spiHandle)
{
  MAX7219_shutdown(dev);
  MAX7219_setIntensity(dev,MAX7219_DEFAULT_INTENSITY);
  MAX7219_setScanLimit(dev,MAX7219_DEFAULT_SCAN_LIMIT);
  MAX7219_disable_displayTest(dev);
  return MAX7219_wake(dev);
}

HAL_StatusTypeDef MAX7219_setIntensity(MAX7219 * dev, uint8_t intensity)
{
  uint8_t intensityVal[2] = {MAX7219_INTENSITY,intensity};
  return WriteRegister(dev->spiHandle, intensityVal, sizeof(intensityVal));
}

HAL_StatusTypeDef MAX7219_setScanLimit(MAX7219 * dev, uint8_t scanLimit)
{
  uint8_t scanVal[2] = {MAX7219_SCAN_LIMIT,scanLimit};
  return WriteRegister(dev->spiHandle, scanVal, sizeof(scanVal));
}

HAL_StatusTypeDef MAX7219_shutdown(MAX7219 * dev)
{
  uint8_t scanVal[2] = {MAX7219_SHUTDOWN_REGISTER,0};
  return WriteRegister(dev->spiHandle, scanVal, sizeof(scanVal));
}

HAL_StatusTypeDef MAX7219_wake(MAX7219 * dev)
{
  uint8_t scanVal[2] = {MAX7219_SHUTDOWN_REGISTER,1};
  return WriteRegister(dev->spiHandle, scanVal, sizeof(scanVal));
}

HAL_StatusTypeDef MAX7219_enable_displayTest(MAX7219 * dev)
{
  uint8_t scanVal[2] = {MAX7219_DISPLAY_TEST,1};
  return WriteRegister(dev->spiHandle, scanVal, sizeof(scanVal));
}

HAL_StatusTypeDef MAX7219_disable_displayTest(MAX7219 * dev)
{
  uint8_t scanVal[2] = {MAX7219_DISPLAY_TEST,0};
  return WriteRegister(dev->spiHandle, scanVal, sizeof(scanVal));
}

HAL_StatusTypeDef MAX7219_setRow(MAX7219 * dev, uint8_t row, uint8_t value)
{
  if(row > 0 && row <= 8)
  {
    dev->rowMatrix[row] = value;
    uint8_t scanVal[2] = {value,row};
    return WriteRegister(dev->spiHandle, scanVal, sizeof(scanVal));
  }
  else
  {
    return HAL_ERROR;
  }
}