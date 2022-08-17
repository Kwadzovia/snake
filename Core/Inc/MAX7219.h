#ifndef __MAX7219_H__
#define __MAX7219_H__

/* --------------------- Includes --------------------- */
#include "spi.h"
#include "stm32f4xx_hal_def.h"

/* --------------------- Defines --------------------- */
#define MAX7219_DEFAULT_INTENSITY 0x9
#define MAX7219_DEFAULT_SCAN_LIMIT 0x7

/* --------------------- Registers --------------------- */
#define MAX7219_DECODE_MODE         0x9
#define MAX7219_INTENSITY           0xA
#define MAX7219_SCAN_LIMIT          0xB
#define MAX7219_SHUTDOWN_REGISTER   0xC
#define MAX7219_DISPLAY_TEST        0xF

/* --------------------- Prototypes --------------------- */
typedef struct 
{

    SPI_HandleTypeDef * spiHandle;
    uint8_t rowMatrix[8];

} MAX7219;

HAL_StatusTypeDef MAX7219_init(MAX7219 * dev, SPI_HandleTypeDef* spiHandle);
HAL_StatusTypeDef MAX7219_setIntensity(MAX7219 * dev, uint8_t intensity);
HAL_StatusTypeDef MAX7219_setScanLimit(MAX7219 * dev, uint8_t scanLimit);
HAL_StatusTypeDef MAX7219_shutdown(MAX7219 * dev);
HAL_StatusTypeDef MAX7219_wake(MAX7219 * dev);
HAL_StatusTypeDef MAX7219_enable_displayTest(MAX7219 * dev);
HAL_StatusTypeDef MAX7219_disable_displayTest(MAX7219 * dev);

HAL_StatusTypeDef ReadRegisters(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef WriteRegister(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size);
#endif /* __MAX7219_H__ */
