#ifndef __MAX7219_H__
#define __MAX7219_H__

/* --------------------- Includes --------------------- */
#include "spi.h"
#include "stm32f4xx_hal_def.h"
#include <stdint.h>

/* --------------------- Defines --------------------- */
#define MAX7219_DEFAULT_INTENSITY 0x01
#define MAX7219_DEFAULT_SCAN_LIMIT 0x07

#define MAX7219_ALL_ROWS 0x08
#define MAX7219_FULL_INTENSITY 0x0F

/* --------------------- Registers --------------------- */
#define MAX7219_DECODE_MODE         0x09
#define MAX7219_INTENSITY           0x0A
#define MAX7219_SCAN_LIMIT          0x0B
#define MAX7219_SHUTDOWN_REGISTER   0x0C
#define MAX7219_DISPLAY_TEST        0x0F

/* --------------------- Prototypes --------------------- */
typedef struct 
{

    SPI_HandleTypeDef * spiHandle;
    uint8_t rowMatrix[8];

} MAX7219;

HAL_StatusTypeDef MAX7219_init(MAX7219 * dev, SPI_HandleTypeDef * spiHandle);
HAL_StatusTypeDef MAX7219_setSegment(MAX7219 * dev, uint8_t row, uint8_t value);
HAL_StatusTypeDef MAX7219_clearCol(MAX7219 * dev, uint8_t col);
HAL_StatusTypeDef MAX7219_setFullCol(MAX7219 * dev, uint8_t col);
HAL_StatusTypeDef MAX7219_setCol(MAX7219 * dev, uint8_t col, uint8_t * pattern);
HAL_StatusTypeDef MAX7219_clearAll(MAX7219 * dev);
HAL_StatusTypeDef MAX7219_setAll(MAX7219 * dev);
HAL_StatusTypeDef MAX7219_shutdown(MAX7219 * dev);
HAL_StatusTypeDef MAX7219_wake(MAX7219 * dev);
HAL_StatusTypeDef MAX7219_setIntensity(MAX7219 * dev, uint8_t intensity);
HAL_StatusTypeDef MAX7219_setScanLimit(MAX7219 * dev, uint8_t scanLimit);
HAL_StatusTypeDef MAX7219_enable_displayTest(MAX7219 * dev);
HAL_StatusTypeDef MAX7219_disable_displayTest(MAX7219 * dev);
HAL_StatusTypeDef MAX7219_blockSet(MAX7219 * dev, uint8_t blockArray[8]);

#endif /* __MAX7219_H__ */
