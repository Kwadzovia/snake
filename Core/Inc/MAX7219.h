#ifndef __MAX7219_H__
#define __MAX7219_H__

/* --------------------- Includes --------------------- */
#include "spi.h"
#include "stm32f4xx_hal_def.h"

/* --------------------- Defines --------------------- */
#define MAX7219_DEFAULT_INTENSITY 0x09
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
HAL_StatusTypeDef MAX7219_clearRow(MAX7219 * dev, uint8_t row);
HAL_StatusTypeDef MAX7219_setRow(MAX7219 * dev, uint8_t row);

#endif /* __MAX7219_H__ */
