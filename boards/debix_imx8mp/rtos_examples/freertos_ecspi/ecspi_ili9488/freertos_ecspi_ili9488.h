#ifndef FREERTOS_ILI9488_DRIVER_H
#define FREERTOS_ILI9488_DRIVER_H

#include "fsl_common.h"


#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

/* Freescale includes. */
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "fsl_ecspi.h"
#include "fsl_i2c.h"
#include "fsl_ecspi_freertos.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#include "lvgl-src/lvgl.h"
#include "fsl_gpio.h"      


typedef struct {
	uint8_t blue;
	uint8_t green;
	uint8_t red;
} lv_color24_t;


#define ECSPI_TRANSFER_SIZE     64
#define ECSPI_TRANSFER_BAUDRATE 50000000U
#define ECSPI_MASTER_BASEADDR   ECSPI1
#define ECSPI_MASTER_CLK_FREQ                                                                 \
    (CLOCK_GetPllFreq(kCLOCK_SystemPll1Ctrl) / (CLOCK_GetRootPreDivider(kCLOCK_RootEcspi1)) / \
     (CLOCK_GetRootPostDivider(kCLOCK_RootEcspi1)))
#define ECSPI_MASTER_TRANSFER_CHANNEL kECSPI_Channel0
#define EXAMPLE_ECSPI_MASTER_IRQN     ECSPI1_IRQn


#define I2C_MASTER_BASEADDR I2C4
#define I2C_MASTER_CLK_FREQ                                                                 \
    (CLOCK_GetPllFreq(kCLOCK_SystemPll1Ctrl) / (CLOCK_GetRootPreDivider(kCLOCK_RootI2c4)) / \
     (CLOCK_GetRootPostDivider(kCLOCK_RootI2c4)) / 5) /* SYSTEM PLL1 DIV5 */

#define I2C_BAUDRATE               100000U



void spi_transaction_one_byte(uint8_t data);
void spi_transaction_array(uint8_t *data, uint16_t len);
#endif