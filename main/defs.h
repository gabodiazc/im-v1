
#ifndef DEFS
#define DEFS

#include <float.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <time.h>
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/i2c_master.h"
#include "driver/spi_master.h"
#include "driver/sdspi_host.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_task.h"
#include "math.h"
#include "sdmmc_cmd.h"
#include "esp_vfs_fat.h"
#include "sdkconfig.h"

/* Frecuencia MASTER */
#define I2C_MASTER_FREQ_HZ              10000

/* Pines I2C */
#define I2C_MASTER_SCL_IO				GPIO_NUM_47				// GPIO pin I2C master
#define I2C_MASTER_SDA_IO				GPIO_NUM_48				// GPIO pin I2C master

/* Direcciones slave de sensores (I2C) */
#define BMM350_SLAVE_ADDR           0x14
#define BMI270_SLAVE_ADDR           0x68
#define BME688_SLAVE_ADDR           0x76

/* Pines SPI (microsd)*/
#define PIN_NUM_CS                          GPIO_NUM_1         // GPIO pin
#define PIN_NUM_MOSI                        GPIO_NUM_2         // GPIO pin
#define PIN_NUM_CLK                         GPIO_NUM_43        // GPIO pin
#define PIN_NUM_MISO                        GPIO_NUM_44        // GPIO pin

/* SD */
#define FORMAT_IF_MOUNT_FAILED              true

/* Data length de BMM350 */
#define BMM350_OTP_DATA_LENGTH              32                 // No confundir con el largo del output

/* Frecuencias */
#define ODR_1_5625                          15625
#define ODR_3_125                           3125
#define ODR_6_25                            625
#define ODR_12_5                            125
#define ODR_25                              25
#define ODR_50                              50
#define ODR_100                             100
#define ODR_200                             200
#define ODR_400                             400
#define ODR_800                             800
#define ODR_1600                            1600
#define ODR_3200                            3200

/* Concatena bytes */
#define CONCAT_BYTES(msb, lsb)      (((uint16_t)msb << 8) | (uint16_t)lsb)

#endif