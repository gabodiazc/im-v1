#ifndef COMMON
#define COMMON

#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "sdkconfig.h"
#include "esp_task.h"
#include <string.h>
#include "schema.pb-c.h"

extern i2c_master_bus_handle_t bus_handle;
extern i2c_master_dev_handle_t device_bmm350;
extern i2c_master_dev_handle_t device_bmi270;
extern i2c_master_dev_handle_t device_bme688;
extern esp_err_t ret;

esp_err_t i2c_master_init(i2c_master_bus_handle_t *bus_handle);
esp_err_t i2c_slave_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *device, int slave_addr, int i2c_master_freq);
esp_err_t device_read(i2c_master_dev_handle_t device, uint8_t *data_address, uint8_t *data_rd, size_t size, const char *tag);
esp_err_t device_write(i2c_master_dev_handle_t device, uint8_t *data_address, uint8_t *data_wr, size_t size, const char *tag);
void mount_sd(void);
void unmount_sd(void);
void save_measure_to_sd(Measure *m, FILE *f, char *file_path);
void read_measures_from_sd(char *file_path);

#endif