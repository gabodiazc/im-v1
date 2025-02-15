#include "common.h"
#include "bmm350.h"
#include "bmi270.h"
#include "bme688.h"
#include "defs.h"
#include "schema.pb-c.h"

/* -------------------- Configuración general ------------------------ */
/* Frecuencia master I2C*/
#define I2C_MASTER_FREQ_HZ		            10000

/* Imprime en bucle mediciones de todos los sensores en la consola.
 * Con esta opción activada no hay guardado de datos en SD */
#define LOOP_MEASURES                       true

/* Cantidad de mediciones a tomar y escribir en la tarjeta SD. 
 * No tiene relevancia si LOOP_MEASURES es true. */
#define NUM_MEASURES                        20

/* Path del archivo de mediciones a guardar. Modificar para 
 * cambiar nombre de archivo. Si el archivo no existe, lo crea. 
 * Y si existe, concatena bytes al final de este. */
#define FILE_PATH "/sdcard/med"

/* Habilita o desabilita de toma de datos de las variables ambientales. 
 * Esto solo hace que se guarden o no dichas mediciones. */
#define MAG_ON_OFF                          true
#define ACC_ON_OFF                          true
#define GYR_ON_OFF                          true
#define TEMP_ON_OFF                         true
#define HUM_ON_OFF                          true
#define PRESS_ON_OFF                        true
#define Q_AIR_ON_OFF                        true


/* -------------------- Configuración BMM350 ------------------------ */
/* Frecuencia de toma de datos magnetómetro. Puede tomar los valores 
 * 400, 200, 100, 50, 25, 12.5, 6.25, 3.125 y 1.5625. Las macros se 
 * encuentran en el script defs.h. */
#define MAG_ODR                             ODR_400

/* Promedio entre muestras. Puede ser 0, 2, 4 u 8. OJO: hay combinaciones 
 * de AVG y ODR NO VÁLIDAS (ver datasheet BMM350). */
#define MAG_AVG                             4


/* -------------------- Configuración BMI270 ------------------------ */
/* Frecuencia de toma de datos acelerómetro. Puede tomar los valores 
 * 12.5, 25, 50, 100, 200, 400, 800 y 1600. Las macros se 
 * encuentran en el script defs.h. */
#define ACC_ODR                             ODR_400

/* Promedio de muestreo acelerómetro. Puede tomar los valores 0, 2, 4
 * 8, 16, 32, 64, 128. */
#define ACC_AVG                             4

/* Rango del acelerómetro. Puede tomar los valores 2, 4, 8 y 16 (+-). */
#define ACC_RANGE                           8

/* Frecuencia de toma de datos giroscopio. Puede tomar los valores 
 * 12.5, 25, 50, 100, 200, 400, 800, 1600 y 3200. Las macros se 
 * encuentran en el script defs.h. */
#define GYR_ODR                             ODR_400

/* Rango del giroscopio. Puede tomar los valores 125, 250, 500, 1000 y 2000 (+-). */
#define GYR_RANGE                           500


/* -------------------- Configuración BME688 ------------------------ */

/* Oversample sensores temperatura, presión y humedad. Pueden tomar los 
 * valores 1, 2, 4, 8, y 16. */
#define TEMP_OVERSAMPLE                     2
#define PRESS_OVERSAMPLE                    16
#define HUM_OVERSAMPLE                      1

void app_main(void) {

    // Inicializa master bus
    ESP_ERROR_CHECK(i2c_master_init(&bus_handle));

    // Inicializa slave BMM350
    ESP_ERROR_CHECK(i2c_slave_init(&bus_handle, &device_bmm350, BMM350_SLAVE_ADDR, I2C_MASTER_FREQ_HZ));

    // Inicializa slave BMI270
    ESP_ERROR_CHECK(i2c_slave_init(&bus_handle, &device_bmi270, BMI270_SLAVE_ADDR, I2C_MASTER_FREQ_HZ));

    // Inicializa slave BME688
    ESP_ERROR_CHECK(i2c_slave_init(&bus_handle, &device_bme688, BME688_SLAVE_ADDR, I2C_MASTER_FREQ_HZ));

    // Inicializa cada uno de los sensores con sus respectivos parámetros
    bmi270_init(ACC_ODR, ACC_AVG, ACC_RANGE, GYR_ODR, GYR_RANGE);
    bmm350_init(MAG_ODR, MAG_AVG);
    bme688_init();

    // Inicializa SD
    mount_sd();
    FILE *f = fopen(FILE_PATH, "ab");
    if (!f) {
        ESP_LOGE("SPI", "Error abriendo archivo en la SD");
        return;
    }

    // While entra en loop cuando LOOP_MEASURES es true
    int count = 0;
    while (count < NUM_MEASURES || LOOP_MEASURES) {

        // Estructura donde se almacena 
        // UNA ventana de datos
        Measure measure = MEASURE__INIT;

        // Lectura de sensores
        readout_data_bmm350(&measure, false, MAG_ON_OFF);
        readout_data_bmi270(&measure, false, ACC_ON_OFF, GYR_ON_OFF);
        readout_data_bme688(&measure, false, TEMP_ON_OFF, PRESS_ON_OFF, HUM_ON_OFF, Q_AIR_ON_OFF, TEMP_OVERSAMPLE, PRESS_OVERSAMPLE, HUM_OVERSAMPLE);   // Se puede mejorar este código

        // Guardado en SD
        if (!LOOP_MEASURES) {
            save_measure_to_sd(&measure, f, FILE_PATH);
            count++;
        }
    }

    // Finaliza SD
    fclose(f);
    unmount_sd();

    // Lee datos guardados en la SD (DEBUGGING)
    mount_sd();    
    read_measures_from_sd(FILE_PATH);
    unmount_sd();
}