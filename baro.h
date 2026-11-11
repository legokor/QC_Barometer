#ifndef BARO_H
#define BARO_H

#include <cstdint>
#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"

/**
 * @brief DPS368 barometric pressure sensor 
 *  The implementation relies on STM32 HAL drivers
 */

typedef struct{
    float pressure;
    float temperature;
    float altitude;

    float referencePressure;
    float referenceTemperature;

    SPI_HandleTypeDef* hspi;
    GPIO_TypeDef* csPort;
    uint16_t csPin;

    //coefficients
    int16_t c0;
    int16_t c1;
    int32_t c00;
    int32_t c10;
    int16_t c01;
    int16_t c11;
    int16_t c20;
    int16_t c21;
    int16_t c30;


    // sensor values
    int32_t pressureData;
    int32_t temperatureData;

    bool initial;
} baro_Baro;

/**
 * @brief Initializes the DPS368 barometer sensor.
 * @param baro - barometer instance
 * @return true on succes, else false
 */

bool baro_init(baro_Baro* baro, SPI_HandleTypeDef* hspi, GPIO_TypeDef* csPort, uint16_t csPin);

/**
 * @brief Checks whether the barometer was detected.
 * @param baro - the barometer instance
 * @return true on succes, else false
 */

bool baro_detected(baro_Baro* baro);

/**
 * @brief Configurates default measurment settings.
 * @param baro - the barometer instance
 * @return true on succes, else false
 */

bool baro_setDefaultSettings(baro_Baro* baro);

/**
 * @brief Reads pressure and temperature data.
 * @param baro - the barometer instance
 * @return true on succes, else false
 */
bool baro_readData(baro_Baro* baro);




/**
 * @brief Calculates relative altitude in meter.
 * @param pressure - [Pa]
 * @param referencePressure - reference pressure [Pa]
 * @param temperature - [°C]
 * @return relative altitude in m
 */
float baro_calculateAltitude(float pressure, float referencePressure, float referenceTemperature, float temperature);

#endif // BARO_H