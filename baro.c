#include "baro.h"
#include "baro_registers.h"
#include "log/log.h"
#include "err/err.h"

#include<math.h>

/**
 * @brief Reads data from sensor registers using SPI communication
 * @param baro sensor
 * @param regAddress the register we want to read
 * @param numBytes number of bytes we want to read
 * @param buffer buffer for the read data
 */
#define _baro_SPI_TIMEOUT 3     //maximum time of wait [ms]
static bool _baro_readBlocking(baro_Baro* baro, uint8_t regAddress, uint8_t numBytes, uint8_t* buffer){
    if (baro == NULL || baro->hspi == NULL || baro->csPort == NULL || baro->csPin == 0){
        return false;
    }
    if(numBytes == 0 || buffer == NULL) {
        return false;
    }
    regAddress |= 0x80;
    HAL_GPIO_WritePin(baro->csPort, baro->csPin, 0);        // selecting device

    HAL_StatusTypeDef status = HAL_SPI_Transmit(baro->hspi, &regAddress, 1, _baro_SPI_TIMEOUT); //transfer

    if(status == HAL_OK){
        status = HAL_SPI_Receive(baro->hspi, buffer, numBytes, _baro_SPI_TIMEOUT * numBytes);
    }

    HAL_GPIO_WritePin(baro->csPort, baro->csPin, 1);        //      deselecting device

    return (status == HAL_OK);
}

static bool _baro_writeBlocking(baro_Baro* baro, uint8_t regAddress, uint8_t data){
    if(baro == NULL || baro->hspi == NULL || baro->csPort == NULL || baro->csPin == 0){
        return false;
    }
    regAddress &= 0x7F;
    HAL_GPIO_WritePin(baro->csPort, baro->csPin, 0);

    uint8_t txData[2] = {regAddress, data}; 
    HAL_StatusTypeDef status = HAL_SPI_Transmit(baro->hspi, txData, 2, _baro_SPI_TIMEOUT);
    
    HAL_GPIO_WritePin(baro->csPort, baro->csPin, 1);
    return (status == HAL_OK);
}


bool baro_detected(baro_Baro* baro){
    uint8_t productId;

    if (!_baro_readBlocking(baro, BARO_PROD_ID, 1, &productId)){
        log_debug("Read failed");
        return false;
    }
    return (productId == 0x10);
}


/**
 * @brief Reads coefficients.
 */

static bool _baro_readCalibrationData(baro_Baro* baro){
    uint8_t buffer[18];

    if(!_baro_readBlocking(baro, BARO_COEF_START, 18, buffer)){
        return false;
    }

    int32_t c0 = (buffer[0] << 4) | (buffer[1]>>4);
    if(c0 & 0x800) c0 -= 0x1000;        //to signed
    baro->c0 = (int16_t)c0;

    int32_t c1 = ((buffer[1] & 0x0F) << 8 | buffer[2]);
    if(c1 & 0x800) c1 -= 0x1000;
    baro->c1 = (int16_t)c1;

    int32_t c00 = ((buffer[3] << 12) | (buffer[4] << 4) | (buffer[5] >> 4));
    if (c00 & 0x80000) c00 -= 0x100000;
    baro->c00 = c00;

    int32_t c10 = ((buffer[5] & 0x0F) << 16 | (buffer[6] << 8) | buffer[7]);
    if (c10 & 0x80000) c10 -= 0x100000;
    baro->c10 = c10;

    int32_t c01 = ((buffer[8] << 8) | buffer[9]);
    if (c01 & 0x8000) c01 -= 0x10000;
    baro->c01 = (int16_t)c01;

    int32_t c11 = ((buffer[10] << 8) | buffer[11]);
    if (c11 & 0x8000) c11 -= 0x10000;
    baro->c11 = (int16_t)c11;

    int32_t c20 = ((buffer[12] << 8) | buffer[13]);
    if (c20 & 0x8000) c20 -= 0x10000;
    baro->c20 = (int16_t)c20;

    int32_t c21 = ((buffer[14] << 8) | buffer[15]);
    if (c21 & 0x8000) c21 -= 0x10000;
    baro->c21 = (int16_t)c21;

    int32_t c30 = ((buffer[16] << 8) | buffer[17]);
    if (c30 & 0x8000) c30 -= 0x10000;
    baro->c30 = (int16_t)c30;


    return true;
}

/**
 * @brief Waits till coefficients readable and sensor is ready.
 */

static bool baro_waitTillReady(baro_Baro* baro){
    uint8_t status;
    uint32_t START_TIME = HAL_GetTick();

    while(HAL_GetTick()-START_TIME < 100){
        if(!_baro_readBlocking(baro, BARO_MEAS_CFG, 1, &status)) {
            return false;
        }

        if((status & 0xC0) == 0xC0){
            return true;
        }

        HAL_Delay(1);

    }

    return false;
}

bool baro_init(baro_Baro* baro, SPI_HandleTypeDef* hspi, GPIO_TypeDef* csPort, uint16_t csPin){
    if(baro == NULL || hspi == NULL || csPort == NULL || csPin == 0){
        return false;
    }
    baro->hspi = hspi;
    baro->csPort = csPort;
    baro->csPin = csPin;

    baro->initial = false;
    log_debug("Initializing Baro...");

    err_try(baro_detected(baro));

    err_try(baro_waitTillReady(baro));

    err_try(_baro_readCalibrationData(baro));

    err_try(baro_setDefaultSettings(baro));

    baro->initial = true;

    //make first measurements
    uint32_t start = HAL_GetTick();
    bool firstMeasurement = false;

    while (HAL_GetTick()-start < 200){
        if(baro_readData(baro)){
            firstMeasurement = true;
            break;
        }

        HAL_Delay(5);
    }

    if(!firstMeasurement){
        baro->initial = false;
        return false;
    }

    baro->referencePressure = baro->pressure;
    baro->referenceTemperature = baro->temperature;
    baro->altitude = 0.0f;

    log_debug("Baro initialized");
    return true;
}


bool baro_setDefaultSettings(baro_Baro* baro){
    //pressure configuration
    err_try(_baro_writeBlocking(baro, BARO_PRS_CFG, 0x42));      // 16 measurment/s for 4x

    //temperature configuration
    err_try(_baro_writeBlocking(baro, BARO_TMP_CFG, 0xC2));      // 16 measurments/s for 4x and MEMS

    //sensor operating mode and status
    err_try(_baro_writeBlocking(baro, BARO_MEAS_CFG, 0x07));       // countinous p and T measurment

    return true;
}

bool baro_readData(baro_Baro* baro){
    if (baro == NULL || baro->initial == 0){
        return false;
    }

    uint8_t status;
    uint8_t buffer[6];

    err_try(_baro_readBlocking(baro, BARO_MEAS_CFG, 1, &status));

    if((status & 0x30) != 0x30){
        return false;
    }

    err_try(_baro_readBlocking(baro, BARO_PSR_B2, 6, buffer));

    int32_t rawPressure = ((buffer[0] << 16) | (buffer[1] << 8) | buffer[2]);
    int32_t rawTemperature = ((buffer[3] << 16) | (buffer[4] << 8) | buffer[5]);

    if(rawPressure & 0x800000) rawPressure -= 0x1000000;
    if(rawTemperature & 0x800000) rawTemperature -= 0x1000000;

    baro->pressureData = rawPressure;
    baro->temperatureData = rawTemperature;

    // scaling factors: 4x
    const float kP = 3670016.0f;
    const float kT = 3670016.0f;

    float pressureScaled = (float)rawPressure/kP;
    float temperatureScaled = (float)rawTemperature/kT;

    float pressure = baro->c00 + pressureScaled*(baro->c10 + pressureScaled*(baro->c20 + pressureScaled*(baro->c30))) +temperatureScaled*(baro->c01) + temperatureScaled*(baro->c11 +pressureScaled*(baro->c21))*pressureScaled;
    float temperature = (baro->c0)*0.5f + (baro->c1)*temperatureScaled;

    baro->pressure = pressure;
    baro->temperature = temperature;

    return true;

}


float baro_calculateAltitude(float pressure, float referencePressure, float referenceTemperature, float temperature){
    if(pressure <= 0.0f || referencePressure <= 0.0f || temperature <= -273.15f || referenceTemperature <= -273.15f){
        return NAN;
    }

    //barometric formula with constant lapse rate
    /*
    const float g0 = 9.80850f;      //gravitational acceleration in Budapest
    const float M0 = 0.0289644f;    //mean molar mass of air at sea level
    const float R = 8.314462618f;   //universal gas constant
    const float L = -0.0065f;       //temperature gardient(laps rate)

    float exponent = (R*L) / (g0*M0);
    float T = referenceTemperature + temperature;
    float altitude = T/L * (powf(referencePressure/pressure, exponent)-1.0f);
    */

    //hypsometric formula with avarage temperature
    const float g = 9.80850f;
    const float R = 287.05f;        //specific gas constant

    float avarageTempKelvin = (referenceTemperature + temperature)/2.0f + 273.15f;
    float altitude = (R*avarageTempKelvin/g)*logf(referencePressure/pressure);

    return altitude;
}
