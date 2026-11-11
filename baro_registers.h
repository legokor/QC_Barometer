#ifndef BARO_REGISTERS_H
#define BARO_REGISTERS_H

// pressure data registers
#define BARO_PSR_B2 0x00
#define BARO_PSR_B1 0x01
#define BARO_PSR_B0 0x02

// temperature data registers
#define BARO_TMP_B2 0x03
#define BARO_TMP_B1 0x04
#define BARO_TMP_B0 0x05

// configuration registers
#define BARO_PRS_CFG 0x06
#define BARO_TMP_CFG 0x07
#define BARO_MEAS_CFG 0x08
#define BARO_CFG_REG 0x09

// interrupt and FIFO status
#define BARO_INT_STS 0x0A
#define BARO_FIFO_STS 0x0B

// reset register
#define BARO_RESET        0x0C

// product identification
#define BARO_PROD_ID      0x0D

// calibration coefficients
#define BARO_COEF_START   0x10
#define BARO_COEF_END     0x21

// tempererature coefficients source
#define BARO_COEF_SRCE    0x28

#endif //BARO_REGISTERS_H