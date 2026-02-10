//
// Created by Pobed on 02.02.2026.
//

#ifndef MPU6050_H
#define MPU6050_H

#include "main.h"

#include <stdbool.h>

#define MPU6050_ADDRESS_AD0_LOW     0x68 << 1 // address pin low (GND), default for InvenSense evaluation board
#define MPU6050_ADDRESS_AD0_HIGH    0x69 << 1 // address pin high (VCC)
#define MPU6050_DEFAULT_ADDRESS     MPU6050_ADDRESS_AD0_LOW

typedef enum {
    MPU6050_OK = 0x00,
    MPU6050_SUCCESS = 0x01,
    MPU6050_FAILED = 0x02,
    MPU6050_ERROR = 0xFF,
} MPU6050_Status;

// Accelerometer Full scale values:
typedef enum {
    A2G,			    // 0 = 2g --> maximum possible resolution
    A4G,				// 1 = 4g
    A8G,				// 2 = 8g
    A16G				// 3 = 16g
} ACC_FS;

#define ACC_FREE_FALL   9.81f

// Gyroscope Full scale values:
typedef enum {
    G250DPS,	        // 0 = 250  °/s --> allows maximum resolution
    G500DPS,	        // 1 = 500  °/s
    G1000DPS,	        // 2 = 1000 °/s
    G2000DPS	        // 3 = 2000 °/s
} GYR_FS;

typedef enum {
    CLOCK_INTERNAL,
    CLOCK_PLL_XGYRO,
    CLOCK_PLL_YGYRO,
    CLOCK_PLL_ZGYRO,
    CLOCK_PLL_EXT32K,
    CLOCK_PLL_EXT19M,
    CLOCK_KEEP_RESET
} MPU6050_CLOCKS;

typedef struct {
    I2C_HandleTypeDef *I2Cx;
    uint8_t addr;

    uint8_t dev_id;

    struct Resolution {
        float acc;
        float gyr;
    } res;
} MPU6050_t;

// *********************  MPU6050 interface functions: ********************* //

MPU6050_Status MPU6050_Initialization(MPU6050_t *dev, I2C_HandleTypeDef *I2Cx, uint8_t addr);

// AUX_VDDIO register
uint8_t MPU6050_GetAuxVDDIOLevel(MPU6050_t *dev);
void MPU6050_SetAuxVDDIOLevel(MPU6050_t *dev, uint8_t level);

// SMPLRT_DIV register
uint8_t MPU6050_GetRate(MPU6050_t *dev);
void MPU6050_SetRate(MPU6050_t *dev, uint8_t rate);

// CONFIG register
uint8_t MPU6050_GetExternalFrameSync(MPU6050_t *dev);
void MPU6050_SetExternalFrameSync(MPU6050_t *dev, uint8_t sync);
uint8_t MPU6050_GetDLPFMode(MPU6050_t *dev);
void MPU6050_SetDLPFMode(MPU6050_t *dev, uint8_t mode);

// GYRO_CONFIG register
uint8_t MPU6050_GetFullScaleGyroRange(MPU6050_t *dev);
void MPU6050_SetFullScaleGyroRange(MPU6050_t *dev, GYR_FS range);

// ACCEL_CONFIG register
bool MPU6050_GetAccelXSelfTest(MPU6050_t *dev);
void MPU6050_SetAccelXSelfTest(MPU6050_t *dev, bool enabled);
bool MPU6050_GetAccelYSelfTest(MPU6050_t *dev);
void MPU6050_SetAccelYSelfTest(MPU6050_t *dev, bool enabled);
bool MPU6050_GetAccelZSelfTest(MPU6050_t *dev);
void MPU6050_SetAccelZSelfTest(MPU6050_t *dev, bool enabled);
uint8_t MPU6050_GetFullScaleAccelRange(MPU6050_t *dev);
void MPU6050_SetFullScaleAccelRange(MPU6050_t *dev, ACC_FS range);
uint8_t MPU6050_GetDHPFMode(MPU6050_t *dev);
void MPU6050_SetDHPFMode(MPU6050_t *dev, uint8_t bandwidth);

// FF_THR register
uint8_t MPU6050_GetFreefallDetectionThreshold(MPU6050_t *dev);
void MPU6050_SetFreefallDetectionThreshold(MPU6050_t *dev, uint8_t threshold);

// FF_DUR register
uint8_t MPU6050_GetFreefallDetectionDuration(MPU6050_t *dev);
void MPU6050_SetFreefallDetectionDuration(MPU6050_t *dev, uint8_t duration);

// MOT_THR register
uint8_t MPU6050_GetMotionDetectionThreshold(MPU6050_t *dev);
void MPU6050_SetMotionDetectionThreshold(MPU6050_t *dev, uint8_t threshold);

// MOT_DUR register
uint8_t MPU6050_GetMotionDetectionDuration(MPU6050_t *dev);
void MPU6050_SetMotionDetectionDuration(MPU6050_t *dev, uint8_t duration);

// ZRMOT_THR register
uint8_t MPU6050_GetZeroMotionDetectionThreshold(MPU6050_t *dev);
void MPU6050_SetZeroMotionDetectionThreshold(MPU6050_t *dev, uint8_t threshold);

// ZRMOT_DUR register
uint8_t MPU6050_GetZeroMotionDetectionDuration(MPU6050_t *dev);
void MPU6050_SetZeroMotionDetectionDuration(MPU6050_t *dev, uint8_t duration);

// FIFO_EN register
bool MPU6050_GetTempFIFOEnabled(MPU6050_t *dev);
void MPU6050_SetTempFIFOEnabled(MPU6050_t *dev, bool enabled);
bool MPU6050_GetXGyroFIFOEnabled(MPU6050_t *dev);
void MPU6050_SetXGyroFIFOEnabled(MPU6050_t *dev, bool enabled);
bool MPU6050_GetYGyroFIFOEnabled(MPU6050_t *dev);
void MPU6050_SetYGyroFIFOEnabled(MPU6050_t *dev, bool enabled);
bool MPU6050_GetZGyroFIFOEnabled(MPU6050_t *dev);
void MPU6050_SetZGyroFIFOEnabled(MPU6050_t *dev, bool enabled);
bool MPU6050_GetAccelFIFOEnabled(MPU6050_t *dev);
void MPU6050_SetAccelFIFOEnabled(MPU6050_t *dev, bool enabled);
bool MPU6050_GetSlave2FIFOEnabled(MPU6050_t *dev);
void MPU6050_SetSlave2FIFOEnabled(MPU6050_t *dev, bool enabled);
bool MPU6050_GetSlave1FIFOEnabled(MPU6050_t *dev);
void MPU6050_SetSlave1FIFOEnabled(MPU6050_t *dev, bool enabled);
bool MPU6050_GetSlave0FIFOEnabled(MPU6050_t *dev);
void MPU6050_SetSlave0FIFOEnabled(MPU6050_t *dev, bool enabled);

// I2C_MST_CTRL register
bool MPU6050_GetMultiMasterEnabled(MPU6050_t *dev);
void MPU6050_SetMultiMasterEnabled(MPU6050_t *dev, bool enabled);
bool MPU6050_GetWaitForExternalSensorEnabled(MPU6050_t *dev);
void MPU6050_SetWaitForExternalSensorEnabled(MPU6050_t *dev, bool enabled);
bool MPU6050_GetSlave3FIFOEnabled(MPU6050_t *dev);
void MPU6050_SetSlave3FIFOEnabled(MPU6050_t *dev, bool enabled);
bool MPU6050_GetSlaveReadWriteTransitionEnabled(MPU6050_t *dev);
void MPU6050_SetSlaveReadWriteTransitionEnabled(MPU6050_t *dev, bool enabled);
uint8_t MPU6050_GetMasterClockSpeed(MPU6050_t *dev);
void MPU6050_SetMasterClockSpeed(MPU6050_t *dev, uint8_t speed);

// I2C_SLV* registers (Slave 0-3)
uint8_t MPU6050_GetSlaveAddress(MPU6050_t *dev, uint8_t num);
void MPU6050_SetSlaveAddress(MPU6050_t *dev, uint8_t num, uint8_t address);
uint8_t MPU6050_GetSlaveRegister(MPU6050_t *dev, uint8_t num);
void MPU6050_SetSlaveRegister(MPU6050_t *dev, uint8_t num, uint8_t reg);
bool MPU6050_GetSlaveEnabled(MPU6050_t *dev, uint8_t num);
void MPU6050_SetSlaveEnabled(MPU6050_t *dev, uint8_t num, bool enabled);
bool MPU6050_GetSlaveWordByteSwap(MPU6050_t *dev, uint8_t num);
void MPU6050_SetSlaveWordByteSwap(MPU6050_t *dev, uint8_t num, bool enabled);
bool MPU6050_GetSlaveWriteMode(MPU6050_t *dev, uint8_t num);
void MPU6050_SetSlaveWriteMode(MPU6050_t *dev, uint8_t num, bool mode);
bool MPU6050_GetSlaveWordGroupOffSet(MPU6050_t *dev, uint8_t num);
void MPU6050_SetSlaveWordGroupOffSet(MPU6050_t *dev, uint8_t num, bool enabled);
uint8_t MPU6050_GetSlaveDataLength(MPU6050_t *dev, uint8_t num);
void MPU6050_SetSlaveDataLength(MPU6050_t *dev, uint8_t num, uint8_t length);

// I2C_SLV* registers (Slave 4)
uint8_t MPU6050_GetSlave4Address(MPU6050_t *dev);
void MPU6050_SetSlave4Address(MPU6050_t *dev, uint8_t address);
uint8_t MPU6050_GetSlave4Register(MPU6050_t *dev);
void MPU6050_SetSlave4Register(MPU6050_t *dev, uint8_t reg);
void MPU6050_SetSlave4OutputByte(MPU6050_t *dev, uint8_t data);
bool MPU6050_GetSlave4Enabled(MPU6050_t *dev);
void MPU6050_SetSlave4Enabled(MPU6050_t *dev, bool enabled);
bool MPU6050_GetSlave4InterruptEnabled(MPU6050_t *dev);
void MPU6050_SetSlave4InterruptEnabled(MPU6050_t *dev, bool enabled);
bool MPU6050_GetSlave4WriteMode(MPU6050_t *dev);
void MPU6050_SetSlave4WriteMode(MPU6050_t *dev, bool mode);
uint8_t MPU6050_GetSlave4MasterDelay(MPU6050_t *dev);
void MPU6050_SetSlave4MasterDelay(MPU6050_t *dev, uint8_t delay);
uint8_t MPU6050_GetSlate4InputByte(MPU6050_t *dev);

// I2C_MST_STATUS register
bool MPU6050_GetPassthroughStatus(MPU6050_t *dev);
bool MPU6050_GetSlave4IsDone(MPU6050_t *dev);
bool MPU6050_GetLostArbitration(MPU6050_t *dev);
bool MPU6050_GetSlave4Nack(MPU6050_t *dev);
bool MPU6050_GetSlave3Nack(MPU6050_t *dev);
bool MPU6050_GetSlave2Nack(MPU6050_t *dev);
bool MPU6050_GetSlave1Nack(MPU6050_t *dev);
bool MPU6050_GetSlave0Nack(MPU6050_t *dev);

// INT_PIN_CFG register
bool MPU6050_GetInterruptMode(MPU6050_t *dev);
void MPU6050_SetInterruptMode(MPU6050_t *dev, bool mode);
bool MPU6050_GetInterruptDrive(MPU6050_t *dev);
void MPU6050_SetInterruptDrive(MPU6050_t *dev, bool drive);
bool MPU6050_GetInterruptLatch(MPU6050_t *dev);
void MPU6050_SetInterruptLatch(MPU6050_t *dev, bool latch);
bool MPU6050_GetInterruptLatchClear(MPU6050_t *dev);
void MPU6050_SetInterruptLatchClear(MPU6050_t *dev, bool clear);
bool MPU6050_GetFSyncInterruptLevel(MPU6050_t *dev);
void MPU6050_SetFSyncInterruptLevel(MPU6050_t *dev, bool level);
bool MPU6050_GetFSyncInterruptEnabled(MPU6050_t *dev);
void MPU6050_SetFSyncInterruptEnabled(MPU6050_t *dev, bool enabled);
bool MPU6050_GetI2CBypassEnabled(MPU6050_t *dev);
void MPU6050_SetI2CBypassEnabled(MPU6050_t *dev, bool enabled);
bool MPU6050_GetClockOutputEnabled(MPU6050_t *dev);
void MPU6050_SetClockOutputEnabled(MPU6050_t *dev, bool enabled);

// INT_ENABLE register
uint8_t MPU6050_GetIntEnabled(MPU6050_t *dev);
void MPU6050_SetIntEnabled(MPU6050_t *dev, uint8_t enabled);
bool MPU6050_GetIntFreefallEnabled(MPU6050_t *dev);
void MPU6050_SetIntFreefallEnabled(MPU6050_t *dev, bool enabled);
bool MPU6050_GetIntMotionEnabled(MPU6050_t *dev);
void MPU6050_SetIntMotionEnabled(MPU6050_t *dev, bool enabled);
bool MPU6050_GetIntZeroMotionEnabled(MPU6050_t *dev);
void MPU6050_SetIntZeroMotionEnabled(MPU6050_t *dev, bool enabled);
bool MPU6050_GetIntFIFOBufferOverflowEnabled(MPU6050_t *dev);
void MPU6050_SetIntFIFOBufferOverflowEnabled(MPU6050_t *dev, bool enabled);
bool MPU6050_GetIntI2CMasterEnabled(MPU6050_t *dev);
void MPU6050_SetIntI2CMasterEnabled(MPU6050_t *dev, bool enabled);
bool MPU6050_GetIntDataReadyEnabled(MPU6050_t *dev);
void MPU6050_SetIntDataReadyEnabled(MPU6050_t *dev, bool enabled);

// INT_STATUS register
uint8_t MPU6050_GetIntStatus(MPU6050_t *dev);
bool MPU6050_GetIntFreefallStatus(MPU6050_t *dev);
bool MPU6050_GetIntMotionStatus(MPU6050_t *dev);
bool MPU6050_GetIntZeroMotionStatus(MPU6050_t *dev);
bool MPU6050_GetIntFIFOBufferOverflowStatus(MPU6050_t *dev);
bool MPU6050_GetIntI2CMasterStatus(MPU6050_t *dev);
bool MPU6050_GetIntDataReadyStatus(MPU6050_t *dev);

// ACCEL_*OUT_* registers
void MPU6050_GetRawMotion6(MPU6050_t *dev, int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz);
void MPU6050_GetMotion6(MPU6050_t *dev, float* ax, float* ay, float* az, float* gx, float* gy, float* gz);
void MPU6050_GetRawAcceleration(MPU6050_t *dev, int16_t* x, int16_t* y, int16_t* z);
void MPU6050_GetAcceleration(MPU6050_t *dev, float* x, float* y, float* z);
int16_t MPU6050_GetAccelerationX(MPU6050_t *dev);
int16_t MPU6050_GetAccelerationY(MPU6050_t *dev);
int16_t MPU6050_GetAccelerationZ(MPU6050_t *dev);

// TEMP_OUT_* registers
int16_t MPU6050_GetRawTemperature(MPU6050_t *dev);
float MPU6050_GetTemperature(MPU6050_t *dev);

// GYRO_*OUT_* registers
void MPU6050_GetRawRotation(MPU6050_t *dev, int16_t* x, int16_t* y, int16_t* z);
void MPU6050_GetRotation(MPU6050_t *dev, float *x, float *y, float *z);
int16_t MPU6050_GetRotationX(MPU6050_t *dev);
int16_t MPU6050_GetRotationY(MPU6050_t *dev);
int16_t MPU6050_GetRotationZ(MPU6050_t *dev);

// EXT_SENS_DATA_* registers
uint8_t MPU6050_GetExternalSensorByte(MPU6050_t *dev, int position);
uint16_t MPU6050_GetExternalSensorWord(MPU6050_t *dev, int position);
uint32_t GetExternalSensorDWord(MPU6050_t *dev, int position);

// MOT_DETECT_STATUS register
bool MPU6050_GetXNegMotionDetected(MPU6050_t *dev);
bool MPU6050_GetXPosMotionDetected(MPU6050_t *dev);
bool MPU6050_GetYNegMotionDetected(MPU6050_t *dev);
bool MPU6050_GetYPosMotionDetected(MPU6050_t *dev);
bool MPU6050_GetZNegMotionDetected(MPU6050_t *dev);
bool MPU6050_GetZPosMotionDetected(MPU6050_t *dev);
bool MPU6050_GetZeroMotionDetected(MPU6050_t *dev);

// I2C_SLV*_DO register
void MPU6050_SetSlaveOutputByte(MPU6050_t *dev, uint8_t num, uint8_t data);

// I2C_MST_DELAY_CTRL register
bool MPU6050_GetExternalShadowDelayEnabled(MPU6050_t *dev);
void MPU6050_SetExternalShadowDelayEnabled(MPU6050_t *dev, bool enabled);
bool MPU6050_GetSlaveDelayEnabled(MPU6050_t *dev, uint8_t num);
void MPU6050_SetSlaveDelayEnabled(MPU6050_t *dev, uint8_t num, bool enabled);

// SIGNAL_PATH_RESet register
void MPU6050_ResetGyroscopePath(MPU6050_t *dev);
void MPU6050_ResetAccelerometerPath(MPU6050_t *dev);
void MPU6050_ResetTemperaturePath(MPU6050_t *dev);

// MOT_DETECT_CTRL register
uint8_t MPU6050_GetAccelerometerPowerOnDelay(MPU6050_t *dev);
void MPU6050_SetAccelerometerPowerOnDelay(MPU6050_t *dev, uint8_t delay);
uint8_t MPU6050_GetFreefallDetectionCounterDecrement(MPU6050_t *dev);
void MPU6050_SetFreefallDetectionCounterDecrement(MPU6050_t *dev, uint8_t decrement);
uint8_t MPU6050_GetMotionDetectionCounterDecrement(MPU6050_t *dev);
void MPU6050_SetMotionDetectionCounterDecrement(MPU6050_t *dev, uint8_t decrement);

// USER_CTRL register
bool MPU6050_GetFIFOEnabled(MPU6050_t *dev);
void MPU6050_SetFIFOEnabled(MPU6050_t *dev, bool enabled);
void MPU6050_ResetFIFO(MPU6050_t *dev);
bool MPU6050_GetI2CMasterModeEnabled(MPU6050_t *dev);
void MPU6050_SetI2CMasterModeEnabled(MPU6050_t *dev, bool enabled);
void MPU6050_SwitchSPIEnabled(MPU6050_t *dev, bool enabled);
void MPU6050_ResetI2CMaster(MPU6050_t *dev);
void MPU6050_ResetSensors(MPU6050_t *dev);

// PWR_MGMT_1 register
void MPU6050_Reset(MPU6050_t *dev);
bool MPU6050_GetSleepEnabled(MPU6050_t *dev);
void MPU6050_SetSleepEnabled(MPU6050_t *dev, bool enabled);
bool MPU6050_GetWakeCycleEnabled(MPU6050_t *dev);
void MPU6050_SetWakeCycleEnabled(MPU6050_t *dev, bool enabled);
bool MPU6050_GetTempSensorEnabled(MPU6050_t *dev);
void MPU6050_SetTempSensorEnabled(MPU6050_t *dev, bool enabled);
uint8_t MPU6050_GetClockSource(MPU6050_t *dev);
void MPU6050_SetClockSource(MPU6050_t *dev, MPU6050_CLOCKS source);

// PWR_MGMT_2 register
uint8_t MPU6050_GetWakeFrequency(MPU6050_t *dev);
void MPU6050_SetWakeFrequency(MPU6050_t *dev, uint8_t frequency);
bool MPU6050_GetStandbyXAccelEnabled(MPU6050_t *dev);
void MPU6050_SetStandbyXAccelEnabled(MPU6050_t *dev, bool enabled);
bool MPU6050_GetStandbyYAccelEnabled(MPU6050_t *dev);
void MPU6050_SetStandbyYAccelEnabled(MPU6050_t *dev, bool enabled);
bool MPU6050_GetStandbyZAccelEnabled(MPU6050_t *dev);
void MPU6050_SetStandbyZAccelEnabled(MPU6050_t *dev, bool enabled);
bool MPU6050_GetStandbyXGyroEnabled(MPU6050_t *dev);
void MPU6050_SetStandbyXGyroEnabled(MPU6050_t *dev, bool enabled);
bool MPU6050_GetStandbyYGyroEnabled(MPU6050_t *dev);
void MPU6050_SetStandbyYGyroEnabled(MPU6050_t *dev, bool enabled);
bool MPU6050_GetStandbyZGyroEnabled(MPU6050_t *dev);
void MPU6050_SetStandbyZGyroEnabled(MPU6050_t *dev, bool enabled);

// FIFO_COUNT_* registers
uint16_t MPU6050_GetFIFOCount(MPU6050_t *dev);

// FIFO_R_W register
uint8_t MPU6050_GetFIFOByte(MPU6050_t *dev);
void MPU6050_SetFIFOByte(MPU6050_t *dev, uint8_t data);
void MPU6050_GetFIFOBytes(MPU6050_t *dev, uint8_t *data, uint8_t length);

// WHO_AM_I register
uint8_t MPU6050_GetDeviceID(MPU6050_t *dev);
void MPU6050_SetDeviceID(MPU6050_t *dev, uint8_t id);

// ======== UNDOCUMENTED/DMP REGISTERS/METHODS ========

// XG_OFFS_TC register
uint8_t MPU6050_GetOTPBankValid(MPU6050_t *dev);
void MPU6050_SetOTPBankValid(MPU6050_t *dev, bool enabled);
int8_t MPU6050_GetXGyroOffsetTC(MPU6050_t *dev);
void MPU6050_SetXGyroOffsetTC(MPU6050_t *dev, int8_t offSet);

// YG_OFFS_TC register
int8_t MPU6050_GetYGyroOffsetTC(MPU6050_t *dev);
void MPU6050_SetYGyroOffsetTC(MPU6050_t *dev, int8_t offSet);

// ZG_OFFS_TC register
int8_t MPU6050_GetZGyroOffsetTC(MPU6050_t *dev);
void MPU6050_SetZGyroOffsetTC(MPU6050_t *dev, int8_t offSet);

// X_FINE_GAIN register
int8_t MPU6050_GetXFineGain(MPU6050_t *dev);
void MPU6050_SetXFineGain(MPU6050_t *dev, int8_t gain);

// Y_FINE_GAIN register
int8_t MPU6050_GetYFineGain(MPU6050_t *dev);
void MPU6050_SetYFineGain(MPU6050_t *dev, int8_t gain);

// Z_FINE_GAIN register
int8_t MPU6050_GetZFineGain(MPU6050_t *dev);
void MPU6050_SetZFineGain(MPU6050_t *dev, int8_t gain);

// XA_OFFS_* registers
int16_t MPU6050_GetXAccelOffset(MPU6050_t *dev);
void MPU6050_SetXAccelOffset(MPU6050_t *dev, int16_t offSet);

// YA_OFFS_* register
int16_t MPU6050_GetYAccelOffset(MPU6050_t *dev);
void MPU6050_SetYAccelOffset(MPU6050_t *dev, int16_t offSet);

// ZA_OFFS_* register
int16_t MPU6050_GetZAccelOffset(MPU6050_t *dev);
void MPU6050_SetZAccelOffset(MPU6050_t *dev, int16_t offSet);

// XG_OFFS_USR* registers
int16_t MPU6050_GetXGyroOffset(MPU6050_t *dev);
void MPU6050_SetXGyroOffset(MPU6050_t *dev, int16_t offSet);

// YG_OFFS_USR* register
int16_t MPU6050_GetYGyroOffset(MPU6050_t *dev);
void MPU6050_SetYGyroOffset(MPU6050_t *dev, int16_t offSet);

// ZG_OFFS_USR* register
int16_t MPU6050_GetZGyroOffset(MPU6050_t *dev);
void MPU6050_SetZGyroOffset(MPU6050_t *dev, int16_t offSet);

void MPU6050_GetOffset(MPU6050_t *dev, int16_t *offSet);
void MPU6050_SetOffset(MPU6050_t *dev, int16_t *offSet);

// INT_ENABLE register (DMP functions)
bool MPU6050_GetIntPLLReadyEnabled(MPU6050_t *dev);
void MPU6050_SetIntPLLReadyEnabled(MPU6050_t *dev, bool enabled);
bool MPU6050_GetIntDMPEnabled(MPU6050_t *dev);
void MPU6050_SetIntDMPEnabled(MPU6050_t *dev, bool enabled);

// DMP_INT_STATUS
bool MPU6050_GetDMPInt5Status(MPU6050_t *dev);
bool MPU6050_GetDMPInt4Status(MPU6050_t *dev);
bool MPU6050_GetDMPInt3Status(MPU6050_t *dev);
bool MPU6050_GetDMPInt2Status(MPU6050_t *dev);
bool MPU6050_GetDMPInt1Status(MPU6050_t *dev);
bool MPU6050_GetDMPInt0Status(MPU6050_t *dev);

// INT_STATUS register (DMP functions)
bool MPU6050_GetIntPLLReadyStatus(MPU6050_t *dev);
bool MPU6050_GetIntDMPStatus(MPU6050_t *dev);

// USER_CTRL register (DMP functions)
bool MPU6050_GetDMPEnabled(MPU6050_t *dev);
void MPU6050_SetDMPEnabled(MPU6050_t *dev, bool enabled);
void MPU6050_ResetDMP(MPU6050_t *dev);

// BANK_SEL register
void MPU6050_SetMemoryBank(MPU6050_t *dev, uint8_t bank, bool prefetchEnabled, bool userBank);

// MEM_START_ADDR register
void MPU6050_SetMemoryStartAddress(MPU6050_t *dev, uint8_t address);

// MEM_R_W register
uint8_t MPU6050_ReadMemoryByte(MPU6050_t *dev);
void MPU6050_WriteMemoryByte(MPU6050_t *dev, uint8_t data);
void MPU6050_ReadMemoryBlock(MPU6050_t *dev, uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address);
/*
bool MPU6050_WriteMemoryBlock(MPU6050_t *dev, const uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address, bool verify, bool useProgMem);
bool MPU6050_WriteProgMemoryBlock(MPU6050_t *dev, const uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address, bool verify);

bool MPU6050_WriteDMPConfigurationSet(MPU6050_t *dev, const uint8_t *data, uint16_t dataSize, bool useProgMem);
bool MPU6050_WriteProgDMPConfigurationSet(MPU6050_t *dev, const uint8_t *data, uint16_t dataSize);
*/

// DMP_CFG_1 register
uint8_t MPU6050_GetDMPConfig1(MPU6050_t *dev);
void MPU6050_SetDMPConfig1(MPU6050_t *dev, uint8_t config);

// DMP_CFG_2 register
uint8_t MPU6050_GetDMPConfig2(MPU6050_t *dev);
void MPU6050_SetDMPConfig2(MPU6050_t *dev, uint8_t config);

MPU6050_Status MPU6050_AverageCalibration(MPU6050_t *dev);

#endif