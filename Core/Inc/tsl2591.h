/**
 ******************************************************************************
 * @file    tsl2591.h
 * @brief   Header for TSL2591 Light Sensor Driver & FreeRTOS Interface Types
 *
 *          Implements the generic "6-point" driver pattern:
 *            ① Acquire bus_mutex (timeout)
 *            ② Validate args + build transaction descriptor (static, no malloc)
 *            ③ Start async transfer (DMA) — non-blocking
 *            ④ Block on completion signal (+timeout) — YIELD POINT
 *            ⑤ Check status + error recovery
 *            ⑥ Release bus_mutex — always, return Drv_Status_t
 ******************************************************************************
 */
#ifndef __TSL2591_H
#define __TSL2591_H

#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* ── LAYER 1: HARDWARE REGISTER DEFINITIONS ────────────────────────────── */
#define TSL2591_I2C_ADDR         (0x29 << 1)

#define TSL2591_CMD              0xA0
#define TSL2591_REG_ENABLE       (TSL2591_CMD | 0x00)
#define TSL2591_REG_CONTROL      (TSL2591_CMD | 0x01)
#define TSL2591_REG_DATA0LOW     (TSL2591_CMD | 0x14)

#define TSL2591_ENABLE_POWERON   0x03    /* PON | AEN */
#define TSL2591_GAIN_MED         0x10    /* medium gain, 100 ms integration */

#define LIGHT_THRESHOLD          2000u
#define SENSOR_POLL_MS           200u

/* ── DRIVER-LAYER TIMEOUTS ──────────────────────────────────────────────── */
#define TSL2591_BUS_MUTEX_TIMEOUT_MS   50u   /* step ① acquire timeout       */
#define TSL2591_XFER_TIMEOUT_MS       100u   /* step ④ completion timeout    */

/* ── LAYER 2: TYPE DEFINITIONS & OBJECT STRUCTS ────────────────────────── */
typedef uint16_t TSL2591_RawData_t;

/* Generic Drv_Status_t equivalent for this peripheral */
typedef enum {
    TSL2591_OK      = 0,
    TSL2591_ERROR   = 1,
    TSL2591_INVALID = 2,
    TSL2591_TIMEOUT = 3,
    TSL2591_BUSY    = 4
} TSL2591_Status_t;

/* Driver handle — opaque to application layer, statically allocated.
 * Owns the FreeRTOS primitives (bus_mutex + completion signal) per the
 * "Driver Layer" box in the architecture diagram. */
typedef struct {
    I2C_HandleTypeDef *hi2c;        /* Assigned HAL I2C bus handle pointer */
    uint8_t            devAddr;     /* 8-bit left-shifted slave address    */
    uint8_t            initialized; /* Flips to 1 when init completes      */

    /* FreeRTOS Primitives (right-hand box in diagram) */
    SemaphoreHandle_t  busMutex;     /* ① / ⑥ : priority-inheriting mutex  */
    TaskHandle_t       xTaskToNotify;/* ④ : task awaiting completion       */

    /* ② Transaction descriptor — static buffers, no malloc */
    uint8_t            regAddr;
    uint8_t            rxBuf[2];

    /* ⑤ Error recovery bookkeeping */
    volatile HAL_StatusTypeDef lastHalStatus;
    TSL2591_Status_t           lastError;
} TSL2591_Handle_t;

/* RTOS Inter-Task Queue Message structure (Application Layer payload) */
typedef struct {
    TSL2591_RawData_t   raw;         /* Channel 0 raw light counts          */
    uint32_t            timestamp;   /* FreeRTOS system OS Tick value       */
} LightSensorMsg_t;

/* ── DRIVER API FUNCTION PROTOTYPES ────────────────────────────────────── */
TSL2591_Status_t TSL2591_Driver_Init(TSL2591_Handle_t *drv, I2C_HandleTypeDef *hi2c, uint8_t devAddr);

/* Performs the full ①-⑥ transaction. Safe to call from any priority task;
 * blocks (yields) at step ④, never spins. */
TSL2591_Status_t TSL2591_Driver_ReadLight(TSL2591_Handle_t *drv, TSL2591_RawData_t *out);

#endif /* __TSL2591_H */
