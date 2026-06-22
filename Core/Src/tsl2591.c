/**
 ******************************************************************************
 * @file    tsl2591.c
 * @brief   TSL2591 Driver Core implementation (I2C, DMA + task notification)
 *
 *          TSL2591_Driver_ReadLight() implements the full driver-layer
 *          transaction exactly as numbered in the architecture diagram:
 *
 *            ① Acquire bus_mutex (timeout)
 *            ② Validate args + build transaction descriptor
 *            ③ Start async transfer (DMA) — non-blocking
 *            ④ Block on completion signal (+timeout)  ★ YIELD POINT
 *            ⑤ Check status + error recovery
 *            ⑥ Release bus_mutex — always — return Drv_Status_t
 ******************************************************************************
 */
#include "tsl2591.h"

/* Routes the HAL ISR-context callbacks below back to the owning handle.
 * For a single I2C1 / single-sensor system one pointer is sufficient.
 * If multiple TSL2591 instances ever share different I2C buses, replace
 * this with a small lookup table keyed on hi2c->Instance. */
static TSL2591_Handle_t *s_activeDrv = NULL;

/* ── PRIVATE: BLOCKING WRITE — init-time only, before scheduler runs ────── */
static TSL2591_Status_t _drv_write_blocking(TSL2591_Handle_t *drv, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    HAL_StatusTypeDef st = HAL_I2C_Master_Transmit(drv->hi2c, drv->devAddr, buf, 2, HAL_MAX_DELAY);
    return (st == HAL_OK) ? TSL2591_OK : TSL2591_ERROR;
}

/* ── PUBLIC DRIVER EXPOSED INTERFACES ──────────────────────────────────── */

/**
 * @brief Initializes peripheral parameters and puts chip in standard execution mode.
 * @note  Runs before vTaskStartScheduler(), so blocking HAL calls are fine here.
 *        This is the ONLY place blocking I2C transmits are permitted.
 */
TSL2591_Status_t TSL2591_Driver_Init(TSL2591_Handle_t *drv, I2C_HandleTypeDef *hi2c, uint8_t devAddr)
{
    if (!drv || !hi2c) return TSL2591_INVALID;

    drv->hi2c          = hi2c;
    drv->devAddr       = devAddr;
    drv->initialized   = 0;
    drv->xTaskToNotify = NULL;
    drv->lastError     = TSL2591_OK;
    drv->lastHalStatus = HAL_OK;

    /* One mutex per bus instance, guards the full transaction (①/⑥) */
    drv->busMutex = xSemaphoreCreateMutex();
    if (drv->busMutex == NULL) return TSL2591_ERROR;

    /* Safe hardware settling time inside FreeRTOS environment context */
    vTaskDelay(pdMS_TO_TICKS(100));

    if (_drv_write_blocking(drv, TSL2591_REG_ENABLE,  TSL2591_ENABLE_POWERON) != TSL2591_OK)
        return TSL2591_ERROR;
    if (_drv_write_blocking(drv, TSL2591_REG_CONTROL, TSL2591_GAIN_MED) != TSL2591_OK)
        return TSL2591_ERROR;

    s_activeDrv      = drv; /* register handle for ISR routing below */
    drv->initialized = 1;
    return TSL2591_OK;
}

/**
 * @brief Captures the raw visual + infrared reading from Channel 0 data registers.
 *        Implements the full ①-⑥ driver transaction.
 */
TSL2591_Status_t TSL2591_Driver_ReadLight(TSL2591_Handle_t *drv, TSL2591_RawData_t *out)
{
    if (!drv || !drv->initialized || !out) return TSL2591_INVALID;

    TSL2591_Status_t result;

    /* ① Acquire bus_mutex (timeout) — serializes bus access,
     *    priority inheritance prevents inversion */
    if (xSemaphoreTake(drv->busMutex, pdMS_TO_TICKS(TSL2591_BUS_MUTEX_TIMEOUT_MS)) != pdTRUE)
    {
        return TSL2591_BUSY;
    }

    /* ② Validate args + build transaction descriptor.
     *    Static buffers live in the handle — no malloc. */
    drv->regAddr       = TSL2591_REG_DATA0LOW;
    drv->rxBuf[0]      = 0;
    drv->rxBuf[1]      = 0;
    drv->lastHalStatus = HAL_BUSY;

    /* Record the calling task so the ISR notifies exactly this task */
    drv->xTaskToNotify = xTaskGetCurrentTaskHandle();

    /* Drop any stale notification left over from a prior timeout/abort,
     * so step ④ cannot return immediately on a leftover signal. */
    (void)ulTaskNotifyValueClear(NULL, 0xFFFFFFFFu);

    /* ③ Start async transfer (DMA) — non-blocking, returns immediately.
     *    Reads 2 bytes starting at DATA0LOW via combined write+read. */
    HAL_StatusTypeDef hst = HAL_I2C_Mem_Read_DMA(drv->hi2c, drv->devAddr,
                                                  drv->regAddr, I2C_MEMADD_SIZE_8BIT,
                                                  drv->rxBuf, sizeof(drv->rxBuf));

    if (hst != HAL_OK)
    {
        /* Could not even start the transfer — nothing to wait on */
        drv->lastError = TSL2591_ERROR;
        result = TSL2591_ERROR;
    }
    else
    {
        /* ④ Block on completion signal (+timeout)  ★ YIELD POINT
         *    Task yields the CPU here. ISR signals on transfer-complete
         *    via vTaskNotifyGiveFromISR() — see callbacks below. */
        uint32_t notified = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(TSL2591_XFER_TIMEOUT_MS));

        /* ⑤ Check status + error recovery */
        if (notified == 0)
        {
            /* Timed out waiting for ISR — abort the in-flight transfer */
            HAL_I2C_Master_Abort_IT(drv->hi2c, drv->devAddr);
            drv->lastError = TSL2591_TIMEOUT;
            result = TSL2591_TIMEOUT;
        }
        else if (drv->lastHalStatus != HAL_OK)
        {
            drv->lastError = TSL2591_ERROR;
            result = TSL2591_ERROR;
        }
        else
        {
            *out = (TSL2591_RawData_t)((drv->rxBuf[1] << 8) | drv->rxBuf[0]);
            drv->lastError = TSL2591_OK;
            result = TSL2591_OK;
        }
    }

    drv->xTaskToNotify = NULL;

    /* ⑥ Release bus_mutex — ALWAYS, success or error, deadlock prevention */
    xSemaphoreGive(drv->busMutex);

    return result;
}

/* ── ISR CONTEXT — completion signal (right-hand "ISR Context" box) ─────
 *
 * Rules enforced here per the diagram:
 *   - only *FromISR() variants
 *   - no mutex operations in ISR (mutex is Take/Give in task context only)
 *   - error path sets lastHalStatus + signals, same as success path
 */

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (s_activeDrv == NULL || s_activeDrv->hi2c != hi2c) return;
    if (s_activeDrv->xTaskToNotify == NULL) return;

    s_activeDrv->lastHalStatus = HAL_OK;

    vTaskNotifyGiveFromISR(s_activeDrv->xTaskToNotify, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (s_activeDrv == NULL || s_activeDrv->hi2c != hi2c) return;
    if (s_activeDrv->xTaskToNotify == NULL) return;

    s_activeDrv->lastHalStatus = HAL_ERROR;

    vTaskNotifyGiveFromISR(s_activeDrv->xTaskToNotify, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
