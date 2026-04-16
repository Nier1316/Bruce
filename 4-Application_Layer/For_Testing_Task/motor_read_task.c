/**
 * @file    motor_read_task.c
 * @brief   多电机参数读取任务
 *
 * 在同一个 10ms 周期内，读取三个电机（ID=1,2,3）的参数：
 *   1. 角度 (angle)
 *   2. 扭矩 (torque)
 *   3. 温度 (temperature)
 *
 * 工作流程：
 *   - 对每台电机依次发送三种参数的读取请求，每次等待回包最多 0.5ms
 *   - 三台电机全部读完后统一写入队列
 *   - 最坏耗时：3 × 3 × 0.5ms = 4.5ms，在 10ms 周期内完成
 *
 * 运行频率：100 Hz（10 ms）
 */

#include "motor_read_task.h"
#include "bsp_usart.h"
#include "robot_task.h"
#include "bsp_dwt.h"

/* 电机ID数组 */
static const uint8_t motor_ids[MOTOR_READ_COUNT] = {
    MOTOR_READ_ELE_ID_1,
    MOTOR_READ_ELE_ID_2,
    MOTOR_READ_ELE_ID_3
};

/* 0.5ms 对应的 DWT 计数周期数 */
#define TIMEOUT_HALF_MS_CYCLES  (SystemCoreClock / 2000U)

/**
 * @brief 向指定电机发送读请求，并在 0.5ms 内等待有效回包
 * @param param_id  要读取的参数索引（ELE_MOTOR_OR_xxx）
 * @param motor_id  目标电机 ID
 * @param out       成功时写入参数值，超时则保持原值不变
 * @return 1 = 收到有效回包，0 = 超时
 */
static uint8_t read_param(uint16_t param_id, uint8_t motor_id, float *out)
{
    Ele_motor_feedback_t fb;
    float val = 0.0f;
    uint8_t is_param = 0U;

    Ele_motor_param_rw(0.0f, 0U, param_id, motor_id);

    uint32_t t0 = DWT->CYCCNT;
    while ((DWT->CYCCNT - t0) < TIMEOUT_HALF_MS_CYCLES)
    {
        if (Ele_motor_fetch_rx(&fb, &val, &is_param) && is_param && fb.id == motor_id)
        {
            *out = val;
            return 1U;
        }
    }
    return 0U;
}

void Motor_read_task(void const *argument)
{
    TickType_t last = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(MOTOR_READ_PERIOD_MS);

    Motor_read_data_t motor_data;
    uint8_t motor_idx;

    (void)argument;

    /* 初始化 motor_data */
    for (motor_idx = 0; motor_idx < MOTOR_READ_COUNT; motor_idx++)
    {
        motor_data.motor[motor_idx].angle        = 0.0f;
        motor_data.motor[motor_idx].torque       = 0.0f;
        motor_data.motor[motor_idx].temperature  = 0.0f;
        motor_data.motor[motor_idx].control_mode = 0U;
    }
    motor_data.timestamp = 0U;

    /* 初始化所有电机 */
    for (motor_idx = 0; motor_idx < MOTOR_READ_COUNT; motor_idx++)
    {
        Ele_motor_init(motor_ids[motor_idx]);
    }

    for (;;)
    {
        /* 对每台电机依次读取四种参数，每次最多等 0.5ms */
        for (motor_idx = 0; motor_idx < MOTOR_READ_COUNT; motor_idx++)
        {
            float mode_float = 0.0f;
            read_param(ELE_MOTOR_OR_ANGLE,       motor_ids[motor_idx], &motor_data.motor[motor_idx].angle);
            read_param(ELE_MOTOR_OR_TORQUE,      motor_ids[motor_idx], &motor_data.motor[motor_idx].torque);
            read_param(ELE_MOTOR_OR_TEMPERATURE, motor_ids[motor_idx], &motor_data.motor[motor_idx].temperature);
            if (read_param(ELE_MOTOR_WR_CONTROL_MODE, motor_ids[motor_idx], &mode_float))
            {
                motor_data.motor[motor_idx].control_mode = (uint8_t)mode_float;
            }
        }

        /* 三台电机均已读完，更新时间戳并推送队列 */
        motor_data.timestamp = xTaskGetTickCount();
        xQueueOverwrite(Motor_read_queue_handle, &motor_data);

        Uart_printf(test_uart,
            "read: M1[a=%.4f t=%.3f T=%.1f mode=%u] M2[a=%.4f t=%.3f T=%.1f mode=%u] M3[a=%.4f t=%.3f T=%.1f mode=%u]\r\n",
            motor_data.motor[0].angle, motor_data.motor[0].torque, motor_data.motor[0].temperature, motor_data.motor[0].control_mode,
            motor_data.motor[1].angle, motor_data.motor[1].torque, motor_data.motor[1].temperature, motor_data.motor[1].control_mode,
            motor_data.motor[2].angle, motor_data.motor[2].torque, motor_data.motor[2].temperature, motor_data.motor[2].control_mode);

        vTaskDelayUntil(&last, period);
    }
}
