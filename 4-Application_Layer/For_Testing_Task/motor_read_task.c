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
 *   - 周期开始时，发送三个电机的参数读取请求
 *   - 由于电机响应时间仅 1μs << 10ms，可在同周期内接收所有回包
 *   - 读取的数据存入队列供其他任务使用
 *
 * 运行频率：100 Hz（10 ms）
 */

#include "motor_read_task.h"
#include "bsp_usart.h"
#include "robot_task.h"

/* 电机ID数组 */
static const uint8_t motor_ids[MOTOR_READ_COUNT] = {
    MOTOR_READ_ELE_ID_1,
    MOTOR_READ_ELE_ID_2,
    MOTOR_READ_ELE_ID_3
};

typedef enum
{
    READ_STATE_ANGLE       = 0,  /* 读取角度 */
    READ_STATE_TORQUE      = 1,  /* 读取扭矩 */
    READ_STATE_TEMPERATURE = 2,  /* 读取温度 */
} Read_state_t;

void Motor_read_task(void const *argument)
{
    TickType_t last = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(MOTOR_READ_PERIOD_MS);

    Ele_motor_feedback_t fb;
    float param_value = 0.0f;
    uint8_t is_param = 0U;
    uint8_t motor_idx;

    Motor_read_data_t motor_data;
    Read_state_t read_state = READ_STATE_ANGLE;

    uint32_t rx_start_tick;

    (void)argument;

    /* 初始化motor_data */
    for (motor_idx = 0; motor_idx < MOTOR_READ_COUNT; motor_idx++)
    {
        motor_data.motor[motor_idx].angle = 0.0f;
        motor_data.motor[motor_idx].torque = 0.0f;
        motor_data.motor[motor_idx].temperature = 0.0f;
    }
    motor_data.timestamp = 0;

    /* 初始化所有电机 */
    for (motor_idx = 0; motor_idx < MOTOR_READ_COUNT; motor_idx++)
    {
        Ele_motor_init(motor_ids[motor_idx]);
    }

    for (;;)
    {
        /* ===== 第一步 & 第二步：逐电机发请求、等回包，避免多帧并发覆盖 ===== */
        for (motor_idx = 0; motor_idx < MOTOR_READ_COUNT; motor_idx++)
        {
            /* 发送当前电机的读取请求 */
            switch (read_state)
            {
                case READ_STATE_ANGLE:
                    Ele_motor_param_rw(0.0f, 0U, ELE_MOTOR_OR_ANGLE, motor_ids[motor_idx]);
                    break;

                case READ_STATE_TORQUE:
                    Ele_motor_param_rw(0.0f, 0U, ELE_MOTOR_OR_TORQUE, motor_ids[motor_idx]);
                    break;

                case READ_STATE_TEMPERATURE:
                    Ele_motor_param_rw(0.0f, 0U, ELE_MOTOR_OR_TEMPERATURE, motor_ids[motor_idx]);
                    break;

                default:
                    break;
            }

            /* 等待该电机回包，超时 5ms */
            rx_start_tick = HAL_GetTick();
            while ((HAL_GetTick() - rx_start_tick) < 5U)
            {
                if (Ele_motor_fetch_rx(&fb, &param_value, &is_param) && is_param
                    && fb.id == motor_ids[motor_idx])
                {
                    switch (read_state)
                    {
                        case READ_STATE_ANGLE:
                            motor_data.motor[motor_idx].angle = param_value;
                            break;

                        case READ_STATE_TORQUE:
                            motor_data.motor[motor_idx].torque = param_value;
                            break;

                        case READ_STATE_TEMPERATURE:
                            motor_data.motor[motor_idx].temperature = param_value;
                            break;

                        default:
                            break;
                    }
                    break;  /* 收到该电机回包，处理下一台 */
                }
            }
        }

        /* ===== 第三步：检查是否所有参数都已读取 ===== */
        if (read_state == READ_STATE_TEMPERATURE)
        {
            /* 三个参数都已读取，发送到队列 */
            motor_data.timestamp = xTaskGetTickCount();
            xQueueOverwrite(Motor_read_queue_handle, &motor_data);

            /* 打印日志 */
            Uart_printf(test_uart, "read: M1[a=%.4f t=%.3f T=%.1f] M2[a=%.4f t=%.3f T=%.1f] M3[a=%.4f t=%.3f T=%.1f]\r\n",
                        motor_data.motor[0].angle, motor_data.motor[0].torque, motor_data.motor[0].temperature,
                        motor_data.motor[1].angle, motor_data.motor[1].torque, motor_data.motor[1].temperature,
                        motor_data.motor[2].angle, motor_data.motor[2].torque, motor_data.motor[2].temperature);
        }


        /* ===== 第四步：状态转移 ===== */
        read_state = (Read_state_t)((read_state + 1) % 3);

        vTaskDelayUntil(&last, period);
    }
}
