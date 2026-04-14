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

    uint32_t received_count;
    uint32_t max_attempts;

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
        /* ===== 第一步：发送请求（同一周期内逐次发送三个电机的请求） ===== */
        for (motor_idx = 0; motor_idx < MOTOR_READ_COUNT; motor_idx++)
        {
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
        }

        /* ===== 第二步：接收回包（同一周期内接收所有三个电机的响应） ===== */
        received_count = 0;
        max_attempts = 100;  /* 防止无限循环 */

        while (received_count < MOTOR_READ_COUNT && max_attempts > 0)
        {
            if (Ele_motor_fetch_rx(&fb, &param_value, &is_param) && is_param)
            {
                /* 找到对应电机的索引 */
                for (motor_idx = 0; motor_idx < MOTOR_READ_COUNT; motor_idx++)
                {
                    if (fb.id == motor_ids[motor_idx])
                    {
                        /* 根据当前读取状态更新对应的参数 */
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
                        received_count++;
                        break;
                    }
                }
            }
            max_attempts--;
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
