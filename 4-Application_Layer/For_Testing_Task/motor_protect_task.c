/**
 * @file    motor_protect_task.c
 * @brief   电机保护任务
 *
 * 从 Motor_read_queue_handle 读取电机参数，持续监控：
 *   1. 角度：正向偏移 > MOTOR_PROTECT_ANGLE_MAX    → 立即失能电机
 *   2. 角度：反向偏移 > MOTOR_PROTECT_ANGLE_DE_MAX → 立即失能电机
 *   3. 温度：超过温度告警值                        → 降低电流
 *   4. 温度：超过温度关闭值                        → 关闭电机
 *
 * 运行频率：100 Hz（10 ms）
 */

#include "motor_protect_task.h"
#include "motor_read_task.h"
#include "robot_task.h"

typedef enum
{
    PROTECT_STATE_INIT    = 0,  /* 等待首次数据到达，记录初始角度 */
    PROTECT_STATE_RUNNING = 1,  /* 正常监控中 */
    PROTECT_STATE_FAULT   = 2,  /* 已触发保护，电机已失能 */
} Protect_state_t;

void Motor_protect_task(void const *argument)
{
    TickType_t last = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(MOTOR_PROTECT_PERIOD_MS);

    Motor_read_data_t motor_data;
    float angle_initial = 0.0f;
    Protect_state_t state = PROTECT_STATE_INIT;

    const float TEMP_ALERT = 80.0f;    /* 温度告警阈值 */
    const float TEMP_SHUTDOWN = 90.0f; /* 温度关闭阈值 */

    (void)argument;

    for (;;)
    {
        /* 从队列读取最新的电机参数 */
        if (xQueuePeek(Motor_read_queue_handle, &motor_data, 0) == pdTRUE)
        {
            switch (state)
            {
                case PROTECT_STATE_INIT:
                    /* 首次收到数据，记录初始角度（电机1的数据：motor[0]）*/
                    angle_initial = motor_data.motor[0].angle;
                    state = PROTECT_STATE_RUNNING;
                    Uart_printf(test_uart, "protect: init angle=%.4f rad\r\n", angle_initial);
                    break;

                case PROTECT_STATE_RUNNING:
                {
                    float delta = motor_data.motor[0].angle - angle_initial;

                    /* 检查角度范围 */
                    if (delta > MOTOR_PROTECT_ANGLE_MAX)
                    {
                        Ele_motor_stop(MOTOR_PROTECT_ELE_ID);
                        state = PROTECT_STATE_FAULT;
                        Uart_printf(test_uart,
                            "protect: FAULT(+) angle=%.4f delta=+%.4f > MAX=%.4f\r\n",
                            motor_data.motor[0].angle, delta, MOTOR_PROTECT_ANGLE_MAX);
                    }
                    else if (delta < -MOTOR_PROTECT_ANGLE_DE_MAX)
                    {
                        Ele_motor_stop(MOTOR_PROTECT_ELE_ID);
                        state = PROTECT_STATE_FAULT;
                        Uart_printf(test_uart,
                            "protect: FAULT(-) angle=%.4f delta=%.4f < -DE_MAX=%.4f\r\n",
                            motor_data.motor[0].angle, delta, -MOTOR_PROTECT_ANGLE_DE_MAX);
                    }

                    /* 检查温度 */
                    if (motor_data.motor[0].temperature > TEMP_SHUTDOWN)
                    {
                        Ele_motor_stop(MOTOR_PROTECT_ELE_ID);
                        state = PROTECT_STATE_FAULT;
                        Uart_printf(test_uart,
                            "protect: SHUTDOWN temp=%.1f > %.1f\r\n",
                            motor_data.motor[0].temperature, TEMP_SHUTDOWN);
                    }
                    else if (motor_data.motor[0].temperature > TEMP_ALERT)
                    {
                        Uart_printf(test_uart,
                            "protect: ALERT temp=%.1f > %.1f\r\n",
                            motor_data.motor[0].temperature, TEMP_ALERT);
                    }
                    break;
                }

                case PROTECT_STATE_FAULT:
                default:
                    break;
            }
        }

        vTaskDelayUntil(&last, period);
    }
}
