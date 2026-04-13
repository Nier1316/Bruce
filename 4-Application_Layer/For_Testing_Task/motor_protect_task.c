/**
 * @file    motor_protect_task.c
 * @brief   电机角度保护任务
 *
 * 首次获取到角度反馈时记录初始角度 Angle_initial，
 * 此后持续监控：
 *   正向偏移 > MOTOR_PROTECT_ANGLE_MAX    → 立即失能电机
 *   反向偏移 > MOTOR_PROTECT_ANGLE_DE_MAX → 立即失能电机
 *
 * 运行频率：100 Hz（10 ms）
 */

#include "motor_protect_task.h"
#include "robot_task.h"

typedef enum
{
    PROTECT_STATE_INIT    = 0,  /* 等待首次角度回包，记录初始角度 */
    PROTECT_STATE_RUNNING = 1,  /* 正常监控中 */
    PROTECT_STATE_FAULT   = 2,  /* 已触发保护，电机已失能 */
} Protect_state_t;

void Motor_protect_task(void const *argument)
{
    TickType_t last = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(MOTOR_PROTECT_PERIOD_MS);

    Ele_motor_feedback_t fb;
    float        param_value  = 0.0f;
    uint8_t      is_param     = 0U;
    float        Angle_initial = 0.0f;
    Protect_state_t state     = PROTECT_STATE_INIT;

    (void)argument;

    Ele_motor_init(MOTOR_PROTECT_ELE_ID);

    for (;;)
    {
        if (state != PROTECT_STATE_FAULT)
        {
            /* 每周期发送机械角度读取请求，电机下一周期回传参数帧 */
            Ele_motor_param_rw(0.0f, 0U, ELE_MOTOR_OR_ANGLE, MOTOR_PROTECT_ELE_ID);
        }

        /* 读取上一周期的回包 */
        if (Ele_motor_fetch_rx(&fb, &param_value, &is_param) && is_param)
        {
            float angle_now = param_value;

            switch (state)
            {
                case PROTECT_STATE_INIT:
                    /* 首次收到角度，记录为初始值 */
                    Angle_initial = angle_now;
                    state = PROTECT_STATE_RUNNING;
                    Uart_printf(test_uart, "protect: init angle=%.4f rad\r\n", Angle_initial);
                    break;

                case PROTECT_STATE_RUNNING:
                {
                    float delta = angle_now - Angle_initial;

                    if (delta > MOTOR_PROTECT_ANGLE_MAX)
                    {
                        Ele_motor_stop(MOTOR_PROTECT_ELE_ID);
                        state = PROTECT_STATE_FAULT;
                        Uart_printf(test_uart,
                            "protect: FAULT(+) angle=%.4f delta=+%.4f > MAX=%.4f\r\n",
                            angle_now, delta, MOTOR_PROTECT_ANGLE_MAX);
                    }
                    else if (delta < -MOTOR_PROTECT_ANGLE_DE_MAX)
                    {
                        Ele_motor_stop(MOTOR_PROTECT_ELE_ID);
                        state = PROTECT_STATE_FAULT;
                        Uart_printf(test_uart,
                            "protect: FAULT(-) angle=%.4f delta=%.4f < -DE_MAX=%.4f\r\n",
                            angle_now, delta, -MOTOR_PROTECT_ANGLE_DE_MAX);
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
