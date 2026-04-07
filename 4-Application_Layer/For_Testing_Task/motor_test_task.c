#include "motor_test_task.h"
#include "robot_task.h"
#include "ele_motor.h"
#include <math.h>

/**
 * @brief 电机力矩正弦测试任务
 *
 * 使用阻抗模式（kp=kd=0）实现纯力矩前馈控制，
 * 向电机持续输出正弦波形力矩指令，并通过串口打印反馈数据。
 *
 * 正弦参数：
 *   幅值  MOTOR_TEST_TORQUE_AMP = 0.5 Nm
 *   频率  MOTOR_TEST_FREQ_HZ    = 0.5 Hz（周期 2 s）
 *   控制周期 MOTOR_TEST_PERIOD_MS = 10 ms（100 Hz）
 */
void Motor_test_task(void const *argument)
{
    TickType_t last = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(MOTOR_TEST_PERIOD_MS);          /* 控制周期 */
    const float dt    = MOTOR_TEST_PERIOD_MS * 0.001f;                      /* 时间步长 (s) */
//    const float amp   = MOTOR_TEST_TORQUE_AMP;                              /* 力矩幅值 (Nm) */
//    const float omega = 2.0f * 3.1415926f * MOTOR_TEST_FREQ_HZ;            /* 角频率 (rad/s) */
    float t = 0.0f;                                /* 累计时间 (s) */

    Ele_motor_feedback_t fb;
    float param_value = 0.0f;
    uint8_t is_param  = 0U;

    (void)argument;

    /* 初始化电机并切换到阻抗控制模式 */
    Ele_motor_init(MOTOR_TEST_ELE_ID);
    Ele_motor_write_control_mode(MOTOR_TEST_ELE_ID, ELE_MOTOR_MODE_IMPEDANCE);

    for (;;)
    {
        /* 读取电机反馈并通过串口输出 */
        if (Ele_motor_fetch_rx(&fb, &param_value, &is_param))
        {
            if (is_param)
            {
                /* 参数读写回包 */
                Uart_printf(test_uart, "ele id=%d param=%.3f\r\n", MOTOR_TEST_ELE_ID, param_value);
            }
//            else
//            {
//                /* 控制反馈回包：位置、速度、力矩 */
//                Uart_printf(test_uart, "ele id=%d p=%.3f v=%.3f tq=%.3f\r\n",
//                            fb.id, fb.position, fb.velocity, fb.torque);
//            }
        }

//        /* 计算当前时刻的正弦力矩目标值 */
//        float torque = amp * sinf(omega * t);
//
//        /* 发送力矩指令：阻抗模式下令 kp=kd=0，仅保留力矩前馈分量
//         * τ_cmd = kp*(p_ref-p) + kd*(v_ref-v) + τ_ff  →  τ_cmd = τ_ff */
//        Ele_motor_set_para(0.0f, 0.0f, 0.0f, 0.0f, torque, ELE_MOTOR_MODE_IMPEDANCE, MOTOR_TEST_ELE_ID);

        t += dt;
        vTaskDelayUntil(&last, period); /* 严格等周期执行 */
    }
}
