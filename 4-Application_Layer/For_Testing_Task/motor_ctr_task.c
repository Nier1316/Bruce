#include "motor_ctr_task.h"
#include "motor_read_task.h"
#include "robot_task.h"
#include "ele_motor.h"
#include <math.h>

/**
 * @brief 电机力矩控制任务
 *
 * 使用阻抗模式（kp=kd=0）实现纯力矩前馈控制。
 * 电机反馈数据（角度、扭矩、温度）由 Motor_read_task 通过队列提供。
 *
 * 正弦参数：
 *   幅值  MOTOR_TEST_TORQUE_AMP = 0.5 Nm
 *   频率  MOTOR_TEST_FREQ_HZ    = 0.5 Hz（周期 2 s）
 *   控制周期 MOTOR_TEST_PERIOD_MS = 10 ms（100 Hz）
 */
void Motor_ctr_task(void const *argument)
{
    TickType_t last = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(MOTOR_TEST_PERIOD_MS);          /* 控制周期 */
    const float dt    = MOTOR_TEST_PERIOD_MS * 0.001f;                      /* 时间步长 (s) */
    const float amp   = MOTOR_TEST_TORQUE_AMP;                              /* 力矩幅值 (Nm) */
    const float omega = 2.0f * 3.1415926f * MOTOR_TEST_FREQ_HZ;            /* 角频率 (rad/s) */
    float t = 0.0f;                                                         /* 累计时间 (s) */

    Motor_read_data_t motor_data;
    uint32_t print_counter = 0;

    (void)argument;

    /* 初始化电机并切换到阻抗控制模式 */
    Ele_motor_init(MOTOR_TEST_ELE_ID);
    Ele_motor_write_control_mode(MOTOR_TEST_ELE_ID, ELE_MOTOR_MODE_IMPEDANCE);

    for (;;)
    {
        /* 从队列读取最新的电机参数（读取电机1的数据：motor[0]）*/
        if (xQueuePeek(Motor_read_queue_handle, &motor_data, 0) == pdTRUE)
        {
            print_counter++;
            /* 每 10 次读取（约 100ms）打印一次反馈数据 */
            if (print_counter >= 10)
            {
                Uart_printf(test_uart, "ctr: angle=%.4f torque=%.3f temp=%.1f\r\n",
                           motor_data.motor[0].angle, motor_data.motor[0].torque, motor_data.motor[0].temperature);
                print_counter = 0;
            }
        }

        /* 计算当前时刻的正弦力矩目标值 */
        float torque = amp * sinf(omega * t);

        /* 发送力矩指令：阻抗模式下令 kp=kd=0，仅保留力矩前馈分量
         * τ_cmd = kp*(p_ref-p) + kd*(v_ref-v) + τ_ff  →  τ_cmd = τ_ff */
        Ele_motor_set_para(0.0f, 0.0f, 0.0f, 0.0f, torque, ELE_MOTOR_MODE_IMPEDANCE, MOTOR_TEST_ELE_ID);

        t += dt;
        vTaskDelayUntil(&last, period); /* 严格等周期执行 */
    }
}
