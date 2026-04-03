#include "motor_test_task.h"
#include "robot_task.h"
#include "ele_motor.h"
#include <math.h>

#define ELE_ID 1U

void Motor_test_task(void const *argument)
{
    TickType_t last = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(10);
    const float dt = 0.01f;
    const float amp = 0.5f;
    const float omega = 2.0f * 3.1415926f * 0.5f;
    float t = 0.0f;

    Ele_motor_feedback_t fb;
    float param_value = 0.0f;
    uint8_t is_param = 0U;

    (void)argument;

    Ele_motor_init(ELE_ID);
    Ele_motor_write_control_mode(ELE_ID, ELE_MOTOR_MODE_POSITION);

    for (;;)
    {
        float target = amp * sinf(omega * t);
        Ele_motor_set_position(ELE_ID, target, 80.0f, 20.0f, 2.0f, 0.0f);

        // if (Ele_motor_fetch_rx(&fb, &param_value, &is_param))
        // {
            if (is_param)
            {
                Uart_printf(test_uart, "ele id=%d param=%.3f\r\n", ELE_ID, param_value);
            }
            else
            {
                Uart_printf(test_uart, "ele id=%d p=%.3f v=%.3f tq=%.3f\r\n", fb.id, fb.position, fb.velocity, fb.torque);
            }
        // }

        t += dt;
        vTaskDelayUntil(&last, period);
    }
}
