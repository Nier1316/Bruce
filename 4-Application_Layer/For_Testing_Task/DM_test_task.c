#include "DM_test_task.h"
#include "dji_motor.h"
#include "can.h"
#include "bsp_usart.h"
#include <stdio.h>

static Djimotor_device_t *test_motor = NULL;
extern Uart_instance_t *test_uart;

void DM_test_task(void const *argument)
{
    TickType_t PreviousWakeTime = xTaskGetTickCount();
    const uint32_t TimeIncrement = 10; // 10ms = 100Hz

    // 初始化3508电机
    Djimotor_init_config_t motor_config = {
        .motor_name = "test_3508",
        .motor_type = M3508,
        .motor_status = MOTOR_ENABLED,
        .deadzone_compensation = 1000,
        .motor_controller_init = {
            .close_loop = OPEN_LOOP,
            .angle_source = MOTOR_FEEDBACK,
            .speed_source = MOTOR_FEEDBACK,
        },
        .can_init = {
            .can_handle = &hcan1,
            .can_id = 0x200,
            .tx_id = 1,
            .rx_id = 0x201,
        },
    };

    test_motor = DJI_Motor_Init(&motor_config);

    if(test_motor == NULL) {
        // 电机初始化失败
        while(1); // 在这里打断点
    }

    // 验证 can_controller 初始化
    if(test_motor->can_controller == NULL) {
        while(1); // can_controller 为空
    }


    uint32_t loop_count = 0;
    for (;;)
    {
        loop_count++;
        
        Djimotor_set_target(test_motor, 2500.0f);
        Djimotor_Calc_Output(test_motor);

        // 每1秒打印一次调试信息
        if(loop_count % 100 == 0 && test_uart != NULL) {
            Uart_printf(test_uart,"Motor: ecd=%d, speed=%.1f, current=%d, out=%d\r\n",
                   test_motor->motor_measure.current_ecd,
                   test_motor->motor_measure.angular_velocity,
                   test_motor->motor_measure.real_current,
                   test_motor->out_current);
        }

        vTaskDelayUntil(&PreviousWakeTime, TimeIncrement);
    }
}













