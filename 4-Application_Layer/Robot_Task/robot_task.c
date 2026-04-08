#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "robot_task.h"
#include "bsp_usart.h"

#include "usart.h"

#include "queue.h"

// 测试任务头文件
#include <stdio.h>

#include "Decision_making_task.h"  // 添加决策任务头文件
#include "Chassis_task.h"
#include "decision_making.h"
#include "motor_task.h"
#include "watchdog_task.h"
#include "Shell_task.h"
#include "buzzer_alarm_task.h"
#include "buzzer_alarm.h"
#include "motor_test_task.h"
#include "spi_motor_bridge_task.h"
#include "uart_motor_bridge_task.h"

  /**任务句柄声明**/
// osThreadId chassis_task_handle;//底盘任务
// osThreadId decision_making_task_handle;     //决策任务
// osThreadId others_task_handle; //处理其他任务，比如与视觉通信，电量读取等琐碎任务，后续根据实际进行修改
// osThreadId watchdog_task_handle; //看门狗任务
// osThreadId buzzer_alarm_task_handle;
osThreadId motor_test_task_handle;//电机测试任务
osThreadId spi_motor_bridge_task_handle;  // SPI-CAN 桥接任务
osThreadId uart_motor_bridge_task_handle; // UART-CAN 桥接任务

/*Shell任务句柄*/
// osThreadId shell_task_handle;
// osThreadId motor_task_handle;

/**创建各个进程控制队列**/
// QueueHandle_t Chassis_cmd_queue_handle;

// QueueHandle_t Buzzer_cmd_queue_handle;

// QueueHandle_t Chassis_feedback_queue_handle;

Uart_instance_t* test_uart = NULL;

/**机器人任务创建**/
void Robot_task_init(void)
{
    test_uart = Uart_register(&huart1,NULL);


    // 创建队列 (必须在任务创建之前!)
    // ============================================================
    // 深度为1，启用覆盖写模式，始终保持最新指令
    // Chassis_cmd_queue_handle = xQueueCreate(1, sizeof(Chassis_cmd_send_t));

    // Chassis_feedback_queue_handle = xQueueCreate(1, sizeof(Chassis_feedback_info_t));

    // Buzzer_cmd_queue_handle = xQueueCreate(5,sizeof(uint8_t));
    // 选择要运行的测试任务（取消注释需要的测试）
  osThreadDef(motor_test_task, Motor_test_task, osPriorityHigh, 0, 512);
  motor_test_task_handle = osThreadCreate(osThread(motor_test_task), NULL);

    // SPI-CAN 桥接任务：接收上位机 SPI 指令 → 电机 CAN，电机状态 → SPI 回传
  // osThreadDef(spi_motor_bridge_task, Spi_motor_bridge_task, osPriorityHigh, 0, 512);
  // spi_motor_bridge_task_handle = osThreadCreate(osThread(spi_motor_bridge_task), NULL);

    // UART-CAN 桥接任务：接收上位机 UART(huart3) 指令 → 电机 CAN，电机状态 → UART 回传
  // osThreadDef(uart_motor_bridge_task, Uart_motor_bridge_task, osPriorityHigh, 0, 512);
  // uart_motor_bridge_task_handle = osThreadCreate(osThread(uart_motor_bridge_task), NULL);

    // === 单元测试 ===

    //看门狗任务
  // osThreadDef(watchdog_control_task, Watchdog_control_task, osPriorityHigh, 0, 512);
  // watchdog_task_handle = osThreadCreate(osThread(watchdog_control_task), NULL);
    // 电机控制任务：1000Hz，聚合并通过 CAN 发送目标值
  // osThreadDef(motor_control_task, Motor_control_task, osPriorityNormal, 0, 512);
  // motor_task_handle = osThreadCreate(osThread(motor_control_task), NULL);
    //决策任务
  // osThreadDef(decision_making_task,Decision_making_task,osPriorityNormal,0,1024);
  // decision_making_task_handle = osThreadCreate(osThread(decision_making_task), NULL);

    // === 启动底盘与电机任务（必需） ===
    // 底盘控制任务：500Hz，接收决策层/测试发布的 chassis_cmd，解算并写入电机目标
    // osThreadDef(chassis_control_task, Chassis_control_task, osPriorityNormal, 0, 512);
    // chassis_task_handle = osThreadCreate(osThread(chassis_control_task), NULL);

    // osThreadDef(buzzer_alarm_task, Buzzer_alarm_control_task, osPriorityBelowNormal, 0, 512);
    // buzzer_alarm_task_handle = osThreadCreate(osThread(buzzer_alarm_task), NULL);
}


