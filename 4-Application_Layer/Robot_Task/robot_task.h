#ifndef _ROBOT_TASK_H
#define _ROBOT_TASK_H

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "bsp_usart.h"

//任务句柄外部声明
extern osThreadId chassis_task_handle; //底盘任务
extern osThreadId others_task_handle; //处理其他任务，比如与视觉通信，电量读取等琐碎任务，后续根据实际进行修改
extern osThreadId watchdog_task_handle;
extern osThreadId buzzer_alarm_task_handle;


/*队列*/
extern QueueHandle_t Chassis_cmd_queue_handle;
extern QueueHandle_t Buzzer_cmd_queue_handle;
extern QueueHandle_t Motor_read_queue_handle;

extern Uart_instance_t* test_uart;

//机器人任务创建
void Robot_task_init(void);

#endif //_ROBOT_TASK_H
