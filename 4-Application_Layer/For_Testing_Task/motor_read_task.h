#ifndef MOTOR_READ_TASK_H
#define MOTOR_READ_TASK_H

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "main.h"
#include "cmsis_os.h"
#include "ele_motor.h"

/* 读取的电机数量及ID */
#define MOTOR_READ_COUNT            3U      /* 读取电机个数 */
#define MOTOR_READ_ELE_ID_1         1U      /* 电机1 ID */
#define MOTOR_READ_ELE_ID_2         2U      /* 电机2 ID */
#define MOTOR_READ_ELE_ID_3         3U      /* 电机3 ID */

/* 读取周期 */
#define MOTOR_READ_PERIOD_MS        10U     /* 读取周期 (ms)，100Hz */

/* 单个电机的参数数据结构 */
typedef struct
{
    float angle;      /* 电机角度 (rad) */
    float torque;     /* 电机扭矩 (Nm) */
    float temperature;/* 电机温度 (°C) */
} Motor_single_data_t;

/* 多电机参数数据结构 */
typedef struct
{
    Motor_single_data_t motor[MOTOR_READ_COUNT];  /* 三个电机的数据 */
    uint32_t timestamp;                           /* 数据时间戳 (ms) */
} Motor_read_data_t;

/* 外部队列句柄 */
extern QueueHandle_t Motor_read_queue_handle;

void Motor_read_task(void const *argument);

#endif /* MOTOR_READ_TASK_H */
