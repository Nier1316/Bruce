#ifndef MOTOR_PROTECT_TASK_H
#define MOTOR_PROTECT_TASK_H

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "ele_motor.h"

/* 保护用电机 CAN ID */
#define MOTOR_PROTECT_ELE_ID        1U

/* 角度保护阈值 */
#define MOTOR_PROTECT_ANGLE_MAX     1.0f    /* 正向最大偏移量 (rad) */
#define MOTOR_PROTECT_ANGLE_DE_MAX  1.0f    /* 反向最大偏移量 (rad) */

/* 监控周期 */
#define MOTOR_PROTECT_PERIOD_MS     10U     /* 监控周期 (ms)，100Hz */

void Motor_protect_task(void const *argument);

#endif /* MOTOR_PROTECT_TASK_H */
