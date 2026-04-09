#ifndef MOTOR_TEST_TASK_H
#define MOTOR_TEST_TASK_H

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "ele_motor.h"

/* 测试用电机 CAN ID */
#define MOTOR_TEST_ELE_ID       1U

/* 正弦力矩参数 */
#define MOTOR_TEST_TORQUE_AMP   5.0f    /* 力矩幅值 (Nm) */
#define MOTOR_TEST_FREQ_HZ      0.5f    /* 正弦频率 (Hz) */

/* 控制周期 */
#define MOTOR_TEST_PERIOD_MS    10U     /* 控制周期 (ms) */

/* 循环读取的电机参数类型（可替换为其他 ELE_MOTOR_OR_xxx） */
#define MOTOR_TEST_READ_PARAM   ELE_MOTOR_OR_ANGLE   /* 读取输出轴角度 */

void Motor_test_task(void const *argument);

#endif /* MOTOR_TEST_TASK_H */
