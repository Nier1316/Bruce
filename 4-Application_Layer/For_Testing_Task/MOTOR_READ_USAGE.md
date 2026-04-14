# 电机参数读取任务使用指南

## 概述

`Motor_read_task` 是一个通用的电机参数读取任务，负责定期读取电机的三个关键参数：
- **角度** (angle) - rad
- **扭矩** (torque) - Nm  
- **温度** (temperature) - °C

读取的数据存放在 `Motor_read_queue_handle` 队列中，供其他任务读取使用。

## 工作原理

### 执行流程

```
周期    状态            操作
----    ----            ----
 0ms    READ_ANGLE      请求读取角度
10ms    READ_TORQUE     接收角度，请求扭矩
20ms    READ_TEMP       接收扭矩，请求温度
30ms    READ_ANGLE      接收温度，请求角度（循环）
...     ...             ...
```

- **执行频率**: 100 Hz (10ms 周期)
- **参数更新频率**: 每个参数约 33 Hz (30ms 周期)
- **队列模式**: 深度为 1，覆盖写模式，始终保持最新数据

### 数据结构

```c
typedef struct
{
    float angle;       /* 电机角度 (rad) */
    float torque;      /* 电机扭矩 (Nm) */
    float temperature; /* 电机温度 (°C) */
} Motor_read_data_t;
```

## 使用方法

### 方法 1: 读取最新数据（推荐）

使用 `xQueuePeek()` 非阻塞地读取最新数据，不移除队列内容。
适合多个任务都需要读取同一数据的场景。

```c
Motor_read_data_t motor_data;

/* 非阻塞模式：立即返回，如果有数据则读取 */
if (xQueuePeek(Motor_read_queue_handle, &motor_data, 0) == pdTRUE)
{
    float angle = motor_data.angle;
    float torque = motor_data.torque;
    float temp = motor_data.temperature;
    
    /* 处理数据... */
}
```

### 方法 2: 阻塞等待新数据

使用 `xQueuePeek()` 带超时时间，等待新数据到达。

```c
Motor_read_data_t motor_data;

/* 阻塞模式：等待数据到达，超时100ms */
if (xQueuePeek(Motor_read_queue_handle, &motor_data, pdMS_TO_TICKS(100)) == pdTRUE)
{
    /* 处理数据... */
}
```

### 方法 3: 定期检查数据

在任务周期内检查是否有新数据。

```c
void My_control_task(void const *argument)
{
    TickType_t last = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(20);
    Motor_read_data_t motor_data;

    for (;;)
    {
        /* 每 20ms 检查一次最新数据 */
        if (xQueuePeek(Motor_read_queue_handle, &motor_data, 0) == pdTRUE)
        {
            /* 基于最新数据进行控制 */
            float control_output = calculate_control(motor_data.angle, motor_data.torque);
            apply_control(control_output);
        }

        vTaskDelayUntil(&last, period);
    }
}
```

## 应用场景

### 场景 1: 电机保护（角度范围监控）

```c
void Motor_protect_task(void const *argument)
{
    Motor_read_data_t motor_data;
    float angle_initial = 0.0f;
    uint8_t first_read = 1;
    const float ANGLE_MAX = 1.0f;    /* rad */
    const float ANGLE_MIN = -1.0f;   /* rad */

    for (;;)
    {
        if (xQueuePeek(Motor_read_queue_handle, &motor_data, pdMS_TO_TICKS(50)) == pdTRUE)
        {
            if (first_read)
            {
                angle_initial = motor_data.angle;
                first_read = 0;
            }
            else
            {
                float delta = motor_data.angle - angle_initial;
                
                if (delta > ANGLE_MAX || delta < ANGLE_MIN)
                {
                    /* 超出范围，执行保护 */
                    Ele_motor_stop(MOTOR_READ_ELE_ID);
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
```

### 场景 2: 温度监控

```c
void Temperature_monitor_task(void const *argument)
{
    Motor_read_data_t motor_data;
    const float TEMP_ALERT = 80.0f;   /* 告警温度 */
    const float TEMP_SHUTDOWN = 90.0f; /* 关闭温度 */

    for (;;)
    {
        if (xQueuePeek(Motor_read_queue_handle, &motor_data, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            if (motor_data.temperature > TEMP_SHUTDOWN)
            {
                /* 立即关闭电机 */
                Ele_motor_stop(MOTOR_READ_ELE_ID);
                Uart_printf(test_uart, "SHUTDOWN: temp=%.1f\r\n", motor_data.temperature);
            }
            else if (motor_data.temperature > TEMP_ALERT)
            {
                /* 降低电流 */
                Uart_printf(test_uart, "ALERT: temp=%.1f\r\n", motor_data.temperature);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
```

### 场景 3: 基于反馈的控制

```c
void Feedback_control_task(void const *argument)
{
    TickType_t last = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(10);
    Motor_read_data_t motor_data;
    float target_torque = 2.0f;

    for (;;)
    {
        if (xQueuePeek(Motor_read_queue_handle, &motor_data, 0) == pdTRUE)
        {
            /* 基于扭矩误差进行 PID 控制 */
            float torque_error = target_torque - motor_data.torque;
            float current_cmd = pid_controller(torque_error);
            
            /* 下发控制命令 */
            Ele_motor_set_para(..., current_cmd, ...);
        }

        vTaskDelayUntil(&last, period);
    }
}
```

### 场景 4: 上位机状态上报

```c
void Status_report_task(void const *argument)
{
    Motor_read_data_t motor_data;
    uint32_t counter = 0;

    for (;;)
    {
        if (xQueuePeek(Motor_read_queue_handle, &motor_data, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            counter++;
            
            /* 每 10 次读取（约 1 秒）上报一次 */
            if (counter >= 10)
            {
                Uart_printf(test_uart, "$MOTOR: angle=%.4f torque=%.3f temp=%.1f\r\n",
                           motor_data.angle, motor_data.torque, motor_data.temperature);
                counter = 0;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
```

## 关键 API

### xQueuePeek()

```c
BaseType_t xQueuePeek(QueueHandle_t xQueue, 
                      void *pvBuffer, 
                      TickType_t xTicksToWait);
```

| 参数 | 说明 |
|------|------|
| `xQueue` | 队列句柄：`Motor_read_queue_handle` |
| `pvBuffer` | 缓冲区指针：`&motor_data` |
| `xTicksToWait` | 等待超时：`0`=非阻塞，`pdMS_TO_TICKS(ms)`=阻塞 |
| **返回值** | `pdTRUE`=成功，`pdFALSE`=失败 |

**特点**：
- ✅ 读取不移除（多任务共享）
- ✅ 支持超时等待
- ✅ 线程安全

### xQueueReceive()

```c
BaseType_t xQueueReceive(QueueHandle_t xQueue, 
                         void *pvBuffer, 
                         TickType_t xTicksToWait);
```

**不推荐使用**：
- Motor_read_task 使用 `xQueueOverwrite()`，深度为 1
- 如果使用 `xQueueReceive()` 可能导致数据丢失

## 初始化配置

### 在 robot_task.c 中的初始化

```c
/* 创建队列（必须在任务创建之前） */
Motor_read_queue_handle = xQueueCreate(1, sizeof(Motor_read_data_t));

/* 创建 Motor_read_task */
osThreadDef(motor_read_task, Motor_read_task, osPriorityHigh, 0, 512);
motor_read_task_handle = osThreadCreate(osThread(motor_read_task), NULL);
```

### 在头文件中的声明

```c
/* robot_task.h */
extern QueueHandle_t Motor_read_queue_handle;
```

## 常见问题

### Q1: 为什么读取的数据有延迟？

**A**: 因为采用轮流读取三个参数的设计：
- 每个参数需要 30ms 才能完整读取一次
- 实际应用中，最新数据的延迟在 0~30ms 之间

如需实时性更强，可修改策略为每周期读取同一参数。

### Q2: 多个任务同时读取会冲突吗？

**A**: 不会。使用 `xQueuePeek()` 是线程安全的：
- FreeRTOS 内部使用临界段保护队列操作
- 多个任务可以安全地同时 `xQueuePeek()` 同一队列

### Q3: 如何只读取某个参数？

**A**: 在应用任务中只使用需要的字段：

```c
Motor_read_data_t motor_data;
if (xQueuePeek(Motor_read_queue_handle, &motor_data, 0) == pdTRUE)
{
    /* 只使用角度 */
    float angle = motor_data.angle;
}
```

### Q4: 如何加快参数更新速率？

**A**: 修改 motor_read_task 的策略：

```c
/* 选项 A: 减少控制周期（需要电机驱动支持） */
#define MOTOR_READ_PERIOD_MS 5  /* 改为 5ms */

/* 选项 B: 改为逐周期读取全部参数（需要电机驱动支持同时返回多个参数） */
/* 详见 ele_motor.c 的接口设计 */
```

## 最佳实践

| 场景 | 推荐方式 | 原因 |
|------|--------|------|
| 保护/监控 | 非阻塞 + 定期检查 | 快速响应，不浪费CPU |
| 反馈控制 | 同周期内读取 | 确保控制和反馈同步 |
| 上位机通信 | 阻塞等待 + 周期上报 | 简化同步逻辑 |
| 多任务共享 | `xQueuePeek()` | 避免数据竞争 |

## 参考链接

- [FreeRTOS Queue API](https://www.freertos.org/a00018.html)
- [电机驱动接口 (ele_motor.h)](../Motor_Driver/ele_motor.h)
- [电机测试任务 (motor_test_task.c)](./motor_test_task.c)
