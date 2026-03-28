#include "motor_test_task.h"
#include "robot_task.h"
#include "can.h"
#include <math.h>

#define ELE_ID 1U
#define ELE_RW_WRITE 1U
#define ELE_PARAM_CONTROL_MODE 0x5BU
#define ELE_MODE_POSITION 2U
#define ELE_PKT_HEAD 0x80U
#define ELE_PKT_TAIL 0xECU

#define ELE_P_MIN -12.5f
#define ELE_P_MAX 12.5f
#define ELE_KP_MIN 0.0f
#define ELE_KP_MAX 500.0f
#define ELE_KD_MIN 0.0f
#define ELE_KD_MAX 100.0f
#define ELE_KI_MIN 0.0f
#define ELE_KI_MAX 10000.0f

static unsigned int ele_float_to_uint(float x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return (unsigned int)((x - offset) * ((1U << bits) - 1U) / span);
}

static void ele_send_raw(uint32_t id, uint8_t data[8])
{
    CAN_TxHeaderTypeDef tx = {0};
    uint32_t mailbox = 0;
    tx.StdId = id;
    tx.IDE = CAN_ID_STD;
    tx.RTR = CAN_RTR_DATA;
    tx.DLC = 8;
    HAL_CAN_AddTxMessage(&hcan1, &tx, data, &mailbox);
}

static void ele_write_mode_position(uint32_t id)
{
    union {
        float f;
        uint8_t u[4];
    } value;
    uint8_t data[8] = {0};
    value.f = (float)ELE_MODE_POSITION;
    data[0] = ELE_PKT_HEAD;
    data[1] = value.u[0];
    data[2] = value.u[1];
    data[3] = value.u[2];
    data[4] = value.u[3];
    data[5] = ELE_RW_WRITE;
    data[6] = ELE_PARAM_CONTROL_MODE;
    data[7] = ELE_PKT_TAIL;
    ele_send_raw(id, data);
}

static void ele_set_position(uint32_t id, float pos, float kvp, float kp, float kd, float kvi)
{
    uint16_t p_int = ele_float_to_uint(pos, ELE_P_MIN, ELE_P_MAX, 15);
    uint16_t kvp_int = ele_float_to_uint(kvp, ELE_KP_MIN, ELE_KP_MAX, 12);
    uint16_t kp_int = ele_float_to_uint(kp, ELE_KP_MIN, ELE_KP_MAX, 12);
    uint16_t kd_int = ele_float_to_uint(kd, ELE_KD_MIN, ELE_KD_MAX, 12);
    uint16_t kvi_int = ele_float_to_uint(kvi, ELE_KI_MIN, ELE_KI_MAX, 12);
    uint8_t data[8];

    data[0] = (uint8_t)((p_int >> 8) & 0x7F);
    data[1] = (uint8_t)(p_int & 0xFF);
    data[2] = (uint8_t)(kvp_int >> 4);
    data[3] = (uint8_t)(((kvp_int & 0x0F) << 4) | (kp_int >> 8));
    data[4] = (uint8_t)(kp_int & 0xFF);
    data[5] = (uint8_t)(kd_int >> 4);
    data[6] = (uint8_t)(((kd_int & 0x0F) << 4) | (kvi_int >> 8));
    data[7] = (uint8_t)(kvi_int & 0xFF);

    ele_send_raw(id, data);
}

void Motor_test_task(void const *argument)
{
    TickType_t last = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(10);
    const float dt = 0.01f;
    const float amp = 0.5f;
    const float omega = 2.0f * 3.1415926f * 0.5f;
    float t = 0.0f;

    (void)argument;
    ele_write_mode_position(ELE_ID);

    for (;;)
    {
        float target = amp * sinf(omega * t);
        ele_set_position(ELE_ID, target, 80.0f, 20.0f, 2.0f, 0.0f);
        t += dt;
        vTaskDelayUntil(&last, period);
    }
}