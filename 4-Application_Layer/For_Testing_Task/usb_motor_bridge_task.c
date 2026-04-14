#include "usb_motor_bridge_task.h"
#include "bsp_usb.h"
#include "ele_motor.h"
#include <string.h>

/* ============================================================
 * 共享状态：接收回调 → 任务
 * 回调在中断上下文中执行，只做最少操作：拷贝 + 置标志
 * ============================================================ */
static uint8_t  s_rx_staging[USB_BRIDGE_FRAME_LEN]; /* 暂存一帧完整数据 */
static volatile uint8_t s_new_cmd = 0U;              /* 1 = 有待处理的新指令 */

static void usb_bridge_rx_callback(uint8_t *buf, uint32_t len)
{
    /* 只接受长度完全匹配的帧，丢弃其他 */
    if (len == USB_BRIDGE_FRAME_LEN)
    {
        memcpy(s_rx_staging, buf, USB_BRIDGE_FRAME_LEN);
        s_new_cmd = 1U;
    }
}

/* ============================================================
 * 帧工具函数（与 UART/SPI 版一致）
 * ============================================================ */

static uint8_t calc_checksum(const uint8_t *buf, uint16_t len)
{
    uint8_t cs = 0U;
    for (uint16_t i = 0U; i < len; i++)
        cs ^= buf[i];
    return cs;
}

static void float_to_le(float f, uint8_t *out)
{
    union { float f; uint8_t u[4]; } v;
    v.f = f;
    out[0] = v.u[0]; out[1] = v.u[1];
    out[2] = v.u[2]; out[3] = v.u[3];
}

static float le_to_float(const uint8_t *in)
{
    union { float f; uint8_t u[4]; } v;
    v.u[0] = in[0]; v.u[1] = in[1];
    v.u[2] = in[2]; v.u[3] = in[3];
    return v.f;
}

/* ============================================================
 * 反馈帧打包  (C板 → Host)
 * ============================================================ */
static void pack_feedback_frame(uint8_t *tx, const Ele_motor_feedback_t *fb, uint8_t data_valid)
{
    memset(tx, 0x00U, USB_BRIDGE_FRAME_LEN);
    tx[0]  = USB_BRIDGE_HDR_FB;
    tx[1]  = fb->id;
    float_to_le(fb->position, &tx[2]);
    float_to_le(fb->velocity, &tx[6]);
    float_to_le(fb->torque,   &tx[10]);
    /* [14-21] reserved，已 memset 为 0 */
    tx[22] = data_valid ? 0x01U : 0x00U;
    tx[23] = calc_checksum(tx, 23U);
}

/* ============================================================
 * 指令帧解包  (Host → C板)
 * 返回 1 = 有效，0 = 帧头或校验错误
 * ============================================================ */
static uint8_t unpack_cmd_frame(const uint8_t *rx,
                                float *pos_ref, float *vel_ref,
                                float *kp,      float *kd,
                                float *torque_ff, uint8_t *flags)
{
    if (rx[0] != USB_BRIDGE_HDR_CMD)
        return 0U;
    if (rx[23] != calc_checksum(rx, 23U))
        return 0U;

    *pos_ref   = le_to_float(&rx[2]);
    *vel_ref   = le_to_float(&rx[6]);
    *kp        = le_to_float(&rx[10]);
    *kd        = le_to_float(&rx[14]);
    *torque_ff = le_to_float(&rx[18]);
    *flags     = rx[22];
    return 1U;
}

/* ============================================================
 * 任务主体
 * ============================================================ */
void Usb_motor_bridge_task(void const *argument)
{
    (void)argument;

    static uint8_t tx_buf[USB_BRIDGE_FRAME_LEN];
    static uint8_t rx_local[USB_BRIDGE_FRAME_LEN]; /* 任务内局部副本，避免处理中被覆盖 */

    /* --- 初始化 USB 虚拟串口（连接上位机） --- */
    Usb_Init(usb_bridge_rx_callback);

    /* --- 初始化电机，切换至阻抗模式 --- */
    Ele_motor_init(USB_BRIDGE_MOTOR_ID);
    Ele_motor_write_control_mode(USB_BRIDGE_MOTOR_ID, ELE_MOTOR_MODE_IMPEDANCE);

    Ele_motor_feedback_t fb = {.id = USB_BRIDGE_MOTOR_ID};
    float param_val  = 0.0f;
    uint8_t is_param = 0U;
    uint8_t data_valid = 0U;

    TickType_t last = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(USB_BRIDGE_PERIOD_MS);

    for (;;)
    {
        /* 1. 读取最新 CAN 反馈（上一周期的电机回包） */
        if (Ele_motor_fetch_rx(&fb, &param_val, &is_param))
        {
            if (!is_param)
                data_valid = 1U;
        }

        /* 2. 若有新指令，解包并发送 CAN 命令 */
        if (s_new_cmd)
        {
            /* 原子拷贝暂存区，防止回调在处理中再次写入 */
            taskENTER_CRITICAL();
            memcpy(rx_local, s_rx_staging, USB_BRIDGE_FRAME_LEN);
            s_new_cmd = 0U;
            taskEXIT_CRITICAL();

            float pos_ref, vel_ref, kp, kd, torque_ff;
            uint8_t flags;
            if (unpack_cmd_frame(rx_local, &pos_ref, &vel_ref, &kp, &kd, &torque_ff, &flags))
            {
                if (flags & 0x01U)  /* bit0 = 1：使能发送 */
                {
                    Ele_motor_set_para(pos_ref, vel_ref, kp, kd, torque_ff,
                                       ELE_MOTOR_MODE_IMPEDANCE, USB_BRIDGE_MOTOR_ID);
                }
            }
        }

        /* 3. 打包反馈帧，通过 USB 发回上位机 */
        pack_feedback_frame(tx_buf, &fb, data_valid);
        Usb_Send(tx_buf, USB_BRIDGE_FRAME_LEN);

        vTaskDelayUntil(&last, period); /* 严格等周期 */
    }
}
