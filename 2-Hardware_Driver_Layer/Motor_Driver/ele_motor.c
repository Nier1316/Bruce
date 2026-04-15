#include "ele_motor.h"
#include "can.h"
#include "bsp_can.h"

#define ELE_RW_WRITE 1U
#define ELE_PKT_HEAD 0x80U
#define ELE_PKT_TAIL 0xECU
#define ELE_CMD_FILL 0xFFU
#define ELE_ENABLE_WAIT_TIMEOUT_MS 20U
#define ELE_ENABLE_RETRY_DELAY_MS 5U

typedef struct
{
    Can_controller_t *can_dev;
    volatile uint8_t has_new;
    volatile uint8_t is_param_frame;
    float param_value;
    Ele_motor_feedback_t feedback;
} Ele_motor_ctx_t;

static Ele_motor_ctx_t g_ele_ctx;

/** 原始CAN帧发送
 *  @param id    电机CAN标准帧ID
 *  @param data  8字节发送数据
 */
static void ele_send_raw(uint8_t id, uint8_t data[8])
{
    CAN_TxHeaderTypeDef tx = {0};
    uint32_t mailbox = 0;
    tx.StdId = id;
    tx.IDE = CAN_ID_STD;
    tx.RTR = CAN_RTR_DATA;
    tx.DLC = 8;
    HAL_CAN_AddTxMessage(&hcan1, &tx, data, &mailbox);
}

/**
 * @brief CAN 接收中断回调，解析电机回传帧
 *
 * 电机回传两种帧，通过 data[0] bit7 和 data[5] 低7位区分：
 *
 * 参数帧（读写参数的响应）：
 *   data[0] bit7=1  且  data[5] & 0x7F != 0
 *   data[0] 低4位 = 电机 ID
 *   data[1..4]    = 参数值（float，小端）
 *   data[5] 低7位 = 参数寄存器地址（如 0x0F=角度，0x11=扭矩）
 *
 * 控制帧（运动控制指令的状态回包）：
 *   其余情况
 *   data[0] 低4位 = 电机 ID
 *   data[1..2]    = 位置（16bit，映射到 P_MIN~P_MAX）
 *   data[3]高8位 + data[4]高4位 = 速度（12bit，映射到 V_MIN~V_MAX）
 *   data[4]低4位 + data[5]     = 扭矩（12bit，映射到 -T_MAX~T_MAX）
 */
static void Ele_motor_can_rx_cb(Can_controller_t *can_dev, void *context)
{
    Ele_motor_ctx_t *ctx = (Ele_motor_ctx_t *)context;
    uint8_t *data = can_dev->rx_buffer;

    if ((data[0] & 0x80U) && (data[5] & 0x7FU))
    {
        /* 参数帧：提取电机 ID 和参数值 */
        ctx->is_param_frame = 1U;
        ctx->feedback.id = data[0] & 0x0FU;
        ctx->param_value = Ele_motor_u8Arry2float(data, 1);
    }
    
    else
    {
        /* 控制帧：解包位置/速度/扭矩（定点数→浮点数） */
        uint16_t p_int = (uint16_t)((data[1] << 8) | data[2]);
        uint16_t v_int = (uint16_t)((data[3] << 4) | (data[4] >> 4));
        uint16_t i_int = (uint16_t)(((data[4] & 0x0F) << 8) | data[5]);

        ctx->is_param_frame = 0U;
        ctx->feedback.id = data[0] & 0x0FU;
        ctx->feedback.position = Ele_motor_uint_to_float(p_int, ELE_MOTOR_P_MIN, ELE_MOTOR_P_MAX, 16);
        ctx->feedback.velocity = Ele_motor_uint_to_float(v_int, ELE_MOTOR_V_MIN, ELE_MOTOR_V_MAX, 12);
        ctx->feedback.torque = Ele_motor_uint_to_float(i_int, -ELE_MOTOR_T_MAX, ELE_MOTOR_T_MAX, 12);
    }

    ctx->has_new = 1U;
}

void Ele_motor_register(uint8_t id)
{
    Can_init_t can_config = {0};

    can_config.can_handle = &hcan1;
    can_config.can_id = id;
    can_config.tx_id = id;
    can_config.rx_id = id + 50;
    can_config.context = &g_ele_ctx;
    can_config.receive_callback = Ele_motor_can_rx_cb;

    g_ele_ctx.can_dev = Can_device_init(&can_config);
    g_ele_ctx.has_new = 0U;
    g_ele_ctx.is_param_frame = 0U;
    g_ele_ctx.param_value = 0.0f;
    g_ele_ctx.feedback.id = 0U;
    g_ele_ctx.feedback.position = 0.0f;
    g_ele_ctx.feedback.velocity = 0.0f;
    g_ele_ctx.feedback.torque = 0.0f;
}

void Ele_motor_init(uint8_t id)
{
    uint8_t data[8] = {ELE_CMD_FILL, ELE_CMD_FILL, ELE_CMD_FILL, ELE_CMD_FILL,
                       ELE_CMD_FILL, ELE_CMD_FILL, ELE_CMD_FILL, ELE_MOTOR_START};
    Ele_motor_feedback_t fb;
    uint8_t is_param = 0U;

    Ele_motor_register(id);

    ele_send_raw(id, data);

    
    for (;;)
    {
        uint32_t start_tick;
        g_ele_ctx.has_new = 0U;
        ele_send_raw(id, data);
        Ele_motor_param_rw(0.0f, 0U, ELE_MOTOR_OR_ANGLE, id);  /* 触发电机回传参数帧 */
        start_tick = HAL_GetTick();

        while ((HAL_GetTick() - start_tick) < ELE_ENABLE_WAIT_TIMEOUT_MS)
        {
            if (Ele_motor_fetch_rx(&fb, 0, &is_param) )
            {
                return;
            }
        }

        HAL_Delay(ELE_ENABLE_RETRY_DELAY_MS);
    }
}

uint8_t Ele_motor_fetch_rx(Ele_motor_feedback_t *feedback, float *param_value, uint8_t *is_param_frame)
{
    if (!g_ele_ctx.has_new)
    {
        return 0U;
    }

    g_ele_ctx.has_new = 0U;

    if (is_param_frame)
    {
        *is_param_frame = g_ele_ctx.is_param_frame;
    }

    if (feedback)
    {
        *feedback = g_ele_ctx.feedback;  /* id 对两种帧都有效 */
    }

    if (g_ele_ctx.is_param_frame)
    {
        if (param_value)
        {
            *param_value = g_ele_ctx.param_value;
        }
    }

    return 1U;
}

////////////////////////////用户使用函数//////////////////////////////////////
/** 读写电机参数
 *  @param parameter   设置的参数值，读时设置为0
 *  @param rw          读写：置0读，置1写
 *  @param type        参数类型，头文件中定义如：ELE_MOTOR_OR_TEMPERATURE
 *  @param id          电机ID号
 */
void Ele_motor_param_rw(float parameter, uint8_t rw, uint8_t type, uint8_t id)
{
    union {
        float f;
        uint8_t u[4];
    } value;
    uint8_t data[8];

    value.f = parameter;
    data[0] = ELE_PKT_HEAD;
    data[1] = value.u[0];
    data[2] = value.u[1];
    data[3] = value.u[2];
    data[4] = value.u[3];
    data[5] = rw;
    data[6] = type;
    data[7] = ELE_PKT_TAIL;

    ele_send_raw(id, data);
}

/** 控制指令
 *  @param p1~p5  对应各控制模式参数
 *  @param mode   控制模式：阻抗/速度/位置
 *  @param id     电机ID号
 *  @note 必须确保电机处于正确模式，如先调用 Ele_motor_write_control_mode
 */
void Ele_motor_set_para(float p1, float p2, float p3, float p4, float p5, uint8_t mode, uint8_t id)
{
    uint8_t data[8];

    if (mode == ELE_MOTOR_MODE_IMPEDANCE)
    {
        /* 扭矩软件限幅，防止上层传入超限值 */
        if (p5 >  ELE_MOTOR_T_MAX) p5 =  ELE_MOTOR_T_MAX;
        if (p5 < -ELE_MOTOR_T_MAX) p5 = -ELE_MOTOR_T_MAX;

        uint16_t p_int = Ele_motor_float_to_uint(p1, ELE_MOTOR_P_MIN, ELE_MOTOR_P_MAX, 15);
        uint16_t v_int = Ele_motor_float_to_uint(p2, ELE_MOTOR_V_MIN, ELE_MOTOR_V_MAX, 12);
        uint16_t kp_int = Ele_motor_float_to_uint(p3, ELE_MOTOR_KP_MIN, ELE_MOTOR_KP_MAX, 12);
        uint16_t kd_int = Ele_motor_float_to_uint(p4, ELE_MOTOR_KD_MIN, ELE_MOTOR_KD_MAX, 12);
        uint16_t t_int = Ele_motor_float_to_uint(p5, ELE_MOTOR_T_MIN, ELE_MOTOR_T_MAX, 12);

        data[0] = (uint8_t)((p_int >> 8) & 0x7F);
        data[1] = (uint8_t)(p_int & 0xFF);
        data[2] = (uint8_t)(v_int >> 4);
        data[3] = (uint8_t)(((v_int & 0x0F) << 4) | (kp_int >> 8));
        data[4] = (uint8_t)(kp_int & 0xFF);
        data[5] = (uint8_t)(kd_int >> 4);
        data[6] = (uint8_t)(((kd_int & 0x0F) << 4) | (t_int >> 8));
        data[7] = (uint8_t)(t_int & 0xFF);
    }
    else if (mode == ELE_MOTOR_MODE_SPEED)
    {
        uint32_t v_int = Ele_motor_float_to_uint(p1, ELE_MOTOR_V_MIN, ELE_MOTOR_V_MAX, 31);
        uint16_t kvp_int = Ele_motor_float_to_uint(p2, ELE_MOTOR_KP_MIN, ELE_MOTOR_KP_MAX, 16);
        uint16_t kvi_int = Ele_motor_float_to_uint(p5, ELE_MOTOR_KI_MIN, ELE_MOTOR_KI_MAX, 16);

        data[0] = (uint8_t)((v_int >> 24) & 0x7F);
        data[1] = (uint8_t)((v_int >> 16) & 0xFF);
        data[2] = (uint8_t)((v_int >> 8) & 0xFF);
        data[3] = (uint8_t)(v_int & 0xFF);
        data[4] = (uint8_t)((kvp_int >> 8) & 0xFF);
        data[5] = (uint8_t)(kvp_int & 0xFF);
        data[6] = (uint8_t)((kvi_int >> 8) & 0xFF);
        data[7] = (uint8_t)(kvi_int & 0xFF);
    }
    else
    {
        uint16_t p_int = Ele_motor_float_to_uint(p1, ELE_MOTOR_P_MIN, ELE_MOTOR_P_MAX, 15);
        uint16_t kvp_int = Ele_motor_float_to_uint(p2, ELE_MOTOR_KP_MIN, ELE_MOTOR_KP_MAX, 12);
        uint16_t kp_int = Ele_motor_float_to_uint(p3, ELE_MOTOR_KP_MIN, ELE_MOTOR_KP_MAX, 12);
        uint16_t kd_int = Ele_motor_float_to_uint(p4, ELE_MOTOR_KD_MIN, ELE_MOTOR_KD_MAX, 12);
        uint16_t kvi_int = Ele_motor_float_to_uint(p5, ELE_MOTOR_KI_MIN, ELE_MOTOR_KI_MAX, 12);

        data[0] = (uint8_t)((p_int >> 8) & 0x7F);
        data[1] = (uint8_t)(p_int & 0xFF);
        data[2] = (uint8_t)(kvp_int >> 4);
        data[3] = (uint8_t)(((kvp_int & 0x0F) << 4) | (kp_int >> 8));
        data[4] = (uint8_t)(kp_int & 0xFF);
        data[5] = (uint8_t)(kd_int >> 4);
        data[6] = (uint8_t)(((kd_int & 0x0F) << 4) | (kvi_int >> 8));
        data[7] = (uint8_t)(kvi_int & 0xFF);
    }

    ele_send_raw(id, data);
}

/** CAN数据解包
 *  @param data      8字节回包
 *  @param feedback  控制回包解析结果输出，传NULL则不输出
 *  @return 参数读写回包时返回解析出的float；控制回包时返回position；否则返回0
 */
float Ele_motor_unpack_cmd(uint8_t *data, Ele_motor_feedback_t *feedback)
{
    if ((data[0] & 0x80U) && (data[5] & 0x7FU))
    {
        return Ele_motor_u8Arry2float(data, 1);
    }

    if (feedback)
    {
        uint16_t p_int = (uint16_t)((data[1] << 8) | data[2]);
        uint16_t v_int = (uint16_t)((data[3] << 4) | (data[4] >> 4));
        uint16_t i_int = (uint16_t)(((data[4] & 0x0F) << 8) | data[5]);

        feedback->id = data[0] & 0x0FU;
        feedback->position = Ele_motor_uint_to_float(p_int, ELE_MOTOR_P_MIN, ELE_MOTOR_P_MAX, 16);
        feedback->velocity = Ele_motor_uint_to_float(v_int, ELE_MOTOR_V_MIN, ELE_MOTOR_V_MAX, 12);
        feedback->torque = Ele_motor_uint_to_float(i_int, -ELE_MOTOR_T_MAX, ELE_MOTOR_T_MAX, 12);
        return feedback->position;
    }

    return 0.0f;
}

/** 将字节数组中的4字节转成float（小端）
 *  @param data  数据源
 *  @param key   起始下标
 */
float Ele_motor_u8Arry2float(uint8_t *data, uint8_t key)
{
    union {
        float f;
        uint8_t u[4];
    } value;

    value.u[0] = data[key + 0U];
    value.u[1] = data[key + 1U];
    value.u[2] = data[key + 2U];
    value.u[3] = data[key + 3U];
    return value.f;
}

/////////////////////////////底层函数////////////////////////////////
/** uint转float */
float Ele_motor_uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

/** float转uint */
unsigned int Ele_motor_float_to_uint(float x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return (unsigned int)(((x - offset) * ((unsigned int)(1U << bits) - 1U)) / span);
}

/** 快捷接口：写控制模式 */
void Ele_motor_write_control_mode(uint8_t id, uint8_t mode)
{
    Ele_motor_param_rw((float)mode, ELE_RW_WRITE, ELE_MOTOR_WR_CONTROL_MODE, id);
}

/** 快捷接口：位置模式参数发送 */
void Ele_motor_set_position(uint8_t id, float pos, float kvp, float kp, float kd, float kvi)
{
    Ele_motor_set_para(pos, kvp, kp, kd, kvi, ELE_MOTOR_MODE_POSITION, id);
}

/** 快捷接口：停止（失能）电机 */
void Ele_motor_stop(uint8_t id)
{
    uint8_t data[8] = {ELE_CMD_FILL, ELE_CMD_FILL, ELE_CMD_FILL, ELE_CMD_FILL,
                       ELE_CMD_FILL, ELE_CMD_FILL, ELE_CMD_FILL, ELE_MOTOR_STOP_CMD};
    ele_send_raw(id, data);
}
