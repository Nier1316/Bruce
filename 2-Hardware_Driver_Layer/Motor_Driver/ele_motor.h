#ifndef ELE_MOTOR_H
#define ELE_MOTOR_H

#include <stdint.h>

/* 测试指令 */
#define ELE_MOTOR_SET_TEST 0x00U        /* 测试帧类型 */

/* 只读参数寄存器地址 (OR: Only Read) */
#define ELE_MOTOR_OR_ERROR 0x01U        /* 错误码 */
#define ELE_MOTOR_OR_IA 0x02U           /* A相电流 */
#define ELE_MOTOR_OR_IB 0x03U           /* B相电流 */
#define ELE_MOTOR_OR_IC 0x04U           /* C相电流 */
#define ELE_MOTOR_OR_ID 0x05U           /* d轴电流 */
#define ELE_MOTOR_OR_IQ 0x06U           /* q轴电流 */
#define ELE_MOTOR_OR_VBUS 0x07U         /* 母线电压 */
#define ELE_MOTOR_OR_VD 0x08U           /* d轴电压 */
#define ELE_MOTOR_OR_VQ 0x09U           /* q轴电压 */
#define ELE_MOTOR_OR_TE 0x0AU           /* 电磁转矩 */
#define ELE_MOTOR_OR_ANGEL 0x0BU        /* 电角度（旧名，保留兼容） */
#define ELE_MOTOR_OR_WE 0x0CU           /* 电角速度 */
#define ELE_MOTOR_OR_TEMPERATURE 0x0DU  /* 电机温度 */
#define ELE_DRIVE_OR_TEMPERATURE 0x0EU  /* 驱动器温度 */
#define ELE_MOTOR_OR_ANGLE 0x0FU        /* 机械角度 */
#define ELE_MOTOR_OR_VELOCITY 0x10U     /* 转速 */
#define ELE_MOTOR_OR_TORQUE 0x11U       /* 输出转矩 */
#define ELE_MOTOR_OR_ERROR_REGISTER 0x12U /* 错误寄存器 */

/* 只写操作指令 (OW: Only Write) */
#define ELE_MOTOR_OW_SAVE_PARAMETER 0x2AU /* 保存参数到Flash */
#define ELE_MOTOR_OW_CLEAR_FAULT 0x2BU    /* 清除故障 */

/* 读写参数寄存器地址 (WR: Write/Read) */
#define ELE_MOTOR_WR_LD 0x40U               /* d轴电感 Ld */
#define ELE_MOTOR_WR_LQ 0x41U               /* q轴电感 Lq */
#define ELE_MOTOR_WR_FLUX 0x42U             /* 永磁磁链 */
#define ELE_MOTOR_WR_RESISTANCE 0x43U       /* 定子电阻 */
#define ELE_MOTOR_WR_GR 0x44U               /* 减速比 */
#define ELE_MOTOR_WR_J 0x45U                /* 转动惯量 J */
#define ELE_MOTOR_WR_B 0x46U                /* 粘滞摩擦系数 B */
#define ELE_MOTOR_WR_P 0x47U                /* 极对数 */
#define ELE_MOTOR_WR_TF 0x48U               /* 库伦摩擦力矩 Tf */
#define ELE_MOTOR_WR_KT_OUT 0x49U           /* 输出侧转矩系数 Kt */
#define ELE_MOTOR_WR_MAJOR 0x50U            /* 主版本号 */
#define ELE_MOTOR_WR_CAN_ID 0x51U           /* CAN节点ID */
#define ELE_MOTOR_WR_CURRENT_RISETIME 0x52U /* 电流环上升时间 */
#define ELE_MOTOR_WR_MAX_ANGLE 0x53U        /* 最大限位角度 */
#define ELE_MOTOR_WR_MIN_ANGLE 0x54U        /* 最小限位角度 */
#define ELE_MOTOR_WR_ANGLE_LIMIT_SWITCH 0x55U /* 角度限位使能 */
#define ELE_MOTOR_WR_CURRENT_LIMIT 0x56U    /* 电流限幅 */
#define ELE_MOTOR_WR_CAN_TIMEOUT 0x57U      /* CAN通信超时时间 */
#define ELE_MOTOR_WR_ECAT_ID 0x58U          /* EtherCAT节点ID */
#define ELE_MOTOR_WR_TEMP_PROTECTION 0x59U  /* 电机过温保护阈值 */
#define ELE_DRIVER_WR_TEMP_PROTECTION 0x5AU /* 驱动器过温保护阈值 */
#define ELE_MOTOR_WR_CONTROL_MODE 0x5BU     /* 控制模式选择 */

/* 控制参数范围限制 */
#define ELE_MOTOR_P_MIN -12.5f      /* 位置最小值 (rad) */
#define ELE_MOTOR_P_MAX 12.5f       /* 位置最大值 (rad) */
#define ELE_MOTOR_V_MIN -14.0f      /* 速度最小值 (rad/s) */
#define ELE_MOTOR_V_MAX 14.0f       /* 速度最大值 (rad/s) */
#define ELE_MOTOR_KP_MIN 0.0f       /* 位置环Kp最小值 */
#define ELE_MOTOR_KP_MAX 500.0f     /* 位置环Kp最大值 */
#define ELE_MOTOR_KD_MIN 0.0f       /* 速度环Kd最小值 */
#define ELE_MOTOR_KD_MAX 100.0f     /* 速度环Kd最大值 */
#define ELE_MOTOR_KI_MIN 0.0f       /* 积分系数Ki最小值 */
#define ELE_MOTOR_KI_MAX 10000.0f   /* 积分系数Ki最大值 */
#define ELE_MOTOR_T_MIN -200.0f     /* 转矩最小值 (N·m) */
#define ELE_MOTOR_T_MAX 200.0f      /* 转矩最大值 (N·m) */

/* 特殊控制指令 */
#define ELE_MOTOR_START 0xFCU                    /* 使能电机 */
#define ELE_MOTOR_STOP_CMD 0xFDU                 /* 停止电机 */
#define ELE_MOTOR_ANGLE_ZERO 0xFEU               /* 设置当前位置为零点 */
#define ELE_MOTOR_CHANGE_ID 0xF9U                /* 修改CAN ID */
#define ELE_MOTOR_ANGLE_CORRECTION 0xF7U         /* 角度校正 */
#define ELE_MOTOR_ABSOLUTE_POSITION_CORRECT 0xF6U /* 绝对位置校正 */
#define ELE_MOTOR_CLEAR_ERROR 0xF4U              /* 清除错误 */
#define ELE_MOTOR_PARAMETER_DECERN 0xEDU         /* 参数辨识 */

/* 控制模式 */
#define ELE_MOTOR_MODE_IMPEDANCE 0U  /* 阻抗控制模式 */
#define ELE_MOTOR_MODE_SPEED 1U      /* 速度控制模式 */
#define ELE_MOTOR_MODE_POSITION 2U   /* 位置控制模式 */

typedef struct
{
    uint8_t id;
    float position;
    float velocity;
    float torque;
} Ele_motor_feedback_t;

void Ele_motor_register(uint8_t id);
void Ele_motor_init(uint8_t id);
uint8_t Ele_motor_fetch_rx(Ele_motor_feedback_t *feedback, float *param_value, uint8_t *is_param_frame);

void Ele_motor_param_rw(float parameter, uint8_t rw, uint8_t type, uint8_t id);
void Ele_motor_set_para(float p1, float p2, float p3, float p4, float p5, uint8_t mode, uint8_t id);
float Ele_motor_unpack_cmd(uint8_t *data, Ele_motor_feedback_t *feedback);
float Ele_motor_u8Arry2float(uint8_t *data, uint8_t key);
float Ele_motor_uint_to_float(int x_int, float x_min, float x_max, int bits);
unsigned int Ele_motor_float_to_uint(float x, float x_min, float x_max, int bits);

void Ele_motor_write_control_mode(uint8_t id, uint8_t mode);
void Ele_motor_set_position(uint8_t id, float pos, float kvp, float kp, float kd, float kvi);

#endif
