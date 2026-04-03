#ifndef ELE_MOTOR_H
#define ELE_MOTOR_H

#include <stdint.h>

#define ELE_MOTOR_SET_TEST 0x00U

#define ELE_MOTOR_OR_ERROR 0x01U
#define ELE_MOTOR_OR_IA 0x02U
#define ELE_MOTOR_OR_IB 0x03U
#define ELE_MOTOR_OR_IC 0x04U
#define ELE_MOTOR_OR_ID 0x05U
#define ELE_MOTOR_OR_IQ 0x06U
#define ELE_MOTOR_OR_VBUS 0x07U
#define ELE_MOTOR_OR_VD 0x08U
#define ELE_MOTOR_OR_VQ 0x09U
#define ELE_MOTOR_OR_TE 0x0AU
#define ELE_MOTOR_OR_ANGEL 0x0BU
#define ELE_MOTOR_OR_WE 0x0CU
#define ELE_MOTOR_OR_TEMPERATURE 0x0DU
#define ELE_DRIVE_OR_TEMPERATURE 0x0EU
#define ELE_MOTOR_OR_ANGLE 0x0FU
#define ELE_MOTOR_OR_VELOCITY 0x10U
#define ELE_MOTOR_OR_TORQUE 0x11U
#define ELE_MOTOR_OR_ERROR_REGISTER 0x12U

#define ELE_MOTOR_OW_SAVE_PARAMETER 0x2AU
#define ELE_MOTOR_OW_CLEAR_FAULT 0x2BU

#define ELE_MOTOR_WR_LD 0x40U
#define ELE_MOTOR_WR_LQ 0x41U
#define ELE_MOTOR_WR_FLUX 0x42U
#define ELE_MOTOR_WR_RESISTANCE 0x43U
#define ELE_MOTOR_WR_GR 0x44U
#define ELE_MOTOR_WR_J 0x45U
#define ELE_MOTOR_WR_B 0x46U
#define ELE_MOTOR_WR_P 0x47U
#define ELE_MOTOR_WR_TF 0x48U
#define ELE_MOTOR_WR_KT_OUT 0x49U
#define ELE_MOTOR_WR_MAJOR 0x50U
#define ELE_MOTOR_WR_CAN_ID 0x51U
#define ELE_MOTOR_WR_CURRENT_RISETIME 0x52U
#define ELE_MOTOR_WR_MAX_ANGLE 0x53U
#define ELE_MOTOR_WR_MIN_ANGLE 0x54U
#define ELE_MOTOR_WR_ANGLE_LIMIT_SWITCH 0x55U
#define ELE_MOTOR_WR_CURRENT_LIMIT 0x56U
#define ELE_MOTOR_WR_CAN_TIMEOUT 0x57U
#define ELE_MOTOR_WR_ECAT_ID 0x58U
#define ELE_MOTOR_WR_TEMP_PROTECTION 0x59U
#define ELE_DRIVER_WR_TEMP_PROTECTION 0x5AU
#define ELE_MOTOR_WR_CONTROL_MODE 0x5BU

#define ELE_MOTOR_P_MIN -12.5f
#define ELE_MOTOR_P_MAX 12.5f
#define ELE_MOTOR_V_MIN -14.0f
#define ELE_MOTOR_V_MAX 14.0f
#define ELE_MOTOR_KP_MIN 0.0f
#define ELE_MOTOR_KP_MAX 500.0f
#define ELE_MOTOR_KD_MIN 0.0f
#define ELE_MOTOR_KD_MAX 100.0f
#define ELE_MOTOR_KI_MIN 0.0f
#define ELE_MOTOR_KI_MAX 10000.0f
#define ELE_MOTOR_T_MIN -200.0f
#define ELE_MOTOR_T_MAX 200.0f

#define ELE_MOTOR_START 0xFCU
#define ELE_MOTOR_STOP_CMD 0xFDU
#define ELE_MOTOR_ANGLE_ZERO 0xFEU
#define ELE_MOTOR_CHANGE_ID 0xF9U
#define ELE_MOTOR_ANGLE_CORRECTION 0xF7U
#define ELE_MOTOR_ABSOLUTE_POSITION_CORRECT 0xF6U
#define ELE_MOTOR_CLEAR_ERROR 0xF4U
#define ELE_MOTOR_PARAMETER_DECERN 0xEDU

#define ELE_MOTOR_MODE_IMPEDANCE 0U
#define ELE_MOTOR_MODE_SPEED 1U
#define ELE_MOTOR_MODE_POSITION 2U

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
