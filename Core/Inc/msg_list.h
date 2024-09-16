#ifndef __MSG_LIST_H__
#define __MSG_LIST_H__

#define RANGERFINDER_MSG_MAX_SIZE 16
#define TDLAS_TX_MSG_MAX_SIZE 16
#define TDLAS_RX_MSG_MAX_SIZE 16
#define CTRL_TX_MSG_MAX_SIZE 16
#define CTRL_RX_MSG_MAX_SIZE 16
#define MOTOR_TX_MSG_MAX_SIZE 16
#define MOTOR_RX_MSG_MAX_SIZE 16

#ifdef __cplusplus
extern "C"
{
#endif

#include "main.h"

    typedef struct
    {
        uint8_t addr;
        uint8_t func_code;
        uint8_t data_len;
        uint8_t data[4];
        uint8_t crc_l;
        uint8_t crc_h;
    } Rangefinder_Msg;

    typedef struct
    {
        uint8_t addr;
        uint8_t func_code;
        uint16_t reg_addr;
        uint16_t reg_cnt;
        uint8_t crc_l;
        uint8_t crc_h;
        uint8_t data_len;
        uint8_t data[2];
    } TDLAS_Tx_Msg;

    typedef struct
    {
        uint8_t addr;
        uint8_t func_code;
        uint8_t data_len;
        uint8_t data[8];
        uint8_t crc_l;
        uint8_t crc_h;
        uint16_t reg_addr;
        uint16_t reg_cnt;
    } TDLAS_Rx_Msg;

    typedef struct
    {
        uint8_t head_1;
        uint8_t head_2;
        uint8_t src_id;
        uint8_t dst_id;
        uint8_t func_code;
        uint8_t data_len;
        uint8_t data[32];
        uint8_t check;
        uint8_t tail_1;
        uint8_t tail_2;
    } Ctrl_Com_Msg;

    int8_t Rangefinder_Msg_Add(Rangefinder_Msg *msg);
    int8_t Rangerfinder_Msg_Get(Rangefinder_Msg *msg);

    int8_t TDLAS_Tx_Msg_Add(TDLAS_Tx_Msg *msg);
    int8_t TDLAS_Tx_Msg_Get(TDLAS_Tx_Msg *msg);
    int8_t TDLAS_Rx_Msg_Add(TDLAS_Rx_Msg *msg);
    int8_t TDLAS_Rx_Msg_Get(TDLAS_Rx_Msg *msg);

    int8_t Ctrl_Tx_Msg_Add(Ctrl_Com_Msg *msg);
    int8_t Ctrl_Tx_Msg_Get(Ctrl_Com_Msg *msg);
    int8_t Ctrl_Rx_Msg_Add(Ctrl_Com_Msg *msg);
    int8_t Ctrl_Rx_Msg_Get(Ctrl_Com_Msg *msg);

#ifdef __cplusplus
}
#endif

#endif /* __MSG_LIST_H__ */
