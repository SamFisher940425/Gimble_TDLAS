#include "msg_list.h"

static uint16_t g_rangefinder_msg_r_index = 0;
static uint16_t g_rangefinder_msg_w_index = 0;
static Rangefinder_Msg g_rangefinder_msg_buf[RANGERFINDER_MSG_MAX_SIZE];

static uint16_t g_tdlas_tx_msg_r_index = 0;
static uint16_t g_tdlas_tx_msg_w_index = 0;
static TDLAS_Tx_Msg g_tdlas_tx_msg_buf[TDLAS_TX_MSG_MAX_SIZE];

static uint16_t g_tdlas_rx_msg_r_index = 0;
static uint16_t g_tdlas_rx_msg_w_index = 0;
static TDLAS_Rx_Msg g_tdlas_rx_msg_buf[TDLAS_TX_MSG_MAX_SIZE];

static uint16_t g_ctrl_tx_msg_r_index = 0;
static uint16_t g_ctrl_tx_msg_w_index = 0;
static Ctrl_Com_Msg g_ctrl_tx_msg_buf[CTRL_TX_MSG_MAX_SIZE];

static uint16_t g_ctrl_rx_msg_r_index = 0;
static uint16_t g_ctrl_rx_msg_w_index = 0;
static Ctrl_Com_Msg g_ctrl_rx_msg_buf[CTRL_RX_MSG_MAX_SIZE];

int8_t Rangefinder_Msg_Add(Rangefinder_Msg *msg)
{
    if (((g_rangefinder_msg_w_index + 1) % RANGERFINDER_MSG_MAX_SIZE) == g_rangefinder_msg_r_index)
    {
        return -1;
    }

    g_rangefinder_msg_buf[g_rangefinder_msg_w_index].addr = msg->addr;
    g_rangefinder_msg_buf[g_rangefinder_msg_w_index].func_code = msg->func_code;
    g_rangefinder_msg_buf[g_rangefinder_msg_w_index].data_len = msg->data_len;
    for (uint8_t i = 0; i < 4; i++)
    {
        g_rangefinder_msg_buf[g_rangefinder_msg_w_index].data[i] = msg->data[i];
    }
    g_rangefinder_msg_buf[g_rangefinder_msg_w_index].crc_l = msg->crc_l;
    g_rangefinder_msg_buf[g_rangefinder_msg_w_index].crc_h = msg->crc_h;
    g_rangefinder_msg_w_index = (g_rangefinder_msg_w_index + 1) % RANGERFINDER_MSG_MAX_SIZE;
    return 0;
}

int8_t Rangerfinder_Msg_Get(Rangefinder_Msg *msg)
{
    if (g_rangefinder_msg_r_index == g_rangefinder_msg_w_index)
    {
        return -1;
    }

    msg->addr = g_rangefinder_msg_buf[g_rangefinder_msg_r_index].addr;
    msg->func_code = g_rangefinder_msg_buf[g_rangefinder_msg_r_index].func_code;
    msg->data_len = g_rangefinder_msg_buf[g_rangefinder_msg_r_index].data_len;
    for (uint8_t i = 0; i < 4; i++)
    {
        msg->data[i] = g_rangefinder_msg_buf[g_rangefinder_msg_r_index].data[i];
    }
    msg->crc_l = g_rangefinder_msg_buf[g_rangefinder_msg_r_index].crc_l;
    msg->crc_h = g_rangefinder_msg_buf[g_rangefinder_msg_r_index].crc_h;
    g_rangefinder_msg_r_index = (g_rangefinder_msg_r_index + 1) % RANGERFINDER_MSG_MAX_SIZE;
    return 0;
}

int8_t TDLAS_Tx_Msg_Add(TDLAS_Tx_Msg *msg)
{
    if (((g_tdlas_tx_msg_w_index + 1) % TDLAS_TX_MSG_MAX_SIZE) == g_tdlas_tx_msg_r_index)
    {
        return -1;
    }

    g_tdlas_tx_msg_buf[g_tdlas_tx_msg_w_index].addr = msg->addr;
    g_tdlas_tx_msg_buf[g_tdlas_tx_msg_w_index].func_code = msg->func_code;
    g_tdlas_tx_msg_buf[g_tdlas_tx_msg_w_index].reg_addr = msg->reg_addr;
    g_tdlas_tx_msg_buf[g_tdlas_tx_msg_w_index].reg_cnt = msg->reg_cnt;
    g_tdlas_tx_msg_buf[g_tdlas_tx_msg_w_index].crc_l = msg->crc_l;
    g_tdlas_tx_msg_buf[g_tdlas_tx_msg_w_index].crc_h = msg->crc_h;
    g_tdlas_tx_msg_buf[g_tdlas_tx_msg_w_index].data_len = msg->data_len;
    g_tdlas_tx_msg_buf[g_tdlas_tx_msg_w_index].data[0] = msg->data[0];
    g_tdlas_tx_msg_buf[g_tdlas_tx_msg_w_index].data[1] = msg->data[1];
    g_tdlas_tx_msg_w_index = (g_tdlas_tx_msg_w_index + 1) % TDLAS_TX_MSG_MAX_SIZE;
    return 0;
}

int8_t TDLAS_Tx_Msg_Get(TDLAS_Tx_Msg *msg)
{
    if (g_tdlas_tx_msg_r_index == g_tdlas_tx_msg_w_index)
    {
        return -1;
    }

    msg->addr = g_tdlas_tx_msg_buf[g_tdlas_tx_msg_r_index].addr;
    msg->func_code = g_tdlas_tx_msg_buf[g_tdlas_tx_msg_r_index].func_code;
    msg->reg_addr = g_tdlas_tx_msg_buf[g_tdlas_tx_msg_r_index].reg_addr;
    msg->reg_cnt = g_tdlas_tx_msg_buf[g_tdlas_tx_msg_r_index].reg_cnt;
    msg->crc_l = g_tdlas_tx_msg_buf[g_tdlas_tx_msg_r_index].crc_l;
    msg->crc_h = g_tdlas_tx_msg_buf[g_tdlas_tx_msg_r_index].crc_h;
    msg->data_len = g_tdlas_tx_msg_buf[g_tdlas_tx_msg_r_index].data_len;
    msg->data[0] = g_tdlas_tx_msg_buf[g_tdlas_tx_msg_r_index].data[0];
    msg->data[1] = g_tdlas_tx_msg_buf[g_tdlas_tx_msg_r_index].data[1];
    g_tdlas_tx_msg_r_index = (g_tdlas_tx_msg_r_index + 1) % TDLAS_TX_MSG_MAX_SIZE;
    return 0;
}

int8_t TDLAS_Rx_Msg_Add(TDLAS_Rx_Msg *msg)
{
    if (((g_tdlas_rx_msg_w_index + 1) % TDLAS_RX_MSG_MAX_SIZE) == g_tdlas_rx_msg_r_index)
    {
        return -1;
    }

    g_tdlas_rx_msg_buf[g_tdlas_rx_msg_w_index].addr = msg->addr;
    g_tdlas_rx_msg_buf[g_tdlas_rx_msg_w_index].func_code = msg->func_code;
    g_tdlas_rx_msg_buf[g_tdlas_rx_msg_w_index].data_len = msg->data_len;
    for (uint8_t i = 0; i < 8; i++)
    {
        g_tdlas_rx_msg_buf[g_tdlas_rx_msg_w_index].data[i] = msg->data[i];
    }
    g_tdlas_rx_msg_buf[g_tdlas_rx_msg_w_index].crc_l = msg->crc_l;
    g_tdlas_rx_msg_buf[g_tdlas_rx_msg_w_index].crc_h = msg->crc_h;
    g_tdlas_rx_msg_buf[g_tdlas_rx_msg_w_index].reg_addr = msg->reg_addr;
    g_tdlas_rx_msg_buf[g_tdlas_rx_msg_w_index].reg_cnt = msg->reg_cnt;
    g_tdlas_rx_msg_w_index = (g_tdlas_rx_msg_w_index + 1) % TDLAS_RX_MSG_MAX_SIZE;
    return 0;
}

int8_t TDLAS_Rx_Msg_Get(TDLAS_Rx_Msg *msg)
{
    if (g_tdlas_rx_msg_r_index == g_tdlas_rx_msg_w_index)
    {
        return -1;
    }

    msg->addr = g_tdlas_rx_msg_buf[g_tdlas_rx_msg_r_index].addr;
    msg->func_code = g_tdlas_rx_msg_buf[g_tdlas_rx_msg_r_index].func_code;
    msg->data_len = g_tdlas_rx_msg_buf[g_tdlas_rx_msg_r_index].data_len;
    for (uint8_t i = 0; i < 8; i++)
    {
        msg->data[i] = g_tdlas_rx_msg_buf[g_tdlas_rx_msg_r_index].data[i];
    }
    msg->crc_l = g_tdlas_rx_msg_buf[g_tdlas_rx_msg_r_index].crc_l;
    msg->crc_h = g_tdlas_rx_msg_buf[g_tdlas_rx_msg_r_index].crc_h;
    msg->reg_addr = g_tdlas_rx_msg_buf[g_tdlas_rx_msg_r_index].reg_addr;
    msg->reg_cnt = g_tdlas_rx_msg_buf[g_tdlas_rx_msg_r_index].reg_cnt;
    g_tdlas_rx_msg_r_index = (g_tdlas_rx_msg_r_index + 1) % TDLAS_RX_MSG_MAX_SIZE;
    return 0;
}

int8_t Ctrl_Tx_Msg_Add(Ctrl_Com_Msg *msg)
{
    if (((g_ctrl_tx_msg_w_index + 1) % CTRL_TX_MSG_MAX_SIZE) == g_ctrl_tx_msg_r_index)
    {
        return -1;
    }

    g_ctrl_tx_msg_buf[g_ctrl_tx_msg_w_index].head_1 = msg->head_1;
    g_ctrl_tx_msg_buf[g_ctrl_tx_msg_w_index].head_2 = msg->head_2;
    g_ctrl_tx_msg_buf[g_ctrl_tx_msg_w_index].src_id = msg->src_id;
    g_ctrl_tx_msg_buf[g_ctrl_tx_msg_w_index].dst_id = msg->dst_id;
    g_ctrl_tx_msg_buf[g_ctrl_tx_msg_w_index].func_code = msg->func_code;
    g_ctrl_tx_msg_buf[g_ctrl_tx_msg_w_index].data_len = msg->data_len;
    for (uint8_t i = 0; i < 32; i++)
    {
        g_ctrl_tx_msg_buf[g_ctrl_tx_msg_w_index].data[i] = msg->data[i];
    }
    g_ctrl_tx_msg_buf[g_ctrl_tx_msg_w_index].check = msg->check;
    g_ctrl_tx_msg_buf[g_ctrl_tx_msg_w_index].tail_1 = msg->tail_1;
    g_ctrl_tx_msg_buf[g_ctrl_tx_msg_w_index].tail_2 = msg->tail_2;
    g_ctrl_tx_msg_w_index = (g_ctrl_tx_msg_w_index + 1) % CTRL_TX_MSG_MAX_SIZE;
    return 0;
}

int8_t Ctrl_Tx_Msg_Get(Ctrl_Com_Msg *msg)
{
    if (g_ctrl_tx_msg_r_index == g_ctrl_tx_msg_w_index)
    {
        return -1;
    }

    msg->head_1 = g_ctrl_tx_msg_buf[g_ctrl_tx_msg_r_index].head_1;
    msg->head_2 = g_ctrl_tx_msg_buf[g_ctrl_tx_msg_r_index].head_2;
    msg->src_id = g_ctrl_tx_msg_buf[g_ctrl_tx_msg_r_index].src_id;
    msg->dst_id = g_ctrl_tx_msg_buf[g_ctrl_tx_msg_r_index].dst_id;
    msg->func_code = g_ctrl_tx_msg_buf[g_ctrl_tx_msg_r_index].func_code;
    msg->data_len = g_ctrl_tx_msg_buf[g_ctrl_tx_msg_r_index].data_len;
    for (uint8_t i = 0; i < 32; i++)
    {
        msg->data[i] = g_ctrl_tx_msg_buf[g_ctrl_tx_msg_r_index].data[i];
    }
    msg->check = g_ctrl_tx_msg_buf[g_ctrl_tx_msg_r_index].check;
    msg->tail_1 = g_ctrl_tx_msg_buf[g_ctrl_tx_msg_r_index].tail_1;
    msg->tail_2 = g_ctrl_tx_msg_buf[g_ctrl_tx_msg_r_index].tail_2;
    g_ctrl_tx_msg_r_index = (g_ctrl_tx_msg_r_index + 1) % CTRL_TX_MSG_MAX_SIZE;
    return 0;
}

int8_t Ctrl_Rx_Msg_Add(Ctrl_Com_Msg *msg)
{
    if (((g_ctrl_rx_msg_w_index + 1) % CTRL_RX_MSG_MAX_SIZE) = g_ctrl_rx_msg_r_index)
    {
        return -1;
    }

    g_ctrl_rx_msg_buf[g_ctrl_rx_msg_w_index].head_1 = msg->head_1;
    g_ctrl_rx_msg_buf[g_ctrl_rx_msg_w_index].head_2 = msg->head_2;
    g_ctrl_rx_msg_buf[g_ctrl_rx_msg_w_index].src_id = msg->src_id;
    g_ctrl_rx_msg_buf[g_ctrl_rx_msg_w_index].dst_id = msg->dst_id;
    g_ctrl_rx_msg_buf[g_ctrl_rx_msg_w_index].func_code = msg->func_code;
    g_ctrl_rx_msg_buf[g_ctrl_rx_msg_w_index].data_len = msg->data_len;
    for (uint8_t i = 0; i < 32; i++)
    {
        g_ctrl_rx_msg_buf[g_ctrl_rx_msg_w_index].data[i] = msg->data[i];
    }
    g_ctrl_rx_msg_buf[g_ctrl_rx_msg_w_index].check = msg->check;
    g_ctrl_rx_msg_buf[g_ctrl_rx_msg_w_index].tail_1 = msg->tail_1;
    g_ctrl_rx_msg_buf[g_ctrl_rx_msg_w_index].tail_2 = msg->tail_2;
    g_ctrl_rx_msg_w_index = (g_ctrl_rx_msg_w_index + 1) % CTRL_RX_MSG_MAX_SIZE;
    return 0;
}

int8_t Ctrl_Rx_Msg_Get(Ctrl_Com_Msg *msg)
{
    if (g_ctrl_rx_msg_r_index == g_ctrl_rx_msg_w_index)
    {
        return -1;
    }

    msg->head_1 = g_ctrl_rx_msg_buf[g_ctrl_rx_msg_r_index].head_1;
    msg->head_2 = g_ctrl_rx_msg_buf[g_ctrl_rx_msg_r_index].head_2;
    msg->src_id = g_ctrl_rx_msg_buf[g_ctrl_rx_msg_r_index].src_id;
    msg->dst_id = g_ctrl_rx_msg_buf[g_ctrl_rx_msg_r_index].dst_id;
    msg->func_code = g_ctrl_rx_msg_buf[g_ctrl_rx_msg_r_index].func_code;
    msg->data_len = g_ctrl_rx_msg_buf[g_ctrl_rx_msg_r_index].data_len;
    for (uint8_t i = 0; i < 32; i++)
    {
        msg->data[i] = g_ctrl_rx_msg_buf[g_ctrl_rx_msg_r_index].data[i];
    }
    msg->check = g_ctrl_rx_msg_buf[g_ctrl_rx_msg_r_index].check;
    msg->tail_1 = g_ctrl_rx_msg_buf[g_ctrl_rx_msg_r_index].tail_1;
    msg->tail_2 = g_ctrl_rx_msg_buf[g_ctrl_rx_msg_r_index].tail_2;
    g_ctrl_rx_msg_r_index = (g_ctrl_rx_msg_r_index + 1) % CTRL_RX_MSG_MAX_SIZE;
    return 0;
}
