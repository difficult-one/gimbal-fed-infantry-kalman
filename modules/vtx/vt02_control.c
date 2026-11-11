#include "vt02_control.h"
#include "bsp_usart.h"
#include "referee_crc.h"
#include "daemon.h"
#include "string.h"

#define VT02_RX_BUFFER_SIZE 256u
#define VT02_DAEMON_RELOAD 10 // 约330ms未收到数据视为离线(30Hz)

#define VT02_SOF 0xA5u
#define VT02_HEADER_SIZE 5u
#define VT02_CMD_ID_SIZE 2u
#define VT02_CRC16_SIZE 2u
#define VT02_DATA_OFFSET (VT02_HEADER_SIZE + VT02_CMD_ID_SIZE)
#define VT02_CONTROL_CMD_ID 0x0304u
#define VT02_CONTROL_DATA_SIZE 12u
#define VT02_CONTROL_STANDARD_DATA_SIZE 10u
#define VT02_MIN_FRAME_SIZE (VT02_HEADER_SIZE + VT02_CMD_ID_SIZE + VT02_CRC16_SIZE)
#define VT02_DEBUG_DATA_SIZE 32u

_Static_assert(sizeof(Vt02_Raw_Data_t) == VT02_CONTROL_DATA_SIZE, "VT02 raw data size mismatch");

typedef struct
{
    uint32_t rx_callback_count;
    uint32_t crc8_ok_count;
    uint32_t crc16_ok_count;
    uint32_t control_frame_count;
    uint16_t last_rx_length;
    uint16_t last_data_length;
    uint16_t last_cmd_id;
    uint16_t last_frame_length;
    uint16_t max_rx_length;
    uint16_t rx_history_length;
    uint16_t last_keyboard_value;
    uint16_t last_key_value[3];
    uint16_t key_rise_value;
    uint8_t last_rx_data[VT02_DEBUG_DATA_SIZE];
} Vt02_Debug_t;

/* 调试器可直接监视该变量,用于区分物理接收和协议解析故障. */
volatile Vt02_Debug_t vt02_debug;

static Vt02_Ctrl_t vt02_ctrl;
static USARTInstance *vt02_usart_instance;
static DaemonInstance *vt02_daemon;
static uint16_t vt02_last_key_value[3];
static uint8_t vt02_last_mode_key;
static uint8_t vt02_stream_buff[VT02_RX_BUFFER_SIZE];
static uint16_t vt02_stream_length;
static uint8_t vt02_init_flag;

/**
 * @brief 根据当前帧和上一帧按键状态更新上升沿计数
 *
 * @param current 当前帧按键状态
 * @param key_count 按键上升沿计数数组
 * @retval none
 */
static void Vt02UpdateKeyCount(const uint16_t current[3], uint8_t key_count[3][16])
{
    uint16_t rise_value = current[KEY_PRESS] & (uint16_t)(~vt02_last_key_value[KEY_PRESS]);

    for (uint32_t i = 0, mask = 0x1u; i < 16u; ++i, mask <<= 1u)
    {
        if (((current[KEY_PRESS] & mask) != 0u) &&
            ((vt02_last_key_value[KEY_PRESS] & mask) == 0u) &&
            ((current[KEY_PRESS_WITH_CTRL] & mask) == 0u) &&
            ((current[KEY_PRESS_WITH_SHIFT] & mask) == 0u))
        {
            if (++key_count[KEY_PRESS][i] >= 240u)
                key_count[KEY_PRESS][i] = 0u;
        }

        if (((current[KEY_PRESS_WITH_CTRL] & mask) != 0u) &&
            ((vt02_last_key_value[KEY_PRESS_WITH_CTRL] & mask) == 0u))
        {
            if (++key_count[KEY_PRESS_WITH_CTRL][i] >= 240u)
                key_count[KEY_PRESS_WITH_CTRL][i] = 0u;
        }

        if (((current[KEY_PRESS_WITH_SHIFT] & mask) != 0u) &&
            ((vt02_last_key_value[KEY_PRESS_WITH_SHIFT] & mask) == 0u))
        {
            if (++key_count[KEY_PRESS_WITH_SHIFT][i] >= 240u)
                key_count[KEY_PRESS_WITH_SHIFT][i] = 0u;
        }
    }

    memcpy(vt02_last_key_value, current, sizeof(vt02_last_key_value));
    vt02_debug.last_key_value[KEY_PRESS] = current[KEY_PRESS];
    vt02_debug.last_key_value[KEY_PRESS_WITH_CTRL] = current[KEY_PRESS_WITH_CTRL];
    vt02_debug.last_key_value[KEY_PRESS_WITH_SHIFT] = current[KEY_PRESS_WITH_SHIFT];
    vt02_debug.key_rise_value = rise_value;
}

/**
 * @brief 将16位按键值填充到Key_t位域
 *
 * @param key 按键位域结构体
 * @param value 16位按键值
 * @retval none
 */
static void Vt02SetKey(Key_t *key, uint16_t value)
{
    memset(key, 0, sizeof(Key_t));
    memcpy(key, &value, sizeof(value));
}

/**
 * @brief 将VT02协议键鼠数据填充到模块输出结构体
 *
 * @param raw VT02协议原始数据
 * @retval none
 */
static void Vt02DataDecode(const Vt02_Raw_Data_t *raw)
{
    uint16_t current_key_value[3];
    /* 标准 VT02 键盘位域中 G 为第 10 位；兼容旧扩展协议 reserved bit0。 */
    uint8_t current_mode_key = (uint8_t)(((raw->keyboard_value & (1u << Key_G)) != 0u) ||
                                         ((raw->reserved & 0x0001u) != 0u));

    vt02_ctrl.mouse.x = raw->mouse_x;
    vt02_ctrl.mouse.y = raw->mouse_y;
    vt02_ctrl.mouse.z = raw->mouse_z;
    vt02_ctrl.mouse.press_l = (uint8_t)raw->left_button;
    vt02_ctrl.mouse.press_r = (uint8_t)raw->right_button;

    current_key_value[KEY_PRESS] = raw->keyboard_value;
    vt02_debug.last_keyboard_value = raw->keyboard_value;

    Vt02SetKey(&vt02_ctrl.key[KEY_PRESS], current_key_value[KEY_PRESS]);

    if ((current_key_value[KEY_PRESS] & (1u << Key_Ctrl)) != 0u)
        current_key_value[KEY_PRESS_WITH_CTRL] = current_key_value[KEY_PRESS];
    else
        current_key_value[KEY_PRESS_WITH_CTRL] = 0u;

    if ((current_key_value[KEY_PRESS] & (1u << Key_Shift)) != 0u)
        current_key_value[KEY_PRESS_WITH_SHIFT] = current_key_value[KEY_PRESS];
    else
        current_key_value[KEY_PRESS_WITH_SHIFT] = 0u;

    Vt02SetKey(&vt02_ctrl.key[KEY_PRESS_WITH_CTRL], current_key_value[KEY_PRESS_WITH_CTRL]);
    Vt02SetKey(&vt02_ctrl.key[KEY_PRESS_WITH_SHIFT], current_key_value[KEY_PRESS_WITH_SHIFT]);
    Vt02UpdateKeyCount(current_key_value, vt02_ctrl.key_count);

    if ((current_mode_key != 0u) && (vt02_last_mode_key == 0u))
    {
        if (++vt02_ctrl.key_l_count >= 240u)
            vt02_ctrl.key_l_count = 0u;
    }
    vt02_ctrl.key_l = current_mode_key;
    vt02_last_mode_key = current_mode_key;
}

/**
 * @brief 解析VT02接收缓冲区内的协议帧
 *
 * @param buff 接收数据首地址
 * @param remaining 缓冲区剩余长度
 * @retval uint16_t 已完成解析或可丢弃的数据长度
 */
static uint16_t Vt02ReadData(uint8_t *buff, uint16_t remaining)
{
    uint16_t offset = 0u;
    uint16_t data_length;
    uint16_t cmd_id;
    uint16_t frame_length;
    Vt02_Raw_Data_t raw;

    if ((buff == NULL) || (remaining == 0u))
        return 0u;

    while (offset < remaining)
    {
        while ((offset < remaining) && (buff[offset] != VT02_SOF))
            ++offset;

        if ((remaining - offset) < VT02_HEADER_SIZE)
            return offset;

        if (Verify_CRC8_Check_Sum(buff + offset, VT02_HEADER_SIZE) == TRUE)
            break;

        ++offset;
    }

    if (offset == remaining)
        return remaining;

    buff += offset;
    remaining -= offset;

    data_length = (uint16_t)buff[1] | ((uint16_t)buff[2] << 8u);
    frame_length = data_length + VT02_HEADER_SIZE + VT02_CMD_ID_SIZE + VT02_CRC16_SIZE;
    vt02_debug.crc8_ok_count++;
    vt02_debug.last_data_length = data_length;
    vt02_debug.last_frame_length = frame_length;
    if ((frame_length > VT02_RX_BUFFER_SIZE) || (frame_length < VT02_MIN_FRAME_SIZE))
        return offset + 1u + Vt02ReadData(buff + 1u, remaining - 1u);

    if (frame_length > remaining)
        return offset;

    cmd_id = (uint16_t)buff[5] | ((uint16_t)buff[6] << 8u);
    vt02_debug.last_cmd_id = cmd_id;
    if (Verify_CRC16_Check_Sum(buff, frame_length) == TRUE)
    {
        vt02_debug.crc16_ok_count++;
        if ((cmd_id == VT02_CONTROL_CMD_ID) &&
            (data_length == VT02_CONTROL_STANDARD_DATA_SIZE || data_length == VT02_CONTROL_DATA_SIZE))
        {
            memset(&raw, 0, sizeof(raw));
            memcpy(&raw, buff + VT02_DATA_OFFSET,
                   data_length < sizeof(raw) ? data_length : sizeof(raw));
            Vt02DataDecode(&raw);
            vt02_debug.control_frame_count++;
        }
    }

    if (remaining > frame_length)
        return offset + frame_length + Vt02ReadData(buff + frame_length, remaining - frame_length);

    return offset + frame_length;
}

/**
 * @brief 将本次DMA数据加入流缓存并解析完整帧
 *
 * @param data 本次DMA接收数据
 * @param data_length 本次DMA接收长度
 * @retval none
 */
static void Vt02StreamDecode(const uint8_t *data, uint16_t data_length)
{
    uint16_t consumed;

    if ((data == NULL) || (data_length == 0u))
        return;

    if (data_length > VT02_RX_BUFFER_SIZE)
    {
        data += data_length - VT02_RX_BUFFER_SIZE;
        data_length = VT02_RX_BUFFER_SIZE;
        vt02_stream_length = 0u;
    }

    if ((vt02_stream_length + data_length) > VT02_RX_BUFFER_SIZE)
        vt02_stream_length = 0u;

    memcpy(vt02_stream_buff + vt02_stream_length, data, data_length);
    vt02_stream_length += data_length;

    consumed = Vt02ReadData(vt02_stream_buff, vt02_stream_length);
    if (consumed > 0u)
    {
        vt02_stream_length -= consumed;
        memmove(vt02_stream_buff, vt02_stream_buff + consumed, vt02_stream_length);
    }
}

/**
 * @brief VT02串口接收回调,喂狗后解析协议数据
 *
 * @retval none
 */
static void Vt02RxCallback(void)
{
    UART_HandleTypeDef *usart_handle = vt02_usart_instance->usart_handle;
    uint16_t received_length = usart_handle->RxXferSize - usart_handle->RxXferCount;

    if (received_length > vt02_usart_instance->recv_buff_size)
        received_length = vt02_usart_instance->recv_buff_size;

    vt02_debug.rx_callback_count++;
    vt02_debug.last_rx_length = received_length;
    if (received_length > vt02_debug.max_rx_length)
        vt02_debug.max_rx_length = received_length;

    if (received_length >= VT02_DEBUG_DATA_SIZE)
    {
        for (uint16_t i = 0u; i < VT02_DEBUG_DATA_SIZE; ++i)
            vt02_debug.last_rx_data[i] = vt02_usart_instance->recv_buff[received_length - VT02_DEBUG_DATA_SIZE + i];
        vt02_debug.rx_history_length = VT02_DEBUG_DATA_SIZE;
    }
    else if (received_length > 0u)
    {
        uint16_t discard_length = 0u;
        if ((vt02_debug.rx_history_length + received_length) > VT02_DEBUG_DATA_SIZE)
            discard_length = vt02_debug.rx_history_length + received_length - VT02_DEBUG_DATA_SIZE;

        for (uint16_t i = discard_length; i < vt02_debug.rx_history_length; ++i)
            vt02_debug.last_rx_data[i - discard_length] = vt02_debug.last_rx_data[i];
        vt02_debug.rx_history_length -= discard_length;

        for (uint16_t i = 0u; i < received_length; ++i)
            vt02_debug.last_rx_data[vt02_debug.rx_history_length + i] = vt02_usart_instance->recv_buff[i];
        vt02_debug.rx_history_length += received_length;
    }

    DaemonReload(vt02_daemon);
    vt02_ctrl.online = 1u;
    Vt02StreamDecode(vt02_usart_instance->recv_buff, received_length);
}

/**
 * @brief VT02离线回调,清除在线标志并重启串口接收
 *
 * @param id 守护进程所有者标识
 * @retval none
 */
static void Vt02LostCallback(void *id)
{
    (void)id;
    vt02_ctrl.online = 0u;
    memset(&vt02_ctrl.mouse, 0, sizeof(vt02_ctrl.mouse));
    memset(vt02_ctrl.key, 0, sizeof(vt02_ctrl.key));
    memset(vt02_last_key_value, 0, sizeof(vt02_last_key_value));
    vt02_ctrl.key_l = 0u;
    vt02_last_mode_key = 0u;
    vt02_stream_length = 0u;
    USARTServiceInit(vt02_usart_instance);
}

Vt02_Ctrl_t *Vt02ControlInit(UART_HandleTypeDef *vt02_usart_handle)
{
    memset(&vt02_ctrl, 0, sizeof(vt02_ctrl));
    memset(vt02_last_key_value, 0, sizeof(vt02_last_key_value));
    vt02_last_mode_key = 0u;
    vt02_stream_length = 0u;

    /* DMA FIFO会在IDLE中止时丢弃不足阈值的尾部字节,变长协议使用direct mode接收. */
    if ((vt02_usart_handle != NULL) &&
        (vt02_usart_handle->hdmarx != NULL) &&
        (vt02_usart_handle->hdmarx->Init.FIFOMode != DMA_FIFOMODE_DISABLE))
    {
        vt02_usart_handle->hdmarx->Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        (void)HAL_DMA_Init(vt02_usart_handle->hdmarx);
    }

    USART_Init_Config_s usart_conf = {
        .module_callback = Vt02RxCallback,
        .usart_handle = vt02_usart_handle,
        .recv_buff_size = (uint8_t)(VT02_RX_BUFFER_SIZE - 1u),
    };
    vt02_usart_instance = USARTRegister(&usart_conf);

    Daemon_Init_Config_s daemon_conf = {
        .reload_count = VT02_DAEMON_RELOAD,
        .callback = Vt02LostCallback,
        .owner_id = NULL,
    };
    vt02_daemon = DaemonRegister(&daemon_conf);

    vt02_init_flag = 1u;
    return &vt02_ctrl;
}

uint8_t Vt02ControlIsOnline(void)
{
    if (vt02_init_flag)
        return DaemonIsOnline(vt02_daemon);
    return 0u;
}
