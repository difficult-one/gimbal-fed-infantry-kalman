#ifndef TEST_BSP_CAN_H
#define TEST_BSP_CAN_H
#include <stdint.h>
typedef struct CANInstance {
    uint8_t rx_buff[8], tx_buff[8], rx_len, tx_len;
    void *id;
    void (*can_module_callback)(struct CANInstance *);
} CANInstance;
typedef struct {
    void *can_handle;
    uint32_t tx_id, rx_id;
    void *id;
    void (*can_module_callback)(CANInstance *);
} CAN_Init_Config_s;
CANInstance *CANRegister(CAN_Init_Config_s *config);
void CANSetDLC(CANInstance *instance, uint8_t length);
uint8_t CANTransmit(CANInstance *instance, uint8_t timeout);
#endif
