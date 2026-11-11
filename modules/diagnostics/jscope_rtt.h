#ifndef JSCOPE_RTT_H
#define JSCOPE_RTT_H

#include <stdint.h>

typedef struct
{
    uint8_t channel;
    const char *buffer_name;
    const char *csv_header;
    uint8_t csv_enable;
    uint16_t csv_div;
    const unsigned *csv_channel; // 初始化时读取可选文本通道，NULL 使用默认通道。
} RTTDebugJScope2FConfig_t;

void RTTDebugJScope2FInit(const RTTDebugJScope2FConfig_t *config);
void RTTDebugJScope2FSend(float value0, float value1);

typedef struct
{
    uint8_t channel;
    const char *buffer_name;
    const char *csv_header;
    uint8_t csv_enable;
    uint16_t csv_div;
    const unsigned *csv_channel; // 初始化时读取可选文本通道，NULL 使用默认通道。
} RTTDebugJScope4FConfig_t;

void RTTDebugJScope4FInit(const RTTDebugJScope4FConfig_t *config);
void RTTDebugJScope4FSend(float v0, float v1, float v2, float v3);

typedef struct
{
    uint8_t channel;
    const char *buffer_name;
    const char *csv_header;
    uint8_t csv_enable;
    uint16_t csv_div;
    const unsigned *csv_channel; // 初始化时读取可选文本通道，NULL 使用默认通道。
} RTTDebugJScope6FConfig_t;

void RTTDebugJScope6FInit(const RTTDebugJScope6FConfig_t *config);
void RTTDebugJScope6FSend(float v0, float v1, float v2, float v3, float v4, float v5);

#endif
