#include "bsp_log.h"

#include "SEGGER_RTT.h"
#include "SEGGER_RTT_Conf.h"
#include <stdio.h>

#if 0 /* 暂停通道 2，连同文本缓冲区一起屏蔽。 */
static char text_rtt_buffer[1024];
#endif

void BSPLogInit(unsigned motion_channel)
{
    SEGGER_RTT_Init();
#if 0 /* 当前只注册通道 0；文本通道暂不启用。 */
    SEGGER_RTT_ConfigUpBuffer(BUFFER_INDEX, "TextLog", text_rtt_buffer,
                              sizeof(text_rtt_buffer), SEGGER_RTT_MODE_NO_BLOCK_SKIP);
#endif
    /* 通道 0 缓冲区由 SEGGER_RTT_Conf.h 的 BUFFER_SIZE_UP 编译期分配。 */
    SEGGER_RTT_SetNameUpBuffer(motion_channel, "ChassisMotion_Text");
    SEGGER_RTT_SetFlagsUpBuffer(motion_channel, SEGGER_RTT_MODE_NO_BLOCK_SKIP);
}

int PrintLog(const char *fmt, ...)
{
#if 0 /* 暂停所有通过 PrintLog 发送的通道 2 日志。 */
    va_list args;
    va_start(args, fmt);
    int n = SEGGER_RTT_vprintf(BUFFER_INDEX, fmt, &args); // 一次可以开启多个buffer(多个终端),我们只用一个
    va_end(args);
    return n;
#else
    (void)fmt;
    return 0;
#endif
}

void Float2Str(char *str, float va)
{
    int flag = va < 0;
    int head = (int)va;
    int point = (int)((va - head) * 1000);
    head = abs(head);
    point = abs(point);
    if (flag)
        sprintf(str, "-%d.%d", head, point);
    else
        sprintf(str, "%d.%d", head, point);
}

