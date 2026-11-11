#include "shoot.h"
#include "robot_msg.h"
#include "config.h"

#include "math.h"
#include "jam_detector.h"
#include "dji_motor.h"
#include "message_center.h"
#include "bsp_dwt.h"
#include "math_units.h"
#include "jscope_rtt.h"
#include "SEGGER_RTT.h"
#include "bsp_log.h"

/* 对于双发射机构的机器人,将下面的数据封装成结构体即可,生成两份shoot应用实例 */
static DJIMotorInstance *friction_l, *friction_r, *loader; // 拨盘电机
static JamDetectorInstance *loader_jam;
// static servo_instance *lid; 需要增加弹舱盖

static Publisher_t *shoot_pub;
static Shoot_Ctrl_Cmd_s shoot_cmd_recv; // 来自cmd的发射控制信息
static Subscriber_t *shoot_sub;
static Shoot_Upload_Data_s shoot_feedback_data; // 来自cmd的发射控制信息

static float loader_target_angle = 0.0f;

void ShootInit()
{
    Motor_Init_Config_s friction_config = g_shoot_config.friction;
    friction_config.can_init_config.can_handle = BSPCANHandle(g_shoot_config.motor_can);
    friction_l = DJIMotorInit(&friction_config);

    friction_config.can_init_config.tx_id = g_shoot_config.friction_right_id;
    friction_config.controller_setting_init_config.motor_reverse_flag = g_shoot_config.friction_right_direction;
    friction_r = DJIMotorInit(&friction_config);

    Motor_Init_Config_s loader_config = g_shoot_config.loader;
    loader_config.can_init_config.can_handle = BSPCANHandle(g_shoot_config.motor_can);
    loader = DJIMotorInit(&loader_config);
    JamDetector_Init_Config_s jam_config = g_shoot_config.jam;
    loader_jam = JamDetectorInit(&jam_config, loader);

    if (g_log_config.shoot_jscope_init)
        g_log_config.shoot_jscope_init(&g_log_config.shoot);

    shoot_pub = PubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));
    shoot_sub = SubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
}

/* 机器人发射机构控制核心任务 */
void ShootTask()
{
    static loader_mode_e last_load_mode = LOAD_STOP;
    static uint8_t jam_handling_active = 0;
    static uint8_t jam_rearm_required = 0;

    // 从cmd获取控制数据
    SubGetMessage(shoot_sub, &shoot_cmd_recv);

    // 单发/三连发执行期间锁存目标角度，避免图传松开鼠标后发送的 LOAD_STOP
    // 在目标到达前把角度环切回速度环。
    uint8_t loader_busy =
        (last_load_mode == LOAD_1_BULLET || last_load_mode == LOAD_3_BULLET) &&
        (fabsf(loader->measure.total_angle - loader_target_angle) > g_shoot_config.loader_tolerance_deg);

    uint8_t jam_detection_active;
    JamState_e jam_state = JAM_NORMAL;

    // 只有"正在发弹/连发/正在处理卡弹"时才检测, 静止时电流瞬态不会被误判
    jam_detection_active = loader_busy ||
                           (shoot_cmd_recv.load_mode == LOAD_BURSTFIRE) ||
                           jam_handling_active;

    // 对shoot mode等于SHOOT_OFF的情况特殊处理,直接停止所有电机(紧急停止)
    if (shoot_cmd_recv.shoot_mode == SHOOT_OFF)
    {
        JamDetectorReset(loader_jam);
        jam_handling_active = 0;
        jam_rearm_required = 0;
        DJIMotorStop(friction_l);
        DJIMotorStop(friction_r);
        DJIMotorStop(loader);
        last_load_mode = LOAD_STOP;
    }
    else // 恢复运行
    {
        DJIMotorEnable(friction_l);
        DJIMotorEnable(friction_r);
        DJIMotorEnable(loader);
    }

    // 轮询卡弹 FSM: 仅在检测使能时运行, 其余情况复位
    if (shoot_cmd_recv.shoot_mode != SHOOT_OFF &&
        loader_jam != NULL && jam_detection_active)
    {
        jam_state = JamDetectorTask(loader_jam);
        if (jam_state == JAM_CONFIRMED || jam_state == JAM_HANDLING)
        {
            jam_handling_active = 1;
            jam_rearm_required = 1;
        }
        else if (jam_handling_active && jam_state == JAM_NORMAL)
        {
            // 回退完成: 退出处理, 但必须等一次 LOAD_STOP 才允许下次触发
            jam_handling_active = 0;
            jam_rearm_required = 1;
            last_load_mode = LOAD_STOP;
            loader_target_angle = loader->measure.total_angle;
            JamDetectorReset(loader_jam);
            loader_busy = 0;
        }
    }
    else if (loader_jam != NULL)
    {
        JamDetectorReset(loader_jam);
        jam_handling_active = 0;
    }

    // 常规拨盘控制: 卡弹处理中 / 等待重新装弹 / 单发三发未到位时都不执行, 避免两个控制器抢写参考值
    if (shoot_cmd_recv.shoot_mode != SHOOT_OFF &&
        !jam_handling_active &&
        !jam_rearm_required &&
        !(loader_busy &&
          (last_load_mode == LOAD_1_BULLET || last_load_mode == LOAD_3_BULLET)))
    {
        switch (shoot_cmd_recv.load_mode)
        {
    // 停止拨盘
    case LOAD_STOP:
        DJIMotorOuterLoop(loader, SPEED_LOOP); // 切换到速度环
        DJIMotorSetRef(loader, 0);             // 同时设定参考值为0,这样停止的速度最快
        break;
    // 单发模式,根据鼠标按下的时间,触发一次之后需要进入不响应输入的状态(否则按下的时间内可能多次进入,导致多次发射)
    case LOAD_1_BULLET:                                                                     // 激活能量机关/干扰对方用,英雄用.
        if (!loader_busy && last_load_mode != LOAD_1_BULLET)
        {
            DJIMotorOuterLoop(loader, ANGLE_LOOP); // 切换到角度环
            loader_target_angle = loader->measure.total_angle + (g_shoot_config.loader_shot_angle_deg * g_shoot_config.loader_reduction);
            DJIMotorSetRef(loader, loader_target_angle);
        }
        break;
    // 三连发,如果不需要后续可能删除
    case LOAD_3_BULLET:
        if (!loader_busy && last_load_mode != LOAD_3_BULLET)
        {
            DJIMotorOuterLoop(loader, ANGLE_LOOP);
            loader_target_angle = loader->measure.total_angle + 3 * (g_shoot_config.loader_shot_angle_deg * g_shoot_config.loader_reduction);
            DJIMotorSetRef(loader, loader_target_angle);
        }
        break;
    // 连发模式,对速度闭环,射频后续修改为可变,目前固定为1Hz
    case LOAD_BURSTFIRE:
        DJIMotorOuterLoop(loader, SPEED_LOOP);
        DJIMotorSetRef(loader, shoot_cmd_recv.shoot_rate * 360 * g_shoot_config.loader_reduction / g_shoot_config.shots_per_turn);
        // x颗/秒换算成速度: 已知一圈的载弹量,由此计算出1s需要转的角度,注意换算角速度(DJIMotor的速度单位是angle per second)
        break;
    // 拨盘反转,对速度闭环,后续增加卡弹检测(通过裁判系统剩余热量反馈和电机电流)
    // 也有可能需要从switch-case中独立出来
    case LOAD_REVERSE:
        DJIMotorOuterLoop(loader, SPEED_LOOP);
        break;
    default:
        while (1)
            ; // 未知模式,停止运行,检查指针越界,内存溢出等问题
        }
    }

    // 触发沿锁存 + re-arm: 卡弹回退完成后必须等到一次 LOAD_STOP 才解除锁定
    if (shoot_cmd_recv.shoot_mode != SHOOT_OFF && !loader_busy)
        last_load_mode = shoot_cmd_recv.load_mode;
    else if (shoot_cmd_recv.shoot_mode == SHOOT_OFF)
        last_load_mode = LOAD_STOP;

    if (!jam_handling_active && jam_rearm_required &&
        shoot_cmd_recv.load_mode == LOAD_STOP)
    {
        jam_rearm_required = 0;
    }

    // 确定是否开启摩擦轮,后续可能修改为键鼠模式下始终开启摩擦轮(上场时建议一直开启)
    if (shoot_cmd_recv.friction_mode == FRICTION_ON)
    {
        // 根据收到的弹速设置设定摩擦轮电机参考值,需实测后填入
        switch (shoot_cmd_recv.bullet_speed)
        {
        case SMALL_AMU_15:
            DJIMotorSetRef(friction_l, g_shoot_config.friction_15_dps);
            DJIMotorSetRef(friction_r, g_shoot_config.friction_15_dps);
            break;
        case SMALL_AMU_18:
            DJIMotorSetRef(friction_l, g_shoot_config.friction_18_dps);
            DJIMotorSetRef(friction_r, g_shoot_config.friction_18_dps);
            break;
        case SMALL_AMU_30:
            DJIMotorSetRef(friction_l, g_shoot_config.friction_30_dps);
            DJIMotorSetRef(friction_r, g_shoot_config.friction_30_dps);
            break;
        default: // 无匹配弹速档位时使用调试参考值。
            DJIMotorSetRef(friction_l, g_shoot_config.friction_default_dps);
            DJIMotorSetRef(friction_r, g_shoot_config.friction_default_dps);
            break;
        }
    }
    else // 关闭摩擦轮
    {
        DJIMotorSetRef(friction_l, 0);
        DJIMotorSetRef(friction_r, 0);
    }

    if (g_log_config.shoot_jscope_send)
        g_log_config.shoot_jscope_send(friction_l->motor_controller.pid_ref,
                         friction_l->measure.speed_aps,
                         friction_r->motor_controller.pid_ref,
                         friction_r->measure.speed_aps);

    // 开关弹舱盖
    if (shoot_cmd_recv.lid_mode == LID_CLOSE)
    {
        //...
    }
    else if (shoot_cmd_recv.lid_mode == LID_OPEN)
    {
        //...
    }

    if (g_log_config.shoot_debug_enabled)
    {
        // 调试:打印拨弹盘角度/目标角度/速度/电流和卡弹状态。
        // 角度乘10输出，避免依赖 RTT printf 的浮点格式支持。
        static uint32_t shoot_dbg_cnt = 0;
        static JamState_e last_jam_state = JAM_NORMAL;
        if (jam_state != last_jam_state)
        {
            SEGGER_RTT_printf(g_chassis_log_config.text_rtt_channel, "[jam] t=%d state=%d ang10=%d target10=%d spd=%d cur=%d\r\n",
                              (int)DWT_GetTimeline_ms(),
                              (int)jam_state,
                              (int)(loader->measure.total_angle * 10.0f),
                              (int)((loader_jam != NULL ? loader_jam->handling_target_angle : 0.0f) * 10.0f),
                              (int)loader->measure.speed_aps,
                              (int)loader->measure.real_current);
            last_jam_state = jam_state;
        }
        if (++shoot_dbg_cnt >= g_log_config.shoot_debug_period)
        {
            shoot_dbg_cnt = 0;
            SEGGER_RTT_printf(g_chassis_log_config.text_rtt_channel, "[loader] t=%d load=%d jam=%d ang10=%d target10=%d ref10=%d spd=%d cur=%d\r\n",
                              (int)DWT_GetTimeline_ms(),
                              (int)shoot_cmd_recv.load_mode,
                              (int)jam_state,
                              (int)(loader->measure.total_angle * 10.0f),
                              (int)((loader_jam != NULL ? loader_jam->handling_target_angle : 0.0f) * 10.0f),
                              (int)(loader->motor_controller.pid_ref * 10.0f),
                              (int)loader->measure.speed_aps,
                              (int)loader->measure.real_current);
           /* SEGGER_RTT_printf(g_chassis_log_config.text_rtt_channel, "[shoot] mod=%d fric=%d ref=%d lSpd=%d rSpd=%d lCur=%d rCur=%d jam=%d\r\n",
                              (int)shoot_cmd_recv.shoot_mode,
                              (int)shoot_cmd_recv.friction_mode,
                              (int)friction_l->motor_controller.pid_ref,
                              (int)friction_l->measure.speed_aps,
                              (int)friction_r->measure.speed_aps,
                              (int)friction_l->measure.real_current,
                              (int)friction_r->measure.real_current,
                              (int)jam_state);*/
        }
    }

    // 反馈数据,目前暂时没有要设定的反馈数据,后续可能增加应用离线监测以及卡弹反馈
    PubPushMessage(shoot_pub, (void *)&shoot_feedback_data);
}
