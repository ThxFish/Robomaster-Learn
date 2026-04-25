/**
 * @file controller.c
 * @author wanghongxi
 * @author modified by neozng
 * @brief  PID控制器定义与各种工业级优化算法的实现。
 *         简单的位置式 PID 公式: Output = Kp*err + Ki*∫err*dt + Kd*(d_err/dt)
 *         但在RoboMaster赛场上，要想云台稳得像锁死一样，或底盘响应快且不超调，只用裸PID是不可能的。
 *         下面实现了一整套完整的、可通过位掩码 (Improve) 动态启停的工业级 PID 优化环节。
 * @version beta
 * @date 2022-11-01
 *
 * @copyrightCopyright (c) 2022 HNU YueLu EC all rights reserved
 */
#include "controller.h"
#include "memory.h"

/* ----------------------------下面是pid优化环节的实现---------------------------- */

// 【优化环节1】梯形积分(Trapezoid Intergral)
// 裸PID是矩形积分(err*dt)。梯形积分将上次和本次误差取平均再乘dt，减小了离散化带来的积分误差。
static void f_Trapezoid_Intergral(PIDInstance *pid)
{
    // 计算梯形的面积,(上底+下底)*高/2
    pid->ITerm = pid->Ki * ((pid->Err + pid->Last_Err) / 2) * pid->dt;
}

// 【优化环节2】变速积分(Changing Integration Rate) / 积分分离
// 当误差(Err)很大时，往往处于剧烈启动阶段，如果死命积分，等到达目标时积分项会积得特别大，从而导致系统严重超调(震荡)。
// 变速积分的逻辑：如果误差小于 CoefB（极近区），全额积分；
// 如果处于 CoefB ~ CoefA+CoefB 之间，按比例削弱积分能力；如果大于这个阈值，直接不积分(积分分离)。
static void f_Changing_Integration_Rate(PIDInstance *pid)
{
    if (pid->Err * pid->Iout > 0)
    {
        // 积分呈累积趋势
        if (abs(pid->Err) <= pid->CoefB)
            return; // Full integral
        if (abs(pid->Err) <= (pid->CoefA + pid->CoefB))
            pid->ITerm *= (pid->CoefA - abs(pid->Err) + pid->CoefB) / pid->CoefA;
        else // 最大阈值,不使用积分
            pid->ITerm = 0;
    }
}

// 【优化环节3】积分抗饱和(Integral Limit)
// 经典的 Anti-Windup 算法。当系统的输出(P+I+D)已经达到执行执行机构的物理上限(比如给电机满电流MaxOut了)，
// 但因为负载太重系统还到不了目标位置，这个时候如果不做限制，I 项会朝着无限大疯狂堆积。
// 当撤去重负载时，无限大的 I 项会让电机疯狂反转飞车。此函数死死掐住 I 值的极大值。
static void f_Integral_Limit(PIDInstance *pid)
{
    static float temp_Output, temp_Iout;
    temp_Iout = pid->Iout + pid->ITerm;
    temp_Output = pid->Pout + pid->Iout + pid->Dout;
    if (abs(temp_Output) > pid->MaxOut)
    {
        if (pid->Err * pid->Iout > 0) // 积分却还在累积
        {
            pid->ITerm = 0; // 当前积分项置零
        }
    }

    if (temp_Iout > pid->IntegralLimit)
    {
        pid->ITerm = 0;
        pid->Iout = pid->IntegralLimit;
    }
    if (temp_Iout < -pid->IntegralLimit)
    {
        pid->ITerm = 0;
        pid->Iout = -pid->IntegralLimit;
    }
}

// 【优化环节4】微分先行(Derivative On Measurement)
// 标准的 D 项是求 d(Error)/dt。但如果外部遥控器操作手手抖，导致参考指令 (Ref) 有个方波突变，
// 那么 Error 瞬间的变化率极高，就会导致 D 项产生一个恐怖的“微分突刺输出”。
// 实际上物理模型的位置 (Measure) 是连续且平滑的。把 D 改成 -d(Measure)/dt 就能避免这种剧烈冲击产生所谓“微分尖峰”。
static void f_Derivative_On_Measurement(PIDInstance *pid)
{
    pid->Dout = pid->Kd * (pid->Last_Measure - pid->Measure) / pid->dt;
}

// 【优化环节5】对极度敏感的 D 测值本身加一阶低通滤波 (Derivative Filter)
// 因为陀螺仪或者微小的震动，会引发噪声被高频放大，这里RC滤波进一步平滑 D 项波折
static void f_Derivative_Filter(PIDInstance *pid)
{
    pid->Dout = pid->Dout * pid->dt / (pid->Derivative_LPF_RC + pid->dt) +
                pid->Last_Dout * pid->Derivative_LPF_RC / (pid->Derivative_LPF_RC + pid->dt);
}

// 【优化环节6】输出低通滤波 (Output Filter)，适合用来抑制系统高频震颤抖动
static void f_Output_Filter(PIDInstance *pid)
{
    pid->Output = pid->Output * pid->dt / (pid->Output_LPF_RC + pid->dt) +
                  pid->Last_Output * pid->Output_LPF_RC / (pid->Output_LPF_RC + pid->dt);
}

// 输出限幅
static void f_Output_Limit(PIDInstance *pid)
{
    if (pid->Output > pid->MaxOut)
    {
        pid->Output = pid->MaxOut;
    }
    if (pid->Output < -(pid->MaxOut))
    {
        pid->Output = -(pid->MaxOut);
    }
}

// 电机堵转检测
static void f_PID_ErrorHandle(PIDInstance *pid)
{
    /*Motor Blocked Handle*/
    if (fabsf(pid->Output) < pid->MaxOut * 0.001f || fabsf(pid->Ref) < 0.0001f)
        return;

    if ((fabsf(pid->Ref - pid->Measure) / fabsf(pid->Ref)) > 0.95f)
    {
        // Motor blocked counting
        pid->ERRORHandler.ERRORCount++;
    }
    else
    {
        pid->ERRORHandler.ERRORCount = 0;
    }

    if (pid->ERRORHandler.ERRORCount > 500)
    {
        // Motor blocked over 1000times
        pid->ERRORHandler.ERRORType = PID_MOTOR_BLOCKED_ERROR;
    }
}

/* ---------------------------下面是PID的外部算法接口--------------------------- */

/**
 * @brief 初始化PID,设置参数和启用的优化环节,将其他数据置零
 *
 * @param pid    PID实例
 * @param config PID初始化设置
 */
void PIDInit(PIDInstance *pid, PID_Init_Config_s *config)
{
    // config的数据和pid的部分数据是连续且相同的的,所以可以直接用memcpy
    // @todo: 不建议这样做,可扩展性差,不知道的开发者可能会误以为pid和config是同一个结构体
    // 后续修改为逐个赋值
    memset(pid, 0, sizeof(PIDInstance));
    // utilize the quality of struct that its memeory is continuous
    memcpy(pid, config, sizeof(PID_Init_Config_s));
    // set rest of memory to 0
    DWT_GetDeltaT(&pid->DWT_CNT);
}

/**
 * @brief  PID闭环强计算核心：把一堆优化器给串上
 *
 * @param  pid        PID结构体句柄
 * @param  measure    传感器或下一级传来的此刻实际值（测量值/反馈值）
 * @param  ref        遥控器或控制器期望它的目标值（期望值/理想值）
 * @retval float      计算完成的本次驱动控制数值量 (比如应该输出的安培数或者RPM数字)
 */
float PIDCalculate(PIDInstance *pid, float measure, float ref)
{
    // 堵转检测
    if (pid->Improve & PID_ErrorHandle)
        f_PID_ErrorHandle(pid);

    pid->dt = DWT_GetDeltaT(&pid->DWT_CNT); // 获取两次pid计算的时间间隔,用于积分和微分

    // 保存上次的测量值和误差,计算当前error
    pid->Measure = measure;
    pid->Ref = ref;
    pid->Err = pid->Ref - pid->Measure;

    // 如果在死区外,则计算PID
    if (abs(pid->Err) > pid->DeadBand)
    {
        // 基本的pid计算,使用位置式
        pid->Pout = pid->Kp * pid->Err;
        pid->ITerm = pid->Ki * pid->Err * pid->dt;
        pid->Dout = pid->Kd * (pid->Err - pid->Last_Err) / pid->dt;

        // 梯形积分
        if (pid->Improve & PID_Trapezoid_Intergral)
            f_Trapezoid_Intergral(pid);
        // 变速积分
        if (pid->Improve & PID_ChangingIntegrationRate)
            f_Changing_Integration_Rate(pid);
        // 微分先行
        if (pid->Improve & PID_Derivative_On_Measurement)
            f_Derivative_On_Measurement(pid);
        // 微分滤波器
        if (pid->Improve & PID_DerivativeFilter)
            f_Derivative_Filter(pid);
        // 积分限幅
        if (pid->Improve & PID_Integral_Limit)
            f_Integral_Limit(pid);

        pid->Iout += pid->ITerm;                         // 累加积分
        pid->Output = pid->Pout + pid->Iout + pid->Dout; // 计算输出

        // 输出滤波
        if (pid->Improve & PID_OutputFilter)
            f_Output_Filter(pid);

        // 输出限幅
        f_Output_Limit(pid);
    }
    else // 进入死区, 则清空积分和输出
    {
        pid->Output = 0;
        pid->ITerm = 0;
    }

    // 保存当前数据,用于下次计算
    pid->Last_Measure = pid->Measure;
    pid->Last_Output = pid->Output;
    pid->Last_Dout = pid->Dout;
    pid->Last_Err = pid->Err;
    pid->Last_ITerm = pid->ITerm;

    return pid->Output;
}