/*-------- 头文件 --------*/
#include "stc8.h"
#include "stdio.h"
#include "stdlib.h"
#include "math.h"
#include "intrins.h"
#include "string.h"
#include "variables.h"

/*-------- 声明函数 --------*/
//硬件初始化
void HardwareInit();
void Uart2Init();
void Uart3Init();

//底层硬件控制（PWM以及LED灯）
void PWMLeftCon(unsigned int duty);
void PWMRightCon(unsigned int duty);
void HallCntFiltering();
void LedToogle(int n);

//软件初始化
void VariablesInit();

//底层软件控制
void GenerateTimeFlag();
void DelayTime(int n);
void ClearRxdBuf(); //清空UART接收缓存区

//发射超声波测距
void TxdData(int n);
unsigned char CheckUartRxd(); //检查接收的值
bit CheckDistance(); //检查距离
void ClearMeaBuf(); //清空每轮测量的位置值
void CalLocation(); //计算人的位置（坐标）
void StoreLocation(); //对每一轮有效测量的坐标值进行存储

//控制电机运动
void LeftVelocityFDBK(float i_fSetV); //左电机速度反馈
void RightVelocityFDBK(float i_fSetV); //右电机速度反馈
void DynamicAngleFDBK(); //动态角度反馈
void StaticAngleFDBK(); //静态角度反馈

//红外避障
void CheckObstacle(); //检测有无阻碍

//主函数//
void main()
{
    Uart2Init();
    Uart3Init();
    HardwareInit();
    VariablesInit();
    EA = 1; //允许总中断
    LedToogle(10); //熄灭LED灯
    TxdData(SoundStart);
    DelayTime(3000);
    while(1)
    {
        /*-------- 时基处理 --------*/
        GenerateTimeFlag();
        //处理时间标志量
        if(g_b10msFlag)
        {
            g_b10msFlag = 0;
        }
        if(g_bSamplingFlag)
        {
            g_bSamplingFlag = 0;

            g_fLeftHallCurCnt = g_ucLeftHallCnt; //滤波前进行初始化，防止数据出错
            g_fRightHallCurCnt = g_ucRightHallCnt;
            g_ucLeftHallCnt = 0;
            g_ucRightHallCnt = 0;

//            HallCntFiltering(); //霍尔缓存的数字滤波
            //语音播报区
            if(g_bSingleLostFlag)
            {
                g_bSingleLostFlag = 0;
                TxdData(SoundSignalLost);
                g_ucFailureNum = 0;
                DelayTime(2000);
            }
            if(g_ucFailureNum > MaxFailureNum)
            {
                g_fLeftSetV = StopPWM;
                g_fRightSetV = StopPWM;
                g_bSingleLostFlag = 1;
            }
            if(g_bSoundStartFlag)
            {
                g_bSoundStartFlag = 0;
                TxdData(SoundStart);
                DelayTime(3000);
            }
            if(g_bSoundSpecialFlag)
            {
                g_bSoundSpecialFlag = 0;
                TxdData(SoundSpecial);
                DelayTime(3000);
            }
            if(g_bSoundObstacleFlag)
            {
                g_bSoundObstacleFlag = 0;
                TxdData(SoundObstacle);
                DelayTime(800);
            }
            if(g_b1minFlag)
            {
                g_b1minFlag = 0;
                g_fLeftSetV = StopPWM;
                g_fRightSetV = StopPWM;
                if(g_b3minFlag == 0)
                {
                    g_bSoundStartFlag = 1; //发送欢迎命令
                    g_bSoundSpecialFlag = 0;
                }
                else if(g_b3minFlag == 1)
                {
                    g_b3minFlag = 0;
                    g_bSoundSpecialFlag = 1;
                    g_bSoundStartFlag = 0;
                }
            }
            CheckObstacle();
            if(g_bNoObstacleFlag == 0)//有障碍
            {
                g_fLeftSetV = StopPWM;
                g_fRightSetV = StopPWM;
                g_bSoundObstacleFlag = 1;
            }

            g_fLeftCurV = g_fLeftHallCurCnt * 3.14 * WheelSize / 11 / 20 / T_Sampling; // 分母中是可调节的时间周期，如50[ms]，整体单位m/sj; 20是减速比
            if(g_fLeftHallCurCnt > 0)
            {
                LedToogle(3);
            }
            LeftVelocityFDBK(g_fLeftSetV);
            if(g_fLeftCurV > 0.95 * g_fLeftSetV && g_fLeftCurV < 1.05 * g_fLeftSetV)
            {
                LedToogle(2);
            }

            g_fRightCurV = g_fRightHallCurCnt * 3.14 * WheelSize / 11 / 20 / T_Sampling; // 分母中是可调节的时间周期，如50[ms]，整体单位m/sj; 20是减速比
            if(g_fRightHallCurCnt > 0)
            {
                LedToogle(4);
            }
            RightVelocityFDBK(g_fRightSetV);

            if(g_fRightCurV > 0.95 * g_fRightSetV && g_fRightCurV < 1.05 * g_fRightSetV)
            {
                LedToogle(5);
            }

//            if(g_bNoObstacleFlag == 0)
//            {
//								g_bNoObstacleFlag = 1;
//                TxdData(SoundObstacle);
//                DelayTime(1000);
//            }
        }
        if(g_b50msFlag)
        {
            g_b50msFlag = 0;
        }
        if(g_b1sFlag)
        {
            g_b1sFlag = 0;
        }
        if(g_b3sFlag)
        {
            g_b3sFlag = 0;
//            TxdData(SoundParameter); //debug
        }
        if(g_b5sFlag)
        {
            g_b5sFlag = 0;
        }
        /*-------- 通信处理 --------*/
        if(g_bWorkFlag)
        {
            g_bWorkFlag = 0;
            //有限状态机测量法
            switch(g_ucWorkStat)
            {
            case WAIT_START: //等待新的一轮测量
            {
                g_ucWorkStat++;
                break;
            }
            case START_TXD: //启动超声波发射
            {
                ClearRxdBuf(); //每次发送指令前清空缓存
                TxdData(StartULS);
                g_ucOverTimeCnt = 0;
                DelayTime(DeltaTime); //延时一段时间，允许信号传递过来
                g_ucWorkStat++;
                break;
            }
            case WAIT_TXD_ACK: //等待超声波T端应答
            {
                if(CheckUartRxd() == TxdULSFrame)
                {
                    g_ucWorkStat += 2; //跳过WAIT_READ
                }
                else
                {
                    g_ucOverTimeCnt += DeltaTime;
                    DelayTime(DeltaTime);
                    if(g_ucOverTimeCnt > MaxOverTime)
                    {
                        g_ucWorkStat = WAIT_START; //超时，回到等待启动状态
                    }
                }
                break;
            }
            case WAIT_READ:
            {
                g_ucWorkStat++;
                break;
            }
            case START_READ:
            {
                ClearRxdBuf();
                TxdData(FindDistance);
                DelayTime(DeltaTime); //延时一段时间，允许信号传递过来
                g_ucWorkStat++;
                break;
            }
            case WAIT_READ_ACK:
            {
                if(CheckUartRxd() == RxdULSFrame)
                {
                    LedToogle(0);
                    g_ucWorkStat++;
                }
                else
                {
                    g_ucOverTimeCnt += DeltaTime;
                    if(g_ucOverTimeCnt > MaxOverTime)
                    {
                        g_ucWorkStat = WAIT_START; //超时，回到等待启动状态
                    }
                }
                break;
            }
            case CAL_XYZ:
            {
                //根据已经接收到的距离数据计算R端传回的4个距离
                for(g_ucTmp = 0; g_ucTmp < 4; g_ucTmp++) //计算真实距离
                {
                    ga_iDistance[g_ucTmp] = 256 * ga_ucUartRxdBuf[11 + 2 * g_ucTmp] + ga_ucUartRxdBuf[10 + 2 * g_ucTmp];
                }
                if(CheckDistance()) //若距离值在误差范围内
                {
                    CalLocation();
                    StoreLocation();
                    LedToogle(1); //亮起绿色LED灯
                    g_ucWorkStat++;
                    break;
                }
                else //若距离值超出误差范围，重新测量
                {
                    g_ucFailureNum++;
                    g_ucWorkStat = WAIT_START;
                    break;
                }
            }
            case CONTROL:
            {
                g_ucFailureNum = 0; //对掉线次数置零
                g_fSetA = 90;
                if(g_bNoObstacleFlag)
                {
                    if(g_fCurR < MinR)
                    {
                        StaticAngleFDBK();
                    }
                    else
                    {
                        DynamicAngleFDBK();
                    }
                    if(g_fCurR > 3 * MinR)
                    {
                        TxdData(SoundTooFar);
                    }
                }
                else if(g_bNoObstacleFlag == 0)//有障碍
                {
                    g_fLeftSetV = StopPWM;
                    g_fRightSetV = StopPWM;
                    g_bSoundObstacleFlag = 1;
                }
                ClearMeaBuf();
                g_ucWorkStat = WAIT_START;
                break;
            }
            }
        }
    }
}

/*-------- 普通函数 --------*/

/******************************/
/*-------- 变量初始化 --------*/
/*****************************/
void VariablesInit()
{
    //上层电机控制初始值
    g_fLeftSetV = 0;
    g_fRightSetV = 0;
    g_uiLeftCurDuty = 0;
    g_uiRightCurDuty = 0;
    g_fSetA = 0;
    g_fAngleCurError = 0;
    g_fAngleLastError = 0;

    //底层电机控制初始值
    g_ucLeftHallCnt = 0;
    g_ucRightHallCnt = 0;
    g_fLeftCurError = 0;
    g_fLeftLastError = 0;
    g_fRightCurError = 0;
    g_fRightLastError = 0;

    g_ucTmp = 0; //临时变量
    g_uiCnt = 0;

    //时基标志量//
    g_b1msFlag = 0;
    g_b1sFlag = 0;
    g_b3sFlag = 0;
    g_bWorkFlag = 0;
    g_bSamplingFlag = 0;
    g_uiMinCnt = 0;
    g_uiCnt = 0;
    g_b1minFlag = 0;
    g_b3minFlag = 0;

    //串口收发标志量//
    g_ucUART2SavePst = 0;
    g_ucWorkStat = 0;
    g_bUart2BusyFlag = 0;
    g_bUart3BusyFlag = 0;

    //初始化距离
    for(g_ucTmp = 0; g_ucTmp < 4; g_ucTmp++)
    {
        ga_iDistance[g_ucTmp] = 0;
    }
    g_ucLocationSavePst = 0;
    g_ucFailureNum = 0;

    //初始化坐标
    g_fCurX = 0;
    g_fCurY = 0;
    g_fCurZ = 0;
    g_fCurA = 90;
    g_fCurR = 0;

    //初始化霍尔缓存
    for(g_ucTmp = 0; g_ucTmp < HallBufMaxBytes; g_ucTmp++)
    {
        ga_ucLeftHallRxdBuf[g_ucTmp] = 0;
        ga_ucRightHallRxdBuf[g_ucTmp] = 0;
    }
    g_ucHallSavePst = 0;
    g_fLeftHallCurCnt = 0;
    g_fRightHallCurCnt = 0;

    //避障标志量
    g_bForwardFlag = 0;
    g_bBackwardFlag = 0;
    g_bLeftwardFlag = 0;
    g_bRightwardFlag = 0;
    g_bNoObstacleFlag = 1;

    g_bSingleLostFlag = 0;
    g_bSoundStartFlag = 0;
    g_bSoundSpecialFlag = 0;
}
/*********************************/
/*-------- 产生时基标志量 --------*/
/*********************************/
void GenerateTimeFlag()
{
    if(g_b1msFlag)
    {
        g_b1msFlag = 0;
        g_uiCnt++;
    }
    //产生时间标志量
    if(g_uiCnt % T_Sampling == 0 & g_uiCnt != 0)
    {
        g_bSamplingFlag = 1;
    }
    if(g_uiCnt % T_Work == 0 & g_uiCnt != 0)
    {
        g_bWorkFlag = 1;
    }
    if(g_uiCnt % 10 == 0 & g_uiCnt != 0)
    {
        g_b10msFlag = 1;
    }

    if(g_uiCnt % 50 == 0 & g_uiCnt != 0)
    {
        g_b50msFlag = 1;
    }
    if(g_uiCnt % 100 == 0 & g_uiCnt != 0)
    {
        g_b100msFlag = 1;
    }
    if(g_uiCnt % 500 == 0 & g_uiCnt != 0)
    {
        g_b500msFlag = 1;
    }
    if(g_uiCnt % 1000 == 0 & g_uiCnt != 0)
    {
        g_b1sFlag = 1;
    }
    if(g_uiCnt % 3000 == 0 & g_uiCnt != 0)
    {
        g_b3sFlag = 1;
    }
    if(g_uiCnt % 5000 == 0 & g_uiCnt != 0)
    {
        g_b5sFlag = 1;
    }
    if(g_uiCnt == 6e4)
//		if(g_uiCnt % 3000 == 0 & g_uiCnt != 0)
    {
        g_b1minFlag = 1;
    }
    if(g_uiCnt > 6e4)
    {
        g_uiCnt = 0;
    }
    //产生分钟标志量
    if(g_b1minFlag == 1)
    {
        g_uiMinCnt++;
    }
    if(g_uiMinCnt % 3 == 0 & g_uiMinCnt != 0)
    {
        g_b3minFlag = 1;
    }
    if(g_uiMinCnt > 250)
    {
        g_uiMinCnt = 0;
    }
}
/******************************************/
/*-------- 清空每一轮测量的临时变量 --------*/
/******************************************/
void ClearMeaBuf()
{
    for(g_ucTmp = 0; g_ucTmp < 4; g_ucTmp++) //清空距离值
    {
        ga_iDistance[g_ucTmp] = 0;
    }
    g_fLeftCurV = 0;
    g_fRightCurV = 0;
    g_fCurX = 0;
    g_fCurY = 0;
    g_fCurZ = 0;
    g_fCurR = 0;
    g_fCurA = 90;
    g_bForwardFlag = 0;
    g_bBackwardFlag = 0;
    g_bLeftwardFlag = 0;
    g_bRightwardFlag = 0;
}
/***********************************/
/*-------- 距离最大差值检测 --------*/
/***********************************/
bit CheckDistance()
{
    int i_aiDstSorted[4], i_aucDstNum[4]; //前者从大到小储存距离值，后者对应储存前者的ULS传感器编号
    int i_iRangeFull4;
    unsigned char i_ucTmpA, i_ucTmpB, i_ucCnt;
    bit i_bDistanceOK;
    i_bDistanceOK = 0;

    for(i_ucTmpA = 0; i_ucTmpA < 4; i_ucTmpA++)//对A位数进行遍历
    {
        i_ucCnt = 0; //每次开始时对计数器置零
        for(i_ucTmpB = 0; i_ucTmpB < 4; i_ucTmpB++) //开始遍历检索，看有几个数比A小
        {
            if(i_ucTmpB == i_ucTmpA)
                continue;
            else
            {
                if(ga_iDistance[i_ucTmpA] > ga_iDistance[i_ucTmpB]) //如果A位数大于B位数，令计数器加一
                {
                    i_ucCnt++;
                }
            }
        }
        //i_ucCnt表示有几个数比A小，3-i_ucCnt可表示i_aiDstSorted数组中的元素序号（从大到小）
        i_aiDstSorted[3 - i_ucCnt] = ga_iDistance[i_ucTmpA];
        i_aucDstNum[3 - i_ucCnt] = i_ucTmpA;
    }

    //进行数据筛选
    i_iRangeFull4 = i_aiDstSorted[0] - i_aiDstSorted[3]; //整体4个数据的极差
    if(i_iRangeFull4 < DeltaDistance) //若整体数据都比较均一
    {
        i_bDistanceOK = 1;
    }
    return i_bDistanceOK;
}
/***************************************/
/*-------- 静态电机角度反馈控制 --------*/
/***************************************/
void StaticAngleFDBK()
{
    float i_fTurningSpeed;

    if(g_fAngleCurError > StaticDeltaAngle & g_fAngleCurError < 50)
    {
        i_fTurningSpeed = UltraLowSpeed;
    }
    if(g_fAngleCurError > 50 & g_fAngleCurError < 120)
    {
        i_fTurningSpeed = LowSpeed;
    }
    if(g_fAngleCurError >  120)
    {
        i_fTurningSpeed = TurningSpeed;
    }

    g_fLeftSetV = 0; //预设零速度，只在特定情况给速度
    g_fRightSetV = 0;

    if(fabs(g_fCurX) > MinX) //排除X的干扰
    {
        if(g_fAngleCurError > StaticDeltaAngle) //如果当前角度和目标角度相差不超DeltaAngle，则无反应
        {
            g_fLeftSetV = abs(min(g_iSign, 0)) * i_fTurningSpeed;
            g_fRightSetV = max(g_iSign, 0) * i_fTurningSpeed;
        }
    }
}
/***************************************/
/*-------- 动态电机角度反馈控制 --------*/
/***************************************/
void DynamicAngleFDBK()
{
    float i_fTurningSpeed;

    if(g_fAngleCurError > StaticDeltaAngle & g_fAngleCurError < 50)
    {
        i_fTurningSpeed = TurningSpeed;
    }
    if(g_fAngleCurError > 50 & g_fAngleCurError < 120)
    {
        i_fTurningSpeed = LowSpeed;
    }
    if(g_fAngleCurError >  120)
    {
        i_fTurningSpeed = UltraLowSpeed;
    }

    g_fLeftSetV = NormalSpeed; //预设等速度，只在特定情况给速度差
    g_fRightSetV = NormalSpeed;

    if(fabs(g_fCurX) > MinX) //排除X的干扰
    {
        if(g_fCurR < 2 * MinR) //距离中等，边走变转
        {
            if(g_fAngleCurError > 90) //大于90°，先停再转
            {
                StaticAngleFDBK();
            }
            else if(g_fAngleCurError > DynamicDeltaAngle) //如果当前角度和目标角度相差不超DeltaAngle，则无反应
            {
                if(g_iSign > 0) //如果在二象限/左边，左电机减速
                {
                    g_fLeftSetV = i_fTurningSpeed;
                }
                else if(g_iSign < 0) //如果人在第一象限/右边，右电机减速
                {
                    g_fRightSetV = i_fTurningSpeed;
                }
            }
        }
        else if(g_fCurR > 2 * MinR) //距离较远，先停再转
        {
            g_fLeftSetV = 1.2*NormalSpeed; //预设等速度，只在特定情况给速度差
            g_fRightSetV = 1.2*NormalSpeed;
            if(g_fAngleCurError > 90) //如果当前角度和目标角度相差不超DeltaAngle，则无反应
            {
                StaticAngleFDBK(); //转入静态转向模式
            }
            else if(g_fAngleCurError > DynamicDeltaAngle) //如果当前角度和目标角度相差不超DeltaAngle，则无反应
            {
                if(g_iSign > 0) //如果在二象限/左边，左电机减速
                {
                    g_fLeftSetV = 1.2 * i_fTurningSpeed;
                }
                else if(g_iSign < 0) //如果人在第一象限/右边，右电机减速
                {
                    g_fRightSetV = 1.2 * i_fTurningSpeed;
                }
            }

//            if(g_fAngleCurError > DynamicDeltaAngle) //如果当前角度和目标角度相差不超DeltaAngle，则无反应
//            {
//                StaticAngleFDBK(); //转入静态转向模式
//            }
//            else
//            {
//                g_fLeftSetV = FastSpeed; //预设等速度，只在特定情况给速度差
//                g_fRightSetV = FastSpeed;
//            }
        }
    }
}
/*******************************/
/*-------- 红外障碍检测 --------*/
/*******************************/
void CheckObstacle()
{
    if(P36 == 1)
    {
        g_bLeftwardFlag = 0;
    }
    else
    {
        g_bLeftwardFlag = 1;
    }
    if(P37 == 1)
    {
        g_bRightwardFlag = 0;
    }
    else
    {
        g_bRightwardFlag = 1;
    }

    if(g_bLeftwardFlag == 0 & g_bRightwardFlag == 0)
    {
        g_bNoObstacleFlag = 1;
    }
    else
    {
        g_bNoObstacleFlag = 0;
    }
}
/************************************/
/*-------- 左电机速度反馈控制 --------*/
/************************************/
void LeftVelocityFDBK(float i_fSetV)
{
    float k_p, k_i, k_d;
    float CurError, LastError, Last2Error;

    Last2Error = g_fLeftLastError;
    g_fLeftLastError = g_fLeftCurError;
    g_fLeftCurError = i_fSetV - g_fLeftCurV;

    LastError = g_fLeftLastError;
    CurError = g_fLeftCurError;

    if(i_fSetV == 0)
    {
        g_uiLeftCurDuty = 10;
        PWMLeftCon(g_uiLeftCurDuty);
    }
    else if(i_fSetV == ConstPWM)
    {
        g_uiLeftCurDuty = ConstDuty;
        PWMLeftCon(g_uiLeftCurDuty);
    }
    else if(i_fSetV == StopPWM)
    {
        g_uiLeftCurDuty = 10;
        PWMLeftCon(g_uiLeftCurDuty);
    }
    else
    {
        k_p = K_P;
        k_i = K_I;
        k_d = K_D;
        g_uiLeftCurDuty += k_p * (CurError - LastError) + k_i * (CurError + LastError) / 2 + k_d * (CurError - 2 * LastError + Last2Error);
        PWMLeftCon(g_uiLeftCurDuty);
    }
}
/************************************/
/*-------- 右电机速度反馈控制 --------*/
/************************************/
void RightVelocityFDBK(float i_fSetV)
{
    float k_p, k_i, k_d;
    float CurError, LastError, Last2Error;

    Last2Error = g_fRightLastError;
    g_fRightLastError = g_fRightCurError;
    g_fRightCurError = i_fSetV - g_fRightCurV;

    LastError = g_fRightLastError;
    CurError = g_fRightCurError;

    if(i_fSetV == 0)
    {
        g_uiRightCurDuty = 10;
        PWMRightCon(g_uiRightCurDuty);
    }
    else if(i_fSetV == ConstPWM)
    {
        g_uiRightCurDuty = ConstDuty;
        PWMRightCon(g_uiRightCurDuty);
    }
    else if(i_fSetV == StopPWM)
    {
        g_uiRightCurDuty = 10;
        PWMRightCon(g_uiLeftCurDuty);
    }
    else
    {
        k_p = K_P;
        k_i = K_I;
        k_d = K_D;
        g_uiRightCurDuty += k_p * (CurError - LastError) + k_i * (CurError + LastError) / 2 + k_d * (CurError - 2 * LastError + Last2Error);
        PWMRightCon(g_uiRightCurDuty);
    }
}
/*********************************/
/*-------- 计算发射端坐标 --------*/
/*********************************/
void CalLocation()
{
    float xdata i_fX1, i_fX2, i_fY1, i_fY2;

    //开始计算，新的三角形面积法
    i_fY1 = (pow(ga_iDistance[0], 2) - pow(ga_iDistance[1], 2)) / 2.0 / DIS_S;
    i_fX1 = (pow(ga_iDistance[1], 2) - pow(ga_iDistance[2], 2)) / 2.0 / DIS_S;
    i_fY2 = (pow(ga_iDistance[3], 2) - pow(ga_iDistance[2], 2)) / 2.0 / DIS_S;
    i_fX2 = (pow(ga_iDistance[0], 2) - pow(ga_iDistance[3], 2)) / 2.0 / DIS_S;

    //对数据进行处理
    g_fCurX = (i_fX1 + i_fX2) / 2.0;
    g_fCurY = (i_fY1 + i_fY2) / 2.0;
    g_fCurR = pow(g_fCurX * g_fCurX + g_fCurY * g_fCurY, 0.5);
    g_fCurA = atan2(g_fCurY, g_fCurX) * 180 / 3.14; //角度值
}
/*********************************/
/*-------- 存储合理的坐标值 --------*/
/*********************************/
void StoreLocation()
{
    ga_fCurXBuf[g_ucLocationSavePst] = g_fCurX;
    ga_fCurYBuf[g_ucLocationSavePst] = g_fCurY;
    g_ucLocationSavePst = (g_ucLocationSavePst + 1) & (LocationBufMaxBytes - 1); //高位屏蔽

    g_fAngleLast2Error = g_fAngleLastError;
    g_fAngleLastError = g_fAngleCurError;
    if(g_fCurX < 0)
    {
        g_iSign = 1; //左旋
        g_fAngleCurError = (g_fCurA - g_fSetA) * g_iSign;
        if(g_fCurY < 0)
        {
            g_fAngleCurError = 180 + g_fCurA + 90; //针对第四象限的发射端位置，对角度进行修正
        }
    }
    else
    {
        g_iSign = -1; //右旋
        g_fAngleCurError = (g_fCurA - g_fSetA) * g_iSign;
    }
}

/*********************************/
/*-------- 清空接收区缓存 --------*/
/*********************************/
void ClearRxdBuf()
{
    for(g_ucTmp = 0; g_ucTmp < UARTBufMaxBytes; g_ucTmp++)
    {
        ga_ucUartRxdBuf[g_ucTmp] = 0;
    }
    g_ucUART2SavePst = 0;
}
/***********************************/
/*-------- 延时函数(ms级别) --------*/
/***********************************/
void DelayTime(int n)
{
    int this_Cnt, this_tmp;
    for(this_Cnt = 0; this_Cnt < n; this_Cnt++)
    {
        for(this_tmp = 0; this_tmp < 1350; this_tmp++)
        {
        }
    }
}
/******************************************/
/*-------- 霍尔传感器数据的数字滤波 --------*/
/******************************************/
void HallCntFiltering()
{
    unsigned char i_ucLeftNonZeroNum, i_ucRightNonZeroNum,  i_ucLeftBiggerCnt, i_ucRightBiggerCnt, i_ucTmpA, i_ucTmpB;
    unsigned char xdata ia_ucLeftHallSorted[HallBufMaxBytes], ia_ucRightHallSorted[HallBufMaxBytes]; //排序后的数组
    int i_iLeftSum, i_iRightSum; //求和数组

    //霍尔传感器数据的存放
    ga_ucLeftHallRxdBuf[g_ucHallSavePst] = g_ucLeftHallCnt;
    ga_ucRightHallRxdBuf[g_ucHallSavePst] = g_ucRightHallCnt;
    g_ucHallSavePst = (g_ucHallSavePst + 1) & (HallBufMaxBytes - 1); //高位屏蔽，防止溢出

    //变量初始化
    g_fLeftHallCurCnt = g_ucLeftHallCnt; //滤波前进行初始化，防止数据出错
    g_fRightHallCurCnt = g_ucRightHallCnt;
    i_ucLeftBiggerCnt = 0;
    i_ucRightBiggerCnt = 0;
    i_ucLeftNonZeroNum = 0; //非0元素个数
    i_ucRightNonZeroNum = 0; //非0元素个数

    //底层变量置零
    g_ucLeftHallCnt = 0;
    g_ucRightHallCnt = 0;

    //对存放的数据进行处理
    for(i_ucTmpA = 0; i_ucTmpA < HallBufMaxBytes; i_ucTmpA++)
    {
        if(ga_ucLeftHallRxdBuf[i_ucTmpA] != 0)
        {
            i_ucLeftNonZeroNum++;
        }
        if(ga_ucRightHallRxdBuf[i_ucTmpA] != 0)
        {
            i_ucRightNonZeroNum++;
        }
        ia_ucLeftHallSorted[i_ucTmpA] = 0; //初始化排序数组
        ia_ucRightHallSorted[i_ucTmpA] = 0; //初始化排序数组
    }

    //如果所有元素都是非零数组，则可以开始滤波了；否则用这一周期的计数值作为最终结果
    if(i_ucLeftNonZeroNum == HallBufMaxBytes & i_ucRightNonZeroNum == HallBufMaxBytes)
    {
        i_ucLeftBiggerCnt = 0; //每次遍历前必须置零！
        i_ucRightBiggerCnt = 0;
        //按大小依次排序
        for(i_ucTmpA = 0; i_ucTmpA < HallBufMaxBytes; i_ucTmpA++)
        {
            //排序左电机HALL缓存
            for(i_ucTmpB = 0; i_ucTmpB < HallBufMaxBytes; i_ucTmpB++)
            {
                //如果A位元素和B位元素都是非零元素，则进行比较
                if(ga_ucLeftHallRxdBuf[i_ucTmpA] > ga_ucLeftHallRxdBuf[i_ucTmpB])
                {
                    i_ucLeftBiggerCnt++; //若A号元素比B号元素大，计数器+1
                }
            }
            //遍历完毕，开始将非零的A号元素存入排序后的数组，第A号元素在非零元素中进行排序，第0位是最大的。
            ia_ucLeftHallSorted[i_ucLeftNonZeroNum - 1 - i_ucLeftBiggerCnt] = ga_ucLeftHallRxdBuf[i_ucTmpA];

            //排序右电机HALL缓存
            for(i_ucTmpB = 0; i_ucTmpB < HallBufMaxBytes; i_ucTmpB++)
            {
                //如果A位元素和B位元素都是非零元素，则进行比较
                if(ga_ucRightHallRxdBuf[i_ucTmpA] > ga_ucRightHallRxdBuf[i_ucTmpB])
                {
                    i_ucRightBiggerCnt++; //若A号元素比B号元素大，计数器+1
                }
            }
            //遍历完毕，开始将非零的A号元素存入排序后的数组，第A号元素在非零元素中进行排序，第0位是最大的。
            ia_ucRightHallSorted[i_ucRightNonZeroNum - 1 - i_ucRightBiggerCnt] = ga_ucRightHallRxdBuf[i_ucTmpA];
        }

        //根据结果将滤波结果存入临时变量g_fLeftHallCurCnt，g_fRightHallCurCnt
        i_iLeftSum = 0;
        i_iRightSum = 0;
        for(i_ucTmpA = 1; i_ucTmpA < HallBufMaxBytes - 1; i_ucTmpA++) //从第2大的数开始统计，直到第2小的数（避开最大和最小）
        {
            i_iLeftSum += ia_ucLeftHallSorted[i_ucTmpA];
            i_iRightSum += ia_ucRightHallSorted[i_ucTmpA];
        }
        g_fLeftHallCurCnt = 1.0 * i_iLeftSum / (HallBufMaxBytes - 2); //输出滤波结果
        g_fRightHallCurCnt = 1.0 * i_iRightSum / (HallBufMaxBytes - 2);
    }
}
/************************************/
/*-------- 面包板LED灯的控制 --------*/
/************************************/
void LedToogle(int n)
{
    //闪烁一下
    P20 = 0;
    P21 = 0;
    P22 = 0;
    P23 = 0;
    P24 = 0;
    P25 = 0;
    if (n == 0)
    {
        P20 = 1;
    }
    else if(n == 1)
    {
        P21 = 1;
    }
    else if(n == 2)
    {
        P22 = 1;
    }
    else if(n == 3)
    {
        P23 = 1;
    }
    else if(n == 4)
    {
        P24 = 1;
    }
    else if(n == 5)
    {
        P25 = 1;
    }
    DelayTime(1);
    P20 = 0;
    P21 = 0;
    P22 = 0;
    P23 = 0;
    P24 = 0;
    P25 = 0;
}

/********************************/
/*-------- PWMLeft的控制 --------*/
/********************************/
void PWMLeftCon(unsigned int duty)
//增强型PWM0(左电机）输出口是P17
{
    unsigned int i_iHighStart;
    unsigned char i_ucPCA;
    if(duty > 1024)
    {
        duty = 1024;
        g_uiLeftCurDuty = duty;
    }
    if(duty < 10)
    {
        duty = 10;
        g_uiLeftCurDuty = duty;
    }
    i_iHighStart = 1024 - g_uiLeftCurDuty;

    i_ucPCA = i_iHighStart >> 8;
    i_ucPCA = (i_ucPCA << 2) | (i_ucPCA << 4);

    CR = 0;
    CCON = 0x00;
    CMOD = 0x04; //PCA 时钟为T0溢出脉冲
    CL = 0x00;
    CH = 0x00;

    CCAPM0 = 0x42; //PCA 模块 0 为 PWM 工作模式
    PCA_PWM0 = 0xc0;//PCA 模块 0 输出 10 位 PWM
    //写入装载值、比较值
    PCA_PWM0 |= i_ucPCA; //写入高位重载值与高位比较值
    CCAP0H = i_iHighStart; //低位重载值
    CCAP0L = CCAP0H; //低位比较值

    CR = 1; //启动 PCA 计时器
}
/********************************/
/*-------- PWMRightCon控制 --------*/
/********************************/
void PWMRightCon(unsigned int duty)
//增强型PWM1输出口是P16
{
    unsigned int i_iHighStart;
    unsigned char i_ucPCA;
    if(duty > 1024)
    {
        duty = 1024;
        g_uiRightCurDuty = duty;
    }
    if(duty < 10)
    {
        duty = 10;
        g_uiRightCurDuty = duty;
    }
    i_iHighStart = 1024 - g_uiRightCurDuty;

    i_ucPCA = i_iHighStart >> 8;
    i_ucPCA = (i_ucPCA << 2) | (i_ucPCA << 4);

    CR = 0;
    CCON = 0x00;
    CMOD = 0x04; //PCA 时钟为T0溢出脉冲
    CL = 0x00;
    CH = 0x00;

    CCAPM1 = 0x42; //PCA 模块 1 为 PWM 工作模式
    PCA_PWM1 = 0xc0;//PCA 模块 1 输出 10 位 PWM
    //写入装载值、比较值
    PCA_PWM1 |= i_ucPCA; //写入高位重载值与高位比较值
    CCAP1H = i_iHighStart; //低位重载值
    CCAP1L = CCAP1H; //低位比较值

    CR = 1; //启动 PCA 计时器
}
/*********************************/
/*-------- 检查接受的数据 --------*/
/*********************************/
unsigned char CheckUartRxd() //检查是否是ULS接收端发回的数据
{
    unsigned char k, ucData_OK;
    ucData_OK = 0;
    k = 0;
    if (ga_ucUartRxdBuf[0] == 0x55) //帧头1
    {
        k++;
    }
    if (ga_ucUartRxdBuf[1] == 0xAA) //帧头2
    {
        k++;
    }
    if (ga_ucUartRxdBuf[2] == 0x56) //上位机（STC8A）的地址
    {
        k++;
    }
    if (k == 3)
    {
        ucData_OK = GeneralULSFrame; //是ULS的数据，但不知道是R端还是T端
        if (ga_ucUartRxdBuf[4] == 0x02 && ga_ucUartRxdBuf[5] == 0xD1)
            //T端响应规律
        {
            ucData_OK = TxdULSFrame;
        }
        if (ga_ucUartRxdBuf[4] == 0x0D)
            //R端响应规律
        {
            ucData_OK = RxdULSFrame;
        }
    }
    return (ucData_OK);
}
/******************************/
/*-------- 硬件初始化 --------*/
/*****************************/
void HardwareInit()
{
    PIN_LGnd = 0;
    PIN_RGnd = 0;

    //推挽输出模式，便于点亮面包板上的LED灯
    P2M0 = 0xff;
    P2M1 = 0x00;

    //外部中断INT0初始化，P32
    IT0 = 1; //INT0 下降沿触发
    EX0 = 1; //使能INT0 中断

    //外部中断INT1初始化，P33
    IT1 = 1; //INT1 下降沿触发
    EX1 = 1; //使能INT1 中断

//    //外部中断INT2初始化，P36
//    INTCLKO = EX2; //使能INT2 下降沿中断

//    //外部中断INT3初始化，P37
//    INTCLKO = EX3; //使能INT3 下降沿中断

    //定时器0用于PWM周期的选择
    AUXR |= AUXR_T0x1; //T0处于不分频模式
    TMOD |= Timer0_M0; //T0处于模式0，16位自动重载
    TL0 = BRT0;
    TH0 = BRT0 >> 8;
    TF0 = 0;
    ET0 = 1; //T0中断使能
    TR0 = 1; //T1开始工作

    //定时器1作为1ms时基中断的初始化
    //重点：T1模式的选择，是否分频，初始高低位置
    AUXR |= AUXR_T1x12; //定时器1工作在12分频
    TMOD |= Timer1_M0; //T1处于模式1，16位自动重载
    TL1 = 0x66;
    TH1 = 0xFC; //设置T1的自动重载值
    ET1 = 1; //允许定时器1触发中断
    TR1 = 1; //允许定时器1开始工作

    //定时器2用于串口2的波特率发生器，在Uart2Init()中初始化
    //定时器3用于串口1的波特率发生器，在Uart3Init()中初始化
}
/*********************************/
/*-------- 串口2的初始化 --------*/
/*********************************/
void Uart2Init()
{
    //设置串口2
    S2CON = UART2_M0; //串口2处于模式0，可变波特率8位数据；用定时器2作波特率发射器
    S2CON |= S2REN; //串口2允许输入

    //设置定时器2
    AUXR |= AUXR_T2x12; // 定时器2进行12分频
    //T2固定为16位重载模式;
    TL2 = BRT2;
    TH2 = BRT2 >> 8; // 设置定时器2的低/高地址初值，确定波特率2

    //定时器使能
    AUXR |= T2R; //允许定时器2开始工作

    //中断使能
    IE2 |= ES2; //允许串口2的中断
}
/*********************************/
/*-------- 串口3的初始化 --------*/
/*********************************/
void Uart3Init()
{
    //设置串口1
    S3CON = UART3_M0T3; //串口3处于模式0，可变波特率8位数据；用定时器3作波特率发射器，允许输入

    //设置定时器3
    T3L = BRT3;
    T3H = BRT3 >> 8;

    //定时器使能
    T4T3M = 0x0a; //1T频率（不分频），使能T3

    //中断使能
    IE2 |= ES3; //允许串口3的中断
}
/**************************************/
/*-------- 串口2发送BYTE型数据 --------*/
/**************************************/
void SendByUart2(unsigned char dat)
{
    while(g_bUart2BusyFlag); //保持空转，等待g_bUart3BusyFlag=0，亦即等待串口的TI中断
    g_bUart2BusyFlag = 1; //发送前，将g_bUart3BusyFlag置1
    S2BUF = dat; //发送数据，发送完成后g_bUart3BusyFlag会被置0
}
/**************************************/
/*-------- 串口3发送BYTE型数据 --------*/
/**************************************/
void SendByUart3(unsigned char dat)
{
    while(g_bUart3BusyFlag); //保持空转，等待g_bUart3BusyFlag=0，亦即等待串口的TI中断
    g_bUart3BusyFlag = 1; //发送前，将g_bUart3BusyFlag置1
    S3BUF = dat; //发送数据，发送完成后g_bUart3BusyFlag会被置0
}
/****************************/
/*-------- 发送数据 --------*/
/****************************/
void TxdData(int mode)
{
    unsigned char i_ucLen;
    unsigned char i_uctmp;
    if(mode == SoundParameter)
    {
        char i_caTmp[20];
        char i_caTmp1[20];
//			  char xdata i_caText[50] = {"[v1][s8][m52]左占空比"};
//        char i_caText1[20] = {"右占空比"};
//        sprintf(i_caTmp, "%3.1f", g_uiLeftCurDuty);
//        strcat(i_caText, i_caTmp);
//        sprintf(i_caTmp1, "%3.1f", g_uiRightCurDuty);
//        strcat(i_caText1, i_caTmp1);
//        strcat(i_caText, i_caText1);

//        char xdata i_caText[50] = {"[v1][s8][m52]左速度"};
//        char i_caText1[20] = {"右速度"};
//        sprintf(i_caTmp, "%0.3f", g_fLeftCurV);
//        strcat(i_caText, i_caTmp);
//        sprintf(i_caTmp1, "%0.3f", g_fRightCurV);
//        strcat(i_caText1, i_caTmp1);
//        strcat(i_caText, i_caText1);

//        char xdata i_caText[50] = {"[v1][s9][m52]左霍尔"};
//        char xdata i_caText1[20] = {"右霍尔"};
//        sprintf(i_caTmp, "%2.1f", g_fLeftHallCurCnt);
//        strcat(i_caText, i_caTmp);
//        sprintf(i_caTmp1, "%2.1f", g_fRightHallCurCnt);
//        strcat(i_caText1, i_caTmp1);
//        strcat(i_caText, i_caText1);

//        char i_caText[40] = {"[v1][s8][m52]X坐标"};
//        char i_caText1[20] = {"Y坐标"};
//        sprintf(i_caTmp, "%4.0f", g_fCurX);
//        strcat(i_caText, i_caTmp);
//        sprintf(i_caTmp1, "%4.0f", g_fCurY);
//        strcat(i_caText1, i_caTmp1);
//        strcat(i_caText, i_caText1);

        char i_caText[40] = {"[v1][s8][m52]距离"};
        char i_caText1[20] = {"角度"};
        sprintf(i_caTmp, "%4.0f", g_fCurR);
        strcat(i_caText, i_caTmp);
        sprintf(i_caTmp1, "%3.1f", g_fCurA);
        strcat(i_caText1, i_caTmp1);
        strcat(i_caText, i_caText1);

//        char xdata i_caText[50] = {"[v1][s9][m52]零通道"};
//        char xdata i_caText1[20] = {"，一通道"};
//				char xdata i_caText2[20] = {"，二通道"};
//				char xdata i_caText3[20] = {"，三通道"};
//        sprintf(i_caTmp, "%u", ga_iDistance[0]);
//        strcat(i_caText, i_caTmp);
//        sprintf(i_caTmp1, "%u", ga_iDistance[1]);
//        strcat(i_caText1, i_caTmp1);
//        strcat(i_caText, i_caText1);
//				sprintf(i_caTmp, "%u", ga_iDistance[2]);
//        strcat(i_caText2, i_caTmp);
//        sprintf(i_caTmp1, "%u", ga_iDistance[3]);
//        strcat(i_caText3, i_caTmp1);
//        strcat(i_caText2, i_caText3);
//				strcat(i_caText, i_caText2);

        i_ucLen = strlen(i_caText);
        ga_ucUartTxdBuf[0] = 0xFD; //帧头
        ga_ucUartTxdBuf[1] = 0x00; //长度高字节
        ga_ucUartTxdBuf[2] = i_ucLen + 2; //长度低字节
        ga_ucUartTxdBuf[3] = 0x01; //命令字
        ga_ucUartTxdBuf[4] = 0x00; //编码格式
        i_ucLen = strlen(ga_ucUartTxdBuf);
        for(i_uctmp = 0; i_uctmp < 5; i_uctmp++)
        {
            SendByUart3(ga_ucUartTxdBuf[i_uctmp]);
        }

        i_ucLen = strlen(i_caText);
        for(i_uctmp = 0; i_uctmp < i_ucLen; i_uctmp++)
        {
            SendByUart3(i_caText[i_uctmp]);
        }
    }
    else if (mode == SoundSignalLost) //语音模块发声
    {
        char i_caText[] = {"[v3][s9][m51]凉凉信号丢失！"};
        i_ucLen = strlen(i_caText);

        ga_ucUartTxdBuf[0] = 0xFD; //帧头
        ga_ucUartTxdBuf[1] = 0x00; //长度高字节
        ga_ucUartTxdBuf[2] = i_ucLen + 2; //长度低字节
        ga_ucUartTxdBuf[3] = 0x01; //命令字
        ga_ucUartTxdBuf[4] = 0x00; //编码格式
        i_ucLen = strlen(ga_ucUartTxdBuf);
        for(i_uctmp = 0; i_uctmp < 5; i_uctmp++)
        {
            SendByUart3(ga_ucUartTxdBuf[i_uctmp]);
        }

        i_ucLen = strlen(i_caText);
        for(i_uctmp = 0; i_uctmp < i_ucLen; i_uctmp++)
        {
            SendByUart3(i_caText[i_uctmp]);
        }
    }

    else  if (mode == SoundStart) //启动
    {
        char i_caText[] = {"[v3][s9][m3]欢迎使用B39展位的小车，喜欢请给B39投票"};
//        char xdata i_caText[] = {"[v2][s9][m3]交大首款智能跟随小车上线啦！智能小车灵活移动，陪您嗨翻天，给您不一样的跟随体验！"};
        i_ucLen = strlen(i_caText);

        ga_ucUartTxdBuf[0] = 0xFD; //帧头
        ga_ucUartTxdBuf[1] = 0x00; //长度高字节
        ga_ucUartTxdBuf[2] = i_ucLen + 2; //长度低字节
        ga_ucUartTxdBuf[3] = 0x01; //命令字
        ga_ucUartTxdBuf[4] = 0x00; //编码格式
        i_ucLen = strlen(ga_ucUartTxdBuf);
        for(i_uctmp = 0; i_uctmp < 5; i_uctmp++)
        {
            SendByUart3(ga_ucUartTxdBuf[i_uctmp]);
        }

        i_ucLen = strlen(i_caText);
        for(i_uctmp = 0; i_uctmp < i_ucLen; i_uctmp++)
        {
            SendByUart3(i_caText[i_uctmp]);
        }
    }
    else  if (mode == SoundTooFar) //语音模块发声
    {
        char i_caText[] = {"[v3][s9][m52]太远了，请等等我。"};
        i_ucLen = strlen(i_caText);

        ga_ucUartTxdBuf[0] = 0xFD; //帧头
        ga_ucUartTxdBuf[1] = 0x00; //长度高字节
        ga_ucUartTxdBuf[2] = i_ucLen + 2; //长度低字节
        ga_ucUartTxdBuf[3] = 0x01; //命令字
        ga_ucUartTxdBuf[4] = 0x00; //编码格式
        i_ucLen = strlen(ga_ucUartTxdBuf);
        for(i_uctmp = 0; i_uctmp < 5; i_uctmp++)
        {
            SendByUart3(ga_ucUartTxdBuf[i_uctmp]);
        }

        i_ucLen = strlen(i_caText);
        for(i_uctmp = 0; i_uctmp < i_ucLen; i_uctmp++)
        {
            SendByUart3(i_caText[i_uctmp]);
        }
    }
    else  if (mode == SoundObstacle) //语音模块发声
    {
        char xdata i_caText[] = {"[v3][s9][m3]前方有障碍"};
        i_ucLen = strlen(i_caText);

        ga_ucUartTxdBuf[0] = 0xFD; //帧头
        ga_ucUartTxdBuf[1] = 0x00; //长度高字节
        ga_ucUartTxdBuf[2] = i_ucLen + 2; //长度低字节
        ga_ucUartTxdBuf[3] = 0x01; //命令字
        ga_ucUartTxdBuf[4] = 0x00; //编码格式
        i_ucLen = strlen(ga_ucUartTxdBuf);
        for(i_uctmp = 0; i_uctmp < 5; i_uctmp++)
        {
            SendByUart3(ga_ucUartTxdBuf[i_uctmp]);
        }

        i_ucLen = strlen(i_caText);
        for(i_uctmp = 0; i_uctmp < i_ucLen; i_uctmp++)
        {
            SendByUart3(i_caText[i_uctmp]);
        }
    }
    else  if (mode == SoundSpecial) //语音模块发声
    {
        char xdata i_caText[120] = {"[v3][s9][m3]交大首款智能跟随小车上线啦！智能小车灵活移动，陪您嗨翻天，给您不一样的跟随体验！"};
        i_ucLen = strlen(i_caText);

        ga_ucUartTxdBuf[0] = 0xFD; //帧头
        ga_ucUartTxdBuf[1] = 0x00; //长度高字节
        ga_ucUartTxdBuf[2] = i_ucLen + 2; //长度低字节
        ga_ucUartTxdBuf[3] = 0x01; //命令字
        ga_ucUartTxdBuf[4] = 0x00; //编码格式
        i_ucLen = strlen(ga_ucUartTxdBuf);
        for(i_uctmp = 0; i_uctmp < 5; i_uctmp++)
        {
            SendByUart3(ga_ucUartTxdBuf[i_uctmp]);
        }

        i_ucLen = strlen(i_caText);
        for(i_uctmp = 0; i_uctmp < i_ucLen; i_uctmp++)
        {
            SendByUart3(i_caText[i_uctmp]);
        }
    }
    else  if (mode == SoundSignalGot) //语音模块发声
    {
        char i_caText[] = {"[v2][s9][m52]收到信号"};
        i_ucLen = strlen(i_caText);

        ga_ucUartTxdBuf[0] = 0xFD; //帧头
        ga_ucUartTxdBuf[1] = 0x00; //长度高字节
        ga_ucUartTxdBuf[2] = i_ucLen + 2; //长度低字节
        ga_ucUartTxdBuf[3] = 0x01; //命令字
        ga_ucUartTxdBuf[4] = 0x00; //编码格式
        i_ucLen = strlen(ga_ucUartTxdBuf);
        for(i_uctmp = 0; i_uctmp < 5; i_uctmp++)
        {
            SendByUart3(ga_ucUartTxdBuf[i_uctmp]);
        }

        i_ucLen = strlen(i_caText);
        for(i_uctmp = 0; i_uctmp < i_ucLen; i_uctmp++)
        {
            SendByUart3(i_caText[i_uctmp]);
        }
    }
    else  if (mode == SoundLeft) //语音模块发声
    {
        char i_caText[] = {"[v3][s10][m52]左"};
        i_ucLen = strlen(i_caText);

        ga_ucUartTxdBuf[0] = 0xFD; //帧头
        ga_ucUartTxdBuf[1] = 0x00; //长度高字节
        ga_ucUartTxdBuf[2] = i_ucLen + 2; //长度低字节
        ga_ucUartTxdBuf[3] = 0x01; //命令字
        ga_ucUartTxdBuf[4] = 0x00; //编码格式
        i_ucLen = strlen(ga_ucUartTxdBuf);
        for(i_uctmp = 0; i_uctmp < 5; i_uctmp++)
        {
            SendByUart3(ga_ucUartTxdBuf[i_uctmp]);
        }

        i_ucLen = strlen(i_caText);
        for(i_uctmp = 0; i_uctmp < i_ucLen; i_uctmp++)
        {
            SendByUart3(i_caText[i_uctmp]);
        }
    }
    else  if (mode == SoundRight) //语音模块发声
    {
        char i_caText[] = {"[v3][s10][m52]右"};
        i_ucLen = strlen(i_caText);

        ga_ucUartTxdBuf[0] = 0xFD; //帧头
        ga_ucUartTxdBuf[1] = 0x00; //长度高字节
        ga_ucUartTxdBuf[2] = i_ucLen + 2; //长度低字节
        ga_ucUartTxdBuf[3] = 0x01; //命令字
        ga_ucUartTxdBuf[4] = 0x00; //编码格式
        i_ucLen = strlen(ga_ucUartTxdBuf);
        for(i_uctmp = 0; i_uctmp < 5; i_uctmp++)
        {
            SendByUart3(ga_ucUartTxdBuf[i_uctmp]);
        }

        i_ucLen = strlen(i_caText);
        for(i_uctmp = 0; i_uctmp < i_ucLen; i_uctmp++)
        {
            SendByUart3(i_caText[i_uctmp]);
        }
    }
    else  if (mode == SoundStraight) //语音模块发声
    {
        char i_caText[] = {"[v3][s10][m52]直"};
        i_ucLen = strlen(i_caText);

        ga_ucUartTxdBuf[0] = 0xFD; //帧头
        ga_ucUartTxdBuf[1] = 0x00; //长度高字节
        ga_ucUartTxdBuf[2] = i_ucLen + 2; //长度低字节
        ga_ucUartTxdBuf[3] = 0x01; //命令字
        ga_ucUartTxdBuf[4] = 0x00; //编码格式
        i_ucLen = strlen(ga_ucUartTxdBuf);
        for(i_uctmp = 0; i_uctmp < 5; i_uctmp++)
        {
            SendByUart3(ga_ucUartTxdBuf[i_uctmp]);
        }

        i_ucLen = strlen(i_caText);
        for(i_uctmp = 0; i_uctmp < i_ucLen; i_uctmp++)
        {
            SendByUart3(i_caText[i_uctmp]);
        }
    }
    else
    {
        ga_ucUartTxdBuf[0] = 0x55; //帧头1
        ga_ucUartTxdBuf[1] = 0xAA; //帧头2
        ga_ucUartTxdBuf[3] = 0x56; //STC8A的地址，为固定值

        if (mode == FindDistance) //向接收端查找距离
        {
            ga_ucUartTxdBuf[2] = 0x10; //
            ga_ucUartTxdBuf[4] = 0x02; //
            ga_ucUartTxdBuf[5] = 0x63; //
            ga_ucUartTxdBuf[6] = 0x01; //
            ga_ucUartTxdBuf[7] = 0x9B; //
            for(i_uctmp = 0; i_uctmp < 8; i_uctmp++)
            {
                SendByUart2(ga_ucUartTxdBuf[i_uctmp]);
            }
        }
        else if (mode == FindTemperature) //向接收端查找温度
        {
            ga_ucUartTxdBuf[2] = 0x10;
            ga_ucUartTxdBuf[3] = 0x56;
            ga_ucUartTxdBuf[4] = 0x02;
            ga_ucUartTxdBuf[5] = 0x52;
            ga_ucUartTxdBuf[6] = 0x01;
            ga_ucUartTxdBuf[7] = 0xAC;
            for(i_uctmp = 0; i_uctmp < 8; i_uctmp++)
            {
                SendByUart2(ga_ucUartTxdBuf[i_uctmp]);
            }
        }
        else if (mode == StartULS) //向发射端启动超声波发送
        {
            ga_ucUartTxdBuf[2] = 0xF1;
            ga_ucUartTxdBuf[3] = 0x56;
            ga_ucUartTxdBuf[4] = 0x02;
            ga_ucUartTxdBuf[5] = 0x51;
            ga_ucUartTxdBuf[6] = 0x20;
            ga_ucUartTxdBuf[7] = 0x8E;
            for(i_uctmp = 0; i_uctmp < 8; i_uctmp++)
            {
                SendByUart2(ga_ucUartTxdBuf[i_uctmp]);
            }
        }
    }
}
/*-------- 中断处理 --------*/
/*********************************************/
/*-------- 外部中断0（左电机霍尔A相） --------*/
/*********************************************/
void External0_Int(void) interrupt 0 using 1
//外部中断0 INT0对应P3.2口
{
    //高位屏蔽，防止溢出
    g_ucLeftHallCnt = min(g_ucLeftHallCnt + 1, 255);
}
/*********************************************/
/*-------- 外部中断1（右电机霍尔A相） --------*/
/*********************************************/
void External1_Int(void) interrupt 2 using 1
//外部中断1 INT1对应P3.3口
{
    //高位屏蔽，防止溢出
    g_ucRightHallCnt = min(g_ucRightHallCnt + 1, 255);
}
///************************************************/
///*-------- 外部中断2（左上角红外传感器） --------*/
///***********************************************/
//void External2_Int(void) interrupt 10 using 1
////外部中断2 INT2对应P3.6口
//{
//    g_bLeftwardFlag = 1;
//}
///************************************************/
///*-------- 外部中断3（右上角红外传感器） --------*/
///***********************************************/
//void External3_Int(void) interrupt 11 using 1
////外部中断2 INT2对应P3.6口
//{
//    g_bRightwardFlag = 1;
//}
/*********************************************/
/*-------- 定时器0时基中断（PWM周期） --------*/
/*********************************************/
void Timer0_Int(void) interrupt 1 using 2
{

}
/*********************************************/
/*-------- 定时器1时基中断（1ms一次） --------*/
/*********************************************/
void Timer1_Int(void) interrupt 3 using 2
{
    g_b1msFlag = 1;
}
/*********************************************/
/*-------- 串口3的中断（收发处理） --------*/
/*********************************************/
void UART3_Int(void) interrupt 17 using 1
{
    if(S3CON & S3RI)
    {
        S3CON &= ~S3RI;
    }
    if(S3CON & S3TI)
    {
        S3CON &= ~S3TI;
        g_bUart3BusyFlag = 0;
    }
}
/*********************************************/
/*-------- 串口2的中断（收发处理） --------*/
/*********************************************/
void UART2_Int(void) interrupt 8 using 1
{
    if(S2CON & S2RI)
    {
        S2CON &= ~S2RI;
        ga_ucUartRxdBuf[g_ucUART2SavePst] = S2BUF;
        g_ucUART2SavePst	= (g_ucUART2SavePst + 1) & (UARTBufMaxBytes - 1); // 利用屏蔽高位的方式实现指针的环形处理
    }
    if(S2CON & S2TI)
    {
        S2CON &= ~S2TI;
        g_bUart2BusyFlag = 0;
    }
}