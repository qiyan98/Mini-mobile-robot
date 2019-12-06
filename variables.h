#include "constants.h"
#include "stc8.h"

/*-------- ULS距离数据 --------*/
unsigned char xdata g_ucOverTimeCnt; //测量过程中超时量
unsigned char xdata g_ucFailureNum; //连续数据错误（掉线）的次数
int xdata ga_iDistance[4]; //4路传感器检测到的距离
unsigned char xdata g_ucSpcDstNum; //特殊距离序号，在ga_iDistance中的序号，计算坐标不使用的距离数据

/*-------- ULS坐标数据 --------*/
float xdata g_fCurX; //当前X坐标
float xdata g_fCurY; //当前Y坐标
float xdata g_fCurZ; //当前Z坐标
float xdata g_fCurR; //当前距离，极坐标描述
float xdata g_fCurA; //当前角度，极坐标描述
float xdata ga_fCurXBuf[LocationBufMaxBytes]; //有效的X坐标缓存
float xdata ga_fCurYBuf[LocationBufMaxBytes]; //有效的Y坐标缓存
unsigned char g_ucLocationSavePst; //保存合理位置的元素编号

/*-------- PID速度控制数据 --------*/
float xdata g_fLeftCurError; //左电机速度误差
float xdata g_fLeftLastError; //左电机上一次速度误差
float xdata g_fRightCurError; //右电机速度误差
float xdata g_fRightLastError; //左电机上一次速度误差

/*-------- PID角度控制数据 --------*/
float xdata g_fAngleCurError; //当前角度误差
float xdata g_fAngleLastError; //上一次角度误差
float xdata g_fAngleLast2Error; //上2次角度误差
int xdata g_iSign; //左旋为1，右旋为-1

/*-------- 速度控制数据 --------*/
float xdata g_fLeftCurV; //当前速度，单位m/s
float xdata g_fLeftSetV; //设置的目标速度，单位m/s
float xdata g_fRightCurV; //当前速度，单位m/s
float xdata g_fRightSetV; //设置的目标速度，单位m/s
float xdata g_fSetA; //设置的目标角度，单位°

/*-------- 主循环变量 --------*/
unsigned char xdata g_ucWorkStat; //主程序分时状态量
unsigned char xdata g_ucTmpFlag; //临时FLAG
unsigned char xdata g_ucTmp;
float xdata g_fTmp;

/*-------- 语音播报变量 --------*/
bit g_bSingleLostFlag;
bit g_bSoundStartFlag;
bit g_bSoundSpecialFlag;
bit g_bSoundObstacleFlag;

/*-------- 红外障碍检测变量 --------*/
bit g_bForwardFlag;
bit g_bBackwardFlag;
bit g_bLeftwardFlag;
bit g_bRightwardFlag;
bit g_bNoObstacleFlag;

/*-------- 时基标志量 --------*/
bit g_b1msFlag;
bit g_b10msFlag;
bit g_b50msFlag;
bit g_b100msFlag;
bit g_b500msFlag;
bit g_b1sFlag;
bit g_b3sFlag;
bit g_b5sFlag;
bit g_bSamplingFlag; //采样周期时基
bit g_bWorkFlag; //有限状态机时基
bit g_b1minFlag;
bit g_b3minFlag;
unsigned int g_uiCnt; //时基计数变量
unsigned char g_uiMinCnt; //分钟计数器

/*-------- PWM数据 --------*/
float xdata g_uiLeftCurDuty; //左电机当前占空比
float xdata g_uiRightCurDuty; //右电机当前占空比

/*-------- 霍尔传感器数据 --------*/
unsigned char xdata ga_ucLeftHallRxdBuf[HallBufMaxBytes]; //左霍尔传感器数组
unsigned char xdata ga_ucRightHallRxdBuf[HallBufMaxBytes]; //右霍尔传感器数组
float g_fLeftHallCurCnt; //左电机霍尔传感器0当前计数变量
float g_fRightHallCurCnt; //右电机霍尔传感器0当前计数变量
unsigned char g_ucLeftHallCnt; //左电机霍尔传感器0临时计数变量
unsigned char g_ucRightHallCnt; //右电机霍尔传感器0临时计数变量
unsigned char g_ucHallSavePst; //左右电机霍尔传感器存数的元素编号

/*-------- 串口收发数据处理变量 --------*/
unsigned char xdata ga_ucUartTxdBuf[UARTBufMaxBytes]; //UART发送缓存区
unsigned char xdata ga_ucUartRxdBuf[UARTBufMaxBytes]; //UART接收缓存区
unsigned char g_ucUART2SavePst; //保存UART2接受数据的元素编号
bit g_bUart3BusyFlag; //串口1发送正忙
bit g_bUart2BusyFlag; //串口2发送正忙