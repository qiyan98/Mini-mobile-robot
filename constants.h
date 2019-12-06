/*-------- MCU固定参数 --------*/
#define FOSC 11059200UL //设定STC8单片机频率
#define PWMCycle (60000) //PWM周期，单位Hz
#define BRT0 (65536 - FOSC/PWMCycle) //定时器0初值设定，用于确定PWM周期
#define BAUD2 115200 //设定串口2通信波特率
#define BRT2 (65536 - FOSC/12/4/BAUD2) //定时器2初值设定
#define BAUD3 9600 //设定串口3通信波特率
#define BRT3 (65536 - FOSC / BAUD3 / 4) //定时器3初值设定

/*-------- 其它固定参数 --------*/
#define UARTBufMaxBytes 64 //UART收发缓存最大字节数
#define HallBufMaxBytes 8 //HALL传感器接收缓存最大字节数
#define LocationBufMaxBytes 8 //坐标数组最大字节数
#define WheelSize 65 //轮子直径，单位mm
#define WheelDistance 175 //2个轮子之间的距离，单位mm

/*-------- 电机速度环PID参数 --------*/
#define K_P 120
#define K_I 50
#define K_D 5

/*-------- 人车角度环PID参数 --------*/
#define K_PA (120 / 1000)
#define K_IA (50 / 1000)
#define K_DA (5 / 1000)

/*-------- 其他可调常数 --------*/
#define DeltaTime 5 //测量过程中每次失败的“耗时”
#define MaxOverTime 30 //每次测量的最大“超时时间”
#define T_Work 20 //有限状态机状态更新周期
#define T_Sampling 30 //采样周期，单位ms

#define DeltaDistance 400 //4个通道的距离所允许的最大差值，数据检测时使用
#define MaxFailureNum 6 // 3 //最多允许的连续失败次数，超出时暂停工作一段时间

/*-------- 角度反馈参数 --------*/
#define StaticDeltaAngle 10 //静态转向时允许的角度差值
#define DynamicDeltaAngle 15 //动态转向时允许的角度差值
#define MinX 100 //小车不转向的最小X值
#define MinR 600 //人车之间的最小距离

/*-------- 速度参数 --------*/ 
#define UltraLowSpeed 0.3
#define LowSpeed 0.5
#define TurningSpeed 0.7
#define NormalSpeed 0.8 // 0.6
#define FastSpeed 1.2 // 0.7
#define LowerPWM -1 //相对另一个电机更低的PWM
#define ConstPWM -2 //固定占空比
#define StopPWM -3 //停止命令
#define DeltaDuty 10
#define ConstDuty 150 //固定占空比的值

/*-------- 计算坐标所用参量 --------*/
//单位均为mm，相对2车轮中点的坐标；各个传感器必须置于同一高度。
#define DIS_S 230

/*-------- PWM特殊管脚 --------*/
//PWM输出的正电压由制定管脚输出，无需自定义
//PWM0（左电机）:P17；PWM1（右电机）：P16
//霍尔传感器使用外部中断，无需自己定义管脚
#define PIN_LGnd P27
#define PIN_RGnd P26

/*-------- 串口2/3发送数据使用的标志量 --------*/
#define SoundSignalLost 1
#define FindDistance 2
#define FindTemperature 3
#define StartULS 4
#define SoundParameter 5
#define SoundLeft 6
#define SoundRight 7
#define SoundStraight 8
#define SoundStart 9
#define SoundTooFar 10
#define SoundSignalGot 11
#define SoundObstacle 12
#define SoundSpecial 13

/*-------- 检验帧数据时所用的标志量 --------*/
#define GeneralULSFrame 1
#define TxdULSFrame 2
#define RxdULSFrame 3

/*-------- 通信循环所使用的标志量 --------*/
#define WAIT_START 0 //等待态，进行通信循环初始化
#define START_TXD 1 //启动超声波发射
#define WAIT_TXD_ACK 2 //等待超声波发射应答
#define WAIT_READ 3 //延时等待接收端收到信号
#define START_READ 4 //开始读取距离数据
#define WAIT_READ_ACK 5 //等待读取距离的应答
#define CAL_XYZ 6 //根据距离数据计算XY坐标
#define CONTROL 7 //根据所得XY坐标其其它传感器进行控制

/*-------- 定义max与min函数 --------*/
#define max(a,b) ( ((a)>(b)) ? (a):(b) )
#define min(a,b) ( ((a)>(b)) ? (b):(a) )

/*-------- 定时器0进行1ms计数 --------*/
// 初始位置值 = 65536 - 11059200/12*t0 (t0为时基单位，如1ms，太大不可以）
#define Timer1msH 0xFC
#define Timer1msL 0x66

/*-------- AUXR寄存器常用量 --------*/
//定时器0、1、2的模式//
#define AUXR_T0x12 0x00 //T0进行12分频
#define AUXR_T0x1 0x80 //T0不进行12分频
#define AUXR_T1x12 0x00 //T1进行12分频
#define AUXR_T1x1 0x40 //T1不进行12分频
#define AUXR_T2x12 0x00 //T2进行12分频
#define AUXR_T2x1 0x04 //T2不进行12分频
//串口通讯//
#define AUXR_UART1_M0x1 0x00 //串口1模式0波特率不加倍
#define AUXR_UART1_M0x6 0x20 //串口1模式0波特率加倍（*6）
#define AUXR_UART1_T1 0x00 //用T1作波特率发射器
#define AUXR_UART1_T2 0x01 //用T2作波特率发生器

/*-------- SCON寄存器常用量 --------*/
//串口1的模式
#define UART1_M0 0x00
#define UART1_M1 0x40
#define UART1_M2 0x80
#define UART1_M3 0xC0

/*-------- S2CON寄存器常用量 --------*/
//串口2的模式
#define UART2_M0 0x00
#define UART2_M1 0x80

/*-------- S3CON寄存器常用量 --------*/
//串口3的模式，均允许UART3的输入（S3REN = 1）
#define UART3_M0T3 0x50
#define UART3_M1T3 0xD0

/*-------- TMOD寄存器常用量 --------*/
//定时器0的模式
#define Timer0_M0 0x00 //模式0；16位自动重载
#define Timer0_M1 0x01 //模式1；16位不自动重载
#define Timer0_M2 0x02 //模式2；8位自动重载
#define Timer0_M3 0x03 //模式3；停止工作
//定时器1的模式
#define Timer1_M0 0x00 //模式0；16位自动重载
#define Timer1_M1 0x10 //模式1；16位不自动重载
#define Timer1_M2 0x20 //模式2；8位自动重载
#define Timer1_M3 0x30 //模式3；停止工作
//定时器2、3、4的模式固定为16位自动重载