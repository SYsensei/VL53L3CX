//配置文件

//1代表上电进入低功耗模式，0代表正常模式
#define PARA_UltraLowPower 1

//en.en.VL53L3CX_UltraLowPower 驱动程序运行时需要调节的参数列表
//default 25ms
#define PARA_Initialization 25
//default 1000ms(20~60000)
#define PARA_Intermeasurement_period 1000
//default 1(1~255)
#define PARA_Macroperiod 1
//default 45mm(1~16383)
#define PARA_Sigma_threshold 45
//default 1500kcps(1~16383)
#define PARA_Signal_threshold 1500
//default 16*16(4*4~16*16)
#define PARA_Region_of_interest 16

//en.STSW-IMG015 驱动程序运行时需要调节的参数列表
//default 33ms
#define PARA_MeasurementTimingBudgetMicroSeconds 33
//default 15000
#define PARA_VL53LX_TUNINGPARM_RESET_MERGE_THRESHOLD 15000
//default 0
#define PARA_VL53LX_TUNINGPARM_PHASECAL_PATCH_POWER 0

