//
// Created by LEGION on 2021/10/4.
//
//This file contains the main function and the interrupt service routines (ISRs).
//The usage of each module follows three steps: initialization, data acquisition, and execution.
//Initialization and execution are performed within the timer interrupt routine.
//Serial (UART) data reception is handled within the UART interrupt routine.
//
//此文件为主函数和中断服务函数所在地
//每个元件的使用包含三个步骤，初始化、获取数据、执行
//初始化和执行在定时器中断函数进行
//获取串口数据在串口中断函数进行
#include "Usermain.h"
#include "IMU.h"
#include "Propeller.h"
#include "Servo.h"
#include "Sensor.h"
#include "Legacy.h"
#include "Watchdog.h"
#include "LED.h"
#include "Buzzer.h"
#include "gpio.h"
#include "UART_Base.h"


//Adjust according to the vehicle version: V30, V31, V32, V33, V40.
//根据潜器版本调整，V30，V31，V32，V33，V40
VERSION_E Robot_Version = V33;


// Statically instantiated objects.
// 静态实例化对象
// IMU imu;
static Servo servo;
static Servo_I2C servo_i2c;
static Propeller_I2C propeller_i2c;
static Sonar sonar;
static Watchdog watchdog;
static LED led;
static Buzzer buzzer;


// Array of device pointers for unified management and invocation of device operations.
// ------ TODO: Select the required devices.
// 设备指针数组，用于统一管理和调用设备操作
//------TODO:选用需要的设备
static Device *device[] = {
        &IMU::imu,                         //IMU 
        &PressureSensor::pressure_sensor,  //Pressure Sensor 水压计
        &propeller_i2c,                    //Propeller（PWM control via the PWM expansion board） 推进器（PWM扩展板控制）
        //&propeller,                      //Propeller（PWM control via the Robomaster_C board） 推进器（C板PWM控制）
				&servo_i2c,                        //Servo（PWM control via the PWM expansion board） 舵机（PWM扩展板控制）
				//&servo,                          //Servo（PWM control via the Robomaster_C board） 舵机（C板PWM控制）
        //&watchdog,                       //Watchdog 窗口看门狗
        &led,                              //LED
        //&buzzer                          //Buzzer 蜂鸣器
};

// Macro definition for the number of devices.
// 定义设备数量宏
#define DEVICE_NUM (sizeof(device) / sizeof(device[0]))

volatile bool system_init_flag = false;
volatile bool tick_300hz = false;
uint8_t RxBuffer[SERIAL_LENGTH_MAX] = {0};
volatile uint8_t key_raw_state = 1;
uint32_t key_last_stamp;
volatile int32_t time_start = 0, time_end = 0, time_interval = 0, cnt = 0;

// Timer interrupt service routine (ISR) for periodic device processing.
// 定时器中断服务函数, 用于周期性处理设备
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(!system_init_flag) return;
    if(htim == &htim1){ //300Hz
        tick_300hz = true;
    }
}

// UART receive interrupt service routine (ISR) for handling device-specific UART reception tasks.
// 串口接收中断服务函数, 用于处理设备的串口接收任务
// NOTE: the 422 upper-computer link was migrated from USART6 to USART1.
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t datasize)//HAL_UART_RxCpltCallback
{
    if(huart->Instance==USART1) {
        // Telemetry selection ("OUT:<what>") belongs to no device - handle it
        // first, while RxBuffer is still intact (a device may strtok() it).
        // 遥测选择命令不属于任何一个器件，先处理，此时 RxBuffer 还没被破坏
        telemetry_parse(RxBuffer);
        // if (datasize >= 2){
            // if (strncmp((char *)RxBuffer, "IM", 2) == 0)
            //     device[0]->Receive();
            // else if (strncmp((char *)RxBuffer, "PS", 2) == 0)
            //     device[1]->Receive();
            // else if (strncmp((char *)RxBuffer, "PR", 2) == 0)
            //     device[2]->Receive();
            // else if (strncmp((char *)RxBuffer, "SE", 2) == 0)
            //     device[3]->Receive();
            // else if (strncmp((char *)RxBuffer, "LE", 2) == 0)
            //     device[4]->Receive();
            for(int i=0;i<DEVICE_NUM;++i){
                device[i]->Receive();
            // }
        }
        
        std::memset(RxBuffer, 0, SERIAL_LENGTH_MAX);
        // Re-enable UART reception.
				// 重新开启接收
        HAL_UARTEx_ReceiveToIdle_IT(&huart1, RxBuffer, SERIAL_LENGTH_MAX);
    }
}

// UART error callback.
// 串口错误回调
// The 422 link must survive overrun/framing errors: on any error the HAL stops
// the reception (UART_EndRxTransfer() clears RXNEIE/IDLEIE) and never restarts
// it, so without this callback a single ORE kills the link until reset.
// 422 链路必须能从溢出/帧错误中恢复：HAL 在出错时会关闭接收中断
// (UART_EndRxTransfer 清掉 RXNEIE/IDLEIE) 且不会自动重开，
// 所以没有这个回调的话，一次 ORE 就会让链路一直收不到数据，直到复位。
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_PEFLAG(huart);
        std::memset(RxBuffer, 0, SERIAL_LENGTH_MAX);
        HAL_UARTEx_ReceiveToIdle_IT(huart, RxBuffer, SERIAL_LENGTH_MAX);
    }
}

// UART transmit interrupt callback function.
// 串口发送中断回调函数
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    #define HANDLE_UART_TX(ID)                            \
        do {                                              \
            if (huart == uartHandleList[ID]) {            \
                UARTBaseLite<ID>::GetInstance().OnTxDone(); \
                return;                                   \
            }                                             \
        } while (0)

    HANDLE_UART_TX(1);
    HANDLE_UART_TX(6);

    #undef HANDLE_UART_TX
    // if(huart == &huart1) {
    //     UARTBaseLite<1>::GetInstance().OnTxDone();
    // }
    // else if(huart == &huart6){
    //     UARTBaseLite<6>::GetInstance().OnTxDone();
    // }
}

// Key/button interrupt (currently unused).
// 按键中断，暂时无用
// void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
//     if(system_init_flag == 0)return;
//     if (GPIO_Pin == GPIO_PIN_0){
//         key_raw_state = HAL_GPIO_ReadPin(KEY_GPIO_Port,KEY_Pin);
//         uint32_t time_interval = HAL_GetTick() - key_last_stamp;
//         key_last_stamp = HAL_GetTick();
//         /*if (time_interval >= 50){
//             if (key_raw_state == 1){
//                 period = 5000;
//             }else{
//                 period = 500;
//             }
//         }*/
//     }
//     IMU::imu.ITHandle(GPIO_Pin);
// }

/**
  * @brief  The application entry point.
  * @retval int
  */
// Entry point of the main function.
// 主函数入口
int main(void)
{
    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* Configure the system clock */
    SystemClock_Config();

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_TIM5_Init();
    MX_TIM1_Init();
    MX_TIM4_Init();
    MX_TIM6_Init();
    MX_TIM7_Init();
    MX_TIM10_Init();
    MX_TIM8_Init();
    MX_ADC1_Init();
    MX_ADC3_Init();
    MX_USART1_UART_Init();
    MX_USART6_UART_Init();
    MX_USART3_UART_Init();
    MX_CAN1_Init();
    MX_CAN2_Init();
    MX_I2C2_Init();
    MX_I2C3_Init();
    MX_SPI1_Init();
    MX_SPI2_Init();
    //MX_IWDG_Init();
    MX_USB_DEVICE_Init();

    HAL_TIM_Base_Start_IT(&htim5);
    HAL_TIM_Base_Start_IT(&htim1);
    HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_4);


    HAL_TIM_Base_Start_IT(&htim8);
    HAL_TIM_PWM_Start_IT(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start_IT(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start_IT(&htim8, TIM_CHANNEL_3);
    //TODO adc校准？
    //RemoteControl::init();
    bsp_flash_read(&flashData);
    HAL_TIM_Base_Start_IT(&htim10);
    HAL_TIM_PWM_Start_IT(&htim10, TIM_CHANNEL_1);
    HAL_TIM_Base_Start_IT(&htim6);
    HAL_TIM_Base_Start_IT(&htim7);

		// Initialize all devices.
    // 初始化所有设备
    for(int i = 0; i < DEVICE_NUM; ++i){
        device[i] -> Init();
    }
		
		// Set the initialization-complete flag.
    // 设置初始化完成标志
    system_init_flag = true;
    time_start = HAL_GetTick();

    while (1){
        // time_start = HAL_GetTick();
        if(tick_300hz){
            tick_300hz = false;
            static int a = 0;
            a++;
            for(int i = 0 ; i < DEVICE_NUM ; ++i){
                device[i]->Handle();
//                if (a==150&&i==4)
//                {
//                    time_end = HAL_GetTick();
//                    time_interval = time_end - time_start;
//                    send_int(time_interval, 1);
//                    a = 0;
//                    time_start = HAL_GetTick();
//                }
            }

            /* Periodic telemetry, emitted from the main loop (~100 Hz) and NOT
             * from the IMU DMA ISR any more - see the note in IMU.cpp.  The SPI
             * DMA chain can stall; when the output lived there, the depth stream
             * died with it and looked like a dead sensor.
             * 遥测改到主循环发（约 100Hz）。以前挂在 IMU 的 DMA 中断上，
             * IMU 链一卡水深就一起停了。 */
            static uint8_t tele_div = 0;
            if (++tele_div >= 3)
            {
                tele_div = 0;
                telemetry_send();
            }
        }
    }
}

void send_float(float value, uint8_t decimalPlaces, bool endSign){
    bool isNegative = 0;
    uint8_t len = 0;
    static char buffer_float[20] = {0};
		
		// Handle negative values.
    // 处理负数
    if(value < 0){
        isNegative = 1;
        value = -value;
    }
    if(isNegative) {
        buffer_float[0] = '-';
        len = 1;
    }

		// Convert the integer part.
    // 转换整数部分
    int32_t intPart = (int)value;
    len += sprintf(buffer_float+len, "%d", intPart);
		
		// Process the fractional part.
    // 处理小数部分
    if (decimalPlaces > 0) {
        buffer_float[len++] = '.';

        float fraction = value - (float)intPart;
        for (uint8_t i = 0; i < decimalPlaces; i++) {
            fraction *= 10;
            int digit = (int)fraction;
            buffer_float[len++] = digit + '0';
            fraction -= digit;
        }
    }

		// If this is the final element, append a newline; otherwise, append a comma.
    // 若结束则换行，否则是逗号
    if(endSign)
        buffer_float[len++] = '\n'; // Null terminator of the string 字符串结束符
    else
        buffer_float[len++] = ',';

    buffer_float[len] = '\0';

    // HAL_UART_Transmit(&huart6, (uint8_t *)buffer, len, 0x00ff);
    UARTBaseLite<1>::GetInstance().Transmit(reinterpret_cast<uint8_t *>(buffer_float),len);
}

void send_int(int32_t value, bool endSign){
    static char buffer_int[20] = {0};
    uint8_t len = 0;
    bool isNegative = 0;

		// Handle negative values.
    // 处理负数
    if(value < 0){
        isNegative = 1;
        value = -value;
    }
    if(isNegative) {
        buffer_int[0] = '-';
        len = 1;
    }

    len += sprintf(buffer_int+len, "%d", value);

    if(endSign)
        buffer_int[len++] = '\n';
    else
        buffer_int[len++] = ',';

    buffer_int[len] = '\0';
    // HAL_UART_Transmit(&huart6, (uint8_t *)buffer, len, 0x00ff);
    UARTBaseLite<1>::GetInstance().Transmit(reinterpret_cast<uint8_t *>(buffer_int),len);
}
/* ---------------------------------------------------------------------------
 * 422 telemetry - see Usermain.h for the wire format.
 * --------------------------------------------------------------------------- */
volatile uint8_t telemetry_mode = TELEMETRY_PS;

static const struct { const char *name; uint8_t mode; } telemetry_table[] = {
    {"PS",   TELEMETRY_PS},
    {"RPY",  TELEMETRY_RPY},
    {"GYRO", TELEMETRY_GYRO},
    {"ACC",  TELEMETRY_ACC},
    {"MAG",  TELEMETRY_MAG},
    {"TEMP", TELEMETRY_TEMP},
    {"ALL",  TELEMETRY_ALL},
    {"PCA",  TELEMETRY_PCA},
    {"OFF",  TELEMETRY_OFF},
};
#define TELEMETRY_TABLE_NUM (sizeof(telemetry_table) / sizeof(telemetry_table[0]))

/* Raw text into the 422 tx queue (send_float()/send_int() only do numbers). */
static void telemetry_write(const char *text)
{
    UARTBaseLite<1>::GetInstance().Transmit(
        reinterpret_cast<uint8_t *>(const_cast<char *>(text)),
        (uint16_t)strlen(text));
}

/* "<name>,v0,v1,...\n" - send_float() already appends ',' or '\n'. */
static void telemetry_line(const char *name, const float *v, uint8_t n, uint8_t dec)
{
    telemetry_write(name);
    for (uint8_t i = 0; i < n; ++i)
        send_float(v[i], dec, (i == (uint8_t)(n - 1)));
}

bool telemetry_parse(const uint8_t *cmd)
{
    if (strncmp((const char *)cmd, "OUT", 3) != 0)
        return false;

    /* "OUT" alone: report what is selected right now. */
    if (((const char *)cmd)[3] != ':')
    {
        for (uint32_t i = 0; i < TELEMETRY_TABLE_NUM; ++i)
        {
            if (telemetry_table[i].mode == telemetry_mode)
            {
                telemetry_write("OUT:");
                telemetry_write(telemetry_table[i].name);
                telemetry_write("\n");
                break;
            }
        }
        return true;
    }

    const char *what = (const char *)cmd + 4;
    for (uint32_t i = 0; i < TELEMETRY_TABLE_NUM; ++i)
    {
        if (strncmp(what, telemetry_table[i].name, strlen(telemetry_table[i].name)) == 0)
        {
            telemetry_mode = telemetry_table[i].mode;
            /* Acknowledge so the upper computer knows it took effect. */
            telemetry_write("OUT:");
            telemetry_write(telemetry_table[i].name);
            telemetry_write("\n");
            return true;
        }
    }
    return false;
}

void telemetry_send(void)
{
    static uint8_t divider = 0;
    uint8_t mode = telemetry_mode;

    if (mode == TELEMETRY_OFF)
        return;

    if (mode == TELEMETRY_ALL)
    {
        /* The full frame is ~150 bytes - send it at ~10 Hz, not 100 Hz. */
        if (++divider < 10)
            return;
        divider = 0;
    }

    bool all = (mode == TELEMETRY_ALL);
    float v[6];

    if (all || mode == TELEMETRY_PS)
    {
        /* PS,<p0..p3>,<ok0..ok3> - the flags say which gauge answered its
         * PROM read, i.e. which reading can be trusted.  PS,<压强x4>,<在位x4> */
        telemetry_write("PS,");
        for (int i = 0; i < SENSOR_NUM; ++i)
            send_float(PressureSensor::pressure_sensor.data_pressure[i], 2, false);
        for (int i = 0; i < SENSOR_NUM; ++i)
            send_int(PressureSensor::pressure_sensor.is_ok(i) ? 1 : 0,
                     (i == (SENSOR_NUM - 1)));
    }

    if (all || mode == TELEMETRY_RPY)
    {
        for (int i = 0; i < 3; ++i)
            v[i] = IMU::imu.pro_data.ins_angle[i] * 180.0f / PI;
        telemetry_line("RPY,", v, 3, 2);
    }

    if (all || mode == TELEMETRY_GYRO)
    {
        for (int i = 0; i < 3; ++i)
            v[i] = IMU::imu.pro_data.gyro[i];
        telemetry_line("GYRO,", v, 3, 4);
    }

    if (all || mode == TELEMETRY_ACC)
    {
        for (int i = 0; i < 3; ++i)
            v[i] = IMU::imu.pro_data.accel[i];
        telemetry_line("ACC,", v, 3, 4);
    }

    if (all || mode == TELEMETRY_MAG)
    {
        for (int i = 0; i < 3; ++i)
        {
            v[i]     = IMU::imu.pro_data.mag[i];   /* calibrated / 校准后 */
            v[i + 3] = IMU::imu.raw_data.mag[i];   /* raw / 原始 */
        }
        telemetry_line("MAG,", v, 6, 3);
    }

    if (all || mode == TELEMETRY_TEMP)
    {
        v[0] = IMU::imu.raw_data.temp;
        telemetry_line("TEMP,", v, 1, 2);
    }

    if (mode == TELEMETRY_PCA)
    {
        /* Read the PWM expander back: if the servos do not move, this says
         * whether the PCA9685 ever received the values.  Motor/servo channels
         * are 0-7 and 8-11.
         * 回读 PWM 扩展板：舵机不动时，用它判断 PCA9685 到底有没有收到数值。 */
        TCA_SetChannel(4);
        v[0] = (float)PCA_Read(PCA9685_MODE1);
        v[1] = (float)PCA_Read(PCA9685_PRESCALE);
        for (int i = 0; i < 4; ++i)
        {
            uint8_t lo = PCA_Read(LED0_OFF_L + 4 * (8 + i));
            uint8_t hi = PCA_Read(LED0_OFF_H + 4 * (8 + i));
            v[2 + i] = (float)(((uint16_t)hi << 8) | lo);
        }
        telemetry_line("PCA,", v, 6, 0);
    }
}
