//
// Created by LEGION on 2021/10/4.
//

#ifndef RM_FRAME_C_DEVICE_H
#define RM_FRAME_C_DEVICE_H
#include "main.h"


#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "iwdg.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"
#include "usart.h"


#include <stdarg.h>
#include <string.h>
#include <cstring>   /* std::memcpy / std::memset under libstdc++ (GCC port) */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

//------TODO:修改串口长度上限、各器件数量

#define CONTROL_FREQUENCY 60        // 更新频率
#define SERIAL_LENGTH_MAX 100       // 串口指令最大长度
#define SENSOR_NUM 4                // 水压计数量
#define SERVO_NUM 4                 // 舵机数量
#define PROPELLER_NUM 8             // 推进器数量
#define LED_PERIOD 500              // 亮灯周期
#define PI 3.14159265358979323846

#define INRANGE(NUM, MIN, MAX) \
{\
    if(NUM<MIN){\
        NUM=MIN;\
    }else if(NUM>MAX){\
        NUM=MAX;\
    }\
}
/*枚举类型定义------------------------------------------------------------*/

/* 422 upper-computer protocol RX buffer. The link runs on USART1
 * (migrated from USART6); reception is driven by HAL_UARTEx_ReceiveToIdle_IT. */
extern uint8_t RxBuffer[SERIAL_LENGTH_MAX];

//extern void ChassisStart();
//extern void ChassisHandle();
//extern void CtrlHandle();
//extern void ARMHandle();

/*
 * 设备类型枚举
 */
typedef enum {
    IM, // imu
    PS, // pressure sensor
    PR, // propeller
    SE, // servo
    LE  // led
} DEVICE_TYPE_E;

typedef enum{
    V30,
    V31,
    V32,
    V33,
    V40
} VERSION_E;

extern VERSION_E Robot_Version;

//extern const int length;

/*extern void I2C_Extension_init();
extern void I2C_Extension();
*/
//extern void Update_level();


extern int Count_ms_dog;
extern int Flag_stop;
extern int Flag_serial;

/*结构体定义--------------------------------------------------------------*/



/*类型定义----------------------------------------------------------------*/



class Device {
public:
    DEVICE_TYPE_E deviceType;
    //uint32_t deviceID;

    virtual void Init() = 0;

    virtual void Handle() = 0;

    virtual void Receive() = 0;

    //virtual void ErrorHandle() = 0;

};

void send_float(float value, uint8_t decimalPlaces, bool endSign);

/* NOTE: int32_t, not int - on ARM `int` and `int32_t` (long int) are different
 * types, and the definition in Usermain.cpp takes an int32_t.  Declaring it as
 * `int` here would create a second, never-defined overload. */
void send_int(int32_t value, bool endSign);

/* ---------------------------------------------------------------------------
 * 422 telemetry: what the periodic stream sends back to the upper computer.
 * 422 遥测：周期性回传的内容，可用串口命令 OUT:<what> 选择。
 *
 * Select it at runtime by sending  "OUT:<what>"  on the 422 link.  Each frame
 * is one text line, always "<NAME>,<v0>,<v1>,...\n":
 *
 *   OUT:PS     PS,<p0>,<p1>,<p2>,<p3>,<k0>,<k1>,<k2>,<k3>
 *                                                    4x pressure (offset removed)
 *                                                    + 1/0 = gauge present
 *   OUT:RPY    RPY,<yaw>,<pitch>,<roll>           [deg]
 *   OUT:GYRO   GYRO,<gx>,<gy>,<gz>                [rad/s]
 *   OUT:ACC    ACC,<ax>,<ay>,<az>                 [m/s^2]
 *   OUT:MAG    MAG,<mx>,<my>,<mz>,<rx>,<ry>,<rz>  [uT] calibrated, then raw
 *   OUT:TEMP   TEMP,<temp>                        [degC] IMU temperature
 *   OUT:ALL    all of the above lines, at ~10 Hz
 *   OUT:PCA    PCA,<MODE1>,<PRESCALE>,<off8>,<off9>,<off10>,<off11>
 *              reads registers back out of the PWM expander - use it to
 *              check the board really received the servo/thruster values /
 *              回读 PWM 扩展板的寄存器, 用来确认舵机/推进器的值真的写进去了
 *   OUT:OFF    stop the stream
 *   OUT        just report the current selection (e.g. "OUT:GYRO")
 *
 * TELEMETRY_PS is the default and reproduces the original behaviour (the four
 * pressure readings, one frame per IMU DMA cycle).
 * TELEMETRY_PS 为默认值，即原来的行为（每个 IMU DMA 周期发 4 路水压）。
 * --------------------------------------------------------------------------- */
typedef enum {
    TELEMETRY_PS = 0,   /* default / 默认 */
    TELEMETRY_RPY,
    TELEMETRY_GYRO,
    TELEMETRY_ACC,
    TELEMETRY_MAG,
    TELEMETRY_TEMP,
    TELEMETRY_ALL,
    TELEMETRY_PCA,   /* read the PCA9685 back / 回读 PWM 扩展板 */
    TELEMETRY_OFF,
} TELEMETRY_E;

extern volatile uint8_t telemetry_mode;

/* Handle an "OUT:..." command from the upper computer.
 * Returns true when the buffer was ours (so nobody else has to look at it). */
bool telemetry_parse(const uint8_t *cmd);

/* Emit one telemetry frame (called from the IMU DMA handler). */
void telemetry_send(void);

/*结构体成员取值定义组------------------------------------------------------*/
/*外部变量声明-------------------------------------------------------------*/
/*外部函数声明-------------------------------------------------------------*/



#endif //RM_FRAME_C_DEVICE_H
