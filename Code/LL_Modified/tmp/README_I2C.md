# I2C 诊断脚本

## 新增：独立 PWM 测试（2026-10-17）

建议当前使用 `pwm_probe.py`。仅依赖 Python 标准库和 OpenOCD，不使用422，
不依赖 ELF/SRAM 地址，不烧录，不写 PWM 配置或占空比寄存器。
它暂停主程序轮询，通过 PF0/PF1 开漏 GPIO 产生低速软件 I2C，
只写 TCA 通道选择及 PCA 的读取指针，读取时逐笔检查 ACK。

先在一个终端启动（若已有 OpenOCD 在运行，不要重复启动）：

```bash
openocd -f interface/cmsis-dap.cfg -c 'transport select swd' -c 'adapter speed 1000' -f target/stm32f4x.cfg -c init
```

另一个终端在项目根目录执行：

```bash
python3 tmp/pwm_probe.py
```

脚本依次停留在以下测量阶段，按回车才继续，输入 `q` 或 Ctrl+C 退出：

1. MCU 释放引脚、关闭内部上下拉，保持原 TCA 选通状态。测 C 板和 PWM 板的 SDA/SCL。
2. TCA 全通道断开。测上游；此时下游高电平不代表上游高电平。
3. 仅选通4号口。再次测两侧，之后回读 PCA MODE1、MODE2、PRESCALE、通道8～11计数。

测试失败时，也会先释放 MCU 引脚并停留，方便在同一故障现场测量电压。
故障本身可能阻止进入后续阶段。脚本不会自动发9脉冲或复位消除现场。

PCA默认All Call地址与TCA的0x70重合，所以选通非零通道后不再回读0x70校验，
改为检查写操作ACK及PCA独立地址回读。原通道读值也可能受此重合影响，退出只能
按读值尽力恢复；若需要严格恢复原通道，应先消除地址重合。脚本不修改PCA的All Call设置。

只保持引脚释放、每秒采样一次，便于用万用表对照：

```bash
python3 tmp/pwm_probe.py --measure-only --seconds 30
```

核对 PF1 是否对应实物 SCL：

```bash
python3 tmp/pwm_probe.py --pin-test
```

此模式保持 SDA 释放，先将 PF1 开漏拉低，按回车后释放 PF1，再按回车退出恢复。
分别记录 C 板 SCL 对 C 板 GND 的电压，正常应从接近0V变为接近VCC。
它不发送 I2C 地址、不修改 TCA 通道；因此 PWM 板侧只有在通道已选通时才应同步。
如需自动定时，每阶段30秒：`python3 tmp/pwm_probe.py --pin-test --no-pause --seconds 30`。

无需万用表等待，自动执行并扫描7位地址：

```bash
python3 tmp/pwm_probe.py --no-pause --scan
```

PCA 默认7位地址为 `0x40`（固件 HAL 中的 `0x80` 是左移后的地址）。
如板上地址不同可传 `--address 0x41`，更换 TCA 端口可传 `--channel 5`。

日志自动写到 `tmp/pwm_probe_日期_时间.log`，请保留所有测量阶段记录。

**运行状态：** 测试期间 MCU 暂停，422遥测停止；PCA原有输出仍可能保持。
退出时尝试恢复原通道、GPIO、I2C及MCU原运行状态；若总线卡死，通道可能恢复失败，
脚本会明确报错。被中断的硬件 I2C 事务不能原样续传，可能由固件超时后重试。
正常退出和 Ctrl+C 会执行清理；拔掉DAP、关闭进程或连接中断时不能保证自动恢复。
不要同时运行旧诊断脚本或其他调试器。

**结果范围：** 软件I2C成功只能证明低速寄存器通信成功，不能证明固件100kHz波形、
动力供电和实际PWM引脚波形正确。脚本未在硬件上运行，本次仅进行了离线验证。

## 旧版：422 + 固件变量诊断（2026-10-16）

`rov_i2c_diagnose.py` 是本次实际使用的诊断脚本，依赖 Python 3、pyserial、OpenOCD。

## 使用

先在一个终端启动 OpenOCD（不复位、不烧录）：

```bash
openocd -f interface/cmsis-dap.cfg -c 'transport select swd' -c 'adapter speed 1000' -f target/stm32f4x.cfg -c init
```

另一个终端在项目根目录执行：

```bash
python3 tmp/rov_i2c_diagnose.py --seconds 45 --mode PS
python3 tmp/rov_i2c_diagnose.py --seconds 30 --mode PCA
python3 tmp/rov_i2c_diagnose.py --seconds 30 --mode PS
```

串口固定为 `/dev/ttyUSB0`、115200；OpenOCD TCL 固定为 `127.0.0.1:6666`。
每 15 秒输出错误计数、GPIO、HAL 状态和原始压强；结束时输出帧数及数据范围。
脚本结束后 MCU 保持运行，遥测模式保持最后选择的模式。脚本不发送运动命令。
不要同时运行其他占用该串口的程序。

## 外部拉低确认

```bash
python3 tmp/rov_i2c_diagnose.py --isolate
```

此选项会暂停 MCU，关闭 I2C，将 PF0/PF1 临时设为无上下拉输入，采样 20 次，
然后恢复寄存器并继续运行。只在机器静止、允许短暂停机时使用。
输出值按位解释：bit0=SDA，bit1=SCL；3=两根均高，1=SDA 高/SCL 低。
正常运行中偶尔读到低电平可能只是通信波形，不能直接判定卡死。

## 固件对应关系

脚本中的 SRAM 地址来自本次 `CubeMX/build/RM_Frame_C.elf`，本次已核对板上 Flash
与其一致。重新编译、烧录其他版本后，必须重新核对地址，否则变量解释可能错误。
GPIO 和 I2C 外设地址针对 STM32F407。

可选 Flash 核对功能仍使用原脚本的 `/tmp` 路径，使用前先恢复参考镜像：

```bash
cp tmp/rov_i2c_current.bin /tmp/rov_i2c_current.bin
python3 tmp/rov_i2c_diagnose.py --seconds 15 --mode PS --verify-image
```

参考镜像是本次 ELF 导出的 84152 字节二进制，仅用于比较，不自动烧录。

## 本次对照结果

仅观察空通道或悬空连接线（不访问PCA）：
`python3 tmp/pwm_probe.py --channel 5 --channel-only --no-pause --seconds 5`。

2026-10-17 21:30，保留杜邦线在5号口、PWM板端悬空：TCA地址应答、
全关闭回读0x00成功，选通0x20后5次每秒采样均SDA=1/SCL=1，正常恢复原状态。
日志为 `pwm_probe_20261017_213055.log`。与接PWM板时选通4/5均立即SCL低形成对照，
故障与PWM板接入相关；尚不能区分接触错位、PWM板供电/共地、板本身或接入后的负载。

### 2026-10-17 补充：引脚对应及独立通道切换

- PF1开漏拉低时，用户在C板SCL实测0.05V；释放时实测3.45V，MCU输入分别为0/1。
  本轮确认了软件PF1与实物SCL的对应关系。
- `pwm_probe_20261017_212801.log`：暂停主程序后，TCA 0x70应答，原通道读值0x08；
  写0x00关闭全部通道并回读成功，SDA/SCL均高。
- 随后写0x10选通4号口，STOP后SCL立即变低，尚未读取PCA寄存器；
  将MCU引脚切成无上下拉输入后仍为SDA高、SCL低。
- 恢复原TCA通道失败（SCL低，不能发送有效时钟）；脚本仍执行GPIO/I2C恢复及MCU resume。
- 下一步可断电后将同一PWM板及线缆从通道4/J5换到通道5/J6，
  使用 `python3 tmp/pwm_probe.py --channel 5 --no-pause --scan` 做对照。
  该测试不修改固件的默认PWM通道，因此不能用它验证原主程序的PWM控制功能。

### 2026-10-16 对照

- 拔掉 PWM 板：PS 45 秒 752 帧；空通道 PCA 回读 30 秒 501 帧；
  再切 PS 30 秒 501 帧，TCA 错误始终为 0。
- PWM 板缺席时 PCA 回读无应答，当前固件会增加错误/恢复计数，回读数值无效。
- 接回 PWM 板并重新上电：PS 30 秒仅 18 帧，TCA 错误从 246 增至 535；
  释放 MCU 引脚后 20 次采样均为 1，即外部 SCL 持续低。
- 优先排查 PWM 连接线和线序、PWM 板本地逻辑供电、PWM 板及 TCA 4 号分支。
- 当前 TIM1 为 50 Hz，遥测每 3 次循环发送一次，正常 PS 约 16.7 Hz。
