# LL_Modified — GCC/Makefile build + DAP-Link flashing

Scratch copy used to validate the CubeMX **Makefile (GCC)** toolchain migration
of the lower-level controller (FW_F4 **V1.27.1** kept).

## Build

```sh
./build.sh              # patch generated code + build
./build.sh clean        # remove CubeMX/build
./build.sh -j8          # extra args go to make
GCC_PATH= ./build.sh    # force the toolchain from PATH (default: STM32CubeCLT)
```

Outputs: `CubeMX/build/RM_Frame_C.{elf,hex,bin,map}`.

Toolchain: `/opt/stm32cubeclt/GNU-tools-for-STM32/bin` (GCC 14.3.1) by default,
system `arm-none-eabi-gcc` as fallback.

## Flash (DAP-Link / CMSIS-DAP, OpenOCD)

```sh
./flash-openocd.sh              # build, then program + verify + reset
./flash-openocd.sh --no-build
./flash-openocd.sh --check      # connection test only
```

`OPENOCD_INTERFACE`, `OPENOCD_TARGET`, `OPENOCD_SPEED`, `IMAGE` override the
defaults (`interface/cmsis-dap.cfg`, `target/stm32f4x.cfg`, 1000 kHz, the ELF).

Hardware side: the C-board's **4-pin UART/SWD headers are labelled confusingly**
— see the manual note below.

## Why a wrapper instead of the plain CubeMX Makefile

The CubeMX-generated `CubeMX/Makefile` only knows `../Src`, `../Drivers`,
`../Middlewares`. It has **no** knowledge of:

* the `userCode/` tree (19 `.cpp` + `bsp_uart.c`) — which contains the real
  `main()` and all application code;
* C++ support (compiler, flags, pattern rule).

`tools/gen_makefile.py` writes `CubeMX/Makefile.gen` (= generated Makefile +
userCode injection) so the original stays pristine. `build.sh` also runs
`tools/patch_generated.py` for the two things CubeMX gets wrong for this
project. Both scripts are idempotent — just re-run `./build.sh` after any
CubeMX regeneration.

## Patches that would be lost on regeneration

| File                 | Issue                                                                                                                          | Patch                                                                                               |
| -------------------- | ------------------------------------------------------------------------------------------------------------------------------ | --------------------------------------------------------------------------------------------------- |
| `Src/main.c`         | CubeMX re-emits `int main(void)` (`.ioc` has `ProjectManager.NoMain=false`) → clashes with the real `main()` in `Usermain.cpp` | generated `main()` wrapped in `#if 0`                                                               |
| `Src/main.c`         | `USER CODE PTD` has the Keil/ARMCC semihosting retarget (`rt_sys.h`, `_sys_*`, `#error Unsupported compiler`)                  | wrapped in `#if defined(__ARMCC_VERSION)`; GCC uses the generated `Src/syscalls.c` + `Src/sysmem.c` |
| `Src/stm32f4xx_it.c` | `DMA2_Stream0_IRQHandler` (SPI1 RX) is re-enabled, but `IMU.cpp` implements it                                                 | handler disabled                                                                                    |

**Proper long-term fixes** (in the main tree / `.ioc`):

* set `ProjectManager.NoMain=true` (or move `main()` back into `main.c`);
* make the `__ARMCC_VERSION` guard permanent in the real `main.c`;
* move the custom SPI1-RX ISR into the `USER CODE` block of `stm32f4xx_it.c`
  (C-callable wrapper) so regeneration keeps it.

## Source fixes applied (portable, no behaviour change)

* `userCode/devices/Inc/Usermain.h`: `+ #include <cstring>`
  (`std::memcpy`/`std::memset` are not pulled in by `<string.h>` under libstdc++).
* Case-sensitive includes (only visible on Linux):
  `matrix.h`→`Matrix.h`, `Kalman_filter.h`→`Kalman_Filter.h`,
  `usermain.h`→`Usermain.h`.
* `Propeller.cpp`: `clamp(pwm, (int32_t)1000, (int32_t)2000)` — GCC deduces
  `int32_t` as `long int`, the bare literals are `int`.

## Result

```
text    data    bss
80836   1940    25104
0 errors, 14 warnings (unused vars, sign-compare, %d vs int32_t)
```

Remaining warnings are cosmetic; the `%d` / `int32_t` one is harmless on
32-bit but worth fixing (`%ld` or cast).

## 422 upper-computer link: USART6 -> USART1

The 422 protocol was on **USART6** and has been moved to **USART1**
(`RxBuffer`, `HAL_UARTEx_ReceiveToIdle_IT`, command parsing and all TX output
share the same port).

| File                                 | Change                                                                                                              |
| ------------------------------------ | ------------------------------------------------------------------------------------------------------------------- |
| `userCode/devices/Src/Usermain.cpp`  | `RxEventCallback`: `USART6`→`USART1`; re-arm `&huart1`; `send_float`/`send_int` `UARTBaseLite<6>`→`UARTBaseLite<1>` |
| `userCode/devices/Src/Servo.cpp`     | RX re-arm `&huart1`                                                                                                 |
| `userCode/devices/Src/Propeller.cpp` | RX re-arm `&huart1`                                                                                                 |
| `userCode/devices/Src/Sensor.cpp`    | pressure-sensor + sonar TX `&huart1`                                                                                |
| `RM_Frame_C.ioc` + `Src/usart.c`     | `USART1_IRQn` preemption priority `0`→`4` (the value USART6 used)                                                   |

USART6 stays initialised but unused (PG9/PG14 remain reserved).

### C-board port naming trap (from the official user manual)

> 外壳丝印 UART1 对应 STM32 的 **UART6**，外壳丝印 UART2 对应 STM32 的 **UART1**

So the 422 cable must go to the **4-pin connector whose case silkscreen says
`UART2`** — that is the STM32 USART1:

| Connector pin | Signal | MCU pin |
| ------------- | ------ | ------- |
| 1             | RXD    | **PB7** |
| 2             | TXD    | **PA9** |
| 3             | GND    | GND     |
| 4             | 5V     | —       |

(The 3-pin connector silkscreened `UART1` is the STM32 **USART6**, PG9/PG14.)

### Link status: WORKING (2026-10-15)

`./test-422.py` (resets the target first, then checks everything):

```
PASS  firmware running / UART configured    system_init_flag=1, CR1=0x203C
PASS  RXD (PB7) idle level sane             PB7 idles HIGH
PASS  ROV -> PC (transmit)                  injected bytes arrive byte-exact
PASS  PC -> ROV (receive)                   "ABCD" arrives and DR reads 0x41
```

Two independent faults had to be fixed to get here:

1. **Hardware: the PC -> ROV RS-422 pair was inverted.** The pair was
   connected, but with T/R+ and T/R- swapped, so the receiver output idled
   LOW. A UART needs an idle HIGH, so the RX pin toggled with the data but the
   peripheral could never frame a byte (`SR` showed `FE=1`, `DR` read 0x00/0xFF
   instead of 0x41). Fix: swap the two wires of that pair only - exactly what
   the README documents.
2. **Firmware: reception died after the first overrun.** On any UART error the
   HAL calls `UART_EndRxTransfer()`, which clears `RXNEIE`/`IDLEIE`, and it is
   never restarted - `CR1` went from `0x203C` to `0x200C` and stayed there, so
   the link was dead until reset. Fix: `HAL_UART_ErrorCallback()` in
   `Usermain.cpp` clears ORE/FE/NE/PE and re-arms
   `HAL_UARTEx_ReceiveToIdle_IT()`.

Note: this firmware does not answer `VA` with the pressure readings
(`flag_read_pres` is set in `PressureSensor::Receive()` but never consumed), so
the periodic pressure line is the only traffic the board sends on its own.

## Telemetry command `OUT:<what>` (added 2026-10-16)

The firmware used to stream **only** the four pressure values; there was no way
to observe the gyro/accel/mag over the link at all (`IVA:BEG`/
`is_output_angle` was dead code). Added a runtime selector:

| Command            | Output line                                                                 |
| ------------------ | --------------------------------------------------------------------------- |
| `OUT:PS` (default) | `PS,<p0..p3>,<ok0..ok3>` — pressure in cm H2O, then 1/0 per gauge           |
| `OUT:RPY`          | `RPY,<yaw>,<pitch>,<roll>` [deg]                                            |
| `OUT:GYRO`         | `GYRO,<gx>,<gy>,<gz>` [rad/s]                                               |
| `OUT:ACC`          | `ACC,<ax>,<ay>,<az>` [m/s²]                                                 |
| `OUT:MAG`          | `MAG,<mx>,<my>,<mz>,<rx>,<ry>,<rz>` [µT] calibrated, then raw               |
| `OUT:TEMP`         | `TEMP,<temp>` [°C]                                                          |
| `OUT:ALL`          | all of the above, at ~10 Hz (single modes run at the IMU DMA rate, ~100 Hz) |
| `OUT:OFF`          | stop streaming                                                              |
| `OUT`              | replies `OUT:<current>`                                                     |

Every `OUT:` command replies `OUT:<NAME>`, so the upper computer can confirm it
took effect. The default (`PS`) reproduces the original behaviour.

Files: `Usermain.h` (enum + `telemetry_parse`/`telemetry_send` declarations),
`Usermain.cpp` (implementation + call first in `HAL_UARTEx_RxEventCallback` —
before the devices, because `Servo_I2C::Receive()` `strtok()`s `RxBuffer`),
`IMU.cpp` (`DMA_IT_Handle()` calls `telemetry_send()` where the hardcoded
pressure output used to be).

### Bug fixes that came with it

* `Usermain.h`: `send_int()` was **declared** `int` but **defined** `int32_t`
  (`long int` on ARM) → the declaration created a second, never-defined
  overload. Nothing called it before, so it only showed up now.
* `Sensor.h/.cpp`: added `PressureSensor::is_ok(id)`. `flag_ok` alone is not a
  presence test — an all-zero PROM (gauge did not answer, I²C read returns
  zeros) also satisfies CRC4 (`0 == 0`). It now also requires `Cal_C[id][0] != 0`,
  which is exactly what `MS5837_30BA_PROM()` already used as its error check.
* `Sensor.cpp`: `Handle_all()` read `pressure[4]` **uninitialised** for a gauge
  that is not present, so `data_pressure[]` / `data_depth` got garbage. Now
  zero-initialised (check `is_ok()` to know it is not a real reading).

## `test-hardware.py` — interactive bench test

```sh
./test-hardware.py                 # auto-detects /dev/ttyUSB*, 115200
./test-hardware.py --port /dev/ttyUSB0
./test-hardware.py --no-safe       # do not send the stop command on exit
```

Screens: gyro/accel/attitude, magnetometer, water depth (4× MS5837 with
presence flags), thrusters (`TES:`), servos (`MOT:`), all-sensors overview and a
raw-command terminal. Sensor screens are live dashboards (single-key, no
Enter); actuator screens are small REPLs.

Safety: thrusters/servos start at their centre value, values outside
1000-2000 / 500-2500 are rejected, a move of more than ±100 from centre asks
for confirmation, and every exit path (including Ctrl-C) sends `OFF` +
`TES:1610×8` + `MOT:1500×4`.

Requires the telemetry firmware above — without it the sensor screens stay
empty (the program says so at startup). No DAP-Link needed at test time.

Wiring it assumes (matches `Making_Instructions/README.md`):
I²C expander ports 0-3 = 4 pressure gauges, port 4 = PWM board;
PWM board channels 0-7 = thrusters, 8-10(11) = servos.

## 2026-10-16 — I²C2 is dead, and it drags the whole firmware down

Symptom: the depth screen showed a "depth" of about -1003 cm, all four gauges
reported `无应答` (presence flag 0), and the min/max/waveform statistics never
filled in.

Read over SWD while the target ran:

| What                       | Value                     | Meaning                                            |
| -------------------------- | ------------------------- | -------------------------------------------------- |
| `hi2c2.ErrorCode`          | `0x20`                    | `HAL_I2C_ERROR_TIMEOUT` — every transfer times out |
| `hi2c2.State`              | `0x20`                    | `HAL_I2C_STATE_BUSY_TX` (never came back)          |
| `I2C2 SR1 / SR2`           | `0x0001` / `0x0000`       | `SB` set, bus idle                                 |
| `PressureSensor::Handle()` | **~0.7 Hz**               | should be 300 Hz                                   |
| `tick_300hz`               | **1 in 100 % of samples** | the main loop is never keeping up                  |
| `PS` telemetry             | **0.2 - 9 Hz**            | should be ~100 Hz                                  |

So the loop is not 300 Hz at all: the PCA9685 writes use a **10 s** HAL timeout
(`PCA_Read/PCA_Write`) and `TCA_SetChannel` uses `0xffff`, so a dead I²C bus
makes one `Propeller_I2C::Handle()`/`Servo_I2C::Handle()` iteration take seconds.

Root cause, measured by halting the core, disabling I2C2 (`CR1=0`) and putting
PF0/PF1 back to plain inputs:

| Config                             | PF0 = I2C2_SDA | PF1 = I2C2_SCL |
| ---------------------------------- | -------------- | -------------- |
| firmware running                   | HIGH           | **LOW**        |
| I²C2 off, internal pull-up (40 kΩ) | HIGH           | **LOW**        |
| I²C2 off, no pull-up (floating)    | HIGH           | **LOW**        |

The MCU drives neither pin, SDA comes up on the same 40 kΩ pull-up, but SCL
stays low — so **something external holds SCL down far harder than 40 kΩ**
(near-short, or a slave clock-stretching forever). Note the asymmetry: this is
not a supply problem (then both would clamp).

Check next, in this order:

1. The 4-wire cable on the C-board IIC header — README says pins 2/4/6/8 =
   GND/VCC/SCL/SDA; a 4-pin shell shifted by one position lands SCL on GND and
   gives exactly this symptom.
2. SCL↔GND resistance with the power off (0 Ω = short).
3. Is the I²C expander powered (VCC) and are the TCA9548A A0-A2 switches tied to
   GND (address 0x70)?
4. Unplug the expander, then the gauges on ports 0-3 one at a time, then the PWM
   board on port 4, to find which device holds SCL.

The screen now says all of this itself: `test-hardware.py` warns when the
telemetry rate is far below ~100 Hz, marks `深度` as `<- 无效` when no gauge
answers, shows `--` for statistics that have not collected at least two
samples, and no longer prints the min/max columns in faint text.

# 
