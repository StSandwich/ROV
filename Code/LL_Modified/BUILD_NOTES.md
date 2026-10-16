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

| File | Issue | Patch |
|---|---|---|
| `Src/main.c` | CubeMX re-emits `int main(void)` (`.ioc` has `ProjectManager.NoMain=false`) → clashes with the real `main()` in `Usermain.cpp` | generated `main()` wrapped in `#if 0` |
| `Src/main.c` | `USER CODE PTD` has the Keil/ARMCC semihosting retarget (`rt_sys.h`, `_sys_*`, `#error Unsupported compiler`) | wrapped in `#if defined(__ARMCC_VERSION)`; GCC uses the generated `Src/syscalls.c` + `Src/sysmem.c` |
| `Src/stm32f4xx_it.c` | `DMA2_Stream0_IRQHandler` (SPI1 RX) is re-enabled, but `IMU.cpp` implements it | handler disabled |

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
79772   1940    25080
0 errors, 14 warnings (unused vars, sign-compare, %d vs int32_t)
```

Remaining warnings are cosmetic; the `%d` / `int32_t` one is harmless on
32-bit but worth fixing (`%ld` or cast).

## 422 upper-computer link: USART6 -> USART1

The 422 protocol was on **USART6** and has been moved to **USART1**
(`RxBuffer`, `HAL_UARTEx_ReceiveToIdle_IT`, command parsing and all TX output
share the same port).

| File | Change |
|---|---|
| `userCode/devices/Src/Usermain.cpp` | `RxEventCallback`: `USART6`→`USART1`; re-arm `&huart1`; `send_float`/`send_int` `UARTBaseLite<6>`→`UARTBaseLite<1>` |
| `userCode/devices/Src/Servo.cpp` | RX re-arm `&huart1` |
| `userCode/devices/Src/Propeller.cpp` | RX re-arm `&huart1` |
| `userCode/devices/Src/Sensor.cpp` | pressure-sensor + sonar TX `&huart1` |
| `RM_Frame_C.ioc` + `Src/usart.c` | `USART1_IRQn` preemption priority `0`→`4` (the value USART6 used) |

USART6 stays initialised but unused (PG9/PG14 remain reserved).

### C-board port naming trap (from the official user manual)

> 外壳丝印 UART1 对应 STM32 的 **UART6**，外壳丝印 UART2 对应 STM32 的 **UART1**

So the 422 cable must go to the **4-pin connector whose case silkscreen says
`UART2`** — that is the STM32 USART1:

| Connector pin | Signal | MCU pin |
|---|---|---|
| 1 | RXD | **PB7** |
| 2 | TXD | **PA9** |
| 3 | GND | GND |
| 4 | 5V | — |

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
the periodic `-1004.22,-1003.28,-1003.02,-1003.79\n` line is the only traffic
the board sends on its own.
