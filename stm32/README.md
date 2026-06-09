# stm32

STM32G431KB firmware: the real-time layer that turns the Pi's commands into PWM,
reads the sensors, and enforces the safety watchdog.

Target board: NUCLEO-32 G431KB (MB1430) in the CoVAPSy Hat's MCU socket. The Hat
sits beside the Pi 5; the Pi-to-STM32 link is the Nucleo's onboard ST-LINK USB
cable, which carries both the runtime serial (VCP) and flashing.

## Pinout in use

| Function | Pin | Peripheral | Net |
|---|---|---|---|
| ESC PWM | `PA8` | `TIM1_CH1` | `PROPULSION_MCU` to `X_PROP` |
| Steering servo PWM | `PA11` | `TIM1_CH4` | `SERVO_DIR_MCU` to `X_DIR` |
| Rear IR (analog) | `PA1` | `ADC1_IN2` | `X_IR_G` |
| Wheel encoder | `PA0` | `TIM2_CH1` | encoder input |
| OLED + IMU (I2C) | `PB7 / PA15` | `I2C1 SDA / SCL` | OLED 0x3C, BNO055 0x28 |
| Buzzer | `PB6` | `TIM4_CH1` | `BUZZER_MCU` |
| VCP TX / RX (to Pi) | `PA2 / PA3` | `LPUART1` (AF12) | ST-LINK VCP |
| SWD | `PA13 / PA14` | reserved | ST-LINK |

Motor and steering PWM is 50 Hz (20 ms period: TIM1 prescaler 169, ARR 20000 at
170 MHz SYSCLK).

## Build and flash on the Pi

```bash
sudo apt install -y gcc-arm-none-eabi binutils-arm-none-eabi libnewlib-arm-none-eabi stlink-tools make
make             # builds build/firmware.{elf,bin,hex}, prints size
make flash       # st-flash --reset write build/firmware.bin 0x08000000
make clean
```

Flashing uses the same USB cable as the Python runtime, so no replug. The
Makefile does not track header dependencies, so run `make clean` after editing
any `.h`.

## Command protocol (ASCII, one command per line, LF-terminated)

| Command | Meaning |
|---|---|
| `S<deg>` | Steering in degrees, clamped to [-18, +18]. |
| `V<m/s>` | Speed in m/s, clamped to [-8, +2]. |
| `P<us>` | Raw ESC pulse width in microseconds (bypasses speed mapping). |
| `DIR<us>` | Raw steering pulse width in microseconds (bypasses the angle clamp). |
| `X` | Neutral (speed 0, steering 0). |
| `WD0` / `WD1` | Disable / enable the 250 ms watchdog. |
| `REV` | Brake, release, then reverse sequence (WP-1060 reverse needs this). |
| `CAL` | ESC throttle-range calibration. |
| `DPING` / `DSCAN` | Dynamixel ping / sweep (legacy diagnostics; steering is now a PWM servo). |

Unknown lines are dropped. `\r` is tolerated. Max line length 32 bytes.

Watchdog: if no valid command arrives within 250 ms, outputs go neutral, so the
Pi must send at >= 4 Hz to hold a non-zero state. Use `WD0` for raw-PWM bench
tests (e.g. `test_motor.py`).

ESC arming: the firmware holds the PWM line low for 2 s after reset, then snaps
to 1500 us neutral. That neutral edge is what Hobbywing-style ESCs arm on.

## Telemetry

The firmware sends periodic `HB` lines back to the Pi with the link state, rear
IR reading, and IMU heading. `stm32_link.py` parses them.

## Source layout (`Core/`)

```
Src/
  main.c          boot, MX_*_Init, command loop, watchdog
  moteurs.c       ESC and steering PWM (set_vitesse_m_s, set_direction_degres)
  cmd_parser.c    IT-driven RX, ring buffer, line parser
  usart.c         LPUART1 on PA2/PA3 (ST-LINK VCP)
  i2c.c sh1106.c  I2C1 bus and OLED dashboard
  bno055.c        IMU heading
  adc.c           rear IR
  encoder.c       wheel encoder
  buzzer.c        startup tone
  dynamixel.c     legacy Dynamixel diagnostics
  tim.c gpio.c stm32g4xx_hal_msp.c stm32g4xx_it.c system_stm32g4xx.c syscalls.c sysmem.c
Inc/
  matching headers
Drivers/  ST HAL + CMSIS (vendor, unmodified)
```

## If the Pi does not see /dev/ttyACM0 after flashing

Some Nucleo-32 G431KB variants route the VCP to USART2 (PA2/PA3, AF7) instead of
LPUART1 (AF12). Same pins, different peripheral and AF index. Swap in `usart.c`
and `stm32g4xx_it.c`:

```c
// usart.c
hlpuart1.Instance = USART2;                  // was LPUART1
__HAL_RCC_USART2_CLK_ENABLE();
PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
GPIO_InitStruct.Alternate = GPIO_AF7_USART2; // was GPIO_AF12_LPUART1
HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);     HAL_NVIC_EnableIRQ(USART2_IRQn);

// stm32g4xx_it.c
void USART2_IRQHandler(void) { HAL_UART_IRQHandler(&huart1); }
```

Reference: ST UM2657 (NUCLEO-32 G431KB), section 4, ST-LINK V2-1 Virtual COM port.
