# Motor library (`elcp_motor.h`)

Driver for the two ELCP DC motor drivers on connectors **J1** and **J3**.
Each motor has three control/feedback lines:

- **PWM** — duty cycle sets speed (0–100 %)
- **DIR** — logic level sets direction
- **FB** — tachometer pulse output, counted automatically by a timer input-capture channel

The hardware mapping is fixed inside `elcp_motor.c` (J1: PWM PB14 / DIR PC6 /
FB PB15 → TIM12_CH2; J3: PWM PC7 / DIR PC9 / FB PC8 → TIM3_CH3) — applications
only use the API below.

**Speed sign convention:** positive `%` = **CW**, negative `%` = **CCW** (default)

> [!IMPORTANT]
> The connector names **J1** and **J3** follow the silkscreen of the
> Schematics given to hardware members in your group.
> If you can't identify them on your boards, ask your hardware team.

## API

| Function                              | Purpose                                                                     |
| ------------------------------------- | --------------------------------------------------------------------------- |
| `ELCP_Motor_Init(id)`                 | Reset state and start PWM output for one motor                              |
| `ELCP_Motor_SetSpeed(id, pct)`        | Command speed, `-100.0f` … `+100.0f`; `+` = CW, `-` = CCW (sign drives DIR) |
| `ELCP_Motor_Stop(id)`                 | Same as `SetSpeed(id, 0)`                                                   |
| `ELCP_Motor_GetState(id)`             | Read-only pointer to the live state struct                                  |
| `ELCP_Motor_SetPulsesPerRev(id, ppr)` | Override the RPM scaling factor at runtime                                  |
| `ELCP_Motor_UpdatePPS(id)`            | Call once per main-loop pass; refreshes `pps`/`rpm` once per second         |

`id` is `MOTOR_J1` or `MOTOR_J3` (`ELCP_Motor_ID` enum).

The state struct (`ELCP_Motor_State_t`) exposes:

```c
const ELCP_Motor_State_t *s = ELCP_Motor_GetState(MOTOR_J1);
// s->current_speed_pct  commanded speed, -100.0 (CCW) .. +100.0 (CW)
// s->pulse_count        signed FB pulse count since init
// s->pps                signed FB pulses per second (refreshed 1 Hz)
// s->rpm                signed speed in RPM (refreshed 1 Hz)
```

> [!CAUTION] 
> **SAFETY NOTICE: BE CAREFUL WHEN FLASHING WITH ELCP MOTORS CONNECTED!**
> Always disconnect motor driver power (or lift robot drive wheels off the ground) while flashing code. During ST-LINK programmer connection, target erase, and MCU reset, timer and GPIO outputs can float momentarily before firmware initializes and establishes zero PWM.

## Example

```c
#include "elcp_motor/elcp_motor.h"

ELCP_Motor_Init(MOTOR_J1);
ELCP_Motor_SetSpeed(MOTOR_J1, 50.0f);      // 50% CW
ELCP_Motor_SetSpeed(MOTOR_J1, -25.0f);     // 25% CCW
ELCP_Motor_Stop(MOTOR_J1);                 // coast at 0%

const ELCP_Motor_State_t *s = ELCP_Motor_GetState(MOTOR_J1);
```

In the main loop, keep the telemetry sampler running:

```c
while (1) {
  ELCP_Motor_UpdatePPS(MOTOR_J1);
  ELCP_Motor_UpdatePPS(MOTOR_J3);
  // ... application logic, display refresh ...
}
```

### Full interactive test example (from skeleton `main.c`)

This example initialises both motors to 50 % CW speed, reads buttons BTN1/BTN2 to adjust speed in 10 % increments, and prints live telemetry and RPM onto the TFT display:

```c
#include "main.h"
#include "lcd/lcd.h"
#include "elcp_motor/elcp_motor.h"

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_TIM8_Init();
  MX_TIM12_Init();

  tft_init(PIN_ON_TOP, BLACK, WHITE, CYAN, DARK_GREY);

  ELCP_Motor_Init(MOTOR_J1);
  ELCP_Motor_Init(MOTOR_J3);

  ELCP_Motor_SetSpeed(MOTOR_J1, 50.0f);
  ELCP_Motor_SetSpeed(MOTOR_J3, 50.0f);

  const ELCP_Motor_State_t *state_J1 = ELCP_Motor_GetState(MOTOR_J1);
  const ELCP_Motor_State_t *state_J3 = ELCP_Motor_GetState(MOTOR_J3);

  bool btn1_prev = false;
  bool btn2_prev = false;

  while (1)
  {
    // Refresh the pulses-per-second telemetry once per second
    ELCP_Motor_UpdatePPS(MOTOR_J1);
    ELCP_Motor_UpdatePPS(MOTOR_J3);

    bool btn1_curr = btn_read(BTN1);
    bool btn2_curr = btn_read(BTN2);

    // BTN1 pressed: -10% to both motors
    if (btn1_curr && !btn1_prev) {
      ELCP_Motor_SetSpeed(MOTOR_J1, state_J1->current_speed_pct - 10.0f);
      ELCP_Motor_SetSpeed(MOTOR_J3, state_J3->current_speed_pct - 10.0f);
    }

    // BTN2 pressed: +10% to both motors
    if (btn2_curr && !btn2_prev) {
      ELCP_Motor_SetSpeed(MOTOR_J1, state_J1->current_speed_pct + 10.0f);
      ELCP_Motor_SetSpeed(MOTOR_J3, state_J3->current_speed_pct + 10.0f);
    }

    btn1_prev = btn1_curr;
    btn2_prev = btn2_curr;

    // Display header and motor states
    tft_prints(4, 0, "RDC 2026");
    tft_prints(4, 1, "Skeleton");

    tft_prints(0, 2, "J1 Spd: %.1f%%  ", state_J1->current_speed_pct);
    tft_prints(0, 3, "J1 Pls: %-6d", (int)state_J1->pulse_count);
    tft_prints(0, 4, "J3 Spd: %.1f%%  ", state_J3->current_speed_pct);
    tft_prints(0, 5, "J3 Pls: %-6d", (int)state_J3->pulse_count);
    tft_prints(0, 7, "%-5d %-5d", (int)(state_J1->rpm), (int)(state_J1->pps));
    tft_prints(0, 8, "%-5d %-5d", (int)(state_J3->rpm), (int)(state_J3->pps));
    tft_prints(0, 9, "B1:-10%%  B2:+10%%");

    tft_update(50);
  }
}
```

## Telemetry details

- FB edges are captured by the timer hardware:
  - **J1**: PB15 → **TIM12_CH2** (AF9), rising edge, `ICFilter = 10`
  - **J3**: PC8 → **TIM3_CH3** (AF2), rising edge, `ICFilter = 10`
- `ELCP_Motor_Init(id)` arms the capture channel (`HAL_TIM_IC_Start_IT`), sets
  the timer NVIC priority and enables the interrupt. Edges are then dispatched
  through `HAL_TIM_IC_CaptureCallback()` (overridden in `elcp_motor.c`). The
  counting rules: rising edges only, and the sign is taken from the commanded
  direction (the tach is single-channel, so it has no direction information of
  its own). Edges are still counted after `ELCP_Motor_Stop()` — while the
  commanded speed is 0 the last non-zero direction is used, so coast-down and
  inertia keep advancing `pulse_count`. If the robot is pushed backwards while
  coasting, the count will still move in the last commanded direction.
- `TIM3_IRQHandler` and `TIM8_BRK_TIM12_IRQHandler` are the CubeMX-generated
  handlers in `stm32f4xx_it.c` (they exist because the two global interrupts are
  enabled in `MainBoard.ioc`; TIM12 shares its vector with TIM8's break event on
  F4). They just call `HAL_TIM_IRQHandler()`, and the NVIC setup lives in
  `tim.c`.
- `ELCP_Motor_UpdatePPS(id)` diffs the pulse counter once per second and
  publishes `pps` and `rpm`. Call it every main-loop pass (it is cheap and
  self-timing). `pps` and `rpm` are signed.
- RPM is computed as `pps * 60 / pulses_per_rev`. The default
  `ELCP_FB_PULSES_PER_REV` is **114.0** (measured for the 36GP-3650 motor at
  a 12 V supply). Override it per motor with
  `ELCP_Motor_SetPulsesPerRev(id, ppr)` if you use a different motor or
  supply voltage.

### Capture clock — and why speed is still measured over a window

Both FB timers hang off APB1: `PCLK1 = 42 MHz`, and because the APB1 prescaler
is not 1, the timer clock is `2 × PCLK1 = 84 MHz`.

| Pin | Capture channel | PSC | Counter clock | Counter wrap |
| --- | --- | --- | --- | --- |
| PB15 (J1 FB) | TIM12_CH2 | 7 | 10.5 MHz (95.2 ns/tick) | 1050 ticks = **100 µs** (same counter as J1's 10 kHz PWM) |
| PC8 (J3 FB) | TIM3_CH3 | 0 | 84 MHz (11.9 ns/tick) | 65536 ticks = **780 µs** |

Expected tach rate is `pps = 1.9 × rpm` for 114 ppr: about 114 pps at 60 rpm
(8.8 ms per pulse) and 760 pps at 400 rpm (1.3 ms per pulse). Every pulse
period is longer than either counter's wrap, so the captured CCR value alone
cannot measure a period; TIM12 is worse still because its counter is reset by
J1's PWM carrier, so tracking overflows there would mean a 10 kHz interrupt.
Counting edges and differentiating over a 1 s window sidesteps all of it, at
the cost of ~1 s of latency and ~0.5 rpm of quantization (`60 / 114` rpm per
count).

The input filter samples at `fDTS/32 = 2.625 MHz` and requires 8 consecutive
samples, so pulses narrower than ≈3.05 µs are rejected. Real tach pulses are
tens to hundreds of µs wide, so this only removes noise — that hardware
filtering is the practical gain over the old EXTI setup.

> [!IMPORTANT]
> PB15 and PC8 are AF inputs and need a pull-up if the ELCP tachometer output
> is open-collector (typical). This repo sets `GPIO_PuPd = GPIO_PULLUP` on both
> pins (in `MainBoard.ioc` and `tim.c`); if you regenerate from an older `.ioc`,
> check that this did not revert to `GPIO_NOPULL`, otherwise the line never
> rises and no pulses are counted.

> [!NOTE]
> The skeleton `main.c` never calls `ELCP_Motor_UpdatePPS()`, so `pps`/`rpm`
> stay 0 until your application adds the two calls in the main loop (see the
> example above). `pulse_count` still updates at interrupt rate.
