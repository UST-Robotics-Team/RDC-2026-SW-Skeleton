#ifndef ELCP_MOTOR_H
#define ELCP_MOTOR_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include "tim.h"

/**
 * @brief FB pulses per output-shaft revolution, used for rpm = pps * 60 / ppr. 
 * @note NOTICE: This number is for power source of 12V
 */
#ifndef ELCP_FB_PULSES_PER_REV
#define ELCP_FB_PULSES_PER_REV 114.0f
#endif

/**
 * @brief Minimum FB LOW time (us) for a valid encoder pulse. FB idles HIGH; a LOW shorter
 *        than this is treated as noise and not counted (see Motor_FB_Filter()).
 */
#ifndef ELCP_FB_MIN_LOW_US
#define ELCP_FB_MIN_LOW_US 1000U  /* caps valid counting at ~1e6 / (2 * this) pps at 50 % duty */
#endif

/**
 * @brief Motor connector selector.
 */
typedef enum {
  MOTOR_J1 = 0, /**< PWM PB14 / DIR PC6 / FB PB15 */
  MOTOR_J3,     /**< PWM PC7 / DIR PC9 / FB PC8 */
  ELCP_MOTOR_COUNT
} ELCP_Motor_ID;

/**
 * @brief Read-only view of a motor's runtime state.
 */
typedef struct {
  volatile float current_speed_pct; /**< Commanded speed, -100.0 (full CCW) to +100.0 (full CW) */
  volatile int32_t pulse_count;     /**< Signed FB pulse count (keeps counting while coasting) */
  volatile float pps;               /**< Signed FB pulses per second (updated once per second) */
  volatile float rpm;               /**< Signed speed in RPM (updated once per second) */
  volatile uint32_t noise_count;    /**< FB pulses rejected as noise (LOW < ELCP_FB_MIN_LOW_US) */
  volatile uint32_t last_low_us;    /**< LOW width of the last complete FB pulse, us (valid or not) */
} ELCP_Motor_State_t;

/* ----- Core API ----- */

/**
  * @brief  Initializes a motor: clears its state, starts PWM output and arms
  *         the FB input-capture channel.
  * @param  id  Motor to initialize (MOTOR_J1 or MOTOR_J3)
  * @retval None
  */
void ELCP_Motor_Init(ELCP_Motor_ID id);

/**
  * @brief  Sets the motor speed and drives the DIR pin.
  * @param  id             Motor to drive (MOTOR_J1 or MOTOR_J3)
  * @param  speed_percent  -100.0 (full CCW) to +100.0 (full CW)
  * @retval None
  */
void ELCP_Motor_SetSpeed(ELCP_Motor_ID id, float speed_percent);

/**
  * @brief  Stops the motor (same as ELCP_Motor_SetSpeed(id, 0)).
  * @param  id  Motor to stop
  * @retval None
  */
void ELCP_Motor_Stop(ELCP_Motor_ID id);

/**
  * @brief  Gets the motor's live state (read-only).
  * @param  id  Motor to query
  * @retval Pointer to the state, or NULL if id is invalid
  */
const ELCP_Motor_State_t *ELCP_Motor_GetState(ELCP_Motor_ID id);

/**
  * @brief  Overrides the FB pulses-per-revolution used for the rpm estimate.
  * @param  id              Motor to configure
  * @param  pulses_per_rev  FB pulses per output-shaft revolution (> 0)
  * @retval None
  */
void ELCP_Motor_SetPulsesPerRev(ELCP_Motor_ID id, float pulses_per_rev);

/* ----- Background / ISR (input capture) -----
 *
 * FB is measured by the timer input-capture channels configured in CubeMX:
 *   J1: PB15 -> TIM12_CH2 (AF9, ICFilter = 10)
 *   J3: PC8  -> TIM3_CH3  (AF2, ICFilter = 10)
 * ELCP_Motor_Init() switches both to BOTH edges for the pulse-width filter: a pulse is
 * counted only if its LOW lasts >= ELCP_FB_MIN_LOW_US, timed with the DWT cycle counter.
 *
 * ELCP_Motor_Init() starts each capture channel. The TIM3 and TIM8_BRK_TIM12
 * interrupt handlers are the CubeMX-generated ones in stm32f4xx_it.c (with the
 * NVIC entries in tim.c) and simply call HAL_TIM_IRQHandler(); edges reach the
 * library through HAL_TIM_IC_CaptureCallback() (overridden in elcp_motor.c).
 * No application code is needed for counting.
 *
 * Edges keep being counted while the commanded speed is 0 (coast-down/inertia):
 * the sign then comes from the last non-zero commanded direction.
 *
 * Note: because J1's capture channel lives on TIM12, the same counter that
 * generates the 10 kHz PWM on TIM12_CH1 also serves the capture. Its counter
 * therefore wraps every 100 us, which is fine for counting edges but useless
 * for timing a 1-30 ms tach period. The library counts edges and derives speed
 * from a time window; do not add period measurement without moving the PWM to
 * another timer.
 */

/**
  * @brief  Publishes pps and rpm into the motor state (refreshes once per second).
  *         Call once per main-loop pass; rpm uses ELCP_FB_PULSES_PER_REV or the
  *         value set by ELCP_Motor_SetPulsesPerRev().
  * @param  id  Motor to update
  * @retval None
  */
void ELCP_Motor_UpdatePPS(ELCP_Motor_ID id);

#endif
