#include "elcp_motor/elcp_motor.h"

/**
 * @brief Internal motor struct: runtime state + hardware mapping.
 */
typedef struct {
  ELCP_Motor_State_t state; /**< State exposed via ELCP_Motor_GetState() */

  /* PWM / DIR hardware, fixed at init */
  TIM_HandleTypeDef *htim;
  uint32_t tim_channel;
  GPIO_TypeDef *dir_port;
  uint16_t dir_pin;
  bool invert_dir;  /**< Swaps CW/CCW polarity */
  bool invert_pwm;  /**< Inverts PWM duty (for active-low inputs) */

  /* FB input capture hardware, fixed at init (see MX_TIM3_Init/MX_TIM12_Init) */
  TIM_HandleTypeDef *fb_htim;
  uint32_t fb_channel;          /**< TIM_CHANNEL_x used for the capture */
  uint32_t fb_active_channel;   /**< HAL_TIM_ACTIVE_CHANNEL_x reported in the callback */
  uint32_t fb_overcapture_flag; /**< TIM_FLAG_CCxOF for that channel */
  IRQn_Type fb_irqn;            /**< Timer interrupt that services the capture */

  float pulses_per_rev; /**< FB pulses per output-shaft revolution (rpm scaling) */

  int8_t last_dir; /**< Last non-zero commanded direction: +1 CW, -1 CCW, 0 none */

  bool initialized; /**< True once ELCP_Motor_Init() has run */

  /* PPS / RPM sampler state (1 Hz) */
  uint32_t last_pps_tick;
  int32_t last_pps_count;
} ELCP_Motor_t;

/* Hardware mapping: one motor per connector. Do not edit. */
static ELCP_Motor_t motor_J1 = {
    .htim = &htim12,
    .tim_channel = TIM_CHANNEL_1,    // PWM PB14 (blue)
    .dir_port = ELCP_J1_DIR_GPIO_Port,
    .dir_pin = ELCP_J1_DIR_Pin,      // DIR PC6 (white)
    .fb_htim = &htim12,
    .fb_channel = TIM_CHANNEL_2,     // FB  PB15 (yellow) -> TIM12_CH2, AF9
    .fb_active_channel = HAL_TIM_ACTIVE_CHANNEL_2,
    .fb_overcapture_flag = TIM_FLAG_CC2OF,
    .fb_irqn = TIM8_BRK_TIM12_IRQn,  // TIM12 shares TIM8_BRK's vector on F4
    .invert_dir = false,
    .invert_pwm = false,
    .pulses_per_rev = ELCP_FB_PULSES_PER_REV
};

static ELCP_Motor_t motor_J3 = {
    .htim = &htim8,
    .tim_channel = TIM_CHANNEL_2,    // PWM PC7 (blue)
    .dir_port = ELCP_J3_DIR_GPIO_Port,
    .dir_pin = ELCP_J3_DIR_Pin,      // DIR PC9 (white)
    .fb_htim = &htim3,
    .fb_channel = TIM_CHANNEL_3,     // FB  PC8 (yellow) -> TIM3_CH3, AF2
    .fb_active_channel = HAL_TIM_ACTIVE_CHANNEL_3,
    .fb_overcapture_flag = TIM_FLAG_CC3OF,
    .fb_irqn = TIM3_IRQn,
    .invert_dir = false,
    .invert_pwm = false,
    .pulses_per_rev = ELCP_FB_PULSES_PER_REV
};

/* Order matches the ELCP_Motor_ID enum (index 0 = MOTOR_J1). */
static ELCP_Motor_t *const motors[ELCP_MOTOR_COUNT] = {&motor_J1, &motor_J3};

/**
  * @brief  Initializes a motor: clears its state, starts PWM output and arms
  *         the FB input-capture channel.
  * @param  id  Motor to initialize (MOTOR_J1 or MOTOR_J3)
  * @retval None
  */
void ELCP_Motor_Init(ELCP_Motor_ID id) {
  if (id >= ELCP_MOTOR_COUNT)
    return;
  ELCP_Motor_t *motor = motors[id];
  motor->state.current_speed_pct = 0.0f;
  motor->state.pulse_count = 0;
  motor->state.pps = 0.0f;
  motor->state.rpm = 0.0f;
  motor->last_dir = 0;
  motor->last_pps_tick = HAL_GetTick();
  motor->last_pps_count = 0;

  HAL_TIM_PWM_Start(motor->htim, motor->tim_channel);
  ELCP_Motor_Stop(id);

  /* Accept FB edges from now on, then arm the capture channel. The counter is
   * already running after HAL_TIM_PWM_Start() (J1) or is started by
   * HAL_TIM_IC_Start_IT() itself (J3). */
  motor->initialized = true;
  /* CubeMX already enables these IRQs in HAL_TIM_*_MspInit(); they are set
   * here again (idempotent) so the capture still works if the NVIC entry is
   * ever missing from the generated code. */
  HAL_NVIC_SetPriority(motor->fb_irqn, 0, 0);
  HAL_NVIC_EnableIRQ(motor->fb_irqn);
  (void)HAL_TIM_IC_Start_IT(motor->fb_htim, motor->fb_channel);
}

/**
  * @brief  Sets the motor speed and drives the DIR pin.
  * @param  id             Motor to drive (MOTOR_J1 or MOTOR_J3)
  * @param  speed_percent  -100.0 (full CCW) to +100.0 (full CW)
  * @retval None
  */
void ELCP_Motor_SetSpeed(ELCP_Motor_ID id, float speed_percent) {
  if (id >= ELCP_MOTOR_COUNT)
    return;
  ELCP_Motor_t *motor = motors[id];

  // Clamp to [-100, 100]
  if (speed_percent > 100.0f)
    speed_percent = 100.0f;
  if (speed_percent < -100.0f)
    speed_percent = -100.0f;
  motor->state.current_speed_pct = speed_percent;

  // Remember the last commanded direction; it is used while coasting
  if (speed_percent > 0.0f)
    motor->last_dir = 1;
  else if (speed_percent < 0.0f)
    motor->last_dir = -1;

  // Direction from the sign of the speed: + = CW, - = CCW
  bool ccw = (speed_percent < 0.0f);
  if (motor->invert_dir)
    ccw = !ccw;

  HAL_GPIO_WritePin(motor->dir_port, motor->dir_pin,
                    ccw ? GPIO_PIN_SET : GPIO_PIN_RESET);

  // Duty cycle from the timer's autoreload value
  float abs_speed = (speed_percent >= 0.0f) ? speed_percent : -speed_percent;
  uint32_t arr = __HAL_TIM_GET_AUTORELOAD(motor->htim);
  uint32_t ccr = (uint32_t)((abs_speed / 100.0f) * (float)arr);

  if (motor->invert_pwm) {
    ccr = arr - ccr;
  }

  __HAL_TIM_SET_COMPARE(motor->htim, motor->tim_channel, ccr);
}

/**
  * @brief  Stops the motor (same as ELCP_Motor_SetSpeed(id, 0)).
  * @param  id  Motor to stop
  * @retval None
  */
void ELCP_Motor_Stop(ELCP_Motor_ID id) { ELCP_Motor_SetSpeed(id, 0.0f); }

/**
  * @brief  Gets the motor's live state (read-only).
  * @param  id  Motor to query
  * @retval Pointer to the state, or NULL if id is invalid
  */
const ELCP_Motor_State_t *ELCP_Motor_GetState(ELCP_Motor_ID id) {
  if (id >= ELCP_MOTOR_COUNT)
    return NULL;
  return &motors[id]->state;
}

/**
  * @brief  Overrides the FB pulses-per-revolution used for the rpm estimate.
  * @param  id              Motor to configure
  * @param  pulses_per_rev  FB pulses per output-shaft revolution (> 0)
  * @retval None
  */
void ELCP_Motor_SetPulsesPerRev(ELCP_Motor_ID id, float pulses_per_rev) {
  if (id >= ELCP_MOTOR_COUNT || pulses_per_rev <= 0.0f)
    return;
  motors[id]->pulses_per_rev = pulses_per_rev;
}

/**
  * @brief  Accounts for one FB edge captured by the timer hardware.
  * @param  motor  Motor whose capture channel fired
  * @retval None
  * @note   Edges are counted even when the commanded speed is 0 so that
  *         coast-down (inertia) keeps updating pulse_count; the direction then
  *         comes from the last commanded direction.
  */
static void Motor_FB_Edge(ELCP_Motor_t *motor) {
  // Single-channel FB: direction comes from the command, never from the signal
  int8_t dir;
  if (motor->state.current_speed_pct > 0.0f) {
    dir = 1;
  } else if (motor->state.current_speed_pct < 0.0f) {
    dir = -1;
  } else {
    dir = motor->last_dir;  // coasting: keep the last commanded direction
  }

  if (dir == 0) {
    return;  // no direction has ever been commanded
  }

  motor->state.pulse_count += dir;
}

/**
  * @brief  Publishes pps and rpm into the motor state (refreshes once per second).
  * @param  id  Motor to update
  * @retval None
  */
void ELCP_Motor_UpdatePPS(ELCP_Motor_ID id) {
  if (id >= ELCP_MOTOR_COUNT)
    return;
  ELCP_Motor_t *motor = motors[id];
  uint32_t now = HAL_GetTick();
  uint32_t elapsed = now - motor->last_pps_tick;
  if (elapsed < 1000U)
    return;

  // Snapshot once so an edge arriving mid-update is not dropped
  int32_t count = motor->state.pulse_count;
  int32_t delta = count - motor->last_pps_count;
  motor->last_pps_tick = now;
  motor->last_pps_count = count;

  float seconds = (float)elapsed / 1000.0f;
  motor->state.pps = (float)delta / seconds;                 // signed, pulses/s
  motor->state.rpm = (float)delta * 60.0f / (motor->pulses_per_rev * seconds);
}

/**
  * @brief  Routes input-capture events from the timer ISRs to the matching motor.
  * @param  htim  Timer handle that raised the capture event
  * @retval None
  * @note   Only the FB channel is used; other channels of the same timer are
  *         ignored so the shared TIM12 counter can also drive J1's PWM.
  */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
  for (uint32_t i = 0U; i < (uint32_t)ELCP_MOTOR_COUNT; i++) {
    ELCP_Motor_t *motor = motors[i];
    if (!motor->initialized || motor->fb_htim != htim)
      continue;
    if (htim->Channel != motor->fb_active_channel)
      continue;
    // Clear a stale over-capture so the flag does not stay latched
    __HAL_TIM_CLEAR_FLAG(htim, motor->fb_overcapture_flag);
    Motor_FB_Edge(motor);
    return;
  }
}
