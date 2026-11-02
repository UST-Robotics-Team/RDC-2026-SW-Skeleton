#include "elcp_motor/elcp_motor.h"

/**
 * @brief Internal motor struct: runtime state + hardware mapping.
 */
typedef struct {
  ELCP_Motor_State_t state; /**< State exposed via ELCP_Motor_GetState() */

  /* Hardware config, fixed at init */
  TIM_HandleTypeDef *htim;
  uint32_t tim_channel;
  GPIO_TypeDef *dir_port;
  uint16_t dir_pin;
  uint16_t fb_pin;
  GPIO_TypeDef *fb_port;
  bool invert_dir;  /**< Swaps CW/CCW polarity */
  bool invert_pwm;  /**< Inverts PWM duty (for active-low inputs) */

  float pulses_per_rev; /**< FB pulses per output-shaft revolution (rpm scaling) */

  bool initialized; /**< True once ELCP_Motor_Init() has run */

  /* PPS / RPM sampler state */
  uint32_t last_pps_tick;
  int32_t last_pps_count;
} ELCP_Motor_t;

/* Hardware mapping: one motor per connector. Do not edit. */
static ELCP_Motor_t motor_J1 = {
    .htim = &htim12,
    .tim_channel = TIM_CHANNEL_1,    // PWM PB14 (blue)
    .dir_port = ELCP_J1_DIR_GPIO_Port,
    .dir_pin = ELCP_J1_DIR_Pin,      // DIR PC6 (white)
    .fb_pin = ELCP_J1_FB_Pin,        // FB  PB15 (yellow)
    .fb_port = ELCP_J1_FB_GPIO_Port,
    .invert_dir = false,
    .invert_pwm = false,
    .pulses_per_rev = ELCP_FB_PULSES_PER_REV
};

static ELCP_Motor_t motor_J3 = {
    .htim = &htim8,
    .tim_channel = TIM_CHANNEL_2,    // PWM PC7 (blue)
    .dir_port = ELCP_J3_DIR_GPIO_Port,
    .dir_pin = ELCP_J3_DIR_Pin,      // DIR PC9 (white)
    .fb_pin = ELCP_J3_FB_Pin,        // FB  PC8 (yellow)
    .fb_port = ELCP_J3_FB_GPIO_Port,
    .invert_dir = false,
    .invert_pwm = false,
    .pulses_per_rev = ELCP_FB_PULSES_PER_REV
};

/* Order matches the ELCP_Motor_ID enum (index 0 = MOTOR_J1). */
static ELCP_Motor_t *const motors[ELCP_MOTOR_COUNT] = {&motor_J1, &motor_J3};

/**
  * @brief  Initializes a motor: clears its state and starts PWM output.
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
  motor->last_pps_tick = HAL_GetTick();
  motor->last_pps_count = 0;
  HAL_TIM_PWM_Start(motor->htim, motor->tim_channel);
  ELCP_Motor_Stop(id);
  motor->initialized = true;
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
  * @brief  Handles one FB pulse; updates the motor's pulse count.
  * @param  motor     Motor that raised the interrupt
  * @param  GPIO_Pin  Pin that triggered the EXTI (must match the motor's FB pin)
  * @retval None
  */
static void Motor_FB_IRQHandler(ELCP_Motor_t *motor, uint16_t GPIO_Pin) {
  if (GPIO_Pin == motor->fb_pin) {
    // Only count clean high levels
    if (HAL_GPIO_ReadPin(motor->fb_port, motor->fb_pin) != GPIO_PIN_SET) {
      return;
    }

    // Ignore pulses while stopped
    if (motor->state.current_speed_pct == 0.0f) {
      return;
    }

    // Single-channel FB: direction comes from the commanded speed
    if (motor->state.current_speed_pct > 0.0f) {
      motor->state.pulse_count++;
    } else {
      motor->state.pulse_count--;
    }
  }
}

/**
  * @brief  Handles one FB pulse; updates the motor's pulse count.
  * @param  id        Motor that raised the interrupt
  * @param  GPIO_Pin  Pin that triggered the EXTI (must match the motor's FB pin)
  * @retval None
  */
void ELCP_Motor_FB_IRQHandler(ELCP_Motor_ID id, uint16_t GPIO_Pin) {
  if (id >= ELCP_MOTOR_COUNT)
    return;
  Motor_FB_IRQHandler(motors[id], GPIO_Pin);
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

  int32_t delta = motor->state.pulse_count - motor->last_pps_count;
  motor->last_pps_tick = now;
  motor->last_pps_count = motor->state.pulse_count;

  float seconds = (float)elapsed / 1000.0f;
  motor->state.pps = (float)delta / seconds;                 // magnitude
  motor->state.rpm = (float)delta * 60.0f / (motor->pulses_per_rev * seconds);
}

/**
  * @brief  Routes EXTI callbacks from stm32f4xx_it.c to the matching motor.
  * @param  GPIO_Pin  Pin that triggered the EXTI
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (motor_J1.initialized && GPIO_Pin == motor_J1.fb_pin)
    ELCP_Motor_FB_IRQHandler(MOTOR_J1, GPIO_Pin);
  else if (motor_J3.initialized && GPIO_Pin == motor_J3.fb_pin)
    ELCP_Motor_FB_IRQHandler(MOTOR_J3, GPIO_Pin);
}
