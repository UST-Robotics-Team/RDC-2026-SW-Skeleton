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
  volatile int32_t pulse_count;     /**< Signed FB pulse count */
  volatile float pps;               /**< FB pulses per second (updated once per second) */
  volatile float rpm;               /**< Signed speed in RPM (updated once per second) */
} ELCP_Motor_State_t;

/* ----- Core API ----- */

/**
  * @brief  Initializes a motor: clears its state and starts PWM output.
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

/* ----- Background / ISR (wired up in stm32f4xx_it.c) ----- */

/**
  * @brief  Handles one FB pulse; updates the motor's pulse count.
  * @param  id        Motor that raised the interrupt
  * @param  GPIO_Pin  Pin that triggered the EXTI (must match the motor's FB pin)
  * @retval None
  */
void ELCP_Motor_FB_IRQHandler(ELCP_Motor_ID id, uint16_t GPIO_Pin);

/**
  * @brief  Publishes pps and rpm into the motor state (refreshes once per second).
  *         Call once per main-loop pass; rpm uses ELCP_FB_PULSES_PER_REV or the
  *         value set by ELCP_Motor_SetPulsesPerRev().
  * @param  id  Motor to update
  * @retval None
  */
void ELCP_Motor_UpdatePPS(ELCP_Motor_ID id);

#endif
