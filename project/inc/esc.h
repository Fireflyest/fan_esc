/**
  **************************************************************************
  * @file     esc.h
  * @brief    sensorless six step bldc driver for the fan ESC board
  *
  * The control algorithm is a trimmed down port of the AM32 firmware
  * (AlkaMotors/AM32-MultiRotor-ESC-firmware): open loop ramp start,
  * bemf zero crossing detection, 30 degree commutation delay, desync and
  * stall protection.  The AM32 sources in project/src/am32_Src are only a
  * reference and are not compiled.
  *
  * The bemf is sampled with the adc (the AT32F421 comparator can not select
  * PA4 as non inverting input, so the three phases are measured against the
  * virtual neutral on PA2 with the adc).
  **************************************************************************
  */
#ifndef __ESC_H
#define __ESC_H

#include <stdint.h>

typedef enum
{
  ESC_STATE_IDLE = 0,     /* outputs off, waiting for throttle            */
  ESC_STATE_ALIGN,        /* rotor alignment                              */
  ESC_STATE_OPENLOOP,     /* forced commutation until bemf is usable      */
  ESC_STATE_RUN,          /* closed loop six step                         */
  ESC_STATE_COAST,        /* short coast before a restart                 */
  ESC_STATE_FAULT         /* desync / stall, throttle must go to zero     */
} esc_state_t;

typedef struct
{
  uint16_t throttle_adc;      /* filtered speed pot value                     */
  uint16_t throttle_pm;       /* requested speed, 0..1000                     */
  uint16_t duty_pm;           /* applied duty, 0..1000                        */
  uint16_t vbus_mv;           /* battery voltage                              */
  uint16_t current_ma;        /* motor current                                */
  uint16_t erpm;              /* electrical rpm                               */
  uint16_t step_interval_us;  /* measured 60 degree time                      */
  uint8_t  state;             /* esc_state_t                                  */
  uint8_t  armed;
} esc_status_t;

/* initialise peripherals and start the 20 kHz control interrupt */
void esc_init(void);

/* speed loop / soft tasks, call from the main loop */
void esc_poll(void);

/* latest status snapshot (valid after the first esc_poll) */
const esc_status_t *esc_status_get(void);

/* ---------------------------------------------------------------------- */
/* hardware hooks implemented in esc_hw.c                                 */
/* ---------------------------------------------------------------------- */
void esc_hw_init(void);
void esc_hw_apply_step(uint8_t step, uint16_t duty_ticks);
void esc_hw_update_duty(uint16_t duty_ticks);
void esc_hw_all_off(void);
uint16_t esc_hw_read_throttle(void);
uint16_t esc_hw_read_vbus_raw(void);
uint16_t esc_hw_read_current_raw(void);

/* 20 kHz control tick, called from the adc interrupt in esc_hw.c */
void esc_fast_tick(uint16_t vfloat, uint16_t vneutral);

#endif /* __ESC_H */
