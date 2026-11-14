/**
  **************************************************************************
  * @file     esc_config.h
  * @brief    user tunable parameters of the fan ESC
  *
  * Hardware (see io.txt):
  *   PA1  phase A voltage sense   ADC_IN1
  *   PA4  phase B voltage sense   ADC_IN4
  *   PA5  phase C voltage sense   ADC_IN5
  *   PA2  virtual neutral         ADC_IN2
  *   PA0  speed pot               ADC_IN0
  *   PA3  battery voltage         ADC_IN3
  *   PA6  current sense           ADC_IN6
  *   PA10 / PB1   phase A high / low gate
  *   PA9  / PB0   phase B high / low gate
  *   PA8  / PA7   phase C high / low gate
  *
  * The speed is set by the pot on ADC_IN0, no ESC/DShot input is used.
  **************************************************************************
  */
#ifndef __ESC_CONFIG_H
#define __ESC_CONFIG_H

/* ------------------------------------------------------------------------- */
/* pwm / time base                                                           */
/* ------------------------------------------------------------------------- */
#define ESC_CORE_CLOCK_HZ            (120000000u)
#define ESC_PWM_FREQ_HZ              (20000u)   /* pwm frequency                     */
#define ESC_PWM_PERIOD_TICKS         (ESC_CORE_CLOCK_HZ / ESC_PWM_FREQ_HZ)

/* adc trigger point: delay from the pwm period start, the bemf is measured
   while the high side switch is on (about 1 us)                          */
#define ESC_ADC_TRIG_DELAY_TICKS     (120u)

/* ------------------------------------------------------------------------- */
/* power stage                                                               */
/* ------------------------------------------------------------------------- */
#define ESC_REVERSE_DIRECTION        (0)        /* set to 1 to reverse the rotation  */
#define ESC_ZC_INVERT                (0)        /* initial guess of the bemf slope, it is
                                                   learned automatically at start up */
#define ESC_GATE_ACTIVE_LOW          (0)        /* set to 1 for active low hin/lin drivers */

/* duty limits, in permille of the pwm period */
#define ESC_MAX_DUTY_PERMILLE        (700u)     /* 70 %, conservative max speed      */
#define ESC_MIN_RUN_DUTY_PERMILLE    (100u)     /* 10 %, must stay above ~10 % so the
                                                   bemf sample window fits inside the
                                                   pwm on time                       */
#define ESC_START_DUTY_PERMILLE      (130u)     /* open loop / alignment torque      */
#define ESC_ALIGN_DUTY_PERMILLE      (70u)      /* rotor alignment                   */

/* duty ramp, in pwm ticks per ms */
#define ESC_RAMP_UP_TICKS_PER_MS     (8u)
#define ESC_RAMP_DOWN_TICKS_PER_MS   (40u)

/* soft speed governor: above this electrical frequency the duty is reduced.
   mechanical rpm = 60 * f / pole_pairs, so 300 Hz is about 2570 rpm with
   7 pole pairs.  Set higher (or 0 to disable) if the fan needs more speed.  */
#define ESC_MAX_ELEC_FREQ_HZ         (300u)

/* ------------------------------------------------------------------------- */
/* speed pot on ADC_IN0 (10k pot + 1k)                                       */
/* ------------------------------------------------------------------------- */
#define ESC_THROTTLE_ADC_LOW         (300u)     /* below -> 0 %                      */
#define ESC_THROTTLE_ADC_HIGH        (3600u)    /* above -> 100 %                    */
#define ESC_THROTTLE_FILTER_DIV      (8)        /* iir filter factor, 1/8 per ms     */
#define ESC_THROTTLE_START_PM        (40u)      /* 4 %, motor starts above this      */
#define ESC_ARM_MAX_PM               (30u)      /* throttle must be low to arm       */
#define ESC_ARM_DELAY_MS             (500u)     /* low throttle time before arming   */

/* ------------------------------------------------------------------------- */
/* startup                                                                   */
/* ------------------------------------------------------------------------- */
#define ESC_ALIGN_MS                 (250u)     /* rotor alignment time              */
#define ESC_OL_START_INTERVAL_US     (10000u)   /* first open loop step time         */
#define ESC_OL_MIN_INTERVAL_US       (1600u)    /* fastest open loop step time       */
#define ESC_OL_ACCEL_PERCENT         (2u)       /* step time reduction per commutation */
#define ESC_OL_TIMEOUT_MS            (3000u)    /* give up when no bemf lock         */
#define ESC_ZC_LOCK_STEPS            (6u)       /* valid steps needed for closed loop */

/* ------------------------------------------------------------------------- */
/* bemf zero crossing detection (adc based)                                  */
/* ------------------------------------------------------------------------- */
#define ESC_ZC_THRESHOLD_COUNTS      (6u)       /* adc counts around neutral         */
#define ESC_ZC_CONFIRM_SAMPLES       (2u)       /* consecutive samples after crossing */
#define ESC_BLANK_MIN_US             (150u)     /* ignore bemf right after commutation */
#define ESC_BLANK_INTERVAL_DIV       (8u)       /* or step_time / 8                  */

/* ------------------------------------------------------------------------- */
/* fault handling                                                            */
/* ------------------------------------------------------------------------- */
#define ESC_DESYNC_TIMEOUT_NUM       (3u)       /* timeout = interval * 3 / 2        */
#define ESC_DESYNC_TIMEOUT_EXTRA     (4u)       /* extra ticks                       */
#define ESC_MAX_DESYNCS              (5u)       /* then stop until throttle is zero  */
#define ESC_RESTART_COAST_MS         (400u)     /* coast time before a restart       */
#define ESC_RUN_OK_MS                (2000u)    /* run time that clears the counters */

/* ------------------------------------------------------------------------- */
/* optional protections (0 = disabled)                                       */
/* ------------------------------------------------------------------------- */
/* current sense scaling: 0 mA/count disables the current reading             */
#define ESC_CURRENT_MA_PER_COUNT     (0u)
#define ESC_CURRENT_ADC_OFFSET       (0u)
#define ESC_CURRENT_LIMIT_MA         (0u)

/* battery voltage divider, value * 10 (11 => 11:1). 0 disables the reading  */
#define ESC_VBAT_DIVIDER_X10         (110u)
#define ESC_VBAT_MIN_MV              (0u)

#endif /* __ESC_CONFIG_H */
