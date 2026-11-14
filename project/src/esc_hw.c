/**
  **************************************************************************
  * @file     esc_hw.c
  * @brief    low level driver of the fan ESC (AT32F421G8U7)
  *
  * PWM:  TMR1 channel 1/2/3 on PA8/PA9/PA10, high side pwm.
  *       The low side pins PA7/PB0/PB1 are plain gpio outputs, so high side
  *       and low side of one leg are never driven at the same time.
  * ADC:  preempt group (triggered by TMR1_CH4) samples the floating phase and
  *       the virtual neutral once per pwm period.  The ordinary group is used
  *       for the slow measurements (pot, battery, current).
  **************************************************************************
  */

#include "at32f421_wk_config.h"
#include "esc.h"
#include "esc_config.h"

/* ------------------------------------------------------------------------- */
/* pin mapping                                                               */
/* ------------------------------------------------------------------------- */
#define PHASE_A                     0u
#define PHASE_B                     1u
#define PHASE_C                     2u

typedef struct
{
  gpio_type            *port;
  uint16_t              pin;
  gpio_pins_source_type src;
} esc_pin_t;

/* phase A = TMR1_CH3, phase B = TMR1_CH2, phase C = TMR1_CH1 */
static const esc_pin_t phase_hin[3] =
{
  { GPIOA, GPIO_PINS_10, GPIO_PINS_SOURCE10 },
  { GPIOA, GPIO_PINS_9,  GPIO_PINS_SOURCE9  },
  { GPIOA, GPIO_PINS_8,  GPIO_PINS_SOURCE8  }
};

static const esc_pin_t phase_lin[3] =
{
  { GPIOB, GPIO_PINS_1, GPIO_PINS_SOURCE1 },
  { GPIOB, GPIO_PINS_0, GPIO_PINS_SOURCE0 },
  { GPIOA, GPIO_PINS_7, GPIO_PINS_SOURCE7 }
};

static const adc_channel_select_type phase_adc[3] =
{
  ADC_CHANNEL_1,   /* PA1, phase A */
  ADC_CHANNEL_4,   /* PA4, phase B */
  ADC_CHANNEL_5    /* PA5, phase C */
};

/* commutation table, same order as the AM32 firmware:
   1: A+ B- C float   2: C+ B- A float   3: C+ A- B float
   4: B+ A- C float   5: B+ C- A float   6: A+ C- B float   */
static const uint8_t step_high[6]  = { PHASE_A, PHASE_C, PHASE_C, PHASE_B, PHASE_B, PHASE_A };
static const uint8_t step_low[6]   = { PHASE_B, PHASE_B, PHASE_A, PHASE_A, PHASE_C, PHASE_C };
static const uint8_t step_float[6] = { PHASE_C, PHASE_A, PHASE_B, PHASE_C, PHASE_A, PHASE_B };

/* ------------------------------------------------------------------------- */
/* gpio helpers                                                              */
/* ------------------------------------------------------------------------- */
/* gate driver input level for "switch off" and "switch on" */
#if ESC_GATE_ACTIVE_LOW
#define GATE_LEVEL_OFF               1u
#define GATE_LEVEL_ON                0u
#else
#define GATE_LEVEL_OFF               0u
#define GATE_LEVEL_ON                1u
#endif

static void pin_config(const esc_pin_t *p, gpio_mode_type mode, uint8_t drive_high)
{
  gpio_init_type gpio_init_struct;

  gpio_default_para_init(&gpio_init_struct);
  gpio_init_struct.gpio_pins           = p->pin;
  gpio_init_struct.gpio_mode           = mode;
  gpio_init_struct.gpio_out_type       = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_pull           = GPIO_PULL_NONE;
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;

  if(mode == GPIO_MODE_OUTPUT)
  {
    if(drive_high != 0u)
    {
      gpio_bits_set(p->port, p->pin);
    }
    else
    {
      gpio_bits_reset(p->port, p->pin);
    }
  }

  gpio_init(p->port, &gpio_init_struct);
}

static void pin_af(const esc_pin_t *p)
{
  pin_config(p, GPIO_MODE_MUX, 0u);
}

/* gate driver input "off" (low side and high side switch open) */
static void pin_off(const esc_pin_t *p)
{
  pin_config(p, GPIO_MODE_OUTPUT, GATE_LEVEL_OFF);
}

/* gate driver input "on" */
static void pin_on(const esc_pin_t *p)
{
  pin_config(p, GPIO_MODE_OUTPUT, GATE_LEVEL_ON);
}

/* ------------------------------------------------------------------------- */
/* pwm                                                                       */
/* ------------------------------------------------------------------------- */
static void esc_hw_pwm_init(void)
{
  tmr_output_config_type oc;
  tmr_brkdt_config_type bd;
  uint8_t i;

  for(i = 0; i < 3; i++)
  {
    gpio_pin_mux_config(phase_hin[i].port, phase_hin[i].src, GPIO_MUX_2);
    pin_off(&phase_hin[i]);
    pin_off(&phase_lin[i]);
  }

  tmr_reset(TMR1);
  tmr_base_init(TMR1, ESC_PWM_PERIOD_TICKS - 1u, 0);
  tmr_cnt_dir_set(TMR1, TMR_COUNT_UP);
  tmr_clock_source_div_set(TMR1, TMR_CLOCK_DIV1);

  tmr_output_default_para_init(&oc);
  oc.oc_mode         = TMR_OUTPUT_CONTROL_PWM_MODE_A;
  oc.oc_output_state = TRUE;
  oc.occ_output_state = FALSE;
#if ESC_GATE_ACTIVE_LOW
  oc.oc_polarity     = TMR_OUTPUT_ACTIVE_LOW;
#else
  oc.oc_polarity     = TMR_OUTPUT_ACTIVE_HIGH;
#endif
  oc.occ_polarity    = TMR_OUTPUT_ACTIVE_HIGH;
  oc.oc_idle_state   = (GATE_LEVEL_OFF != 0u) ? TRUE : FALSE;
  oc.occ_idle_state  = (GATE_LEVEL_OFF != 0u) ? TRUE : FALSE;

  tmr_output_channel_config(TMR1, TMR_SELECT_CHANNEL_1, &oc);
  tmr_output_channel_config(TMR1, TMR_SELECT_CHANNEL_2, &oc);
  tmr_output_channel_config(TMR1, TMR_SELECT_CHANNEL_3, &oc);

  /* channel 4 is not routed to a pin, it only triggers the adc.
     pwm mode b gives the trigger event at the compare value (after the
     pwm edge), so the bemf is sampled inside the on time */
  oc.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_B;
  tmr_output_channel_config(TMR1, TMR_SELECT_CHANNEL_4, &oc);
  tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_4, ESC_ADC_TRIG_DELAY_TICKS);

  tmr_brkdt_default_para_init(&bd);
  bd.brk_enable         = FALSE;
  bd.auto_output_enable = FALSE;
  bd.deadtime           = 0;
  tmr_brkdt_config(TMR1, &bd);

  tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_1, 0);
  tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_2, 0);
  tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_3, 0);

  tmr_output_enable(TMR1, TRUE);
  tmr_counter_enable(TMR1, TRUE);
}

void esc_hw_apply_step(uint8_t step, uint16_t duty_ticks)
{
  uint8_t i;
  uint8_t hi;
  uint8_t lo;
  uint8_t fl;

  if(step < 1u || step > 6u)
  {
    return;
  }

  hi = step_high[step - 1u];
  lo = step_low[step - 1u];
  fl = step_float[step - 1u];

  /* high side compare value, the low side pins are gpio driven */
  tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_1, duty_ticks);
  tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_2, duty_ticks);
  tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_3, duty_ticks);

  /* both switches of the floating phase off */
  for(i = 0; i < 3; i++)
  {
    if(i != hi && i != lo)
    {
      pin_off(&phase_hin[i]);
      pin_off(&phase_lin[i]);
    }
  }

  /* high side pwm, low side off */
  pin_off(&phase_lin[hi]);
  pin_af(&phase_hin[hi]);

  /* high side off, low side on */
  pin_off(&phase_hin[lo]);
  pin_on(&phase_lin[lo]);

  /* float phase first, neutral second (adc preempt sequence) */
  adc_preempt_channel_set(ADC1, phase_adc[fl], 1, ADC_SAMPLETIME_7_5);
}

void esc_hw_update_duty(uint16_t duty_ticks)
{
  tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_1, duty_ticks);
  tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_2, duty_ticks);
  tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_3, duty_ticks);
}

void esc_hw_all_off(void)
{
  uint8_t i;

  tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_1, 0);
  tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_2, 0);
  tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_3, 0);

  for(i = 0; i < 3; i++)
  {
    pin_off(&phase_hin[i]);
    pin_off(&phase_lin[i]);
  }
}

/* ------------------------------------------------------------------------- */
/* adc                                                                       */
/* ------------------------------------------------------------------------- */
static uint16_t adc_read_once(adc_channel_select_type channel)
{
  uint8_t attempt;

  adc_ordinary_channel_set(ADC1, channel, 1, ADC_SAMPLETIME_71_5);

  for(attempt = 0; attempt < 3u; attempt++)
  {
    uint32_t guard = 5000u;

    adc_flag_clear(ADC1, ADC_CCE_FLAG);
    adc_ordinary_software_trigger_enable(ADC1, TRUE);

    while(adc_flag_get(ADC1, ADC_CCE_FLAG) == RESET && guard > 0u)
    {
      guard--;
    }

    if(adc_flag_get(ADC1, ADC_CCE_FLAG) == SET)
    {
      break;
    }
  }

  return adc_ordinary_conversion_data_get(ADC1);
}

uint16_t esc_hw_read_throttle(void)
{
  return adc_read_once(ADC_CHANNEL_0);
}

uint16_t esc_hw_read_vbus_raw(void)
{
  return adc_read_once(ADC_CHANNEL_3);
}

uint16_t esc_hw_read_current_raw(void)
{
  return adc_read_once(ADC_CHANNEL_6);
}

static void esc_hw_adc_init(void)
{
  adc_base_config_type adc_base;
  volatile uint32_t delay;

  crm_periph_clock_enable(CRM_ADC1_PERIPH_CLOCK, TRUE);
  crm_adc_clock_div_set(CRM_ADC_DIV_6);       /* 120 MHz / 6 = 20 MHz */

  adc_reset(ADC1);

  adc_base_default_para_init(&adc_base);
  adc_base.sequence_mode            = TRUE;
  adc_base.repeat_mode              = FALSE;
  adc_base.data_align               = ADC_RIGHT_ALIGNMENT;
  adc_base.ordinary_channel_length  = 1;
  adc_base_config(ADC1, &adc_base);

  /* preempt group: floating phase, virtual neutral and two unused slots.
     the sequence length has to be 4 so that rank 1 lands in pdt1/psn1 */
  adc_preempt_channel_length_set(ADC1, 4);
  adc_preempt_channel_set(ADC1, ADC_CHANNEL_1, 1, ADC_SAMPLETIME_7_5);
  adc_preempt_channel_set(ADC1, ADC_CHANNEL_2, 2, ADC_SAMPLETIME_7_5);
  adc_preempt_channel_set(ADC1, ADC_CHANNEL_2, 3, ADC_SAMPLETIME_1_5);
  adc_preempt_channel_set(ADC1, ADC_CHANNEL_2, 4, ADC_SAMPLETIME_1_5);

  adc_preempt_conversion_trigger_set(ADC1, ADC12_PREEMPT_TRIG_TMR1CH4, TRUE);
  adc_ordinary_conversion_trigger_set(ADC1, ADC12_ORDINARY_TRIG_SOFTWARE, TRUE);
  adc_interrupt_enable(ADC1, ADC_PCCE_INT, TRUE);
  nvic_irq_enable(ADC1_CMP_IRQn, 0, 0);

  adc_enable(ADC1, TRUE);

  for(delay = 0; delay < 2000u; delay++)
  {
  }

  adc_calibration_init(ADC1);
  while(adc_calibration_init_status_get(ADC1) == SET)
  {
  }
  adc_calibration_start(ADC1);
  while(adc_calibration_status_get(ADC1) == SET)
  {
  }
}

/* ------------------------------------------------------------------------- */
/* init                                                                      */
/* ------------------------------------------------------------------------- */
void esc_hw_init(void)
{
  gpio_init_type gpio_init_struct;

  crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_TMR1_PERIPH_CLOCK, TRUE);

  /* analog inputs: pa0 pot, pa1/pa4/pa5 phase sense, pa2 neutral,
     pa3 battery, pa6 current */
  gpio_default_para_init(&gpio_init_struct);
  gpio_init_struct.gpio_pins = GPIO_PINS_0 | GPIO_PINS_1 | GPIO_PINS_2 |
                               GPIO_PINS_3 | GPIO_PINS_4 | GPIO_PINS_5 |
                               GPIO_PINS_6;
  gpio_init_struct.gpio_mode           = GPIO_MODE_ANALOG;
  gpio_init_struct.gpio_out_type       = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_pull           = GPIO_PULL_NONE;
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
  gpio_init(GPIOA, &gpio_init_struct);

  esc_hw_adc_init();
  esc_hw_pwm_init();
}

/* ------------------------------------------------------------------------- */
/* 20 kHz interrupt                                                          */
/* ------------------------------------------------------------------------- */
void ADC1_CMP_IRQHandler(void)
{
  if(adc_interrupt_flag_get(ADC1, ADC_PCCE_FLAG) == SET)
  {
    adc_flag_clear(ADC1, ADC_PCCE_FLAG);

    /* rank 1: floating phase, rank 2: virtual neutral */
    esc_fast_tick(ADC1->pdt1_bit.pdt1, ADC1->pdt2_bit.pdt2);
  }
}
