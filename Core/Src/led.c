#include "led.h"
#include "main.h"
#include "stm32h5xx_hal.h"
#include "usb_device_status.h"

/* WS2812 GRB, битбанг по DWT CYCCNT (на 250 МГц __NOP даёт «все единицы» → белый). */

#define LED_CAN_ACTIVITY_MS     2000u
#define LED_BLINK_HALF_MS       1000u
#define LED_RGB_STEP_MS         333u
#define LED_RGB_STEPS           6u

#define LED_BRIGHT_R            32u
#define LED_BRIGHT_G            32u
#define LED_BRIGHT_B            32u

/* Длительности по даташиту WS2812B (нс), фронты укорочены — запас против чтения «0» как «1». */
#define WS_T0H_NS               220U
#define WS_T0L_NS               1050U
#define WS_T1H_NS               700U
#define WS_T1L_NS               600U
#define WS_RESET_NS             120000U

/* Вычитаем примерную стоимость BSRR + ветвления из фазы HIGH (иначе T0H раздувается). */
#define WS_HIGH_PHASE_OVERHEAD  14U
#define WS_LOW_PHASE_OVERHEAD   6U

static uint32_t s_t0h;
static uint32_t s_t0l;
static uint32_t s_t1h;
static uint32_t s_t1l;
static uint32_t s_reset_cy;

static volatile uint32_t s_last_can_tx_tick;
static uint32_t s_phase_start_tick;
static uint8_t s_usb_blink_on;
static uint8_t s_rgb_step;
static uint8_t s_prev_usb_configured;
static volatile uint8_t s_usb_cdc_active;

static uint8_t s_out_r;
static uint8_t s_out_g;
static uint8_t s_out_b;
static uint8_t s_out_valid;
static uint8_t s_was_can_active;

static uint32_t ws2812_ns_to_cycles(uint32_t f_hz, uint32_t ns)
{
  uint64_t c = ((uint64_t)f_hz * (uint64_t)ns) / 1000000000ULL;
  if (c < 1ULL) {
    c = 1ULL;
  }
  if (c > 0xFFFFFFFFULL) {
    c = 0xFFFFFFFFULL;
  }
  return (uint32_t)c;
}

static void ws2812_dwt_enable(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  __DSB();
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  __DSB();
}

static inline void ws2812_delay_cycles(uint32_t cycles)
{
  uint32_t const t0 = DWT->CYCCNT;
  if (cycles < 2u) {
    cycles = 2u;
  }
  while ((DWT->CYCCNT - t0) < cycles) {
    __NOP();
  }
}

static void ws2812_recalc_timings(void)
{
  uint32_t f = HAL_RCC_GetSysClockFreq();
  if (f < 1000000u) {
    f = 250000000u;
  }

  s_t0h = ws2812_ns_to_cycles(f, WS_T0H_NS);
  s_t0l = ws2812_ns_to_cycles(f, WS_T0L_NS);
  s_t1h = ws2812_ns_to_cycles(f, WS_T1H_NS);
  s_t1l = ws2812_ns_to_cycles(f, WS_T1L_NS);
  s_reset_cy = ws2812_ns_to_cycles(f, WS_RESET_NS);

  if (s_t0h > WS_HIGH_PHASE_OVERHEAD) {
    s_t0h -= WS_HIGH_PHASE_OVERHEAD;
  }
  if (s_t1h > WS_HIGH_PHASE_OVERHEAD) {
    s_t1h -= WS_HIGH_PHASE_OVERHEAD;
  }
  if (s_t0l > WS_LOW_PHASE_OVERHEAD) {
    s_t0l -= WS_LOW_PHASE_OVERHEAD;
  }
  if (s_t1l > WS_LOW_PHASE_OVERHEAD) {
    s_t1l -= WS_LOW_PHASE_OVERHEAD;
  }
}

static void ws2812_send_grb(uint8_t g, uint8_t r, uint8_t b)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();

  for (uint32_t n = 0u; n < 3u; n++) {
    uint8_t byte = (n == 0u) ? g : ((n == 1u) ? r : b);
    for (int32_t i = 7; i >= 0; i--) {
      if ((byte >> (uint32_t)i) & 1u) {
        LED_GPIO_Port->BSRR = (uint32_t)LED_Pin;
        ws2812_delay_cycles(s_t1h);
        LED_GPIO_Port->BSRR = (uint32_t)LED_Pin << 16U;
        ws2812_delay_cycles(s_t1l);
      } else {
        LED_GPIO_Port->BSRR = (uint32_t)LED_Pin;
        ws2812_delay_cycles(s_t0h);
        LED_GPIO_Port->BSRR = (uint32_t)LED_Pin << 16U;
        ws2812_delay_cycles(s_t0l);
      }
    }
  }

  LED_GPIO_Port->BSRR = (uint32_t)LED_Pin << 16U;
  ws2812_delay_cycles(s_reset_cy);

  if (primask == 0u) {
    __enable_irq();
  }
}

static void output_if_changed(uint8_t r, uint8_t g, uint8_t b)
{
  if (s_out_valid && s_out_r == r && s_out_g == g && s_out_b == b) {
    return;
  }
  ws2812_send_grb(g, r, b);
  s_out_r = r;
  s_out_g = g;
  s_out_b = b;
  s_out_valid = 1u;
}

static uint8_t led_usb_configured_poll(void)
{
  if (s_usb_cdc_active != 0u) {
    return 1u;
  }
  return UsbDevice_IsConfigured();
}

void Led_Init(void)
{
	return;
	ws2812_dwt_enable();
	ws2812_recalc_timings();

  s_last_can_tx_tick = 0u;
  s_phase_start_tick = HAL_GetTick();
  s_usb_blink_on = 1u;
  s_rgb_step = 0u;
  s_out_valid = 0u;
  s_was_can_active = 0u;
  s_prev_usb_configured = 0u;
  s_usb_cdc_active = 0u;
  output_if_changed(0, 0u, 0u);
}

void Led_SetUsbConfigured(uint8_t configured)
{
  s_usb_cdc_active = (configured != 0u) ? 1u : 0u;
}

void Led_NotifyCanTx(void)
{
  s_last_can_tx_tick = HAL_GetTick();
  s_phase_start_tick = s_last_can_tx_tick;
  s_rgb_step = 0u;
}

__attribute__((optimize("O0")))
void Led_Process(void)
{
  return;
	uint32_t now = HAL_GetTick();
  uint8_t usb_cfg = led_usb_configured_poll();

  if (usb_cfg && !s_prev_usb_configured) {
    s_phase_start_tick = now;
    s_usb_blink_on = 0u;
    s_out_valid = 0u;
  }
  s_prev_usb_configured = usb_cfg;

  uint8_t can_active = (s_last_can_tx_tick != 0u) &&
                       ((now - s_last_can_tx_tick) < LED_CAN_ACTIVITY_MS);

  if (can_active && !s_was_can_active) {
    s_was_can_active = 1u;
    s_phase_start_tick = now;
    s_rgb_step = 0u;
  } else if (!can_active && s_was_can_active) {
    s_was_can_active = 0u;
    s_phase_start_tick = now;
    s_usb_blink_on = 1u;
  }

  if (can_active) {
    while ((now - s_phase_start_tick) >= LED_RGB_STEP_MS) {
      s_phase_start_tick += LED_RGB_STEP_MS;
      s_rgb_step = (uint8_t)((s_rgb_step + 1u) % LED_RGB_STEPS);
    }
    uint8_t r = 0u, g = 0u, b = 0u;
    switch (s_rgb_step) {
      case 0u:
        r = LED_BRIGHT_R;
        break;
      case 1u:
        g = LED_BRIGHT_G;
        break;
      case 2u:
        b = LED_BRIGHT_B;
        break;
      case 3u:
        r = LED_BRIGHT_R;
        g = LED_BRIGHT_G;
        break;
      case 4u:
        g = LED_BRIGHT_G;
        b = LED_BRIGHT_B;
        break;
      default:
        r = LED_BRIGHT_R;
        b = LED_BRIGHT_B;
        break;
    }
    output_if_changed(r, g, b);
    return;
  }

  if (usb_cfg) {
    if ((now - s_phase_start_tick) >= LED_BLINK_HALF_MS) {
      s_phase_start_tick += LED_BLINK_HALF_MS;
      s_usb_blink_on ^= 1u;
    }
    if (s_usb_blink_on) {
      output_if_changed(LED_BRIGHT_R, 0u, 0u);
    } else {
      output_if_changed(0u, 0u, 0u);
    }
    return;
  }

  output_if_changed(LED_BRIGHT_R, 0u, 0u);
}
