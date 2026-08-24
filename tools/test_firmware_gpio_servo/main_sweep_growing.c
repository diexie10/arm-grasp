/* =========================================================================
 * GROWING-SWEEP SERVO TEST FIRMWARE (bit-bang, no timers, no HAL init)
 * =========================================================================
 * Behavior: pulse width oscillates around center 1500us as a triangle wave.
 *           Amplitude starts at +-100us and grows +-100us per completed
 *           cycle up to +-900us (600us..2400us full travel), then restarts.
 *           Motion is continuous and smooth - a working servo visibly
 *           swings wider and wider, then repeats.
 *           PC13 LED toggles once per amplitude step as heartbeat.
 *
 * Frame: 50 Hz (20 ms), width stepped 20 us per frame.
 * One full cycle at amplitude A takes (2*A/20)*20ms = 0.8s..7.2s.
 * Full ramp 100..900us takes ~29 s, then repeats from small amplitude.
 *
 * Restore: production firmware lives in Core/Src/main.c.bak - copy back
 *          and rebuild after the test.
 * ========================================================================= */
#include "main.h"

/* Linker dummies: other translation units reference these symbols. */
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

/* Rough busy-wait, ~0.5 us per loop at the default 8 MHz HSI reset clock */
static void delay_us(uint32_t us)
{
  volatile uint32_t n = us * 2U;
  while (n--)
  {
    /* busy wait */
  }
}

/* One 50 Hz frame with the given high-time in microseconds on PA8+PA0 */
static void servo_frame(uint32_t high_us)
{
  GPIOA->BSRR = GPIO_BSRR_BS8 | GPIO_BSRR_BS0;   /* both pins HIGH */
  delay_us(high_us);
  GPIOA->BSRR = GPIO_BSRR_BR8 | GPIO_BSRR_BR0;   /* both pins LOW  */
  delay_us(20000U - high_us);                    /* rest of frame  */
}

int main(void)
{
  uint32_t amp;
  int32_t w;

  /* Enable GPIOA and GPIOC peripheral clocks (register level, RM0008) */
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPCEN;

  /* PA8: output push-pull 2 MHz */
  GPIOA->CRH &= ~(GPIO_CRH_CNF8_Msk | GPIO_CRH_MODE8_Msk);
  GPIOA->CRH |= GPIO_CRH_MODE8_1;

  /* PA0: output push-pull 2 MHz */
  GPIOA->CRL &= ~(GPIO_CRL_CNF0_Msk | GPIO_CRL_MODE0_Msk);
  GPIOA->CRL |= GPIO_CRL_MODE0_1;

  /* PC13: onboard LED, output push-pull 2 MHz, active LOW */
  GPIOC->CRH &= ~(GPIO_CRH_CNF13_Msk | GPIO_CRH_MODE13_Msk);
  GPIOC->CRH |= GPIO_CRH_MODE13_1;

  for (;;)
  {
    for (amp = 100U; amp <= 900U; amp += 100U)
    {
      GPIOC->BSRR = GPIO_BSRR_BR13;              /* LED on for this cycle */

      /* ramp UP: (1500-amp) -> (1500+amp), 20us per 20ms frame */
      for (w = (int32_t)(1500U - amp); w <= (int32_t)(1500U + amp); w += 20)
      {
        servo_frame((uint32_t)w);
      }

      GPIOC->BSRR = GPIO_BSRR_BS13;              /* LED off */

      /* ramp DOWN: (1500+amp) -> (1500-amp) */
      for (w = (int32_t)(1500U + amp); w >= (int32_t)(1500U - amp); w -= 20)
      {
        servo_frame((uint32_t)w);
      }
    }
  }
}
