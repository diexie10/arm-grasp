/* =========================================================================
 * MINIMAL BIT-BANG SERVO TEST FIRMWARE (archived copy)
 * =========================================================================
 * Purpose: drive servo pulses with NOTHING but GPIO register writes.
 *          No timers, no HAL init, no configuration layers - if a servo
 *          does not respond to THIS, the servo/power/wiring is at fault,
 *          not any peripheral setup code.
 *
 * Behavior: PA8 and PA0 both output 50 Hz servo frames.
 *           Pulse width alternates 2000us <-> 1000us every ~2 seconds,
 *           producing a large visible sweep on any working hobby servo.
 *           PC13 LED toggles each phase as a heartbeat.
 *
 * Expected: connected servo sweeps back and forth continuously.
 *           DMM (DC 200mV, yellow-vs-brown at servo plug) shows
 *           ~330mV <-> ~165mV alternating every 2 seconds.
 *
 * Restore:  production firmware lives in Core/Src/main.c.bak - copy back
 *           and rebuild after the test. See README.md in this folder.
 * ========================================================================= */
#include "main.h"

/* Linker dummies: other translation units reference these symbols. */
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

/* Rough busy-wait. At the default 8 MHz HSI reset clock one volatile
 * decrement loop costs ~0.5 us, so iterations = 2 per microsecond.
 * Exact timing is NOT critical: hobby servos accept 40..60 Hz frames
 * and +-10% pulse width variation. */
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
  uint32_t f;

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
    /* Phase A: 2000 us pulses for ~2 s (100 frames) */
    GPIOC->BSRR = GPIO_BSRR_BR13;
    for (f = 0U; f < 100U; f++)
    {
      servo_frame(2000U);
    }

    /* Phase B: 1000 us pulses for ~2 s */
    GPIOC->BSRR = GPIO_BSRR_BS13;
    for (f = 0U; f < 100U; f++)
    {
      servo_frame(1000U);
    }
  }
}
