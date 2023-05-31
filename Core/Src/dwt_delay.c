#include "dwt_delay.h"
// #include "core_cm33.h"
#include "stm32h5xx.h"

static uint32_t SystemCoreClockInMHz;

void dwt_delay_init(void)
{
    // uint32_t SystemCoreClock = 64000000U;
    SystemCoreClockUpdate(); // Update the system core clock
    SystemCoreClockInMHz = SystemCoreClock / 1000000;

    if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk))
    {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // Enable TRC
    }

    DWT->CYCCNT = 0;                     // Reset the counter
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; // Enable the cycle counter
}

void dwt_delay_us(uint32_t us)
{
    uint32_t target_tick = us * SystemCoreClockInMHz;
    uint32_t start_tick = DWT->CYCCNT;
    while ((DWT->CYCCNT - start_tick) < target_tick)
    {
        // Do nothing, just wait
    }
}
