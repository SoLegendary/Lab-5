/*!
**  @file PIT.c
**
**  @brief Routines for controlling Periodic Interrupt Timer (PIT) on the TWR-K70F120M.
**         This contains the functions for operating the periodic interrupt timer (PIT).
*/
/*!
**  @addtogroup main_module main module documentation
**
**  @author Thanit Tangson & Emile Fadel
**  @{
*/
/* MODULE PIT */

#include "PIT.h"
#include "MK70F12.h"

// Private global variables for user callback function and its arguments
static void (*PITCallback)(void*) = 0;
static void *PITCallbackArg       = 0;



bool PIT_Init(const uint32_t moduleClk, void (*userFunction)(void*), void* userArguments)
{
  PITCallback    = userFunction;
  PITCallbackArg = userArguments;

  // Enabling clock gate for PIT
  SIM_SCGC6 |= SIM_SCGC6_PIT_MASK;
	
  // Clearing the Module Disable bit to enable the PIT
  PIT_MCR &= ~PIT_MCR_MDIS_MASK;
  
  // Freezes the PIT while debugging
  PIT_MCR |= PIT_MCR_FRZ_MASK;
  
  // Note that there are 4 timers that can be used simultaneously (0-3), but only 1 is needed
  // write 1 to clear the TIF bit to avoid an unwanted interrupt during initialization
  PIT_TFLG0  |= PIT_TFLG_TIF_MASK;
  
  // Timer Interrupt Enable and Timer Enable
  PIT_TCTRL0 |= PIT_TCTRL_TIE_MASK;
  PIT_TCTRL0 |= PIT_TCTRL_TEN_MASK;
	
  // Sets Timer to a 500ms period (as specified in software requirements)
  // LDVAL = (period*moduleClk) - 1
  PIT_LDVAL0 = (moduleClk/2) - 1;
	
  // Setting up NVIC for PIT see K70 manual pg 97, 99
  // Vector=84, IRQ=68
  // NVIC non-IPR=2 IPR=17
  // Clear any pending interrupts on PIT
  NVICICPR2 = (1 << 4); // 68mod32 = 4
  // Enable interrupts from the PIT
  NVICISER2 = (1 << 4); 

  return true;
}



void PIT_Set(const uint32_t period, const bool restart)
{
  // *10^9 to convert to seconds from nanoseconds, then /50*10^6 as moduleClk should be 50Mhz
  // This works out to be division by 20
  uint32_t LDVal = (period/20) - 1;
	
  if (restart)
  {
    PIT_TCTRL0 &= ~PIT_TCTRL_TEN_MASK; // new period is enacted immediately, restarting the timer
    PIT_LDVAL0 = LDVal;
    PIT_TCTRL0 |= PIT_TCTRL_TEN_MASK;
  }
  else
    PIT_LDVAL0 = LDVal; // new period is enacted after the next interrupt
}



void PIT_Enable(const bool enable)
{
  if (enable)
    PIT_TCTRL0 |= PIT_TCTRL_TEN_MASK;
  else
    PIT_TCTRL0 &= ~PIT_TCTRL_TEN_MASK;
}



void __attribute__ ((interrupt)) PIT_ISR(void)
{
  // Clear the interrupt flag
  PIT_TFLG0 |= PIT_TFLG_TIF_MASK;

  // Check if the call back function pointer is not NULL, then call it
  if (PITCallback)
    (*PITCallback)(PITCallbackArg);
}



/* END PIT */
/*!
** @}
*/