/*!
**  @file FTM.c
**
**  @brief Routines for setting up the FlexTimer module (FTM) on the TWR-K70F120M.
**         The FTM is primarily used for timing the LEDs, allowing them to be turned on and off
**         automatically. Eg. Having a blue LED turn on for 1s whenever a valid packet is received
*/
/*!
**  @addtogroup main_module main module documentation
**
**  @author Thanit Tangson & Emile Fadel
**  @{
*/
/* MODULE FTM */

#include "FTM.h"
#include "MK70F12.h"
#include "OS.h"

// Private global variable for the FTM thread semaphore for every channel
static ECB* FTMSemaphore[8];



bool FTM_Init(void)
{
  // Enabling clock gate for FTM0=
  SIM_SCGC6 |= SIM_SCGC6_FTM0_MASK;
	
  FTM0_CNTIN |= FTM_CNTIN_INIT(0); // Initial FTM value
  FTM0_MOD   |= FTM_MOD_MOD(0);    // Modulo value
  FTM0_CNT   |= FTM_CNT_COUNT(0);  // Counter value
  FTM0_SC    |= FTM_SC_CLKS(0x2);    // Clock source selection set to 'fixed freq. clock'
  
  // Setting up NVIC for FTM see K70 manual pg 97
  // Vector=78, IRQ=62
  // NVIC non-IPR=1 IPR=15
  // Clear any pending interrupts on FTM
  NVICICPR1 = (1 << 30); // 62mod32 = 30
  // Enable interrupts from the FTM
  NVICISER1 = (1 << 30);
	
  return true;
}



bool FTM_Set(const TFTMChannel* const aFTMChannel)
{
  // See table 43.67 on pg 1219 of the K70 manual for the IO options for FTM channels
 
  uint8_t channelNb = aFTMChannel->channelNb;
  
  // Channel Interrupt Enable
  FTM0_CnSC(channelNb) |= FTM_CnSC_CHIE_MASK;
  
  // Saving semaphore for this channel
  FTMSemaphore[channelNb]    = aFTMChannel->semaphore;
  
  // If channel function is for input capture
  if (aFTMChannel->timerFunction == TIMER_FUNCTION_INPUT_CAPTURE)
  {
    FTM0_CnSC(channelNb) &= ~FTM_CnSC_MSB_MASK; // MSnB:MSnA = 00
    FTM0_CnSC(channelNb) &= ~FTM_CnSC_MSA_MASK;
	  
    switch (aFTMChannel->ioType.inputDetection)
    {
      case TIMER_INPUT_OFF:
        FTM0_CnSC(channelNb) &= ~FTM_CnSC_ELSB_MASK; // ELSnB:ELSnA = 00
        FTM0_CnSC(channelNb) &= ~FTM_CnSC_ELSA_MASK;
        break;
      case TIMER_INPUT_RISING:
        FTM0_CnSC(channelNb) &= ~FTM_CnSC_ELSB_MASK; // ELSnB:ELSnA = 01
        FTM0_CnSC(channelNb) |=  FTM_CnSC_ELSA_MASK;
        break;
      case TIMER_INPUT_FALLING:
        FTM0_CnSC(channelNb) |=  FTM_CnSC_ELSB_MASK; // ELSnB:ELSnA = 10
        FTM0_CnSC(channelNb) &= ~FTM_CnSC_ELSA_MASK;
        break;
      case TIMER_INPUT_ANY:
        FTM0_CnSC(channelNb) |=  FTM_CnSC_ELSB_MASK; // ELSnB:ELSnA = 11
        FTM0_CnSC(channelNb) |=  FTM_CnSC_ELSA_MASK;
        break;
    }
  }
  
  // If channel function is for output compare
  else if (aFTMChannel->timerFunction == TIMER_FUNCTION_OUTPUT_COMPARE)
  {
    FTM0_CnSC(channelNb) &= ~FTM_CnSC_MSB_MASK; // MSnB:MSnA = 01
    FTM0_CnSC(channelNb) |=  FTM_CnSC_MSA_MASK;
	  
    switch (aFTMChannel->ioType.outputAction)
    {
      case TIMER_OUTPUT_DISCONNECT:
        FTM0_CnSC(channelNb) &= ~FTM_CnSC_ELSB_MASK; // ELSnB:ELSnA = 00
        FTM0_CnSC(channelNb) &= ~FTM_CnSC_ELSA_MASK;
        break;
      case TIMER_OUTPUT_TOGGLE:
        FTM0_CnSC(channelNb) &= ~FTM_CnSC_ELSB_MASK; // ELSnB:ELSnA = 01
        FTM0_CnSC(channelNb) |=  FTM_CnSC_ELSA_MASK;
        break;
      case TIMER_OUTPUT_LOW:
        FTM0_CnSC(channelNb) |=  FTM_CnSC_ELSB_MASK; // ELSnB:ELSnA = 10
        FTM0_CnSC(channelNb) &= ~FTM_CnSC_ELSA_MASK;
        break;
      case TIMER_OUTPUT_HIGH:
        FTM0_CnSC(channelNb) |=  FTM_CnSC_ELSB_MASK; // ELSnB:ELSnA = 11
        FTM0_CnSC(channelNb) |=  FTM_CnSC_ELSA_MASK;
        break;
    }
  }

  else // If channel function was undefined
    return false;
	
  return true;
}



bool FTM_StartTimer(const TFTMChannel* const aFTMChannel)
{
  // See chapter 6, pg 5 of lecture notes for how this timer works
  
  uint8_t channelNb = aFTMChannel->channelNb;

  // If channel was not set up for output compare (ie. MSnB:MSnA != 01)
  if ((FTM0_CnSC(channelNb) & FTM_CnSC_MSB_MASK) ||
     !(FTM0_CnSC(channelNb) & FTM_CnSC_MSA_MASK))
    return false;

  // 1. Read FTM_CNT value
  uint16_t counterValue = FTM0_CNT;
  // 2. Set output compare register to CNT + delay
  FTM0_CnV(channelNb) = counterValue + aFTMChannel->delayCount;
  // 3. Clear output compare flag
  FTM0_CnSC(channelNb) = FTM_CnSC_CHF_MASK;
  // 4. Output compare flag will now set and trigger an interrupt after the delay
  
  return true;
}



void __attribute__ ((interrupt)) FTM0_ISR(void)
{
  // Checks for the interrupt source from each channel - probably only need 1 channel for lab 3 anyway
  for (uint8_t channelNb = 0; channelNb < 8; channelNb++)
  {
    // Clear interrupt flag for each channel
    FTM0_CnSC(channelNb) &= ~FTM_CnSC_CHF_MASK;
  
    // If channel is set up for output compare (ie. MSnB:MSnA == 01)
    if (!(FTM0_CnSC(channelNb) & FTM_CnSC_MSB_MASK) &&
         (FTM0_CnSC(channelNb) & FTM_CnSC_MSA_MASK))
      OS_SemaphoreSignal(FTMSemaphore[channelNb]);
  }
}



/* END FTM */
/*!
** @}
*/
