/*!
**  @file RTC.c
**
**  @brief Routines for controlling the Real Time Clock (RTC) on the TWR-K70F120M.
**         This contains the functions for operating the real time clock (RTC).
*/
/*!
**  @addtogroup main_module main module documentation
**
**  @author Thanit Tangson & Emile Fadel
**  @{
*/
/* MODULE RTC */

#include "RTC.h"
#include "MK70F12.h"
#include "OS.h"

// Private global variable for the RTC thread semaphore
static ECB* RTCSemaphore;



bool RTC_Init(ECB* semaphore)
{
  // saving semaphore into global variable
  RTCSemaphore = semaphore;

  // Enabling clock gate for RTC
  SIM_SCGC5 |= SIM_SCGC6_RTC_MASK;

  // Enabling 18pF capacitance load on crystal
  RTC_CR |= RTC_CR_SC16P_MASK;
  RTC_CR |= RTC_CR_SC2P_MASK;
	
  // Enables the 32.768 kHz oscillator. After setting this bit, wait the oscillator startup 
  // time before enabling the time counter to allow the 32.768 kHz clock time to stabilize.
  RTC_CR |= RTC_CR_OSCE_MASK;
	
  for (uint32_t i; i < 50000000; i++){;}
  
  // Time Seconds Interrupt Enable (allows interrupts every second using the time seconds register)
  RTC_IER |= RTC_IER_TSIE_MASK;
  
  // Locks the control register until reset (0 == locked, 1 == unlocked)
  RTC_LR &= ~RTC_LR_CRL_MASK;

  // For loop to wait for the clock to stabilize before enabling the Time Counter
  // We need to test and find out the oscillator startup time, and then get the for loop to emulate this time
  // for (uint8_t i = 0; i < 1000; i++){}

  // Time Counter Enabled
  RTC_SR |= RTC_SR_TCE_MASK;
	
  // Setting up NVIC for RTC see K70 manual pg 97, 99
  // Vector=83, IRQ=67
  // NVIC non-IPR=2 IPR=16
  // Clear any pending interrupts on RTC
  NVICICPR2 = (1 << 3); // 67mod32 = 3
  // Enable interrupts from the RTC
  NVICISER2 = (1 << 3);

  return true;
}



void RTC_Set(const uint8_t hours, const uint8_t minutes, const uint8_t seconds)
{
  // NOTE: If the clock overflowed recently (RTC_SR[TOF]) or is invalid due to error or reset (RTC_SR[TIF])
  // it will reset to 0 and stay there until the TSR is written to again while the clock is disabled

  // Time Seconds Register is read-only while the clock is enabled
  RTC_SR &= ~RTC_SR_TCE_MASK;
	
  // The RTC only has one time-keeping register which is only in seconds
  uint32_t secondsTotal = ((hours*3600) + (minutes*60) + (seconds));
  
  RTC_TSR = RTC_TSR_TSR(secondsTotal);
	
  RTC_SR |= RTC_SR_TCE_MASK;
}



void RTC_Get(uint8_t* const hours, uint8_t* const minutes, uint8_t* const seconds)
{
  // Time Seconds Register needs to be broken down back into hours, minutes and seconds
  uint32_t secondsTotal = (uint32_t)RTC_TSR;
	
  *hours = (secondsTotal/3600); // Should always round down
  secondsTotal -= (*hours*3600);
  
  *minutes = (secondsTotal/60);
  secondsTotal -= (*minutes*60);
  
  *seconds = secondsTotal; // Leftover after previous decrements should be seconds left
}



void __attribute__ ((interrupt)) RTC_ISR(void)
{
  OS_ISREnter();
	
  // Every second an interrupt should happen when the clock increments by 1 second
  // According to the TSIE register bit, there is no corresponding flag to clear

  // Allow RTCThread to run
  OS_SemaphoreSignal(RTCSemaphore);
  
  OS_ISRExit();
}



/* END RTC */
/*!
** @}
*/
