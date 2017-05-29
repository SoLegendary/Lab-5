/*!
**  @file LEDs.c
**
**  @brief Contains functions for initialising and controlling LED pins on the Tower
**         Initialises the appropriate Pin Control Registers (PCRs) and allows for the turning on, off
**         and toggling of four different LEDs (Orange, Yellow, Green, Blue) on the Tower.
*/
/*!
**  @addtogroup main_module main module documentation
**
**  @author Thanit Tangson & Emile Fadel
**  @{
*/
/* MODULE LEDs */

#include "LEDs.h"
#include "MK70F12.h"



bool LEDs_Init(void)
{
  // Turning on PORTA System Clock Gate
  SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;
  
  // Enabling drive strength for LED pins
  PORTA_PCR11 |= PORT_PCR_DSE_MASK;
  PORTA_PCR28 |= PORT_PCR_DSE_MASK;
  PORTA_PCR29 |= PORT_PCR_DSE_MASK;
  PORTA_PCR10 |= PORT_PCR_DSE_MASK;
  
  // Writing 1 to all the MUXes
  PORTA_PCR11 |= PORT_PCR_MUX(0x1);
  PORTA_PCR28 |= PORT_PCR_MUX(0x1);
  PORTA_PCR29 |= PORT_PCR_MUX(0x1);
  PORTA_PCR10 |= PORT_PCR_MUX(0x1);
  
  return true;
}



void LEDs_On(const TLED color)
{
  // writing to the PortA Set Output Register
  GPIOA_PSOR = color;
}



void LEDs_Off(const TLED color)
{
  // writing to the PortA Clear Output Register
  GPIOA_PCOR = color;
}



void LEDs_Toggle(const TLED color)
{
  // writing to the PortA Toggle Output Register
  GPIOA_PTOR = color;
}



/* END LEDs */
/*!
** @}
*/
