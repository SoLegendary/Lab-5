/*!
**  @file FIFO.c
**
**  @brief Contains functions for initializing and manipulating data in FIFO arrays
**         Initializes a FIFO by resetting values to 0 and also allows for the input and output
**         of data to the UART module using a 256 bit shifting array.
*/
/*!
**  @addtogroup main_module main module documentation
**
**  @author Thanit Tangson & Emile Fadel
**  @{
*/
/* MODULE FIFO */

#include "FIFO.h"
#include "Cpu.h"



/*!
 * Setting all parameters of the FIFO to 0 for initialization.
 */
void FIFO_Init(TFIFO * const FIFO)
{
  FIFO->Start   = 0;
  FIFO->End     = 0;
  FIFO->NbBytes = 0;
}



/*!
 * Checks if the Buffer has room and then puts data into the buffer, returning true.
 * If the buffer is full it loops back around to the start of the buffer and returns false.
 */
bool FIFO_Put(TFIFO * const FIFO, const uint8_t data)
{
  // If the FIFO has room, it places some data in to it and returns true.
  if (FIFO->NbBytes < FIFO_SIZE)
  {
    EnterCritical(); // Nesting-compatible Interrupt Disable

    FIFO->Buffer[FIFO->End] = data;
    FIFO->NbBytes++;
    FIFO->End++;

    if (FIFO->End >= FIFO_SIZE) // If the FIFO is full, it loops back around
      FIFO->End = 0;

    ExitCritical(); // Nesting-compatible Interrupt Enable
    return true;
  }

  return false; // Returns false if the FIFO is full
}



/*!
 * Gets data from the buffer.
 */
bool FIFO_Get(TFIFO * const FIFO, uint8_t * const dataPtr)
{
  // Checks to see if FIFO has something in it
  if (FIFO->NbBytes > 0)
  {
    EnterCritical();

    // Gets the oldest data from the FIFO
    *dataPtr = FIFO->Buffer[FIFO->Start];
    // Decrements the number of bytes
    FIFO->NbBytes--;
    // Increments the start to move to the next available position for oldest data
    FIFO->Start++;

    // If the FIFO is now full, resets the start position
    if (FIFO->Start >= FIFO_SIZE)
      FIFO->Start = 0;

    // Returns true if the get process was able to retrieve data
    ExitCritical();
    return true;
  }
  // Returns false if the FIFO was empty and no data was retrieved
  return false;
}



/* END FIFO */
/*!
** @}
*/
