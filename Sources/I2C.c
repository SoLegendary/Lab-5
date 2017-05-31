/*! @file I2C.c
 *
 *  @brief I/O routines for the K70 I2C interface.
 *
 *  Includes functions to initialise the I2C with appropriate user settings and
 *  perform a single-byte write and multi-byte read to a slave device using both
 *  polling and interrupt methods
 *
 *  @author Thanit Tangson
 *  @date 2017-5-9
 */
/*!
**  @addtogroup main_module main module documentation
**
**  @author Thanit Tangson
*/
#include "types.h"
#include "I2C.h"
#include "MK70F12.h"
#include "OS.h"

// Private global variable for the I2C thread semaphore
ECB* ReadCompleteSemaphore;


static uint8_t PrimarySlaveAddress = 0; // private global variable to track accelerometer slave address
static uint8_t SlaveAddressWrite   = 0; // write mode address has first bit set
static uint8_t SlaveAddressRead    = 0; // read mode address has first bit cleared

// private globals used in I2C_ISR which are copies of parameters sent to I2C_IntRead()
static uint8_t IsrNbBytes;
static uint8_t *IsrData;

// icr determines SCL divider (see K70 manual pg. 1885)
// use icr register value as the index to get the SCL divider value used in the baud rate formula
static const uint16_t SclDivider[64] = {
20,22,24,26,28,32,36,40,28,32,26,40,44,48,56,68, // these first 16 values are unused due to unreliability
48,56,64,72,80,88,104,128,80,96,112,128,144,160,192,204,
160,192,224,256,288,320,384,480,320,384,448,512,576,640,768,960,
640,768,896,1024,1152,1280,1536,1920,1280,1536,1792,2048,2304,2560,3072,3840};

	
bool I2C_Init(const TI2CModule* const aI2CModule, const uint32_t moduleClk)
{
  // System clock gate enable
  SIM_SCGC4 |= SIM_SCGC4_IIC0_MASK;
	
  // I2C enable
  I2C0_C1 |= I2C_C1_IICEN_MASK;

  // AK signal - SCL is held low until this is written (ie. STOP signal can't happen)
  I2C0_C1 &= ~I2C_C1_TXAK_MASK;
	
  // Saving semaphore into global variable
  I2C_SelectSlaveDevice(aI2CModule->primarySlaveAddress);
  ReadCompleteSemaphore = aI2CModule->readCompleteSemaphore;
	

  uint8_t mult; // value to use in baud rate formula
  uint8_t multSave; // saves the best value for the mult register
  uint8_t icrSave; // saves the best value for the icr register
  uint32_t baudRateActual; // actual baud rate calculated using the current mult and icr combination
  uint32_t baudRateError; // current error range calculated using baudRateActual
  uint32_t baudRateErrorMin = aI2CModule->baudRate; // current lowest value for baud rate error range
  

  for (uint8_t multReg = 0; multReg < 3; multReg++)
  {
    switch (multReg) // mult is used in the formula for baud rate,
    {                // whereas multReg is the value to be written into the MULT register
      case 0:
        mult = 1;
	break;
      case 1:
	mult = 2;
	break;
      case 2:
	mult = 4;
	break;
    }

    for (uint8_t icr = 0x10; icr <= 0x3F; icr++)
    {
      baudRateActual = (moduleClk/(mult*SclDivider[icr]));

      if (baudRateActual < aI2CModule->baudRate)
        baudRateError = aI2CModule->baudRate - baudRateActual; // positive error
      else
        baudRateError = baudRateActual - aI2CModule->baudRate; // negative error

      if (baudRateError < baudRateErrorMin)
      {
        baudRateErrorMin = baudRateError;
        multSave = multReg;
        icrSave = icr;
      }
    }
  }
  // Write in register values for the most accurate baud rate
  I2C0_F |= I2C_F_MULT(multSave);
  I2C0_F |= I2C_F_ICR(icrSave);
  
  // Setting up NVIC for I2C0 see K70 manual pg 97
  // Vector=40, IRQ=24
  // NVIC non-IPR=0 IPR=6
  // Clear any pending interrupts on I2C0
  NVICICPR0 = (1 << 24); // 24mod32 = 24
  // Enable interrupts from the I2C0
  NVICISER0 = (1 << 24);
  
  return true;
}



void I2C_SelectSlaveDevice(const uint8_t slaveAddress)
{
  PrimarySlaveAddress = slaveAddress; 
  
  SlaveAddressWrite = (slaveAddress << 1);
  SlaveAddressWrite &= ~0x1; // write mode address has first bit cleared

  SlaveAddressRead = (slaveAddress << 1);
  SlaveAddressRead |= 0x1; // read mode address has first bit set
}



// follows pg. 19 of accelerometer manual - single-byte write
void I2C_Write(const uint8_t registerAddress, const uint8_t data)
{

  while (I2C0_S & I2C_S_BUSY_MASK)
  {
  }  // wait until bus is idle


  I2C0_C1 |= I2C_C1_MST_MASK; // START signal
  I2C0_C1 |= I2C_C1_TX_MASK; // I2C is in Tx mode (write)

  for (uint8_t i = 0; i < 3; i++)
  {
    switch (i)
    {
      case 0:
        I2C0_D = SlaveAddressWrite; // send accelerometer address with R/W bit set to Write
        break;
      case 1:
        I2C0_D = registerAddress; // send address of register to be written
        break;
      case 2:
        I2C0_D = data; // send data to write into register
        break;
    }

  	// Wait for AK (IICIF also sets from a transfer complete)
    while (!(I2C0_S & I2C_S_IICIF_MASK));
    I2C0_S |= I2C_S_IICIF_MASK;

    if (I2C0_S & I2C_S_RXAK_MASK) // if no AK received, end the communication
      break;
  }
  
  I2C0_C1 &= ~I2C_C1_MST_MASK; // STOP signal
}



// follows pg. 19 of accelerometer manual - multi-byte read
void I2C_PollRead(const uint8_t registerAddress, uint8_t* const data, const uint8_t nbBytes)
{
  while (I2C0_S & I2C_S_BUSY_MASK)
  {
  } // wait until bus is idle

  I2C0_C1 |= I2C_C1_MST_MASK; // START signal
  I2C0_C1 |= I2C_C1_TX_MASK; // I2C is in Tx mode (write)
  
  for (uint8_t i = 0; i < (nbBytes + 3); i++)
  {
    switch (i)
    {
      case 0:
        I2C0_D = SlaveAddressWrite; // send accelerometer address with R/W bit set to Write
        break;
      case 1:
        I2C0_D = registerAddress; // send address of register to be written
        break;
      case 2:
        I2C0_C1 &= ~I2C_C1_TX_MASK; // I2C is in Rx mode (read)
        I2C0_C1 |= I2C_C1_RSTA_MASK; // REPEAT START signal
        I2C0_D   = SlaveAddressRead; // send accelerometer address with R/W bit cleared to Read
        break;
      default:
      // Data is put into I2C0_D here

      if (i == (nbBytes + 1)) // 2nd last byte to be read - different between figure 11 diagram vs flowchart?
        I2C0_C1 |= I2C_C1_TXAK_MASK; // NAK signal
      else if (i == (nbBytes + 2)) // last byte to be read
        I2C0_C1 &= ~I2C_C1_MST_MASK; // STOP signal
      else
        I2C0_C1 &= ~I2C_C1_TXAK_MASK; // AK signal
		
      data[i - 3] = I2C0_D; // place data into array

      break;
    }

    // Wait for current transfer to be completed
    while (!(I2C0_S & I2C_S_IICIF_MASK));
    I2C0_S |= I2C_S_IICIF_MASK;

    if ((i < 3) && (I2C0_S & I2C_S_RXAK_MASK)) // if no AK received during write stages, end the communication
      break;
  }
}



void I2C_IntRead(const uint8_t registerAddress, uint8_t* const data, const uint8_t nbBytes)
{
  while (I2C0_S & I2C_S_BUSY_MASK)
  {
  } // wait until bus is idle
  
  // Save parameters to global private variables for later communication with ISR
  IsrNbBytes = nbBytes;
  IsrData    = data;
  
  I2C0_C1 |= I2C_C1_MST_MASK; // START signal
  I2C0_C1 |= I2C_C1_TX_MASK; // I2C is in Tx mode (write)
  I2C0_C1 |= I2C_C1_IICIE_MASK; // enable I2C interrupts
  
  for (uint8_t i = 0; i < 3; i++)
  {
    switch (i)
    {
      case 0:
        I2C0_D = SlaveAddressWrite; // send accelerometer address with R/W bit set to Write
        break;
      case 1:
        I2C0_D = registerAddress; // send address of register to be written
        break;
      case 2:
        I2C0_C1 &= ~I2C_C1_TX_MASK; // I2C is in Rx mode (read)
        I2C0_C1 |= I2C_C1_RSTA_MASK; // REPEAT START signal
        I2C0_D   = SlaveAddressRead; // send accelerometer address with R/W bit cleared to Read
        break;
    }
	
    // Wait for current transfer to be completed - once completed the ISR should trigger
    while (!(I2C0_S & I2C_S_IICIF_MASK));

    if (I2C0_S & I2C_S_RXAK_MASK) // if no AK received end the communication
      break;
  }
  
  // ISR should trigger here, following the flowchart
}



void __attribute__ ((interrupt)) I2C_ISR(void)
{
  I2C0_S |= I2C_S_IICIF_MASK; // w1c interrupt flag
  static uint8_t dataIndex = 0; // index to data pointer
  
  // Flowchart 55-42 on pg 1896 of K70 manual
  if (I2C0_C1 & I2C_C1_MST_MASK)
  {
    if (I2C0_C1 & ~I2C_C1_TX_MASK); // ISR is only for reading
    {
      if (dataIndex == IsrNbBytes) //last byte to read
      {
        I2C0_C1 &= ~I2C_C1_MST_MASK; // STOP signal
        I2C0_C1 &= ~I2C_C1_IICIE_MASK; // disable I2C interrupts
      }
      else if (dataIndex == (IsrNbBytes - 1)) //2nd last byte to read
        I2C0_C1 |= I2C_C1_TXAK_MASK;
      
	  
      IsrData[dataIndex] = I2C0_D; // Read from data reg and store
	  
      // If last byte to read, reset index and callback
      if (dataIndex == IsrNbBytes)
      {
        dataIndex = 0;

        // Allow I2CThread to run
        OS_SemaphoreSignal(ReadCompleteSemaphore);
      }
      // else, increment index and the ISR should reoccur
      else
        dataIndex++;
    }
  }
}


