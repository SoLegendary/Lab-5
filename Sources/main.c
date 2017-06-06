/* ###################################################################
**     Filename    : main.c
**     Project     : Lab3
**     Processor   : MK70FN1M0VMJ12
**     Version     : Driver 01.01
**     Compiler    : GNU C Compiler
**     Date/Time   : 2015-07-20, 13:27, # CodeGen: 0
**     Abstract    :
**         Main module.
**         This module contains user's application code.
**     Settings    :
**     Contents    :
**         No public methods
**
** ###################################################################*/
/*!
** @file main.c
** @version 1.0
** @brief
**         Main module.
**         This module utilises the K70 Tower Device to input and output basic data packets
**         in accordance to a Communication Protocol as described on UTSOnline.
**         It is to be used in conjunction with TowerPC.exe which sends and receives these
**         packets to and from the Tower via a Windows PC.
*/
/*!
**  @addtogroup main_module main module documentation
**
**  @author Thanit Tangson
**  @{
*/
/* MODULE main */


// CPU module - contains low level hardware initialization routines
#include "Cpu.h"
#include "packet.h"
#include "UART.h"
#include "Flash.h"
#include "LEDs.h"
#include "RTC.h"
#include "PIT.h"
#include "FTM.h"
#include "accel.h"
#include "I2C.h"
#include "median.h"
#include "OS.h"
#include "PE_Types.h"
#include "IO_Map.h"


// Tower Protocols based on the command byte of each packet
#define CMD_STARTUP   0x04
#define CMD_VERSION   0x09
#define CMD_NUMBER    0x0B
#define CMD_TOWERMODE 0x0D
#define CMD_PROGBYTE  0x07
#define CMD_READBYTE  0x08
#define CMD_SETTIME   0x0C
#define CMD_MODE      0x0A
#define CMD_ACCEL     0x10

#define THREAD_STACK_SIZE 1024


static TFTMChannel FTM0Channel0; // Struct to set up channel 0 in the FTM module
static TAccelSetup accelSetup; // Struct to set up the accelerometer via I2C0

volatile uint16union_t *towerNumber = NULL; // Currently set tower number and mode
volatile uint16union_t *towerMode   = NULL;

static TAccelData accelDataOld; // oldest XYZ accelerometer data - only used in asynchronous polling mode
static TAccelData accelDataNew; // latest XYZ accelerometer data

static bool synchronousMode = false; // variable to track current I2C mode (synchronous by default)

// RTOS Threads stacks
static uint32_t InitThreadStack[THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));
static uint32_t RTCThreadStack[THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));
static uint32_t PacketThreadStack[THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));
static uint32_t FTM0ThreadStack[THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));
static uint32_t PITThreadStack[THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));
static uint32_t AccelThreadStack[THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));
static uint32_t I2CThreadStack[THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));

// RTOS Semaphores as private globals
static OS_ECB* RTCSemaphore;
static OS_ECB* FTM0Semaphore;
static OS_ECB* PITSemaphore;
static OS_ECB* AccelSemaphore;
static OS_ECB* I2CSemaphore;


// Function Initializations

/*!
 * @brief The startup packet will send four packets back to the PC by default
 * 	  It will send the startup, version, tower number and mode packets.
 *        If flash is reset, tower number and mode will be set to studentNumber and defaultTowerMode
 *
 * Parameter1 = 0, Parameter2 = 0, Parameter3 = 0
 *
 * @return bool - TRUE if all of the packets were handled successfully.
 */
bool HandleStartupPacket(void)
{
  bool success;

  uint16union_t studentNumber;
  studentNumber.l    = 5696; // Last 4 digits of Emile Fadel's student number
  studentNumber.s.Hi = 0x16;
  studentNumber.s.Lo = 0x40;

  uint16union_t defaultTowerMode;
  defaultTowerMode.l    = 1;    // Default mode 1 according to Lab 2 notes
  defaultTowerMode.s.Hi = 0x00;
  defaultTowerMode.s.Lo = 0x01;

  if (towerNumber == NULL) // If towerNumber is not yet programmed, save it to flash
  {
    // Note that the void* typecast does not appear in lab 2 notes
    success = Flash_AllocateVar((void*)&towerNumber, sizeof(*towerNumber));
    success = Flash_Write16((uint16_t*)towerNumber, studentNumber.l);
    if (!success) // If Flash_Allocate or Flash_Write failed
      return false;
  }

  if (towerMode == NULL) // If towerMode is not yet programmed, save it to flash
  {
    success = Flash_AllocateVar((void*)&towerMode, sizeof(*towerMode));
    success = Flash_Write16((uint16_t*)towerMode, defaultTowerMode.l);
    if (!success)
      return false;
  }


  return ((Packet_Put(CMD_STARTUP, 0x00, 0x00, 0x00)) &&
	  (Packet_Put(CMD_VERSION, 'v', 0x01, 0x00)) &&
	  (Packet_Put(CMD_NUMBER, 0x01, towerNumber->s.Lo, towerNumber->s.Hi)) &&
	  (Packet_Put(CMD_TOWERMODE, 0x01, towerMode->s.Lo, towerMode->s.Hi)) &&
	  (Packet_Put(CMD_MODE, 0x01, synchronousMode, 0x00)));
}



/*!
 * @brief Handles the tower version packet in accordance with the Tower Serial Communication Protocol document
 *
 * Parameter1 = 'v', Parameter2 = 1, Parameter3 = 0 (V1.0)
 * @return bool - TRUE if the packet was handled successfully.
 */
bool HandleVersionPacket(void)
{
  return Packet_Put(CMD_VERSION, 'v', 0x01, 0x00);
}



/*!
 * @brief Handles the tower number packet in accordance with the Tower Serial Communication Protocol document
 *
 * Parameter1 = 0x01, Parameter2 = LSB, Parameter3 = MSB
 * If Parameter1 is 0x02, you are able to set LSB and MSB by passing these in through Parameter2 and Parameter3
 *
 * @return bool - TRUE if the packet was handled successfully.
 */
bool HandleNumberPacket(void)
{
  if (Packet_Parameter1 == 0x02) // If the packet is in SET mode, set the tower number by storing it in flash before returning the packet
  {
    bool success = Flash_Write16((uint16_t*)towerNumber, Packet_Parameter23);

    return Packet_Put(CMD_NUMBER, 0x01, towerNumber->s.Lo, towerNumber->s.Hi) && success;
  }
  else if (Packet_Parameter1 == 0x01) // If the packet is in GET mode, just return the current tower number
  {
    return Packet_Put(CMD_NUMBER, 0x01, towerNumber->s.Lo, towerNumber->s.Hi);
  }

  // If the packet is not in either SET or GET mode, return false
  return false;
}



/*!
 * @brief Handles the tower mode packet in accordance with the Tower Serial Communication Protocol document
 *
 * Parameter1 = 0x01, Parameter2 = LSB, Parameter3 = MSB
 * If Parameter1 is 0x02, you are able to set LSB and MSB by passing these in through Parameter2 and Parameter3
 *
 * @return bool - TRUE if the packet was handled successfully.
 */
bool HandleTowerModePacket(void)
{
  if (Packet_Parameter1 == 0x02) // If the packet is in SET mode, set the tower mode by storing it in flash before returning the packet
  {
    bool success = Flash_Write16((uint16_t*)towerMode, Packet_Parameter23);
    return Packet_Put(CMD_TOWERMODE, 0x01, towerMode->s.Lo, towerMode->s.Hi) && success;
  }
  else if (Packet_Parameter1 == 0x01) // If the packet is in GET mode, just return the current tower mode
    return Packet_Put(CMD_TOWERMODE, 0x01, towerMode->s.Lo, towerMode->s.Hi);

  // If the packet is not in either SET or GET mode, return false
  return false;
}



/*!
 * @brief Handles a Flash Program Byte packet as per the Tower Serial Communication Protocol document
 * by writing the data in parameter3 to the address given in parameter1
 *
 * Parameter1 = address offset (0-7), Parameter2 = 0, Parameter3 = data
 *
 * @return bool - TRUE if the packet was handled successfully.
 */
bool HandleProgBytePacket(void)
{
  // Return false if the address is out of range
  if ((Packet_Parameter1 < 0) || (Packet_Parameter1 > 8))
    return false;
	  
  if (Packet_Parameter1 == 0x08) // 0x08 erases the flash
    return Flash_Erase();
	  
  // Writes the data in parameter3 to the given address starting at FLASH_DATA_START and offsetted according to Parameter1
  return Flash_Write8((uint8_t *)(FLASH_DATA_START + Packet_Parameter1), Packet_Parameter3);
}



/*!
 * @brief Handles a Flash Read Byte packet as per the Tower Serial Communication Protocol document
 * by putting in a packet with the address of the flash byte read and the data contained in that address.
 *
 * Parameter1 = address offset (0-7), Parameter2 = 0, Parameter3 = 0
 *
 * @return bool - TRUE if the packet was handled successfully.
 */
bool HandleReadBytePacket(void)
{
  // Return false if the address is out of range
  if ((Packet_Parameter1 < 0) || (Packet_Parameter1 > 7))
    return false;

  // Data is accessed using a Flash.h macro by starting at address FLASH_DATA_START and offsetting according to Parameter1
  return (Packet_Put(CMD_READBYTE, Packet_Parameter1, 0x00, _FB(FLASH_DATA_START + Packet_Parameter1)));
}

  
  
/*!
 * @brief Handles a Set Time packet as per the Tower Serial Communication Protocol document
 * by setting sending the parameters of the packet to the RTC module to be set.
 *
 * Parameter1 = hours(0-23), Parameter2 = minutes(0-59), Parameter3 = seconds (0-59)
 *
 * @return bool - TRUE if the packet was handled successfully, FALSE if parameters out of range.
 */
bool HandleSetTimePacket(void)
{
  // Return false if any time values are out of range
  if ((Packet_Parameter1 < 0) || (Packet_Parameter1 > 23) ||
      (Packet_Parameter2 < 0) || (Packet_Parameter2 > 59) ||
      (Packet_Parameter3 < 0) || (Packet_Parameter3 > 59))
    return false;
    
  RTC_Set(Packet_Parameter1, Packet_Parameter2, Packet_Parameter3);
  
  uint8_t seconds, minutes, hours;
  RTC_Get(&seconds, &minutes, &hours);
    
  return (Packet_Put(CMD_SETTIME, seconds, minutes, hours));
}



/*!
 * @brief Handles a Protocol - Mode packet as per the Tower Serial Communication Protocol document
 * - either getting or setting the mode of operation for the accelerometer module (polling vs interrupts)
 *
 * Parameter1 = 1 for GET, 2 for SET
 * Parameter2 = 0 for asynchronous (polling)
 *              1 for synchronous (interrupts)
 * Parameter3 = 0
 *
 * @return bool - TRUE if the packet was handled successfully, FALSE if parameters out of range.
 */
bool HandleModePacket(void)
{
  if (Packet_Parameter1 == 0x02) // If the packet is for SET change the mode using Accel_SetMode()
  {
    switch (Packet_Parameter2)
    {
      case 0:
        synchronousMode = false;
        Accel_SetMode(ACCEL_POLL);
	PIT_Enable(true);
        return true;
      case 1:
        synchronousMode = true;
        Accel_SetMode(ACCEL_INT);
        PIT_Enable(false);
	return true;
      default:
	return false;
    }
  }
  
  else if (Packet_Parameter1 == 0x01) // If the packet is for GET, just return the current mode
    return (Packet_Put(CMD_MODE, 1, synchronousMode, 0));

  // If the packet is not in either SET or GET mode, return false
  return false;
}


  
/*!
 * @brief Handles the packet by first checking to see what type of packet it is and processing it
 * as per the Tower Serial Communication Protocol document.
 */
void HandlePacket(void)
{
  bool success = false; // Holds the success or failure of the packet handlers
  bool ackReq  = false; // Holds whether an Acknowledgment is required

  if (Packet_Command & PACKET_ACK_MASK) // Check if ACK bit is set
  {
    ackReq = true;
    Packet_Command &= 0x7F; //Strips the top bit of the Command Byte to ignore ACK bit
  }

  switch (Packet_Command)
  {
    case CMD_STARTUP:
      success = HandleStartupPacket();
      break;
    case CMD_VERSION:
      success = HandleVersionPacket();
      break;
    case CMD_NUMBER:
      success = HandleNumberPacket();
      break;
    case CMD_TOWERMODE:
      success = HandleTowerModePacket();
      break;
    case CMD_PROGBYTE:
      success = HandleProgBytePacket();
      break;
    case CMD_READBYTE:
      success = HandleReadBytePacket();
      break;
    case CMD_SETTIME:
      success = HandleSetTimePacket();
      break;
    case CMD_MODE:
      success = HandleModePacket();
      break;
    default:
      success = false;
      break;
  }

  /*!
   * Check if the handling of the packet was a success and an ACK packet was requested
   * If that checks out, set the ACK bit to 1
   * Else, if the handling of the packet failed and an ACK packet was requested
   * Clear the ACK bit in order to indicate a NAK (command could not be carried out)
   * Finally, return the ACK packet to the Tower
   */

  if (ackReq)
  {
    if (success)
      Packet_Command |= PACKET_ACK_MASK; // Set the ACK bit
    else
      Packet_Command &= ~PACKET_ACK_MASK; // Clear the ACK bit

    Packet_Put(Packet_Command, Packet_Parameter1, Packet_Parameter2, Packet_Parameter3); // Send the ACK/NAK packet back out
  }
}
  





/***************************************************************************/
/** RTOS THREADS FOR ALL MODULES - In order of priority HIGHEST -> LOWEST **/
/***************************************************************************/


/*! @brief Very first thread to run, but runs only once to initialise all tower modules
 * and then deletes itself at the end; priority = 0 (highest)
 */
static void InitThread(void* pData)
{
  __DI(); // Disable interrupts

  // RTOS Semaphores - all initialised as 0 (ie. threads can't run before being signaled)
  RTCSemaphore    = OS_SemaphoreCreate(0);
  FTM0Semaphore   = OS_SemaphoreCreate(0);
  PITSemaphore    = OS_SemaphoreCreate(0);
  AccelSemaphore  = OS_SemaphoreCreate(0);
  I2CSemaphore    = OS_SemaphoreCreate(0);

  const uint32_t BAUDRATE      = 115200;
  const uint32_t ACCELBAUDRATE = 100000;

  // Setting up FTM channel 0 struct
  FTM0Channel0.channelNb           = 0;
  FTM0Channel0.timerFunction       = TIMER_FUNCTION_OUTPUT_COMPARE;
  FTM0Channel0.ioType.outputAction = TIMER_OUTPUT_LOW;
  FTM0Channel0.semaphore           = FTM0Semaphore;

  // Setting up accel init struct
  accelSetup.moduleClk             = CPU_BUS_CLK_HZ;
  accelSetup.dataReadySemaphore    = AccelSemaphore;
  accelSetup.readCompleteSemaphore = I2CSemaphore;

  Packet_Init(BAUDRATE, CPU_BUS_CLK_HZ);
  Flash_Init();
  LEDs_Init();
  FTM_Init();
  FTM_Set(&FTM0Channel0);
  PIT_Init(CPU_BUS_CLK_HZ, PITSemaphore);
  RTC_Init(RTCSemaphore);
  Accel_Init(&accelSetup);

  // Polling mode by default for accelerometer
  PIT_Set(1000000000, true);
  PIT_Enable(true);
  synchronousMode = false;
  Accel_SetMode(ACCEL_POLL);

  // Startup protocol
  LEDs_On(LED_ORANGE);
  HandleStartupPacket();

  __EI(); // Enable interrupts

  OS_ThreadDelete(OS_PRIORITY_SELF);
}


/*! @brief Thread to send the time from the RTC back to the PC
 *  set as high priority to avoid clock desyncing
 */
static void RTCThread(void* pData)
{
  for (;;)
  {
    // wait for RTC_ISR to signal
    OS_SemaphoreWait(RTCSemaphore,0);

    // Get and send time back to PC, just as in HandleSetTimePacket
    uint8_t seconds, minutes, hours;
    RTC_Get(&seconds, &minutes, &hours);

    LEDs_Toggle(LED_YELLOW);
    Packet_Put(CMD_SETTIME, seconds, minutes, hours);
  }
}


/*! @brief Thread to do something once the FTM0 timer expires
 */
static void FTM0Thread(void* pData)
{
  for (;;)
  {
    // wait for FTM0_ISR to signal
    OS_SemaphoreWait(FTM0Semaphore,0);

    LEDs_Off(LED_BLUE);
  }
}

/*! @brief Thread to do something periodically according to the PIT period setting
 * currently used for Lab 4 polling mode readings (asynchronous mode)
 */
static void PITThread(void* pData)
{
  for (;;)
  {
    // wait for PIT_ISR to signal
    OS_SemaphoreWait(PITSemaphore,0);

    // shift new data into old before getting new data
    accelDataOld.bytes[0] = accelDataNew.bytes[0];
    accelDataOld.bytes[1] = accelDataNew.bytes[1];
    accelDataOld.bytes[2] = accelDataNew.bytes[2];

    Accel_ReadXYZ(accelDataNew.bytes);

   // If any axes are new, send the packet and toggle green LED
     if((accelDataOld.bytes[0] != accelDataNew.bytes[0]) ||
        (accelDataOld.bytes[1] != accelDataNew.bytes[1]) ||
        (accelDataOld.bytes[2] != accelDataNew.bytes[2]))
    {
      Packet_Put(CMD_ACCEL, accelDataNew.bytes[0], accelDataNew.bytes[1], accelDataNew.bytes[2]);
      LEDs_Toggle(LED_GREEN);
    }
  }
}

/*! @brief Thread to read accelerometer data via Accel_ISR signaling
 */
static void AccelThread(void* pData)
{
  for (;;)
  {
    // wait for FIFO or UART to signal
    OS_SemaphoreWait(AccelSemaphore,0);

    Accel_ReadXYZ(accelDataNew.bytes);
  }
}


/*! @brief Thread to median filter and send data back to PC via I2C_ISR signaling
 */
static void I2CThread(void* pData)
{
  for (;;)
  {
    // wait for FIFO or UART to signal
    OS_SemaphoreWait(I2CSemaphore,0);

    // This thread copies some code from Accel_ReadXYZ due to order problems with synchronous mode

    // array of 3 unions and to save data from the 3 most recent Accel_ReadXYZ calls
    static TAccelData accelData[3] = {{0,0,0},{0,0,0}};

    // shifts data in the array unions back (index 0 is most recent data, 2 is oldest data)
    for (uint8_t i = 0; i < 3; i++)
    {
      accelData[2].bytes[i] = accelData[1].bytes[i];
      accelData[1].bytes[i] = accelData[0].bytes[i];
    }

    // Populate newest array
    accelData[0].bytes[0] = accelDataNew.bytes[0];
    accelData[0].bytes[1] = accelDataNew.bytes[1];
    accelData[0].bytes[2] = accelDataNew.bytes[2];

    // Median filters the last 3 sets of XYZ data
    for (uint8_t i = 0; i < 3; i++)
      accelDataNew.bytes[i] = Median_Filter3(accelData[0].bytes[i], accelData[1].bytes[i], accelData[2].bytes[i]);

    // Send data back to PC
    Packet_Put(CMD_ACCEL, accelDataNew.bytes[0], accelDataNew.bytes[1], accelDataNew.bytes[2]);
    LEDs_Toggle(LED_GREEN);
  }
}


/*! @brief Thread to handle packets taken from the FIFO
 *  does not wait for any semaphore (ie. would run forever), but has lowest priority
 *  and so can be interrupted by any ISR and be placed on waiting for any other thread
 */
static void PacketThread(void* pData)
{
  for (;;)
  {
    if (Packet_Get()) // If a packet is received.
    {
      LEDs_On(LED_BLUE);
      FTM_StartTimer(&FTM0Channel0);
      HandlePacket(); // Handle the packet appropriately.
    }
  }
}



/*lint -save  -e970 Disable MISRA rule (6.3) checking. */
int main(void)
/*lint -restore Enable MISRA rule (6.3) checking. */
{
  OS_ERROR error; // error object for RTOS
  

  /*** Processor Expert internal initialization. DON'T REMOVE THIS CODE!!! ***/
  PE_low_level_init();
  /*** End of Processor Expert internal initialization.                    ***/


  // Initialize the RTOS - without flashing the orange LED "heartbeat"
  OS_Init(CPU_CORE_CLK_HZ, false);


  /*  Creating all threads; parameters are:
   *  1. Thread name (address)
   *  2. Arguments (which are all of type void* pData)
   *  3. Pointer to thread stack with [STACK_SIZE]
   *  4. Priority (0 is highest)
   */
  error = OS_ThreadCreate(InitThread,
          NULL,
          &InitThreadStack[THREAD_STACK_SIZE - 1],
	  0);

  error = OS_ThreadCreate(RTCThread,
          NULL,
          &RTCThreadStack[THREAD_STACK_SIZE - 1],
	  1);

	  
  // threads 2 & 3 are inside UART.c
  

  error = OS_ThreadCreate(FTM0Thread,
          NULL,
          &FTM0ThreadStack[THREAD_STACK_SIZE - 1],
	  4);

  //error = OS_ThreadCreate(PITThread,
  //        NULL,
  //        &PITThreadStack[THREAD_STACK_SIZE - 1],
//	  5);

  error = OS_ThreadCreate(AccelThread,
          NULL,
          &AccelThreadStack[THREAD_STACK_SIZE - 1],
  	  5);

  error = OS_ThreadCreate(I2CThread,
          NULL,
          &I2CThreadStack[THREAD_STACK_SIZE - 1],
  	  6);
	  
  error = OS_ThreadCreate(PacketThread,
          NULL,
          &PacketThreadStack[THREAD_STACK_SIZE - 1],
	  7);
	  
  // Start multithreading - never returns!
  // NOTE that this still runs threads that are created in lower levels inside modules
  OS_Start();



  /*** Don't write any code pass this line, or it will be deleted during code generation. ***/
  /*** RTOS startup code. Macro PEX_RTOS_START is defined by the RTOS component. DON'T MODIFY THIS CODE!!! ***/
  #ifdef PEX_RTOS_START
    PEX_RTOS_START();                  /* Startup of the selected RTOS. Macro is defined by the RTOS component. */
  #endif
  /*** End of RTOS startup code.  ***/
  /*** Processor Expert end of main routine. DON'T MODIFY THIS CODE!!! ***/
  for(;;){}
  /*** Processor Expert end of main routine. DON'T WRITE CODE BELOW!!! ***/
} /*** End of main routine. DO NOT MODIFY THIS TEXT!!! ***/

/* END main */
/*!
** @}
*/
/*
** ###################################################################
**
**     This file was created by Processor Expert 10.5 [05.21]
**     for the Freescale Kinetis series of microcontrollers.
**
** ###################################################################
*/
