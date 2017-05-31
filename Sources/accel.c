/*! @file accel.c
 *
 *  @brief HAL for the accelerometer.
 *
 *  This contains the functions for interfacing to the MMA8451Q accelerometer via
 *  the I2C module. All register writes and data reads are performed via functions
 *  included from I2C.h. Also contains address definitions for all accelerometer registers.
 *
 *  @author Thanit Tangson
 *  @date 2017-5-9
 */
/*!
**  @addtogroup main_module main module documentation
**
*/

// Accelerometer functions
#include "accel.h"

// Inter-Integrated Circuit
#include "I2C.h"

// Median filter
#include "median.h"

// K70 module registers
#include "MK70F12.h"

// CPU and PE_types are needed for critical section variables and the defintion of NULL pointer
#include "CPU.h"
#include "PE_types.h"
#include "OS.h"

// Accelerometer registers
#define ADDRESS_OUT_X_MSB 0x01

#define ADDRESS_INT_SOURCE 0x0C

static union
{
  uint8_t byte;			/*!< The INT_SOURCE bits accessed as a byte. */
  struct
  {
    uint8_t SRC_DRDY   : 1;	/*!< Data ready interrupt status. */
    uint8_t               : 1;
    uint8_t SRC_FF_MT  : 1;	/*!< Freefall/motion interrupt status. */
    uint8_t SRC_PULSE  : 1;	/*!< Pulse detection interrupt status. */
    uint8_t SRC_LNDPRT : 1;	/*!< Orientation interrupt status. */
    uint8_t SRC_TRANS  : 1;	/*!< Transient interrupt status. */
    uint8_t SRC_FIFO   : 1;	/*!< FIFO interrupt status. */
    uint8_t SRC_ASLP   : 1;	/*!< Auto-SLEEP/WAKE interrupt status. */
  } bits;			/*!< The INT_SOURCE bits accessed individually. */
} INT_SOURCE_Union;

#define INT_SOURCE     		INT_SOURCE_Union.byte
#define INT_SOURCE_SRC_DRDY	INT_SOURCE_Union.bits.SRC_DRDY
#define INT_SOURCE_SRC_FF_MT	CTRL_REG4_Union.bits.SRC_FF_MT
#define INT_SOURCE_SRC_PULSE	CTRL_REG4_Union.bits.SRC_PULSE
#define INT_SOURCE_SRC_LNDPRT	CTRL_REG4_Union.bits.SRC_LNDPRT
#define INT_SOURCE_SRC_TRANS	CTRL_REG4_Union.bits.SRC_TRANS
#define INT_SOURCE_SRC_FIFO	CTRL_REG4_Union.bits.SRC_FIFO
#define INT_SOURCE_SRC_ASLP	CTRL_REG4_Union.bits.SRC_ASLP

#define ADDRESS_CTRL_REG1 0x2A

typedef enum
{
  DATE_RATE_800_HZ,
  DATE_RATE_400_HZ,
  DATE_RATE_200_HZ,
  DATE_RATE_100_HZ,
  DATE_RATE_50_HZ,
  DATE_RATE_12_5_HZ,
  DATE_RATE_6_25_HZ,
  DATE_RATE_1_56_HZ
} TOutputDataRate;

typedef enum
{
  SLEEP_MODE_RATE_50_HZ,
  SLEEP_MODE_RATE_12_5_HZ,
  SLEEP_MODE_RATE_6_25_HZ,
  SLEEP_MODE_RATE_1_56_HZ
} TSLEEPModeRate;

static union
{
  uint8_t byte;			/*!< The CTRL_REG1 bits accessed as a byte. */
  struct
  {
    uint8_t ACTIVE    : 1;	/*!< Mode selection. */
    uint8_t F_READ    : 1;	/*!< Fast read mode. */
    uint8_t LNOISE    : 1;	/*!< Reduced noise mode. */
    uint8_t DR        : 3;	/*!< Data rate selection. */
    uint8_t ASLP_RATE : 2;	/*!< Auto-WAKE sample frequency. */
  } bits;			/*!< The CTRL_REG1 bits accessed individually. */
} CTRL_REG1_Union;

#define CTRL_REG1     		    CTRL_REG1_Union.byte
#define CTRL_REG1_ACTIVE	    CTRL_REG1_Union.bits.ACTIVE
#define CTRL_REG1_F_READ  	  CTRL_REG1_Union.bits.F_READ
#define CTRL_REG1_LNOISE  	  CTRL_REG1_Union.bits.LNOISE
#define CTRL_REG1_DR	    	  CTRL_REG1_Union.bits.DR
#define CTRL_REG1_ASLP_RATE	  CTRL_REG1_Union.bits.ASLP_RATE

#define ADDRESS_CTRL_REG2 0x2B

#define ADDRESS_CTRL_REG3 0x2C

static union
{
  uint8_t byte;			/*!< The CTRL_REG3 bits accessed as a byte. */
  struct
  {
    uint8_t PP_OD       : 1;	/*!< Push-pull/open drain selection. */
    uint8_t IPOL        : 1;	/*!< Interrupt polarity. */
    uint8_t WAKE_FF_MT  : 1;	/*!< Freefall/motion function in SLEEP mode. */
    uint8_t WAKE_PULSE  : 1;	/*!< Pulse function in SLEEP mode. */
    uint8_t WAKE_LNDPRT : 1;	/*!< Orientation function in SLEEP mode. */
    uint8_t WAKE_TRANS  : 1;	/*!< Transient function in SLEEP mode. */
    uint8_t FIFO_GATE   : 1;	/*!< FIFO gate bypass. */
  } bits;			/*!< The CTRL_REG3 bits accessed individually. */
} CTRL_REG3_Union;

#define CTRL_REG3     		    CTRL_REG3_Union.byte
#define CTRL_REG3_PP_OD		    CTRL_REG3_Union.bits.PP_OD
#define CTRL_REG3_IPOL		    CTRL_REG3_Union.bits.IPOL
#define CTRL_REG3_WAKE_FF_MT	CTRL_REG3_Union.bits.WAKE_FF_MT
#define CTRL_REG3_WAKE_PULSE	CTRL_REG3_Union.bits.WAKE_PULSE
#define CTRL_REG3_WAKE_LNDPRT	CTRL_REG3_Union.bits.WAKE_LNDPRT
#define CTRL_REG3_WAKE_TRANS	CTRL_REG3_Union.bits.WAKE_TRANS
#define CTRL_REG3_FIFO_GATE	  CTRL_REG3_Union.bits.FIFO_GATE

#define ADDRESS_CTRL_REG4 0x2D

static union
{
  uint8_t byte;			/*!< The CTRL_REG4 bits accessed as a byte. */
  struct
  {
    uint8_t INT_EN_DRDY   : 1;	/*!< Data ready interrupt enable. */
    uint8_t               : 1;
    uint8_t INT_EN_FF_MT  : 1;	/*!< Freefall/motion interrupt enable. */
    uint8_t INT_EN_PULSE  : 1;	/*!< Pulse detection interrupt enable. */
    uint8_t INT_EN_LNDPRT : 1;	/*!< Orientation interrupt enable. */
    uint8_t INT_EN_TRANS  : 1;	/*!< Transient interrupt enable. */
    uint8_t INT_EN_FIFO   : 1;	/*!< FIFO interrupt enable. */
    uint8_t INT_EN_ASLP   : 1;	/*!< Auto-SLEEP/WAKE interrupt enable. */
  } bits;			/*!< The CTRL_REG4 bits accessed individually. */
} CTRL_REG4_Union;

#define CTRL_REG4            		CTRL_REG4_Union.byte
#define CTRL_REG4_INT_EN_DRDY	  CTRL_REG4_Union.bits.INT_EN_DRDY
#define CTRL_REG4_INT_EN_FF_MT	CTRL_REG4_Union.bits.INT_EN_FF_MT
#define CTRL_REG4_INT_EN_PULSE	CTRL_REG4_Union.bits.INT_EN_PULSE
#define CTRL_REG4_INT_EN_LNDPRT	CTRL_REG4_Union.bits.INT_EN_LNDPRT
#define CTRL_REG4_INT_EN_TRANS	CTRL_REG4_Union.bits.INT_EN_TRANS
#define CTRL_REG4_INT_EN_FIFO	  CTRL_REG4_Union.bits.INT_EN_FIFO
#define CTRL_REG4_INT_EN_ASLP	  CTRL_REG4_Union.bits.INT_EN_ASLP

#define ADDRESS_CTRL_REG5 0x2E

static union
{
  uint8_t byte;			/*!< The CTRL_REG5 bits accessed as a byte. */
  struct
  {
    uint8_t INT_CFG_DRDY   : 1;	/*!< Data ready interrupt enable. */
    uint8_t                : 1;
    uint8_t INT_CFG_FF_MT  : 1;	/*!< Freefall/motion interrupt enable. */
    uint8_t INT_CFG_PULSE  : 1;	/*!< Pulse detection interrupt enable. */
    uint8_t INT_CFG_LNDPRT : 1;	/*!< Orientation interrupt enable. */
    uint8_t INT_CFG_TRANS  : 1;	/*!< Transient interrupt enable. */
    uint8_t INT_CFG_FIFO   : 1;	/*!< FIFO interrupt enable. */
    uint8_t INT_CFG_ASLP   : 1;	/*!< Auto-SLEEP/WAKE interrupt enable. */
  } bits;			/*!< The CTRL_REG5 bits accessed individually. */
} CTRL_REG5_Union;

#define CTRL_REG5     		      	CTRL_REG5_Union.byte
#define CTRL_REG5_INT_CFG_DRDY		CTRL_REG5_Union.bits.INT_CFG_DRDY
#define CTRL_REG5_INT_CFG_FF_MT		CTRL_REG5_Union.bits.INT_CFG_FF_MT
#define CTRL_REG5_INT_CFG_PULSE		CTRL_REG5_Union.bits.INT_CFG_PULSE
#define CTRL_REG5_INT_CFG_LNDPRT	CTRL_REG5_Union.bits.INT_CFG_LNDPRT
#define CTRL_REG5_INT_CFG_TRANS		CTRL_REG5_Union.bits.INT_CFG_TRANS
#define CTRL_REG5_INT_CFG_FIFO		CTRL_REG5_Union.bits.INT_CFG_FIFO
#define CTRL_REG5_INT_CFG_ASLP		CTRL_REG5_Union.bits.INT_CFG_ASLP

/*!
 * @}
*/


// Private global variable for the Accel thread semaphore
ECB* DataReadySemaphore;

static bool SynchronousMode = false; // private global to track whether we are in polling or interrupt mode



bool Accel_Init(const TAccelSetup* const accelSetup)
{
  // Accelerometer is connected to PORTB pin 4 via INT1 (see tower schematics)
  SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
  // Accelerometer is connected to PORTE pins 18-19 via SDA and SCL (see tower schematics)
  SIM_SCGC5 |= SIM_SCGC5_PORTE_MASK;
  PORTE_PCR18 |= PORT_PCR_MUX(4) | PORT_PCR_ODE_MASK; // ALT4 in the pin MUX -> I2C0_SDA
  PORTE_PCR19 |= PORT_PCR_MUX(4) | PORT_PCR_ODE_MASK; // ALT4 in the pin MUX -> I2C0_SCL

  // Initialising I2C which controls the accelerometer
  // Using a TI2CModule struct defined in I2C.h
  TI2CModule aI2CModule;
  aI2CModule.primarySlaveAddress   = 0x1D; // address 0011101 (see accelerometer manual pg. 17) - requires pin 7 (SA0) to be high logic level
  aI2CModule.baudRate              = 100000;
  aI2CModule.readCompleteSemaphore = accelSetup->readCompleteSemaphore;

  if(!I2C_Init(&aI2CModule, accelSetup->moduleClk))
    return false;
	
  //uint8_t blank;
  //I2C_PollRead(0x0D, &blank, 1);
  
  
  // Setting fast-read bit for 8-bit data resolution - set F_READ
  // Set sampling frequency to 1.56Hz - set DR[2:0] to 1:1:1
  // Standby mode during initialisation - clear ACTIVE
  I2C_Write(ADDRESS_CTRL_REG1, 0x3A); // writing 00111010
  
  // Allow data ready interrupts - set INT_EN_DRDY - done in main via Accel_SetMode()
  I2C_Write(ADDRESS_CTRL_REG4, 0x1);
  // Route Data Ready interrupts through the INT1 pin (tied to PTB4) - set INT_CFG_DRDY
  I2C_Write(ADDRESS_CTRL_REG5, 0x1);

  // Same as first step but taking accelerometer out of standby
  I2C_Write(ADDRESS_CTRL_REG1, 0x3B); // writing 00111011


  // Saving semaphore
  DataReadySemaphore  = accelSetup->dataReadySemaphore
  
  // Setting up NVIC for PORTB see K70 manual pg 97
  // Vector=104, IRQ=88
  // NVIC non-IPR=2 IPR=22
  // Clear any pending interrupts on PORTB
  NVICICPR2 = (1 << 24); // 88mod32 = 24
  // Enable interrupts from PORTB
  NVICISER2 = (1 << 24);
  
  return true;
}



void Accel_ReadXYZ(uint8_t data[3])
{
  // array of 3 unions and to save data from the 3 most recent Accel_ReadXYZ calls
  static TAccelData accelData[3] = {{0,0,0},{0,0,0}};

  // shifts data in the array unions back (index [0] is most recent data, [2] is oldest data)
  for (uint8_t i = 0; i < 3; i++)
  {
    accelData[2].bytes[i] = accelData[1].bytes[i];
    accelData[1].bytes[i] = accelData[0].bytes[i];
  }

  // call Int or PollRead based on current mode to populate the newest data array
  if (SynchronousMode)
    I2C_IntRead(ADDRESS_OUT_X_MSB, accelData[0].bytes, 3);
  else
  {
    I2C_PollRead(ADDRESS_OUT_X_MSB, accelData[0].bytes, 3);
    // Median filters the last 3 sets of XYZ data - median filtering for IntRead is done in I2CCallback
    for (uint8_t i = 0; i < 3; i++)
      data[i] = Median_Filter3(accelData[0].bytes[i], accelData[1].bytes[i], accelData[2].bytes[i]);
  }
}



void Accel_SetMode(const TAccelMode mode)
{
  EnterCritical(); // critical section as PIT could trigger in the middle of changing
	
  // Starting standby mode (while preserving init bits)
  I2C_Write(ADDRESS_CTRL_REG1, 0x3A); // writing 00111010

  switch (mode)
  {
    case ACCEL_POLL: // disable data ready interrupts
      I2C_Write(ADDRESS_CTRL_REG4, 0x0);
      SynchronousMode = false;
      break;
	
    case ACCEL_INT: // enable data ready interrupts
      I2C_Write(ADDRESS_CTRL_REG4, 0x1);
      SynchronousMode = true;
      break;
  }

  // Ending standby mode
  I2C_Write(ADDRESS_CTRL_REG1, 0x3B); // writing 00111011
  
  ExitCritical();
}



void __attribute__ ((interrupt)) AccelDataReady_ISR(void)
{
  OS_ISREnter();
	
  // clear interrupt flag for INT1 (PTB4)
  PORTB_PCR4 &= ~PORT_PCR_ISF_MASK;

  // Allow AccelThread to run
  OS_SemaphoreSignal(DataReadySemaphore);
  
  OS_ISRExit();
}
