/*!
**  @file Flash.c
**
**  @brief Functions to control writing and erasing of the Flash memory module of the K70 Tower
**         These functions allocate memory for and allow the programming of said memory in the Flash module.
**         This is done via LaunchCommand calls which write or erase the necessary bytes via the FCCOB register.
**         Note that for Lab 2, only one Phrase (8 bytes) maximum may be allocated at a time.
*/
/*!
**  @addtogroup main_module main module documentation
**
**  @author Thanit Tangson & Emile Fadel
**  @{
*/
/* MODULE Flash */

#include "Flash.h"
#include "MK70F12.h"

typedef struct // Struct containing all the bytes to be written into the FTFE_FFCOB register
{
  uint8_t commandByte;
  uint8_t addressHi;
  uint8_t addressMed;
  uint8_t addressLo;
  uint8_t dataByte[8];
} TFCCOB;



/*! @brief Private function which performs an FMC command, following the flowchart on pg 813 of the K70 manual
 *  
 *  @param TFCCOB struct (defined above) containing all of the bytes for the register
 *  
 *  @return TRUE if the command was successfully carried out by the Flash module
 */
static bool LaunchCommand(TFCCOB* command)
{
  bool success = false; // Default as false if CCIF is not set
  
  if (FTFE_FSTAT & FTFE_FSTAT_CCIF_MASK) // If CCIF flag indicates the last command is completed
  {
    if ((FTFE_FSTAT & FTFE_FSTAT_FPVIOL_MASK) || (FTFE_FSTAT & FTFE_FSTAT_ACCERR_MASK))
      FTFE_FSTAT = 0x30; // If there are errors, return false later, stopping main.c functions
    else
      success = true; 
    
    // Writes the command struct to the FTFE_FCCOB registers; See K70 manual pg 796 for details
    // Note that these are big-endian (MSB goes in the lowest memory address)
    FTFE_FCCOB0 = FTFE_FCCOB0_CCOBn(command->commandByte);
    FTFE_FCCOB1 = FTFE_FCCOB1_CCOBn(command->addressHi);
    FTFE_FCCOB2 = FTFE_FCCOB2_CCOBn(command->addressMed);
    FTFE_FCCOB3 = FTFE_FCCOB3_CCOBn(command->addressLo);
  
    if (command->commandByte == 0x07) // dataBytes only needed for Program Phrase command
    {
      FTFE_FCCOB4 = FTFE_FCCOB4_CCOBn(command->dataByte[3]); // Flipped each aligned longword (4 byte) (see K70 manual page 797)
      FTFE_FCCOB5 = FTFE_FCCOB5_CCOBn(command->dataByte[2]);
      FTFE_FCCOB6 = FTFE_FCCOB6_CCOBn(command->dataByte[1]);
      FTFE_FCCOB7 = FTFE_FCCOB7_CCOBn(command->dataByte[0]);

      FTFE_FCCOB8 = FTFE_FCCOB8_CCOBn(command->dataByte[7]);
      FTFE_FCCOB9 = FTFE_FCCOB9_CCOBn(command->dataByte[6]);
      FTFE_FCCOBA = FTFE_FCCOBA_CCOBn(command->dataByte[5]);
      FTFE_FCCOBB = FTFE_FCCOBB_CCOBn(command->dataByte[4]);
    }
    
    FTFE_FSTAT = FTFE_FSTAT_CCIF_MASK;
  }
  
  return success; // If there was a violation or access error, return false
}



/*! @brief Private function to erase a sector of the flash
 *
 *  @param starting address of the sector to be erased
 *
 *  @return BOOL of the following LaunchCommand call
 */
static bool EraseSector(const uint32_t address)
{
  TFCCOB command;

  command.commandByte = 0x09; // Byte for 'Erase Sector' command

  command.addressHi  = ((address & 0xFF0000) >> 16);
  command.addressMed = ((address & 0xFF00) >> 8);
  command.addressLo  =  (address & 0xFF);

  return LaunchCommand(&command);
}



/*! @brief Private function to write a phrase into the flash
 *  
 *  @param 32-bit address for the flash phrase to be written
 *  @param 64-bit union containing the phrase to be written
 *  
 *  @return BOOL of the following LaunchCommand call
 */
static bool WritePhrase(const uint32_t address, const uint64union_t phrase)
{
  uint8_t flashPhraseOld[8] = {0}; // Array to save the current phrase data before calling EraseSector or Flash_Erase
  uint8_t flashPhraseNew[8] = {0}; // Array to hold the new phrase data before sending it to LaunchCommand
  
  for (uint8_t i = 0; i < 8; i++)
    flashPhraseOld[i] = _FB(FLASH_DATA_START + i); // Populating the 'Old' array with the flash phrase data before erasing
  
  
  // Populating the 'New' array using bit masks and shifts
  // Note that data is written from the highest index down with LSB on a higher index
  flashPhraseNew[0] = ((phrase.s.Hi & 0xFF000000) >> 24);
  flashPhraseNew[1] = ((phrase.s.Hi & 0xFF0000) >> 16);
  flashPhraseNew[2] = ((phrase.s.Hi & 0xFF00) >> 8);
  flashPhraseNew[3] =  (phrase.s.Hi & 0xFF);
  flashPhraseNew[4] = ((phrase.s.Lo & 0xFF000000) >> 24);
  flashPhraseNew[5] = ((phrase.s.Lo & 0xFF0000) >> 16);
  flashPhraseNew[6] = ((phrase.s.Lo & 0xFF00) >> 8);
  flashPhraseNew[7] =  (phrase.s.Lo & 0xFF);
  
  uint8_t dataStart = 0; // Tracks at which index above in flashPhraseNew the MSB of the data resides
  while (flashPhraseNew[dataStart] == 0)
    dataStart++;
  
  
  // Data needs to be written matching the address given in this function's parameters but by default
  // it resides in the highest indexes, so shuffle the array back until it reaches that address
  for (uint8_t i = 0; i < (dataStart - (address - FLASH_DATA_START)); i++) // For loop based on how many shuffles are needed
  {
    for (uint8_t j = 0; j < 7; j++) // For loop to actually shuffle the array
    {
      flashPhraseNew[j] = flashPhraseNew[j+1];
      if (j == 6)
        flashPhraseNew[j+1] = 0;
    }
  }
  
  
  // Declaring a command struct for use in LaunchCommand
  TFCCOB command;
  command.commandByte = 0x07; // Byte for 'Program Phrase' command
  
  // Stores the address of the start of the flash phrase using the same method as above
  // Unsure of whether FLASH_DATA_START macro can be bit masked and shifted or not, so put it in a constant
  const uint32_t PHRASE_ADDRESS = FLASH_DATA_START;
  command.addressHi  = ((PHRASE_ADDRESS & 0xFF0000) >> 16);
  command.addressMed = ((PHRASE_ADDRESS & 0xFF00) >> 8);
  command.addressLo  =  (PHRASE_ADDRESS & 0xFF);
  
  
  if (_FP(FLASH_DATA_START) == 0xFFFFFFFFFFFFFFFF) // If flash is empty (reset to 0xFFFF), then just copy in the new phrase only
  {
    for (uint8_t i = 0; i < 8; i++)
      command.dataByte[i] = flashPhraseNew[i];
  }
  else // Else, combine the old and new phrases
  {
    for (uint8_t i = 0; i < 8; i++)
      command.dataByte[i] = flashPhraseOld[i] | flashPhraseNew[i];
  }

  if (!(EraseSector(FLASH_DATA_START))) // Flash must always be bulk erased before writing
    return false;

  return LaunchCommand(&command);
}



bool Flash_Init(void)
{
  // MCG does not require user initialization, this is done automatically
  return true;
}



bool Flash_AllocateVar(volatile void** variable, const uint8_t size)
{
  // Note that memory hierarchy for the flash is Byte (8 bits) < Phrase (8 bytes) < Sector (4KiB) < Block (64KiB) < 2 Banks (128KiB each)
  // This function only needs to be called until all 8 bytes of the phrase are used, then will continually return false
  // Assigns the FIRST address available in a block (eg. if 0123 are taken size == 4, then allocate 4 and mark 567 as taken)
  
  
  uint32_t address[8]; // Array to hold all addresses in the flash phrase
  static uint8_t freeAddress[8] = {0}; // Array to track which addresses are free (0 == free)
                                       // This should deallocate (with the flash data retained) upon tower reset
  
  for (int i = 0; i < 8; i++)
  {
    address[i] = FLASH_DATA_START + i; // Allocating addresses to the array
  }
  
  
  if (size == 1)
  {
    for (int i = 0; i < 8; i++)
    {
      if (freeAddress[i] == 0)
      {
        *variable = (void*)address[i]; // Allocates the first available memory space
        freeAddress[i] = 1; // Set to indicate address is taken
        return true;
      }
    }
  }
      
  else if (size == 2)
  {
    for (int i = 0; i <= 6; i+=2)
    {
      if ((freeAddress[i] == 0) && (freeAddress[i+1] == 0))
      {
        *variable = (void*)address[i]; // Allocates the first even address followed by a free
        freeAddress[i]   = 1; // Set to indicate addresses are taken
        freeAddress[i+1] = 1;
        return true;
      }
    }
  } 
  
  else if (size == 4)
  {
    for (int i = 0; i <= 4; i+=4)
    {
      if ((freeAddress[i] == 0) && (freeAddress[i+1] == 0) && (freeAddress[i+2] == 0) && (freeAddress[i+3] == 0))
      {
        *variable = (void*)address[i]; // Allocates the first free address divisible by 4 and followed by 3 free bytes
        freeAddress[i]   = 1; // Set to indicate addresses are taken
        freeAddress[i+1] = 1;
        freeAddress[i+2] = 1;
        freeAddress[i+3] = 1;
        return true;
      }
    }
  }
  
  return false;
}



bool Flash_Write32(volatile uint32_t* const address, const uint32_t data)
{
  //addressOffset = address - FLASH_DATA_START;
  //if (addressOffset != 0 &&
  //    addressOffset != 4) // If address is not aligned to 4-byte boundary
  //  return false;
  
  // 64-bit union to store the data to send along to WritePhrase
  uint64union_t phrase;
  phrase.l    = data;
  phrase.s.Hi = 0;
  phrase.s.Lo = data;
  
  return WritePhrase((uint32_t)address, phrase);
}



bool Flash_Write16(volatile uint16_t* const address, const uint16_t data)
{
  //addressOffset = address - FLASH_DATA_START;
  //if (addressOffset != 0 &&
  //    addressOffset != 2 &&
  //    addressOffset != 4 &&
  //    addressOffset != 6) // If address is not aligned to 2-byte boundary
  //  return false;
  
  // Typecasts the 16-bit address and data to 32-bit integers to send along to Flash_Write32
  return Flash_Write32((uint32_t*)address, (uint32_t)data);
}



bool Flash_Write8(volatile uint8_t* const address, const uint8_t data)
{
  // Typecasts the 8-bit address and data to 16-bit integers to send along to Flash_Write16
  return Flash_Write16((uint16_t*)address, (uint16_t)data);
}



bool Flash_Erase(void)
{
  // Erases the sector used in Lab 2 specifically
  return EraseSector(FLASH_DATA_START);
}



/* END Flash */
/*!
** @}
*/
