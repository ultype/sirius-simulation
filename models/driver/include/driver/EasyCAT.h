#ifndef MODELS_DRIVER_INCLUDE_DRIVER_EASYCAT_H_
#define MODELS_DRIVER_INCLUDE_DRIVER_EASYCAT_H_
#include <stddef.h>
#include <stdint.h>


//---- AB&T EasyCAT library V_1.3
//-----------------------------------------------------------------
//
// AB&T Tecnologie Informatiche - Ivrea Italy
// http://www.bausano.net
// https://www.ethercat.org/en/products/791FFAA126AD43859920EA64384AD4FD.htm
//
// This library has been tested with Arduino IDE 1.6.8
// https://www.arduino.cc



//****** configuration parameters
//*****************************************************************

#define SPI_fast_transfer  // enable fast SPI transfert (default)
#define SPI_fast_SS        // enable fast SPI chip select management (default)


//*************************************************************************************************


//---- LAN9252 registers
//--------------------------------------------------------------------------

//---- access to EtherCAT registers -------------------

#define ECAT_CSR_DATA 0x0300  // EtherCAT CSR Interface Data Register
#define ECAT_CSR_CMD 0x0304   // EtherCAT CSR Interface Command Register


//---- access to EtherCAT process RAM -----------------

#define ECAT_PRAM_RD_ADDR_LEN \
    0x0308  // EtherCAT Process RAM Read Address and Length Register
#define ECAT_PRAM_RD_CMD 0x030C  // EtherCAT Process RAM Read Command Register
#define ECAT_PRAM_WR_ADDR_LEN \
    0x0310  // EtherCAT Process RAM Write Address and Length Register
#define ECAT_PRAM_WR_CMD 0x0314  // EtherCAT Process RAM Write Command Register

#define ECAT_PRAM_RD_DATA 0x0000  // EtherCAT Process RAM Read Data FIFO
#define ECAT_PRAM_WR_DATA 0x0020  // EtherCAT Process RAM Write Data FIFO

//---- EtherCAT registers -----------------------------

#define AL_STATUS 0x0130    // AL status
#define WDOG_STATUS 0x0440  // watch dog status


//---- LAN9252 registers ------------------------------

#define HW_CFG 0x0074     // hardware configuration register
#define BYTE_TEST 0x0064  // byte order test register
#define RESET_CTL 0x01F8  // reset register
#define ID_REV 0x0050     // chip ID and revision


//---- LAN9252 flags
//------------------------------------------------------------------------------

#define ECAT_CSR_BUSY 0x80
#define PRAM_READ_BUSY 0x80
#define PRAM_READ_AVAIL 0x01
#define PRAM_WRITE_AVAIL 0x01
#define READY 0x08000000

#define DIGITAL_RST 0x00000001
#define ETHERCAT_RST 0x00000040


//---- EtherCAT flags
//-----------------------------------------------------------------------------

// EtherCAT state machine

#define ESM_INIT 0x01    // init
#define ESM_PREOP 0x02   // pre-operational
#define ESM_BOOT 0x03    // bootstrap
#define ESM_SAFEOP 0x04  // safe-operational
#define ESM_OP 0x08      // operational


//--- ESC commands
//--------------------------------------------------------------------------------

#define ESC_WRITE 0x80
#define ESC_READ 0xC0


//---- SPI
//----------------------------------------------------------------------------------------

#define COMM_SPI_READ 0x03
#define COMM_SPI_WRITE 0x02

#define DUMMY_BYTE 0xFF

#ifdef STM32F4
#define ECAT_SPI_CLK 21000000
#else
#define ECAT_SPI_CLK 25000000
#endif

//----- standard SPI chip select management
//------------------------------------------------------
#define SCS_Low_macro
// bcm2835_spi_setChipSelectPolarity(SPI_CS, LOW);

#define SCS_High_macro
// bcm2835_spi_setChipSelectPolarity(SPI_CS, HIGH);


//---- typedef
//------------------------------------------------------------------------------------

typedef union {
    uint16_t Word;
    uint8_t Byte[2];
} UWORD;

typedef union {
    uint32_t Long;
    uint16_t Word[2];
    uint8_t Byte[4];
} ULONG;

typedef union {
    uint8_t Byte[32];
    uint32_t Long[8];
} PROCBUFFER;


//-------------------------------------------------------------------------------------------------
void SPIWriteRegisterDirect(uint16_t Address, uint32_t DataOut);

unsigned long SPIReadRegisterDirect(uint16_t Address, uint8_t Len);

void SPIWriteRegisterIndirect(uint32_t DataOut, uint16_t Address);
unsigned long SPIReadRegisterIndirect(uint16_t Address,
                                      uint8_t Len);

void SPIReadProcRamFifo(char *out, size_t size);
void SPIWriteProcRamFifo(char *data, size_t size);

int Init(void);  // EasyCAT board initialization


// EtherCAT main task
// must be called cyclically by the application
void MainTask(void *MasterToSlave,
              size_t MasterToSlaveSize,
              void *SlaveToMaster,
              size_t SlaveToMasterSize);


#endif  // MODELS_DRIVER_INCLUDE_DRIVER_EASYCAT_H_
