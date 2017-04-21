
#include "driver/EasyCAT.h"
#include <string.h>
#include <unistd.h>
#include <stddef.h>
#include "driver/rpi/spi.h"
#include "driver/rpi/spi_dev.h"
#include "driver/console.h"

//---- AB&T EasyCAT library V_1.3
//-----------------------------------------------------------------
//
// AB&T Tecnologie Informatiche - Ivrea Italy
// http://www.bausano.net
// https://www.ethercat.org/en/products/791FFAA126AD43859920EA64384AD4FD.htm

//--- V_1.3 ---
// Replace delay(100) in Init() function with a wait loop
//
//
//--- V_1.2 ---
// SPI chip select is configured by the application, setting a constructor
// parameter.
// If no chip select is declared in the constructor, pin 9 will be used as
// default.
// Code cleaning.
// Comments in english.
//
//
//--- V_1.1 ---
// First official release.
// SPI chip select is configured editing the library ".h" file.

//---- EasyCAT board initialization
//---------------------------------------------------------------
#define MAX_RETRY 10000

int Init(void)
{
    ULONG TempLong;
    int i = 0;

    if (spi_configure(ECAT_SPI_DEV, ECAT_SPI_CLK,
		      SPI_BIT_MSBFIRST, SPI_MODE0) < 0) {
        dbg_printf("%s: Fail to configure spi\n", __func__);
        return -1;
    }

    TempLong.Long = 0;
    do {
        TempLong.Long = SPIReadRegisterDirect(BYTE_TEST, 8);  // read test register
	i++;
    } while (i < MAX_RETRY && TempLong.Long != 0x87654321);

    if (TempLong.Long == 0x87654321) {  // if the test register is ok
                                        // check also the READY flag
        TempLong.Long = SPIReadRegisterDirect(HW_CFG, 4);  //
        if (TempLong.Long & READY)
            return 0;  // initalization completed
    }

    dbg_printf("LAN9252 is not ready\n");
    return -1;  // initialization failed
};


//---- EtherCAT task
//------------------------------------------------------------------------------
// must be called cyclically by the application
void MainTask(void *MasterToSlave,
              size_t MasterToSlaveSize,
              void *SlaveToMaster,
              size_t SlaveToMasterSize)
{
    int WatchDog = 0;
    int Operational = 0;
    uint32_t WatchDogStatus = 0;
    uint32_t ALStatus = 0;

    if (spi_configure(ECAT_SPI_DEV, ECAT_SPI_CLK,
		      SPI_BIT_MSBFIRST, SPI_MODE0) < 0) {
        dbg_printf("%s: Fail to configure spi\n", __func__);
        return;
    }

    if (MasterToSlave == NULL || MasterToSlaveSize == 0 ||
        MasterToSlaveSize > 64) {
        dbg_printf("MasterToSlave is NULL or its size %d is invalid\n",
                   MasterToSlaveSize);
        return;
    }

    if (SlaveToMaster == NULL || SlaveToMasterSize == 0 ||
        SlaveToMasterSize > 64) {
        dbg_printf("SlaveToMaster is NULL or its size %d is invalid\n",
                   SlaveToMaster);
        return;
    }

    // set SPI parameters
    WatchDogStatus =
        SPIReadRegisterIndirect(WDOG_STATUS, 1);  // read watchdog status
    if (WatchDogStatus & 0x01)
        WatchDog = 0;  // set/reset the corrisponding flag
    else               //
        WatchDog = 1;  //

    ALStatus = SPIReadRegisterIndirect(
        AL_STATUS, 1);                // read the EtherCAT State Machine status
    if ((ALStatus & 0x0F) == ESM_OP)  // to see if we are in operational state
        Operational = 1;              //
    else                              // set/reset the corrisponding flag
        Operational = 0;              //

    //--- process data transfert ----------
    //
    if (WatchDog | !Operational) {  // Debug
        if (!Operational) {
            dbg_printf("Not operational\n");
            dbg_printf("AL Status = 0x%x\n", ALStatus);
        }
    } else {
        // otherwise transfer process data from
        // the EtherCAT core to the output buffer
        SPIReadProcRamFifo(MasterToSlave, MasterToSlaveSize);
    }

    // we always transfer process data from
    // the input buffer to the EtherCAT core
    SPIWriteProcRamFifo(SlaveToMaster, SlaveToMasterSize);
}


//---- read a directly addressable registers
//-----------------------------------------------------

unsigned long SPIReadRegisterDirect(unsigned short Address, unsigned char Len)

// Address = register to read
// Len = number of bytes to read (1,2,3,4)
//
// a long is returned but only the requested bytes
// are meaningful, starting from LsByte
{
    uint32_t Result;
    UWORD Addr;
    Addr.Word = Address;
    char buff[64];

    // SCS_Low_macro  // SPI chip select enable
    memset(buff, 0xff, sizeof(buff));
    memset(&Result, 0xff, sizeof(Result));
    buff[0] = COMM_SPI_READ;
    buff[1] = Addr.Byte[1];
    buff[2] = Addr.Byte[0];

    spi_transfern(ECAT_SPI_DEV, buff, Len + 3);

    memcpy(&Result, buff + 3, Len);

    return Result;  // return the result
}


//---- write a directly addressable registers
//----------------------------------------------------

void SPIWriteRegisterDirect(unsigned short Address, unsigned long DataOut)

// Address = register to write
// DataOut = data to write
{
    ULONG Data;
    UWORD Addr;
    char buff[64];

    Addr.Word = Address;
    Data.Long = DataOut;

    memset(buff, 0xff, sizeof(buff));

    buff[0] = COMM_SPI_WRITE;  // SPI write command
    buff[1] = Addr.Byte[1];    // address of the register
    buff[2] = Addr.Byte[0];    // to write MsByte first

    buff[3] = Data.Byte[0];  // data to write
    buff[4] = Data.Byte[1];  // LsByte first
    buff[5] = Data.Byte[2];  //
    buff[6] = Data.Byte[3];  //

    spi_transfern(ECAT_SPI_DEV, buff, 7);
}


//---- read an undirectly addressable registers
//--------------------------------------------------

unsigned long SPIReadRegisterIndirect(unsigned short Address, unsigned char Len)

// Address = register to read
// Len = number of bytes to read (1,2,3,4)
//
// a long is returned but only the requested bytes
// are meaningful, starting from LsByte
{
    ULONG TempLong;
    UWORD Addr;

    Addr.Word = Address;
    // compose the command
    TempLong.Byte[0] = Addr.Byte[0];  // address of the register
    TempLong.Byte[1] = Addr.Byte[1];  // to read, LsByte first
    TempLong.Byte[2] = Len;           // number of bytes to read
    TempLong.Byte[3] = ESC_READ;      // ESC read

    SPIWriteRegisterDirect(ECAT_CSR_CMD, TempLong.Long);  // write the command

    do {  // wait for command execution
        TempLong.Long = SPIReadRegisterDirect(ECAT_CSR_CMD, 4);  //
    }                                                            //
    while (TempLong.Byte[3] & ECAT_CSR_BUSY);                    //


    TempLong.Long = SPIReadRegisterDirect(ECAT_CSR_DATA,
                                          Len);  // read the requested register
    return TempLong.Long;                        //
}


//---- write an undirectly addressable registers
//-------------------------------------------------

void SPIWriteRegisterIndirect(unsigned long DataOut, unsigned short Address)

// Address = register to write
// DataOut = data to write
{
    ULONG TempLong;
    UWORD Addr;

    Addr.Word = Address;
    SPIWriteRegisterDirect(ECAT_CSR_DATA, DataOut);  // write the data

    // compose the command
    TempLong.Byte[0] = Addr.Byte[0];  // address of the register
    TempLong.Byte[1] = Addr.Byte[1];  // to write, LsByte first
    TempLong.Byte[2] = 4;             // we write always 4 bytes
    TempLong.Byte[3] = ESC_WRITE;     // ESC write

    SPIWriteRegisterDirect(ECAT_CSR_CMD, TempLong.Long);  // write the command

    do {  // wait for command execution
        TempLong.Long = SPIReadRegisterDirect(ECAT_CSR_CMD, 4);  //
    }                                                            //
    while (TempLong.Byte[3] & ECAT_CSR_BUSY);                    //
}


//---- read from process ram fifo
//----------------------------------------------------------------

void SPIReadProcRamFifo(char *data,
                        size_t size)  // read from the output process
// ram, through the fifo
// these are the bytes received from the EtherCAT master and
// that will be use by our application to write the outputs
{
    ULONG TempLong;

    /*
     *  0x00201000);	// we always read 32 bytes
     *				0x0020----
     *			// output process ram offset 0x----1000
     */

    char buff[64];

    if (size > 64)
        return;

    SPIWriteRegisterDirect(ECAT_PRAM_RD_ADDR_LEN, (size << 16 | 0x1000));
    SPIWriteRegisterDirect(ECAT_PRAM_RD_CMD, 0x80000000);  // start command

    do {  // wait for data to be transferred
          // from the output process ram
        TempLong.Long =
            SPIReadRegisterDirect(ECAT_PRAM_RD_CMD, 4);  // to the read fifo
    }                                                    //
    while (!(TempLong.Byte[0] & PRAM_READ_AVAIL) /*|| (TempLong.Byte[1] != 8)*/);

    memset(buff, 0xff, sizeof(buff));
    buff[0] = COMM_SPI_READ;  // SPI read command
    buff[1] = 0;
    buff[2] = 0;
    spi_transfern(ECAT_SPI_DEV, buff, size + 3);
    memcpy(data, buff + 3, size);
}


//---- write to the process ram fifo
//--------------------------------------------------------------

void SPIWriteProcRamFifo(char *data,
                         size_t size)  // write to the input process ram,
                                       // through the fifo
                                       //
// these are the bytes that we have read from the inputs of our
// application and that will be sent to the EtherCAT master
{
    ULONG TempLong;
    char buff[64];

    if (size > 64)
        return;

    SPIWriteRegisterDirect(ECAT_PRAM_WR_ADDR_LEN, (size << 16 | 0x1200));
    // we always write 32 bytes 0x0020----
    // input process ram offset 0x----1200

    SPIWriteRegisterDirect(ECAT_PRAM_WR_CMD, 0x80000000);  // start command

    do {
        // check fifo has available space
        // for data to be written
        TempLong.Long = SPIReadRegisterDirect(ECAT_PRAM_WR_CMD, 4);  //
    }                                                                //
    while (!(TempLong.Byte[0] & PRAM_WRITE_AVAIL) || (TempLong.Byte[1] < 8));


    memset(buff, 0, sizeof(buff));
    buff[0] = COMM_SPI_WRITE;  // SPI write command
    buff[1] = 0x00;            // address of the write fifo
    buff[2] = 0x20;            // MsByte first

    memcpy(buff + 3, data, size);
    spi_transfern(ECAT_SPI_DEV, buff, size + 3);
}
