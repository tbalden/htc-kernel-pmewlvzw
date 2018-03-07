/******************************************************************************

Copyright (c) 2016, Analogix Semiconductor, Inc.

PKG Ver  : V2.1.11

Filename : eeprom.c

Project  : ANX7688

Created  : 28 Nov. 2016

Devices  : ANX7688

Toolchain: Android

Description:

Revision History:

******************************************************************************/
/*
 * Copyright(c) 2014, Analogix Semiconductor. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#define EEPROM_C

#include "anx7688_driver.h"
#include "anx7688_private_interface.h"
#include "eeprom.h"

#define CURRENT_VERSION  0x2112
#define PREVIOUS_VERSION 0x2082

#define AUTO_UPDATE_RETRY 0 // 0:without retry
#define USE_PAGE_COMPARE 0
/* FW 2.1.12 has issue with page_update*/
#if (CURRENT_VERSION == 0x2112)
#define USE_PAGE_UPDATE 0
#else
#define USE_PAGE_UPDATE 1
#endif /* CURRENT_VERSION == 0x2112 */
#define USE_CRC_ERASE 1

#define REG_USBC_RESET_CTRL 0x05
#define REG_USBC_RESET_CTRL_OCM_RESET_SHIFT 4
#define REG_USBC_RAM_CTRL   0xE7

#define BITTEST(x, bitindex)            (((x) & 1<<bitindex) != 0)
#define BITTESTFALSE(x, bitindex)       (((x) & 1<<bitindex) == 0)
#define BITSET(x, bitindex)             ((x) |= 1<<bitindex)
#define BITCLEAR(x, bitindex)           ((x) &= ~(1<<bitindex))

#define HEX_LINE_SIZE 44
#define MAX_WRITE_CYCLE_TIME 5000 // 5 ms

#define EEP_PAGE_SIZE 16

#if USE_CRC_ERASE
typedef struct SegmentAddrData_tag {
    unsigned char Number;
    unsigned short int Addr[16];
    unsigned short int Size[16];
} SegmentAddrData;
#endif

char GetLineData(unsigned char *pLine, unsigned char *pByteCount, unsigned short int *pAddress, unsigned char *pRecordType, unsigned char *pData);
void SetLineData(unsigned char *pLine, unsigned char ByteCount, unsigned short int Address, unsigned char RecordType, unsigned char *pData);
void ProgHex(void);
void TRACE_ARRAY(unsigned char array[], unsigned char len);

unsigned char readhexver(unsigned char *pData, unsigned char selectHex);

static unsigned short int  BurnHexIndex=0;
static unsigned short int  BurnHexSelect=0;
static unsigned short int  BurnHexError=0;
static unsigned short int  BurnHexI2cError=0;
#if USE_PAGE_COMPARE
static unsigned short int  BurnHexSamePageCount=0;
#endif
static unsigned char g_bBurnHex=0;
unsigned char *g_CmdLineBuf;
unsigned short int BurnHexWriteCycleTime = MAX_WRITE_CYCLE_TIME;

unsigned char const OCM_FW_HEX[][HEX_LINE_SIZE]= {
#include "MI1_burn_hex.h"
};
unsigned char const OCM_FW_HEX_PAGE[][HEX_LINE_SIZE]= {
#include "MI1_page_hex.h"
};
unsigned char const OCM_FW_HEX_TEST[][HEX_LINE_SIZE]= {
#include "MI1_test_hex.h"
};

extern void MI1_power_on(void);
extern void anx7688_hardware_reset(int enable);
extern unsigned char debug_on;
extern unsigned char device_addr;


#if USE_CRC_ERASE
void ReadPage(unsigned short int Address,unsigned char *pData)
{
    unsigned char AddrBuf[2];
    unsigned char RegData;
    unsigned char DataBuf[16];

    AddrBuf[0] = Address>>8;
    AddrBuf[1] = Address&0xFF;

    if(WriteBlockReg( 0xE0, 2, AddrBuf) < 0) { // address [15:8] and [7:0]
        pr_info("NAK!\n");
        return;
    }

    WriteReg(0xE2, 0x06); // start read

    do {
        RegData=ReadReg( 0xE2 ); // wait for read done
    } while(BITTESTFALSE(RegData, 3)) ; // wait for read done
    if(ReadBlockReg( 0xD0, 16, DataBuf) < 0) { // read data
        pr_info("NAK!\n");
        return;
    }

    if(pData == 0) {
        pr_info("EEP Read %x:\n",Address);
        TRACE_ARRAY(DataBuf,16);
    } else {
        memcpy(pData,DataBuf,16);
    }

}

void WritePage(unsigned short int Address,unsigned char *pData)
{
    unsigned char WriteDataBuf[16];
    unsigned char ReadDataBuf[16];
    unsigned char AddrBuf[2];
    unsigned char RegData;

    AddrBuf[0] = Address>>8;
    AddrBuf[1] = Address&0xFF;

    if(pData == 0) {
        memset(WriteDataBuf, 0, 16);
    } else {
        memcpy(WriteDataBuf,pData,16);
    }

    if(WriteBlockReg(0xE0, 2, AddrBuf) < 0) { // address [15:8] and [7:0]
        pr_info("NAK!\n");
        return;
    }

    if(WriteBlockReg(0xD0, 16, WriteDataBuf) < 0) { // write 16 bytes
        pr_info("NAK!\n");
        return;
    }
    WriteReg(0xE2, 0x01); // start write

    do {
        RegData = ReadReg(0xE2);  // wait for write done
    } while(BITTESTFALSE(RegData, 3));
    mdelay(5); // wait for program done
    // verify
    WriteReg(0xE2, 0x06); // start read
    do {
        RegData=ReadReg( 0xE2 ); // wait for read done
    } while(BITTESTFALSE(RegData, 3)) ; // wait for read done
    if(ReadBlockReg( 0xD0, 16, ReadDataBuf) < 0) { // read data
        pr_info("NAK!\n");
        return;
    }
    if(memcmp(WriteDataBuf, ReadDataBuf, 16) != 0) {
        pr_info("Verify FAIL!\n");
        pr_info("Read back:");
        TRACE_ARRAY(ReadDataBuf, 16);
        pr_info("Expecting:");
        TRACE_ARRAY(WriteDataBuf, 16);
        return;
    }

}

//#define SEGDATA_ADDR (0x0780 + 0x10) // for TCPM
#define SEGDATA_ADDR (0x0890 + 0x10) // for Standalone
#define SEGDATA_MAX_SIZE 0x80
#define SEGDATA_FW_START 0x10
#define SEGDATA_FW_END 0x8010
unsigned char GetSegData(SegmentAddrData *pData)
{
    unsigned char DataBuf[EEP_PAGE_SIZE];
    unsigned short int Address;
    unsigned short int SegAddr;
    unsigned short int SegSize;

    pData->Number = 0;

    for(Address = SEGDATA_ADDR ; Address < SEGDATA_ADDR + SEGDATA_MAX_SIZE ; Address += EEP_PAGE_SIZE) {
        ReadPage(Address, DataBuf);
        // 1-0: Addr
        // 2: Type (0 for main FW)
        // 6-5: Size

        if(DataBuf[2] != 0)
            break;

        SegAddr = DataBuf[0];
        SegAddr <<= 8;
        SegAddr |= DataBuf[1];

        SegSize = DataBuf[5];
        SegSize <<= 8;
        SegSize |= DataBuf[6];
        SegSize <<= 4;

        if(!(SegAddr >= SEGDATA_FW_START && (SegAddr + SegSize) <= SEGDATA_FW_END))
            break;

        pData->Addr[pData->Number] = SegAddr;
        pData->Size[pData->Number] = SegSize;
        pData->Number++;

        if(DataBuf[2+8] != 0)
            break;

        SegAddr = DataBuf[0+8];
        SegAddr <<= 8;
        SegAddr |= DataBuf[1+8];

        SegSize = DataBuf[5+8];
        SegSize <<= 8;
        SegSize |= DataBuf[6+8];
        SegSize <<= 4;

        if(!(SegAddr >= SEGDATA_FW_START && (SegAddr + SegSize) <= SEGDATA_FW_END))
            break;

        pData->Addr[pData->Number] = SegAddr;
        pData->Size[pData->Number] = SegSize;
        pData->Number++;
    }

    return 0;
}

unsigned char EraseSegCrc(void)
{
    unsigned char DataBuf[EEP_PAGE_SIZE];
    SegmentAddrData SegAddrData;
    unsigned short int Address;
    unsigned char i;

    GetSegData(&SegAddrData);
    memset(DataBuf, 0, EEP_PAGE_SIZE);

    for(i=0; i<SegAddrData.Number; i++) {
        Address = SegAddrData.Addr[i] + SegAddrData.Size[i] - EEP_PAGE_SIZE;
        //printk("erase crc addr 0x%04x\n",Address);
        WritePage(Address,DataBuf);
    }
    return SegAddrData.Number;
}
#endif

#define MAX_WAIT_OCM_TIME 1000
#define EEPROM_LOAD_STA 0x12
#define EEPROM_LOAD_STA_FW 0x3F
unsigned char WaitOcmFwReady(void)
{
    unsigned short int count=0;
    unsigned char ret = 0;

    do {
        mdelay(1);
        if((ReadReg(EEPROM_LOAD_STA) & EEPROM_LOAD_STA_FW) == EEPROM_LOAD_STA_FW) {
            printk("OcmFwReady(%d)\n", count+1);
            ret = 1;
            break;
        }
    } while(count++ < MAX_WAIT_OCM_TIME);

    return ret;
}

unsigned short int burnhexauto(void)
{
    unsigned char RegData;
    unsigned char VerBuf[2];
    unsigned short int VersionHex;
    unsigned short int VersionChip;
    unsigned char OcmReady=0;
    unsigned char AutoUpdateRetry=0;

    debug_on = 1;

    pr_info("%s %s: burnhexauto.\n", LOG_TAG, __func__);

    do {
        if (atomic_read(&anx7688_power_status) == 0) {
            // power on chip first
            MI1_power_on();
        } else {
            pr_info("%s %s: Chip already power on!\n", LOG_TAG, __func__);
            anx7688_hardware_reset(0);
            mdelay(10);
            anx7688_hardware_reset(1);
            mdelay(10);
            pr_info("%s %s: Chip reset and release!\n", LOG_TAG, __func__);
        }
        OcmReady=WaitOcmFwReady();

        readhexver(VerBuf, 0);
        VersionHex = ((unsigned short int)VerBuf[0]<<8) + VerBuf[1];
        pr_info("%s %s: Version from hex: %2hx.%2hx\n", LOG_TAG, __func__, VersionHex>>8,VersionHex&0xFF);
#if VERSION_LOCATION
        device_addr= OCM_SLAVE_I2C_ADDR2;
        VerBuf[0] = ReadReg(0x80);
        VerBuf[1] = ReadReg(0x81);
        device_addr= OCM_SLAVE_I2C_ADDR1;
#else
        VerBuf[0] = ReadReg(0x15);
        VerBuf[1] = ReadReg(0x16);
#endif
        VersionChip = ((unsigned short int)VerBuf[0]<<8) + VerBuf[1];
        pr_info("%s %s: Version from chip: %2hx.%2hx\n", LOG_TAG, __func__, VersionChip>>8, VersionChip&0xFF);

        // Check load done
        if(OcmReady == 1 && VersionHex == VersionChip) {
            if (atomic_read(&anx7688_power_status) == 0) {
                anx7688_power_standby();
            }
            pr_info("%s %s: Same version not need update.\n", LOG_TAG, __func__);
            debug_on = 0;
            return VersionHex;
        } else {
            if(VersionHex == VersionChip)
                pr_info("%s %s: Update version for load fail.\n", LOG_TAG, __func__);
            else {
#if USE_PAGE_UPDATE
                if(VersionChip == PREVIOUS_VERSION)
                    BurnHexSelect = 1;
                else
                    BurnHexSelect = 0;
#endif
                pr_info("%s %s: Update version.\n", LOG_TAG, __func__);
            }
        }

        // reset OCM
        RegData = ReadReg(REG_USBC_RESET_CTRL);

        BITSET(RegData, REG_USBC_RESET_CTRL_OCM_RESET_SHIFT);

        WriteReg(REG_USBC_RESET_CTRL, RegData);
        mdelay(100); // avoid unstable

        // write password
        RegData=ReadReg(0x3F);
        BITSET(RegData, 5); // KEY0
        WriteReg(0x3F, RegData);

        RegData = ReadReg(0x44);
        BITSET(RegData, 0); // KEY1
        BITSET(RegData, 7); // KEY2
        WriteReg(0x44, RegData);

        RegData=ReadReg(0x66);
        BITSET(RegData, 3); // KEY3
        WriteReg(0x66, RegData);

        g_bBurnHex = 1;
        BurnHexIndex = 0;
        BurnHexError = 0;
        BurnHexI2cError = 0;
#if USE_PAGE_COMPARE
        BurnHexSamePageCount = 0;
#endif
#if AUTO_UPDATE_RETRY
        if(AutoUpdateRetry == 0)
            BurnHexWriteCycleTime = MAX_WRITE_CYCLE_TIME/2; // 2.5 ms for auto update
#endif
#if USE_CRC_ERASE
        EraseSegCrc();
#endif
        while(g_bBurnHex) {
            ProgHex();
        }
        BurnHexWriteCycleTime = MAX_WRITE_CYCLE_TIME; // 5 ms for default

        anx7688_hardware_reset(0);
        pr_info("%s %s: Chip reset!\n", LOG_TAG, __func__);
        mdelay(5);

        if (atomic_read(&anx7688_power_status) == 0) {
            anx7688_power_standby();
        } else {
            anx7688_hardware_reset(1);
            pr_info("%s %s: Chip reset release!\n", LOG_TAG, __func__);
            mdelay(5);
        }

        if(BurnHexError || BurnHexI2cError) {
            AutoUpdateRetry++;
            if(AutoUpdateRetry <= AUTO_UPDATE_RETRY)
                pr_info("%s %s: AutoUpdate Retry %d!\n", LOG_TAG, __func__,AutoUpdateRetry);

        } else {
            break;
        }

    } while(AutoUpdateRetry <= AUTO_UPDATE_RETRY);

#if USE_PAGE_UPDATE
    BurnHexSelect = 0;
#endif
    debug_on = 0;

    if(BurnHexError || BurnHexI2cError) {
        return VersionChip;
    }

    return VersionHex;

}

void burnhex(void)
{
    unsigned char RegData;
    unsigned char VerBuf[2];

    debug_on = 1;

    if (atomic_read(&anx7688_power_status) == 0) {
        // power on chip first
        MI1_power_on();
    } else {
        pr_info("Chip already power on!\n");
        anx7688_hardware_reset(0);
        mdelay(10);
        anx7688_hardware_reset(1);
        mdelay(10);
        pr_info("Chip reset and release!\n");
    }
    mdelay(500);
    mdelay(500);

    readhexver(VerBuf, 0);
    pr_info("Version from hex: %2hhx.%2hhx\n",VerBuf[0],VerBuf[1]);
    // VerBuf[1] = 0 ;
#if VERSION_LOCATION
    device_addr= OCM_SLAVE_I2C_ADDR2;
    VerBuf[0] = ReadReg(0x80);
    VerBuf[1] = ReadReg(0x81);
    device_addr= OCM_SLAVE_I2C_ADDR1;
#else
    VerBuf[0] = ReadReg(0x15);
    VerBuf[1] = ReadReg(0x16);
#endif
    pr_info("Version from chip: %2hhx.%2hhx\n",VerBuf[0],VerBuf[1]);

    // reset OCM
    RegData = ReadReg(REG_USBC_RESET_CTRL);

    BITSET(RegData, REG_USBC_RESET_CTRL_OCM_RESET_SHIFT);

    WriteReg(REG_USBC_RESET_CTRL, RegData);

    // write password
    RegData=ReadReg(0x3F);
    BITSET(RegData, 5); // KEY0
    WriteReg(0x3F, RegData);

    RegData = ReadReg(0x44);
    BITSET(RegData, 0); // KEY1
    BITSET(RegData, 7); // KEY2
    WriteReg(0x44, RegData);

    RegData=ReadReg(0x66);
    BITSET(RegData, 3); // KEY3
    WriteReg(0x66, RegData);

    g_bBurnHex = 1;
    BurnHexIndex = 0;
    BurnHexError = 0;
    mdelay(50); // wait power ready
#if USE_CRC_ERASE
    EraseSegCrc();
#endif
    while(g_bBurnHex) {
        ProgHex();
    }
    anx7688_power_standby();
    debug_on = 0;
}

void burntest(unsigned short int select)
{
    BurnHexSelect = select;
    burnhex();
    BurnHexSelect = 0;
}

void ProgHex(void)
{
    unsigned char WriteDataBuf[16];
    unsigned char ReadDataBuf[16];
    unsigned char AddrBuf[2];
    unsigned char ByteCount;
    unsigned short int Address;
    unsigned char RecordType;
    unsigned char RegData;

    if(BurnHexSelect == 1)
        g_CmdLineBuf = (unsigned char *)OCM_FW_HEX_PAGE[BurnHexIndex++];
    else if(BurnHexSelect == 2)
        g_CmdLineBuf = (unsigned char *)OCM_FW_HEX_TEST[BurnHexIndex++];
    else
        g_CmdLineBuf = (unsigned char *)OCM_FW_HEX[BurnHexIndex++];

    if(GetLineData(g_CmdLineBuf, &ByteCount, &Address, &RecordType, WriteDataBuf) == 0) {
        AddrBuf[0] = Address>>8;
        AddrBuf[1] = Address&0xFF;
        if(RecordType == 0) {
            // data record
            if((Address & 0x0F) == 0) {
                // address alignment
                if(Address == 0x0000) {
                    // fuse data
//                    if(WriteReg(USBC_I2C_ADDR, 0xEA, WriteDataBuf[2]) != 0) // write FUSE_WR_DATA0
//                    {
//                        TRACE("NAK!\n");
//                        return;
//                    }
//                    if(WriteReg(USBC_I2C_ADDR, 0xEB, WriteDataBuf[1]) != 0) // write FUSE_WR_DATA1
//                    {
//                        TRACE("NAK!\n");
//                        return;
//                    }
//                    if(WriteReg(USBC_I2C_ADDR, 0xEC, WriteDataBuf[0]) != 0) // write FUSE_WR_DATA2
//                    {
//                        TRACE("NAK!\n");
//                        return;
//                    }
//                    if(WriteReg(USBC_I2C_ADDR, 0xE9, 0x02) != 0) // start fuse write
//                    {
//                        TRACE("NAK!\n");
//                        return;
//                    }
//                    do
//                    {
//                        if(ReadReg(USBC_I2C_ADDR, 0xE9, &RegData) != 0) // wait for write done
//                        {
//                            TRACE("NAK!\n");
//                            return;
//                        }
//                    }while(BITTEST(RegData, 2));
//                    DELAY_US(5000); // wait for program done
                } else {
                    // key or fw
                    if(ByteCount == 16) {
                        // 16 bytes
                        if(WriteBlockReg( 0xE0, 2, AddrBuf) < 0) { // update [15:8] and [7:0]
                            pr_info("NAK!\n");
                            BurnHexI2cError++;
                            return;
                        }
#if USE_PAGE_COMPARE
                        if(BurnHexWriteCycleTime < MAX_WRITE_CYCLE_TIME) {
                            WriteReg(0xE2, 0x06); // start read
                            do {
                                RegData = ReadReg( 0xE2); // wait for read done
                            } while(BITTESTFALSE(RegData, 3)) ; // wait for read done
                            if(ReadBlockReg( 0xD0, 16, ReadDataBuf) < 0) { // read data
                                pr_info("NAK!\n");
                                BurnHexI2cError++;
                                return;
                            }
                            if(memcmp(WriteDataBuf, ReadDataBuf, 16) == 0) {
                                //pr_info("Same page!\n");
                                BurnHexSamePageCount++;
                                return;
                            }
                        }
#endif
                        if(WriteBlockReg( 0xD0, 16, WriteDataBuf) < 0) { // write 16 bytes
                            pr_info("NAK!\n");
                            BurnHexI2cError++;
                            return;
                        }
                        WriteReg(0xE2, 0x01); // start write
                        do {
                            //mdelay(1); // avoid unstable
                            RegData = ReadReg( 0xE2); // wait for write done
                        } while(BITTESTFALSE(RegData, 3));
                        //mdelay(5); // wait for program done
                        udelay(BurnHexWriteCycleTime); // wait for program done

                        // verify
                        WriteReg(0xE2, 0x06); // start read
                        do {
                            RegData = ReadReg( 0xE2); // wait for read done
                        } while(BITTESTFALSE(RegData, 3)) ; // wait for read done
                        if(ReadBlockReg( 0xD0, 16, ReadDataBuf) < 0) { // read data
                            pr_info("NAK!\n");
                            BurnHexI2cError++;
                            return;
                        }
                        if(memcmp(WriteDataBuf, ReadDataBuf, 16) != 0) {
                            pr_info("Verify FAIL!\n");
                            pr_info("Read back:");
                            TRACE_ARRAY(ReadDataBuf, 16);
                            pr_info("Expecting:");
                            TRACE_ARRAY(WriteDataBuf, 16);
                            BurnHexError++;
                            return;
                        }
                        //pr_info("BHL%hX ",Address); // test
                    } else {
                        pr_info("Byte count in Hex file MUST be 16!\n");
                    }
                }
            } else {
                pr_info("Address alignment error: Address=%04X\n", Address);
            }
        } else if(RecordType == 1) {
            // End Of File record

            g_bBurnHex = 0;
            // close password
            RegData=ReadReg(0x3F);
            BITCLEAR(RegData, 5); // KEY0
            WriteReg( 0x3F, RegData);
            RegData=ReadReg(0x44);
            BITCLEAR(RegData, 0); // KEY1
            BITCLEAR(RegData, 7); // KEY2
            WriteReg(0x44, RegData);
            RegData=ReadReg(0x66 );
            BITCLEAR(RegData, 3); // KEY3
            WriteReg(0x66, RegData);
            pr_info("Program finished.\n");
#if USE_PAGE_COMPARE
            pr_info("Update rate %d%%.\n",100*(BurnHexIndex-BurnHexSamePageCount)/BurnHexIndex);
#endif
#if USE_PAGE_UPDATE
            pr_info("Update pages %d.\n",BurnHexIndex);
#endif
        }
    } else {
        pr_info("Hex file error!\n");
    }
}

#define READ_HEX_END 0x8010
void readhex(void)
{
    unsigned short int addr;
    unsigned char AddrBuf[2];
    unsigned char LineBuf[48];
    unsigned char DataBuf[16];
    unsigned char RegData;

    addr = 0;
    do {
        AddrBuf[0] = addr>>8;
        AddrBuf[1] = addr&0xFF;

        if(addr % 256 == 0) {
            if(WriteBlockReg( 0xE0, 2, AddrBuf) < 0) { // update [15:8] and [7:0]
                pr_info("NAK!\n");
                return;
            }
        } else {
            WriteReg( 0xE1, addr % 256); // update [7:0]
        }
        WriteReg( 0xE2, 0x06); // start read
        do {
            RegData=ReadReg( 0xE2 ); // wait for read done
        } while(BITTESTFALSE(RegData, 3)) ; // wait for read done
        if(ReadBlockReg( 0xD0, 16, DataBuf) < 0) { // read data
            pr_info("NAK!\n");
            return;
        }
        SetLineData(LineBuf, 16, addr, 0, DataBuf);
        pr_info("%s\n", LineBuf);
        addr += 16;
        if(addr >= READ_HEX_END)
            break;
    } while(addr != 0);
    // End Of File record
    SetLineData(LineBuf, 0, 0x0000, 1, DataBuf);
    pr_info("%s\n", LineBuf);
    pr_info("Read all finished.\n");
}

#define VERSION_ADDR 0x1010

unsigned char readhexver(unsigned char *pData, unsigned char selectHex)
{
    unsigned char *LineBuf;
    unsigned short int Index=0;
    unsigned short int Lines=0;
    unsigned char WriteDataBuf[16];
    unsigned char ByteCount;
    unsigned short int Address;
    unsigned char RecordType;

    if(selectHex == 1)
        Lines = sizeof(OCM_FW_HEX_PAGE)/sizeof(OCM_FW_HEX_PAGE[0]);
    else
        Lines = sizeof(OCM_FW_HEX)/sizeof(OCM_FW_HEX[0]);

    while(Index < Lines) {
        if(selectHex == 1)
            LineBuf = (unsigned char *)OCM_FW_HEX_PAGE[Index++];
        else
            LineBuf = (unsigned char *)OCM_FW_HEX[Index++];

        if(GetLineData(LineBuf, &ByteCount, &Address, &RecordType, WriteDataBuf) == 0) {
            if(RecordType == 0) {
                if(Address== VERSION_ADDR /* && Address+ByteCount >= 0x80b*/) {
                    pData[0] = WriteDataBuf[VERSION_ADDR - Address]; // main version
                    pData[1] = WriteDataBuf[VERSION_ADDR + 1 - Address] ; // minor version
                    break;
                }
            } else if(RecordType == 1) {
                pr_info("Hex file end!\n");
                return 1;
            }
        } else {
            pr_info("Hex file error!\n");
            return 1;
        }
    }
    return 0;
}

unsigned short int strtoval(unsigned char *str,unsigned char n)
{
    unsigned char c;
    unsigned char i;
    unsigned short int rsult=0;

    for(i=0; i<n; i++) {
        c = str[i];

        rsult = rsult * 16;

        if (c >= '0' && c <= '9')
            rsult += (c - '0');
        else if (c >= 'A' && c <= 'F')
            rsult += (c - 'A' + 10);
        else if (c >= 'a' && c <= 'f')
            rsult += (c - 'a' + 10);

    }
    return rsult;
}

char GetLineData(unsigned char *pLine, unsigned char *pByteCount, unsigned short int *pAddress, unsigned char *pRecordType, unsigned char *pData)
{
    unsigned int ValBuf[3];
    unsigned char checksum;
    unsigned char sum;
    unsigned char i;

    if(sscanf(pLine, ":%2x%4x%2x", ValBuf, ValBuf+1, ValBuf+2) == 3) {
#if 1 // workaround for sscanf incorrect
        *pByteCount = strtoval(pLine+1,2);
        *pAddress = strtoval(pLine+1+2,4);
        *pRecordType = strtoval(pLine+1+2+4,2);
#else
        *pByteCount = ValBuf[0];
        *pAddress = ValBuf[1];
        *pRecordType = ValBuf[2];
#endif
        sum = *pByteCount + *pAddress / 256 + *pAddress % 256 + *pRecordType;

//TRACE4("L%sBHB%02hhXA%04hXT%02hhX \n",pLine,*pByteCount,*pAddress,*pRecordType); // test
        pLine += 1 + 2 + 4 + 2;
        for(i = *pByteCount; i != 0; i--) {
            if(sscanf(pLine, "%2x", ValBuf) == 1) {
#if 1 // workaround for sscanf incorrect
                *pData = strtoval(pLine,2);
#else
                *pData = ValBuf[0];
#endif
                sum += *pData;
                pData++;
                pLine += 2;
            } else {
                return -1;
            }
        }
        if(sscanf(pLine, "%2x", ValBuf) == 1) {
#if 1 // workaround for sscanf incorrect
            checksum = strtoval(pLine,2);
#else
            checksum = ValBuf[0];
#endif
            if((char)(sum + checksum) == 0) {
                //TRACE1("BHBA sum%2hhx ",sum); // test
                return 0;
            } else {
                return -1;
            }
        } else {
            return -1;
        }
    } else {
        return -1;
    }
}

void SetLineData(unsigned char *pLine, unsigned char ByteCount, unsigned short int Address, unsigned char RecordType, unsigned char *pData)
{
    unsigned char checksum;

    sprintf(pLine, ":%02hhX%04hX%02hhX", ByteCount, Address, RecordType);
    pLine += 1 + 2 + 4 + 2;
    checksum = ByteCount + Address / 256 + Address % 256 + RecordType;
    for(; ByteCount;  ByteCount--) {
        sprintf(pLine, "%02hhX", *pData);
        pLine += 2;
        checksum += *pData;
        pData++;
    }
    sprintf(pLine, "%02hhX", -checksum);
    pLine += 2;
    *pLine = '\0';
}

void TRACE_ARRAY(unsigned char array[], unsigned char len)
{
    unsigned char i;

    i = 0;
    while(1) {
        printk("%02X", array[i]);
        i++;
        if(i != len) {
            printk(" ");
        } else {
            printk("\n");
            break;
        }
    }
}



