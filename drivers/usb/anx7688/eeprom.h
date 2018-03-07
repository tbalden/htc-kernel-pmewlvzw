/******************************************************************************

Copyright (c) 2016, Analogix Semiconductor, Inc.

PKG Ver  : V2.1.11

Filename : eeprom.h

Project  : ANX7688

Created  : 28 Nov. 2016

Devices  : ANX7688

Toolchain: Android

Description:

Revision History:

******************************************************************************/

#ifndef __EEPROM_H
#define __EEPROM_H

#define AUTO_UPDATE_OCM_FW 1 // 0:disable, 1:enable
#define VERSION_LOCATION 1 // 1: 0x58:0x80~0x81 / 0: 0x50:0x15~0x16
#if AUTO_UPDATE_OCM_FW
unsigned short int burnhexauto(void);
#endif
void burnhex(void);
void readhex(void);

#endif  /* __EEPROM_H */

