#ifndef _CYPD4225_DRV_H
#define _CYPD4225_DRV_H

#include <linux/async.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/wakelock.h>
#include "usb_typec_fw_update.h"

/* CCG4 Register definitions */
#define REG_ADDR_DEVICE_MODE            0x0000
#define REG_ADDR_BOOT_MODE_REASON       0x0001
#define REG_ADDR_READ_SILICON_ID        0x0002
#define REG_ADDR_BOOT_LOADER_LAST_ROW   0x0004
#define REG_ADDR_INTR_REG               0x0006
#define REG_ADDR_JUMP_TO_BOOT           0x0007
#define REG_ADDR_RESET                  0x0008
#define REG_ADDR_ENTER_FLASHING_MODE    0x000A
#define REG_ADDR_VALIDATE_FW            0x000B
#define REG_ADDR_FLASH_ROW_READ_WRITE   0x000C
#define REG_ADDR_READ_ALL_VERSION       0x0010
#define REG_ADDR_FW2_VERSION            0x0020
#define REG_ADDR_FIRMWARE_LOCATION      0x0028
#define REG_ADDR_PDPORT_ENABLE          0x002C
#define REG_ADDR_SLEEP_CTRL             0x002D
#define REG_ADDR_BATTERY_STAT           0x002E
#define REG_ADDR_SET_APP_PRIORITY       0x002F
#define REG_ADDR_READ_CUSTOMER_INFO     0x0030
#define REG_ADDR_VENDOR_SPECIFIC        0x0040
#define REG_ADDR_RESPONSE_REGISTER      0x007E

#define REG_ADDR_FLASH_READ_WRITE_MEMORY    0x0200

#define REG_ADDR_VDM_CTRL                   0x1000
#define REG_ADDR_EFFECTIVE_SOURCE_PDO_MASK  0x1002
#define REG_ADDR_EFFECTIVE_SINK_PDO_MASK    0x1003
#define REG_ADDR_SELECT_SOURCE_PDO          0x1004
#define REG_ADDR_SELECT_SINK_PDO            0x1005
#define REG_ADDR_PD_CONTROL                 0x1006
#define REG_ADDR_PD_STATUS                  0x1008
#define REG_ADDR_TYPE_C_STATUS              0x100C
#define REG_ADDR_CURRENT_PDO                0x1010
#define REG_ADDR_CURRENT_RDO                0x1014
#define REG_ADDR_CURRENT_CABLE_VDO          0x1018
#define REG_ADDR_ALT_MODE_CMD               0x101C
#define REG_ADDR_APP_HW_CMD                 0x1020
#define REG_ADDR_EVENT_MASK                 0x1024
#define REG_ADDR_SWAP_RESPONSE              0x1028
#define REG_ADDR_ACTIVE_EC_MODES            0x1029
#define REG_ADDR_VDM_EC_CONTROL             0x102A
#define REG_ADDR_CMD_TIMEOUT                0x1030
#define REG_ADDR_PORT_INTR_STATUS           0x1034
#define REG_ADDR_DISABLE_BILLBOARD_RESET    0x1050
#define REG_ADDR_BILLBOARD_ALTMODE_STATUS   0x1052
#define REG_ADDR_BILLBOARD_OPER_MODEL       0x1054
#define REG_ADDR_PD_RESPONSE                0x1400

#define REG_ADDR_VDM_CTRL_P1                    0x2000
#define REG_ADDR_EFFECTIVE_SOURCE_PDO_MASK_P1   0x2002
#define REG_ADDR_EFFECTIVE_SINK_PDO_MASK_P1     0x2003
#define REG_ADDR_SELECT_SOURCE_PDO_P1           0x2004
#define REG_ADDR_SELECT_SINK_PDO_P1             0x2005
#define REG_ADDR_PD_CONTROL_P1                  0x2006
#define REG_ADDR_PD_STATUS_P1                   0x2008
#define REG_ADDR_TYPE_C_STATUS_P1               0x200C
#define REG_ADDR_CURRENT_PDO_P1                 0x2010
#define REG_ADDR_CURRENT_RDO_P1                 0x2014
#define REG_ADDR_CURRENT_CABLE_VDO_P1           0x2018
#define REG_ADDR_ALT_MODE_CMD_P1                0x201C
#define REG_ADDR_APP_HW_CMD_P1                  0x2020
#define REG_ADDR_EVENT_MASK_P1                  0x2024
#define REG_ADDR_SWAP_RESPONSE_P1               0x2028
#define REG_ADDR_ACTIVE_EC_MODES_P1             0x2029
#define REG_ADDR_VDM_EC_CONTROL_P1              0x202A
#define REG_ADDR_CMD_TIMEOUT_P1                 0x2030
#define REG_ADDR_PORT_INTR_STATUS_P1            0x2034
#define REG_ADDR_DISABLE_BILLBOARD_RESET_P1     0x2050
#define REG_ADDR_BILLBOARD_ALTMODE_STATUS_P1    0x2052
#define REG_ADDR_BILLBOARD_OPER_MODEL_P1        0x2054
#define REG_ADDR_PD_RESPONSE_P1                 0x2400

/* INTR_REG definitions */
#define INTR_REG_DEV_INTR   0x1
#define INTR_REG_PORT0_INTR 0x2
#define INTR_REG_PORT1_INTR 0x4

/* FLASH_ROW_READ_WRITE definitions */
#define FLASH_ROW_READ  0x00
#define FLASH_ROW_WRITE 0x01

/* PD_CONTROL definitions */
#define PD_CONTROL_SET_TYPE_C_DEFAULT_PROFILE                   0x00
#define PD_CONTROL_SET_TYPE_C_1500mA_PROFILE                    0x01
#define PD_CONTROL_SET_TYPE_C_3A_PROFILE                        0x02
#define PD_CONTROL_TRIGGER_DATA_ROLE_SWAP                       0x05
#define PD_CONTROL_TRIGGER_POWER_ROLE_SWAP                      0x06
#define PD_CONTROL_SWITCH_ON_VCONN                              0x07
#define PD_CONTROL_SWITCH_OFF_VCONN                             0x08
#define PD_CONTROL_TRIGGER_VCONN_ROLE_SWAP                      0x09
#define PD_CONTROL_RETRIEVE_PORT_PARTNER_SOURCE_CAPABILITIES    0x0A
#define PD_CONTROL_RETRIEVE_PORT_PARTNER_SINK_CAPABIITIES       0x0B
#define PD_CONTROL_SEND_GOTOMIN_MESAGE                          0x0C
#define PD_CONTROL_SEND_HARD_RESET                              0x0D
#define PD_CONTROL_SEND_SOFT_RESET                              0x0E
#define PD_CONTROL_SEND_CABLE_RESET                             0x0F
#define PD_CONTROL_EC_INITIALIZATION_COMPLETE                   0x10
#define PD_CONTROL_PORT_DISABLE                                 0x11
#define PD_CONTROL_SEND_SOFT_RESET_SOP_PRIME                    0x12
#define PD_CONTROL_SEND_SOFT_RESET_SOP_DPRIME                   0x13
#define PD_CONTROL_CHANGE_PD_PORT_PARAMETERS                    0x14
#define PD_CONTROL_ABORT_PENDING_PD_COMMAND                     0x15
#define PD_CONTROL_READ_SRC_PDO                                 0x20
#define PD_CONTROL_READ_SINK_PDO                                0x21

/* EVENT_MASK definitions */
#define EVENT_MASK_OVER_CURRENT_DETECTED                (1 << 1)
#define EVENT_MASK_OVER_VOLTAGE_DETECTED                (1 << 2)
#define EVENT_MASK_TYPE_C_PORT_CONNECT_DETECTED         (1 << 3)
#define EVENT_MASK_TYPE_C_PORT_DISCONNECT_DETECTED      (1 << 4)
#define EVENT_MASK_PD_CONTRACT_NEGOTIATION_COMPLETE     (1 << 5)
#define EVENT_MASK_PD_CONTROL_MESSAGE_RECEIVED          (1 << 6)
#define EVENT_MASK_VDM_RECEIVED                         (1 << 7)
#define EVENT_MASK_SOURCE_CAPABILITIES_MESSAGE_RECEIVED (1 << 8)
#define EVENT_MASK_SINK_CAPABILITIES_MESSAGE_RECEIVED   (1 << 9)
#define EVENT_MASK_DP_ALTERNATE_MODE_RELATED_EVENTS     (1 << 10)
#define EVENT_MASK_ERROR_AND_TIMEOUT_RELEATED_EVENTS    (1 << 11)
#define EVENT_MASK_EMCA_RELATED_EVENTS                  (1 << 12)
#define EVENT_MASK_MISCELLANEOUS_EVENTS                 (1 << 13)
#define EVENT_MASK_BILLBOARD_RELATED_EVENTS             (1 << 14)
#define EVENT_MASK_EC_POWER_CONTROL_RELATED_EVENTS      (1 << 15)

/* Events definitions */
#define RESP_TYPE_EVENT_ASYNC_MESSAGE               0x80

#define RESP_NO_RESPONSE                            0x00
#define RESP_SUCCESS                                0x02
#define RESP_FLASH_DATA_AVAILABLE                   0x03
#define RESP_INVALID_COMMAND                        0x05
#define RESP_FLASH_UPDATE_FAILED                    0x07
#define RESP_INVALID_FW                             0x08
#define RESP_INVALID_ARGUMENT                       0x09
#define RESP_NOT_SUPPORT                            0x0A
#define RESP_TRANSACTION_FAILED                     0x0C
#define RESP_PD_COMMAND_FAILED                      0x0D
#define RESP_UNDEFINED_ERROR                        0x0F
#define RESP_READ_PDO_DATA                          0x10
#define RESP_CMD_ABORTED                            0x11
#define RESP_PORT_BUSY                              0x12
#define RESP_RESET_COMPLETE                         0x80
#define RESP_MESSAGE_QUEUE_OVERFLOW                 0x81
#define RESP_OVER_CURRENT_DETECTED                  0x82
#define RESP_OVER_VOLTAGE_DETECTED                  0x83
#define RESP_TYPE_C_PORT_CONNECTED_DETECTED         0x84
#define RESP_TYPE_C_PORT_DISCONNECT_DETECTED        0x85
#define RESP_PD_CONTRACT_NEGOTIATION_COMPLETE       0x86
#define RESP_SWAP_COMPLETE                          0x87
#define RESP_PS_RDY_MESSAGE_RECEIVED                0x8A
#define RESP_GOTOMIN_MESSAGE_RECEIVED               0x8B
#define RESP_ACCEPT_MESSAGE_RECEIVED                0x8C
#define RESP_REJECT_MESSAGE_RECEIVED                0x8D
#define RESP_WAIT_MESSAGE_RECEIVED                  0x8E
#define RESP_HARD_RESET_RECEIVED                    0x8F
#define RESP_VDM_RECEIVED                           0x90
#define RESP_SOURCE_CAPABILITIES_MESSAGE_RECEIVED   0x91
#define RESP_SINK_CAPABILITIES_MESSAGE_RECEIVED     0x92
#define RESP_HARD_RESET_SENT_TO_PORT_PARTNER        0x9A
#define RESP_SOFT_RESET_SENT_TO_PORT_PARTNER        0x9B
#define RESP_CABLE_RESET_SENT_TO_EMCA               0x9C
#define RESP_SOURCE_DISABLED_STATE_ENTERED          0x9D
#define RESP_SENDER_RESPONSE_TIMER_TIMEOUT          0x9E
#define RESP_NO_VDM_RESPONSE_RECEIVED               0x9F
#define RESP_UNEXPECTED_VOLTAGE_ON_VBUS             0xA0
#define RESP_TYPE_C_ERROR_RECOVERY                  0xA1
#define RESP_EMCA_DETECTED                          0xA6
#define RESP_CABLE_DISCOVERY_FAILED                 0xA7
#define RESP_RP_CHANGE_DETECTED                     0xAA
#define RESP_BILLBOARD_CONNECT                      0xAB
#define RESP_SEND_VENDOR_DATA                       0xAC
#define RESP_ALTERNATE_MODE_EVENT                   0xB0
#define RESP_ALTERNATE_MODE_HARDWARE_EVENT          0xB1

#endif