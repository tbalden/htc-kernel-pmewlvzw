#include "cypd4225_driver.h"

#define CYPD4225_LOG_NAME "[CYPD4225]"

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) CYPD4225_LOG_NAME ": " fmt

/* to access global platform data */
static struct cypd4225_platform_data *g_pdata;
static struct cypd4225_data *g_data;
static struct cypd4225_data *platform = NULL;
atomic_t cypd4225_power_status;
atomic_t cypd4225_p1_power_status;
struct i2c_client *cypd4225_client;
struct power_supply *usb_psy = NULL;

struct cypd4225_platform_data {
    int gpio_CCG4_XRES;
    int gpio_CCG4_INTz;
};

struct cypd4225_data {
	struct cypd4225_platform_data *pdata;
    struct power_supply *usb_psy;
    struct power_supply *usb_dp_psy;
    struct power_supply *batt_psy;
    struct usb_typec_fwu_notifier *usbc_fwu_notifier;
    struct completion response_completion;
    struct completion response_pd_completion;
    struct completion response_pd_p1_completion;
    u8 waiting_response_completion;
    u8 waiting_response_pd_completion;
    u8 waiting_response_pd_p1_completion;
    u8 last_response;
    u8 last_response_pd;
    u8 last_response_pd_p1;
    struct mutex command_lock;
    struct mutex command_pd_lock;
    struct mutex command_pd_p1_lock;
    struct workqueue_struct *workqueue;
    struct delayed_work reset_complete_work;
    unsigned int online;
    unsigned int current_max;
    unsigned int health_status;
    u8 vbus_active;
};

unsigned char device_addr = 0x08;

int workable_charging_cable(void)
{
    return 1;
}
EXPORT_SYMBOL(workable_charging_cable);

static int ReadCCG4Reg(u16 RegAddr, u16 len, u8 *dat)
{
    u8 i;
    int status;
    u8 buf[I2C_SMBUS_BLOCK_MAX];
    u16 addr = RegAddr;
    u16 read = 0;
    struct i2c_msg msg[2] = {
		{
			.addr = device_addr,
            .flags = 0,
			.len = 2,
		}, {
			.addr = device_addr,
			.flags = I2C_M_RD,
			.buf = buf,
		},
	};

    while (read < len)
    {
        u8 size = ((len - read) > I2C_SMBUS_BLOCK_MAX) ? I2C_SMBUS_BLOCK_MAX : (len - read);

        msg[0].buf = (u8 *)&addr;
        msg[1].len = size;
        status = i2c_transfer(cypd4225_client->adapter, msg, 2);
        if (status < 0)
        {
            pr_err("%s: failed to read %x\n", __func__, addr);
            break;
        }

        for (i = 0; i < size; i++)
            dat[read + i] = buf[i];

        read += size;
        addr += size;
    }

    return status;
}

static int WriteCCG4Reg(u16 RegAddr, u16 len, u8 *dat)
{
    u8 i;
    int status;
    u8 buf[I2C_SMBUS_BLOCK_MAX + 2];
    u16 addr = RegAddr;
    u16 written = 0;
    struct i2c_msg msg = {
        .addr = device_addr,
        .flags = 0,
        .buf = buf,
	};

    while (written < len)
    {
        u8 size = ((len - written) > I2C_SMBUS_BLOCK_MAX) ? I2C_SMBUS_BLOCK_MAX : (len - written);

        msg.len = 2 + size;
        memcpy(buf, (u8 *)&addr, 2);
        for (i = 0; i < size; i++)
            buf[2 + i] = dat[written + i];

        status = i2c_transfer(cypd4225_client->adapter, &msg, 1);
        if (status < 0)
        {
            pr_err("%s: failed to write %x\n", __func__, addr);
            break;
        }

        written += size;
        addr += size;
    }

    return status;
}

static int cypd4225_issue_command(u16 RegAddr, u8 len, u8 *dat)
{
    u8 u8Data;

    pr_info("%s @ 0x%x\n", __func__, RegAddr);

    pr_debug("%s: wait until no DEV_INTR\n", __func__);
    do
    {
        ReadCCG4Reg(REG_ADDR_INTR_REG, 1, &u8Data);
    } while (u8Data & INTR_REG_DEV_INTR);
    pr_debug("%s: no DEV_INTR now\n", __func__);

    pr_debug("%s: wait until command_lock is available\n", __func__);
    mutex_lock(&g_data->command_lock);
    pr_debug("%s: got command_lock\n", __func__);

    g_data->waiting_response_completion = 1;

    WriteCCG4Reg(RegAddr, len, dat);

    pr_debug("%s: wait until new response\n", __func__);
    wait_for_completion(&g_data->response_completion);
    pr_debug("%s: new response received\n", __func__);

    g_data->waiting_response_completion = 0;

    u8Data = g_data->last_response;

    mutex_unlock(&g_data->command_lock);

    pr_info("%s: resp 0x%x\n", __func__, u8Data);

    return u8Data;
}

static int cypd4225_issue_command_pd(u16 RegAddr, u8 len, u8 *dat)
{
    u8 u8Data;

    pr_info("%s @ 0x%x\n", __func__, RegAddr);

    pr_debug("%s: wait until no PORT0_INTR\n", __func__);
    do
    {
        ReadCCG4Reg(REG_ADDR_INTR_REG, 1, &u8Data);
    } while (u8Data & INTR_REG_PORT0_INTR);
    pr_debug("%s: no PORT0_INTR now\n", __func__);

    pr_debug("%s: wait until command_pd_lock is available\n", __func__);
    mutex_lock(&g_data->command_pd_lock);
    pr_debug("%s: got command_pd_lock\n", __func__);

    g_data->waiting_response_pd_completion = 1;

    WriteCCG4Reg(RegAddr, len, dat);

    pr_debug("%s: wait until new response\n", __func__);
    wait_for_completion(&g_data->response_pd_completion);
    pr_debug("%s: new response received\n", __func__);

    g_data->waiting_response_pd_completion = 0;

    u8Data = g_data->last_response_pd;

    mutex_unlock(&g_data->command_pd_lock);

    pr_info("%s: resp 0x%x\n", __func__, u8Data);

    return u8Data;
}

static int cypd4225_issue_command_pd_p1(u16 RegAddr, u8 len, u8 *dat)
{
    u8 u8Data;

    pr_info("%s @ 0x%x\n", __func__, RegAddr);

    pr_debug("%s: wait until no PORT1_INTR\n", __func__);
    do
    {
        ReadCCG4Reg(REG_ADDR_INTR_REG, 1, &u8Data);
    } while (u8Data & INTR_REG_PORT1_INTR);
    pr_debug("%s: no PORT1_INTR now\n", __func__);

    pr_debug("%s: wait until command_pd_p1_lock is available\n", __func__);
    mutex_lock(&g_data->command_pd_p1_lock);
    pr_debug("%s: got command_pd_p1_lock\n", __func__);

    g_data->waiting_response_pd_p1_completion = 1;

    WriteCCG4Reg(RegAddr, len, dat);

    pr_debug("%s: wait until new response\n", __func__);
    wait_for_completion(&g_data->response_pd_p1_completion);
    pr_debug("%s: new response received\n", __func__);

    g_data->waiting_response_pd_p1_completion = 0;

    if (g_data->last_response_pd_p1 != RESP_SUCCESS)
    {
        pr_err("%s: failed to issue command, resp %x\n", __func__, g_data->last_response_pd_p1);
        mutex_unlock(&g_data->command_pd_p1_lock);
        return -1;
    }

    u8Data = g_data->last_response_pd_p1;

    mutex_unlock(&g_data->command_pd_p1_lock);

    pr_info("%s: resp 0x%x\n", __func__, u8Data);

    return u8Data;
}

static void reset_complete_work_func(struct work_struct *work)
{
    u32 u32Event_Mask;
    u8 u8Data;

    pr_info("reset_complete_work_func\n");

    u32Event_Mask = EVENT_MASK_TYPE_C_PORT_CONNECT_DETECTED | EVENT_MASK_TYPE_C_PORT_DISCONNECT_DETECTED | EVENT_MASK_PD_CONTRACT_NEGOTIATION_COMPLETE;
    cypd4225_issue_command_pd(REG_ADDR_EVENT_MASK, 4, (u8 *)&u32Event_Mask);
    cypd4225_issue_command_pd_p1(REG_ADDR_EVENT_MASK_P1, 4, (u8 *)&u32Event_Mask);

    u8Data = PD_CONTROL_EC_INITIALIZATION_COMPLETE;
    cypd4225_issue_command_pd(REG_ADDR_PD_CONTROL, 1, &u8Data);
    cypd4225_issue_command_pd_p1(REG_ADDR_PD_CONTROL_P1, 1, &u8Data);

    ReadCCG4Reg(REG_ADDR_TYPE_C_STATUS, 1, &u8Data);
    pr_info("%s: TYPE_C_STATUS 0x%x\n", __func__, u8Data);

    if (u8Data & 0x01)
    {
        atomic_set(&cypd4225_power_status, 1);
        power_supply_set_supply_type(g_data->usb_dp_psy, POWER_SUPPLY_TYPE_USB);
        power_supply_set_present(g_data->usb_dp_psy, 1);
    }
    else
    {
        atomic_set(&cypd4225_power_status, 0);
        power_supply_set_present(g_data->usb_dp_psy, 0);
    }

    ReadCCG4Reg(REG_ADDR_TYPE_C_STATUS_P1, 1, &u8Data);
    pr_info("%s: TYPE_C_STATUS_P1 0x%x\n", __func__, u8Data);

    if (u8Data & 0x01)
    {
        atomic_set(&cypd4225_p1_power_status, 1);
    }
    else
    {
        atomic_set(&cypd4225_p1_power_status, 0);
    }
}

static irqreturn_t cypd4225_CCG4_INTz_isr(int irq, void *data)
{
    u8 u8Intr;

    ReadCCG4Reg(REG_ADDR_INTR_REG, 1, &u8Intr);
    pr_debug("%s: INTR_REG 0x%x\n", __func__, u8Intr);

    if (u8Intr & INTR_REG_DEV_INTR)
    {
        u8 u8Response[2];

        ReadCCG4Reg(REG_ADDR_RESPONSE_REGISTER, sizeof(u8Response), u8Response);
        pr_debug("%s: RESPONSE_REGISTER 0x%x 0x%x\n", __func__, u8Response[0], u8Response[1]);

        g_data->last_response = u8Response[0];
        if ((g_data->last_response <= RESP_RESET_COMPLETE) && (g_data->waiting_response_completion == 1))
            complete(&g_data->response_completion);

        if (u8Response[0] == RESP_RESET_COMPLETE)
        {
            u8 u8Data;

            ReadCCG4Reg(REG_ADDR_DEVICE_MODE, 1, &u8Data);
            pr_info("%s: DEVICE_MODE 0x%x\n", __func__, u8Data);

            if ((u8Data & 0x03) != 0)
                queue_delayed_work(g_data->workqueue, &g_data->reset_complete_work, 0);
        }
    }

    if (u8Intr & INTR_REG_PORT0_INTR)
    {
        u8 u8PD_Response[4];

        ReadCCG4Reg(REG_ADDR_PD_RESPONSE, sizeof(u8PD_Response), u8PD_Response);
        pr_debug("%s: PD_RESPONSE 0x%x 0x%x 0x%x 0x%x\n", __func__, u8PD_Response[0], u8PD_Response[1], u8PD_Response[2], u8PD_Response[3]);

        g_data->last_response_pd = u8PD_Response[0];
        if (!(g_data->last_response_pd & RESP_TYPE_EVENT_ASYNC_MESSAGE) && (g_data->waiting_response_pd_completion == 1))
            complete(&g_data->response_pd_completion);

        if (u8PD_Response[0] == RESP_TYPE_C_PORT_CONNECTED_DETECTED)
        {
            u8 u8Data;

            ReadCCG4Reg(REG_ADDR_TYPE_C_STATUS, 1, &u8Data);
            pr_info("%s: TYPE_C_STATUS 0x%x\n", __func__, u8Data);

            if ((atomic_read(&cypd4225_power_status) == 0) && (u8Data & 0x01))
            {
                atomic_set(&cypd4225_power_status, 1);
                power_supply_set_supply_type(g_data->usb_dp_psy, POWER_SUPPLY_TYPE_USB);
                power_supply_set_present(g_data->usb_dp_psy, 1);
            }
        }
        else if (u8PD_Response[0] == RESP_TYPE_C_PORT_DISCONNECT_DETECTED)
        {
            u8 u8Data;

            ReadCCG4Reg(REG_ADDR_TYPE_C_STATUS, 1, &u8Data);
            pr_info("%s: TYPE_C_STATUS 0x%x\n", __func__, u8Data);

            if ((atomic_read(&cypd4225_power_status) == 1) && !(u8Data & 0x01))
            {
                atomic_set(&cypd4225_power_status, 0);
                power_supply_set_present(g_data->usb_dp_psy, 0);
            }
        }
    }

    if (u8Intr & INTR_REG_PORT1_INTR)
    {
        u8 u8PD_Response[4];

        ReadCCG4Reg(REG_ADDR_PD_RESPONSE_P1, sizeof(u8PD_Response), u8PD_Response);
        pr_debug("%s: PD_RESPONSE_P1 0x%x 0x%x 0x%x 0x%x\n", __func__, u8PD_Response[0], u8PD_Response[1], u8PD_Response[2], u8PD_Response[3]);

        g_data->last_response_pd_p1 = u8PD_Response[0];
        if (!(g_data->last_response_pd_p1 & RESP_TYPE_EVENT_ASYNC_MESSAGE) && (g_data->waiting_response_pd_p1_completion == 1))
            complete(&g_data->response_pd_p1_completion);

        if (u8PD_Response[0] == RESP_TYPE_C_PORT_CONNECTED_DETECTED)
        {
            u8 u8Data;

            ReadCCG4Reg(REG_ADDR_TYPE_C_STATUS_P1, 1, &u8Data);
            pr_info("%s: TYPE_C_STATUS_P1 0x%x\n", __func__, u8Data);

            if ((atomic_read(&cypd4225_p1_power_status) == 0) && (u8Data & 0x01))
            {
                atomic_set(&cypd4225_p1_power_status, 1);
            }
        }
        else if (u8PD_Response[0] == RESP_TYPE_C_PORT_DISCONNECT_DETECTED)
        {
            u8 u8Data;

            ReadCCG4Reg(REG_ADDR_TYPE_C_STATUS_P1, 1, &u8Data);
            pr_info("%s: TYPE_C_STATUS_P1 0x%x\n", __func__, u8Data);

            if ((atomic_read(&cypd4225_p1_power_status) == 1) && !(u8Data & 0x01))
            {
                atomic_set(&cypd4225_p1_power_status, 0);
            }
        }
    }

    if (u8Intr != 0)
        WriteCCG4Reg(REG_ADDR_INTR_REG, 1, &u8Intr);

    return IRQ_HANDLED;
}

static void cypd4225_free_gpio(struct cypd4225_data *platform)
{
    gpio_free(platform->pdata->gpio_CCG4_XRES);
    gpio_free(platform->pdata->gpio_CCG4_INTz);
}

static int cypd4225_init_gpio(struct cypd4225_data *platform)
{
    int ret = 0;

    ret = gpio_request(platform->pdata->gpio_CCG4_XRES, "cypd4225_CCG4_XRES");
    if (ret) {
        pr_err("%s : failed to request CCG4_XRES gpio %d\n", __func__, platform->pdata->gpio_CCG4_XRES);
        goto err0;
    }
    gpio_direction_output(platform->pdata->gpio_CCG4_XRES, 1);

    ret = gpio_request(platform->pdata->gpio_CCG4_INTz, "cypd4225_CCG4_INTz");
    if (ret) {
        pr_err("%s : failed to request CCG4_INTz gpio %d\n", __func__, platform->pdata->gpio_CCG4_INTz);
        goto err0;
    }
    gpio_direction_input(platform->pdata->gpio_CCG4_INTz);

    goto out;

err0:
	return -EINVAL;
out:
    return 0;
}

#ifdef CONFIG_OF
static int cypd4225_parse_dt(struct device *dev, struct cypd4225_platform_data *pdata)
{
    struct device_node *np = dev->of_node;

    pdata->gpio_CCG4_XRES = of_get_named_gpio_flags(np, "cypd4225,CCG4_XRES", 0, NULL);
    pr_info("%s: CCG4_XRES %d\n", __func__, pdata->gpio_CCG4_XRES);

    pdata->gpio_CCG4_INTz = of_get_named_gpio_flags(np, "cypd4225,CCG4_INTz", 0, NULL);
    pr_info("%s: CCG4_INTz %d\n", __func__, pdata->gpio_CCG4_INTz);

    return 0;
}
#else
static int cypd4225_parse_dt(struct device *dev, struct cypd4225_platform_data *pdata)
{
	return -ENODEV;
}
#endif

int GetHexVal(char c)
{
    if ((c >= '0') && (c <= '9'))
        return (c - '0');
    if ((c >= 'A') && (c <= 'F'))
        return (10 + (c - 'A'));
    if ((c >= 'a') && (c <= 'f'))
        return (10 + (c - 'a'));
    return 0;
}

int cypd4225_usbc_fw_update(struct usb_typec_fwu_notifier *notifier, struct firmware *fw)
{
    u8 u8Data, u8Version[16], u8Firmware_Location[4];
    u8 flashbuf[256];
    u8 firmware_mode;
    u16 flash_row_size;
    u16 fw1_start, fw2_start;
    u32 read_index;
    u8 check_row_first_time = 1;

    pr_info("%s\n", __func__);

    ReadCCG4Reg(REG_ADDR_DEVICE_MODE, 1, &u8Data);
    pr_info("%s: DEVICE_MODE 0x%x\n", __func__, u8Data);

#if 0
    if ((u8Data & 0x03) != 0)
    {
        u8Data = 0;
        if (cypd4225_issue_command(REG_ADDR_PDPORT_ENABLE, 1, &u8Data) != RESP_SUCCESS)
        {
            pr_err("%s: failed to disable ports\n", __func__);
            return -1;
        }

        u8Data = 'J';
        if (cypd4225_issue_command(REG_ADDR_JUMP_TO_BOOT, 1, &u8Data) != RESP_RESET_COMPLETE)
        {
            pr_err("%s: failed to jump to boot\n", __func__);
            return -1;
        }
    }

    ReadCCG4Reg(REG_ADDR_DEVICE_MODE, 1, &u8Data);
    pr_info("%s: DEVICE_MODE 0x%x\n", __func__, u8Data);
#endif

    firmware_mode = u8Data & 0x03;
    if ((u8Data & 0x30) == 0)
        flash_row_size = 128;
    else
        flash_row_size = 256;
    pr_info("%s: firmware_mode %x flash_row_size %d\n", __func__, firmware_mode, flash_row_size);

    ReadCCG4Reg(REG_ADDR_READ_ALL_VERSION, 16, u8Version);
    pr_info("%s: bootloader: base version 0x%x 0x%x 0x%x 0x%x application version 0x%x 0x%x 0x%x 0x%x\n", __func__, u8Version[0], u8Version[1], u8Version[2], u8Version[3], u8Version[4], u8Version[5], u8Version[6], u8Version[7]);
    pr_info("%s: FW1: base version 0x%x 0x%x 0x%x 0x%x application version 0x%x 0x%x 0x%x 0x%x\n", __func__, u8Version[8], u8Version[9], u8Version[10], u8Version[11], u8Version[12], u8Version[13], u8Version[14], u8Version[15]);

    ReadCCG4Reg(REG_ADDR_FW2_VERSION, 8, u8Version);
    pr_info("%s: FW2: base version 0x%x 0x%x 0x%x 0x%x application version 0x%x 0x%x 0x%x 0x%x\n", __func__, u8Version[0], u8Version[1], u8Version[2], u8Version[3], u8Version[4], u8Version[5], u8Version[6], u8Version[7]);

    ReadCCG4Reg(REG_ADDR_FIRMWARE_LOCATION, 4, u8Firmware_Location);
    fw1_start = u8Firmware_Location[0] | (u8Firmware_Location[1] << 8);
    fw2_start = u8Firmware_Location[2] | (u8Firmware_Location[3] << 8);
    pr_info("%s: fw1_start %d fw2_start %d\n", __func__, fw1_start, fw2_start);

    pr_info("%s: fw->size %d\n", __func__, (int)fw->size);
    read_index = 0;
    while (read_index < fw->size)
    {
        if (fw->data[read_index] == ':')
        {
            u16 row, len;
            u16 i;
            u8 u8Flash_Row_Read_Write[4];

            row = (GetHexVal(fw->data[read_index + 3]) << 12) | (GetHexVal(fw->data[read_index + 4]) << 8) | (GetHexVal(fw->data[read_index + 5]) << 4) | GetHexVal(fw->data[read_index + 6]);
            len = (GetHexVal(fw->data[read_index + 7]) << 12) | (GetHexVal(fw->data[read_index + 8]) << 8) | (GetHexVal(fw->data[read_index + 9]) << 4) | GetHexVal(fw->data[read_index + 10]);

            if (check_row_first_time == 1)
            {
                check_row_first_time = 0;

                if (firmware_mode == 1)
                {
                    if (row < fw2_start)
                    {
                        pr_err("%s: FW1 cannot be updated while FW1 is running\n", __func__);
                        return 0;
                    }

                    u8Data = 'P';
                    if (cypd4225_issue_command(REG_ADDR_ENTER_FLASHING_MODE, 1, &u8Data) != RESP_SUCCESS)
                    {
                        pr_err("%s: failed to enter flashing mode\n", __func__);
                        return -1;
                    }

                    // clear the metadata
                    memset(flashbuf, 0, flash_row_size);

                    WriteCCG4Reg(REG_ADDR_FLASH_READ_WRITE_MEMORY, len, flashbuf);

                    u8Flash_Row_Read_Write[0] = 'F';
                    u8Flash_Row_Read_Write[1] = FLASH_ROW_WRITE;
                    u8Flash_Row_Read_Write[2] = (510 & 0xFF);
                    u8Flash_Row_Read_Write[3] = (510 >> 8);
                    if (cypd4225_issue_command(REG_ADDR_FLASH_ROW_READ_WRITE, 4, u8Flash_Row_Read_Write) != RESP_SUCCESS)
                    {
                        pr_err("%s: failed to clear FW2 metadata\n", __func__);
                        return -1;
                    }
                }

                if (firmware_mode == 2)
                {
                    if (row >= fw2_start)
                    {
                        pr_err("%s: FW2 cannot be updated while FW2 is running\n", __func__);
                        return 0;
                    }

                    u8Data = 'P';
                    if (cypd4225_issue_command(REG_ADDR_ENTER_FLASHING_MODE, 1, &u8Data) != RESP_SUCCESS)
                    {
                        pr_err("%s: failed to enter flashing mode\n", __func__);
                        return -1;
                    }

                    // clear the metadata
                    memset(flashbuf, 0, flash_row_size);

                    WriteCCG4Reg(REG_ADDR_FLASH_READ_WRITE_MEMORY, len, flashbuf);

                    u8Flash_Row_Read_Write[0] = 'F';
                    u8Flash_Row_Read_Write[1] = FLASH_ROW_WRITE;
                    u8Flash_Row_Read_Write[2] = (511 & 0xFF);
                    u8Flash_Row_Read_Write[3] = (511 >> 8);
                    if (cypd4225_issue_command(REG_ADDR_FLASH_ROW_READ_WRITE, 4, u8Flash_Row_Read_Write) != RESP_SUCCESS)
                    {
                        pr_err("%s: failed to clear FW1 metadata\n", __func__);
                        return -1;
                    }
                }
            }

            for (i = 0; i < len; i++)
                flashbuf[i] = (GetHexVal(fw->data[read_index + 11 + 2 * i]) << 4) | GetHexVal(fw->data[read_index + 12 + 2 * i]);

            read_index += (11 + 2 * len);

            WriteCCG4Reg(REG_ADDR_FLASH_READ_WRITE_MEMORY, len, flashbuf);

            u8Flash_Row_Read_Write[0] = 'F';
            u8Flash_Row_Read_Write[1] = FLASH_ROW_WRITE;
            u8Flash_Row_Read_Write[2] = row & 0xFF;
            u8Flash_Row_Read_Write[3] = row >> 8;
            if (cypd4225_issue_command(REG_ADDR_FLASH_ROW_READ_WRITE, 4, u8Flash_Row_Read_Write) != RESP_SUCCESS)
            {
                pr_err("%s: failed to write data to row %d\n", __func__, row);
                return -1;
            }

#if 0
            u8Flash_Row_Read_Write[0] = 'F';
            u8Flash_Row_Read_Write[1] = FLASH_ROW_READ;
            u8Flash_Row_Read_Write[2] = row & 0xFF;
            u8Flash_Row_Read_Write[3] = row >> 8;
            if (cypd4225_issue_command(REG_ADDR_FLASH_ROW_READ_WRITE, 4, u8Flash_Row_Read_Write) != RESP_FLASH_DATA_AVAILABLE)
            {
                pr_err("%s: failed to read data from row %d\n", __func__, row);
                return -1;
            }

            ReadCCG4Reg(REG_ADDR_FLASH_READ_WRITE_MEMORY, len, readbuf);

            if (memcmp(readbuf, flashbuf, flash_row_size) != 0)
            {
                pr_err("%s: data miscompare on row %d\n", __func__, row);
                if (row < 510)
                    return -1;
            }
#endif
        }
        else
        {
            read_index++;
            continue;
        }
    }

    if (firmware_mode != 1)
    {
        u8Data = 1;
        if (cypd4225_issue_command(REG_ADDR_VALIDATE_FW, 1, &u8Data) != RESP_SUCCESS)
        {
            pr_err("%s: failed to validate FW2\n", __func__);
            return -1;
        }
    }
    if (firmware_mode != 2)
    {
        u8Data = 2;
        if (cypd4225_issue_command(REG_ADDR_VALIDATE_FW, 1, &u8Data) != RESP_SUCCESS)
        {
            pr_err("%s: failed to validate FW1\n", __func__);
            return -1;
        }
    }

    return 0;
}

int register_usb_typec_fw_update(struct usb_typec_fwu_notifier *notifier)
{
	notifier->fwupdate = cypd4225_usbc_fw_update;
	notifier->flash_timeout = 360;
	snprintf(notifier->fw_vendor, sizeof(notifier->fw_vendor) - 1, "%s", "CYPRESS");
	return register_usbc_fw_update(notifier);
}

static void usb_typec_fw_update_deinit(struct usb_typec_fwu_notifier *notifier)
{
	unregister_usbc_fw_update(notifier);
	kfree(notifier);
	notifier = NULL;
}

static int cypd4225_power_get_property_usb(struct power_supply *psy, enum power_supply_property psp, union power_supply_propval *val)
{
    pr_debug("%s: %d\n", __func__, psp);

    switch (psp) {
    case POWER_SUPPLY_PROP_HEALTH:  // 2
        val->intval = g_data->health_status;
        break;

    case POWER_SUPPLY_PROP_PRESENT: // 3
        val->intval = g_data->vbus_active;
        break;

    case POWER_SUPPLY_PROP_ONLINE:  // 4
        val->intval = g_data->online;
        break;

    case POWER_SUPPLY_PROP_CURRENT_MAX: // 16
        val->intval = g_data->current_max;
        break;

    case POWER_SUPPLY_PROP_TYPE:    // 61
        val->intval = psy->type;
        break;

    default:
        pr_debug("%s, unsupported property %d\n", __func__, psp);
        return -EINVAL;
    }

    pr_debug("%s: %d\n", __func__, val->intval);

    return 0;
}

static int cypd4225_power_set_property_usb(struct power_supply *psy, enum power_supply_property psp, const union power_supply_propval *val)
{

    pr_debug("%s: %d %d\n", __func__, psp, val->intval);

    switch (psp) {
    case POWER_SUPPLY_PROP_HEALTH:  // 2
        g_data->health_status = val->intval;
        break;

    case POWER_SUPPLY_PROP_PRESENT: // 3
        g_data->vbus_active = val->intval;
        break;

    case POWER_SUPPLY_PROP_ONLINE:  // 4
        g_data->online = val->intval;
        break;

    case POWER_SUPPLY_PROP_CURRENT_MAX: // 16
        g_data->current_max = val->intval;
        break;

    case POWER_SUPPLY_PROP_TYPE:    // 61
        psy->type = val->intval;
        break;

    case POWER_SUPPLY_PROP_DP_DM:
        break;

    default:
        pr_debug("%s, unsupported property %d\n", __func__, psp);
        return -EINVAL;
    }

    power_supply_changed(g_data->usb_psy);

    return 0;
}

static int cypd4225_property_is_writeable(struct power_supply *psy, enum power_supply_property psp)
{
    switch (psp) {
    case POWER_SUPPLY_PROP_HEALTH:
    case POWER_SUPPLY_PROP_PRESENT:
    case POWER_SUPPLY_PROP_ONLINE:
    case POWER_SUPPLY_PROP_CURRENT_MAX:
    case POWER_SUPPLY_PROP_TYPE:
        return 1;
    default:
        break;
    }

    return 0;
}

static char *cypd4225_pm_power_supplied_to[] = {
    "battery",
    "bms",
};

static enum power_supply_property cypd4225_pm_power_props_usb[] = {
    POWER_SUPPLY_PROP_HEALTH,
    POWER_SUPPLY_PROP_PRESENT,
    POWER_SUPPLY_PROP_ONLINE,
    POWER_SUPPLY_PROP_CURRENT_MAX,
    POWER_SUPPLY_PROP_TYPE,
};

static int cypd4225_i2c_probe(struct i2c_client *client,
                             const struct i2c_device_id *id)
{
    struct cypd4225_platform_data *pdata;
    int ret = 0;
    struct pinctrl *cypd4225_pinctrl;
    struct pinctrl_state *cypd4225_pinctrl_state;
    struct power_supply *usb_dp_psy;
    struct power_supply *batt_psy;

    pr_info("cypd4225_i2c_probe\n");

    if (platform == NULL)
    {
        platform = kzalloc(sizeof(struct cypd4225_data), GFP_KERNEL);
        if (!platform) {
            pr_err("%s: failed to allocate driver data\n", __func__);
            ret = -ENOMEM;
            goto exit;
        }

        platform->vbus_active = 0;

        g_data = platform;
    }

    if (usb_psy == NULL)
    {
        usb_psy = kzalloc(sizeof(struct power_supply), GFP_KERNEL);
        if (!usb_psy) {
            pr_err("%s: failed to allocate driver data\n", __func__);
            ret = -ENOMEM;
            goto exit;
        }

        platform->usb_psy = usb_psy;

        usb_psy->name = "usb";
        usb_psy->type = POWER_SUPPLY_TYPE_USB;
        usb_psy->supplied_to = cypd4225_pm_power_supplied_to;
        usb_psy->num_supplicants = ARRAY_SIZE(cypd4225_pm_power_supplied_to);
        usb_psy->properties = cypd4225_pm_power_props_usb;
        usb_psy->num_properties = ARRAY_SIZE(cypd4225_pm_power_props_usb);
        usb_psy->get_property = cypd4225_power_get_property_usb;
        usb_psy->set_property = cypd4225_power_set_property_usb;
        usb_psy->property_is_writeable = cypd4225_property_is_writeable;
        ret = power_supply_register(&client->dev, usb_psy);
        if (ret < 0) {
            pr_err("%s: power_supply_register usb failed\n", __func__);
            kfree(usb_psy);
            kfree(platform);
            goto exit;
        }
    }

    usb_dp_psy = power_supply_get_by_name("usb_dp");
    if (!usb_dp_psy) {
		pr_err("%s: USB supply not found, deferring probe\n", __func__);
		return -EPROBE_DEFER;
	}

    batt_psy = power_supply_get_by_name("battery");
    if (!batt_psy) {
        pr_err("%s: battery supply not found, deferring probe\n", __func__);
        return -EPROBE_DEFER;
    }

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_I2C_BLOCK))
    {
        pr_err("%s: cypd4225's i2c bus doesn't support I2C_FUNC_SMBUS_I2C_BLOCK\n", __func__);
        ret = -ENODEV;
        goto exit;
    }

    if (client->dev.of_node)
    {
        pdata = devm_kzalloc(&client->dev, sizeof(struct cypd4225_platform_data), GFP_KERNEL);
        if (!pdata) {
            pr_err("%s: Failed to allocate memory\n", __func__);
            ret = -ENOMEM;
            goto err0;
        }

        client->dev.platform_data = pdata;

        /* device tree parsing function call */
        ret = cypd4225_parse_dt(&client->dev, pdata);
        if (ret != 0)   /* if occurs error */
            goto err0;

        platform->pdata = pdata;
    }
    else
    {
        platform->pdata = client->dev.platform_data;
    }

    platform->waiting_response_completion = 0;
    platform->waiting_response_pd_completion = 0;
    platform->waiting_response_pd_p1_completion = 0;
    init_completion(&platform->response_completion);
    init_completion(&platform->response_pd_completion);
    init_completion(&platform->response_pd_p1_completion);
    mutex_init(&platform->command_lock);
    mutex_init(&platform->command_pd_lock);
    mutex_init(&platform->command_pd_p1_lock);

    platform->usbc_fwu_notifier = kzalloc(sizeof(struct usb_typec_fwu_notifier), GFP_KERNEL);
	if (!platform->usbc_fwu_notifier) {
		pr_err("%s: Failed to allocate memory\n", __func__);
		ret = -ENOMEM;
		goto err0;
	}

    register_usb_typec_fw_update(platform->usbc_fwu_notifier);

    /* to access global platform data */
	g_pdata = platform->pdata;
	cypd4225_client = client;
	cypd4225_client->addr = device_addr;
	i2c_set_clientdata(client, platform);

    atomic_set(&cypd4225_power_status, 0);
    atomic_set(&cypd4225_p1_power_status, 0);
    platform->usb_dp_psy = usb_dp_psy;
	platform->batt_psy = batt_psy;

    if (!platform->pdata) {
        ret = -EINVAL;
        goto err1;
    }

    ret = cypd4225_init_gpio(platform);
    if (ret) {
        pr_err("%s: failed to initialize gpio\n", __func__);
        goto err1;
    }

    INIT_DELAYED_WORK(&platform->reset_complete_work, reset_complete_work_func);

    platform->workqueue = create_singlethread_workqueue("cypd4225_work");
	if (platform->workqueue == NULL) {
		pr_err("%s: failed to create workqueue\n", __func__);
		ret = -ENOMEM;
		goto err1;
	}

    cypd4225_pinctrl = devm_pinctrl_get(&client->dev);
    if (IS_ERR_OR_NULL(cypd4225_pinctrl)) {
#ifdef CONFIG_OF
        if (of_property_read_bool(client->dev.of_node, "pinctrl-names")) {
            pr_err("%s: error encountered while getting pinctrl\n", __func__);
            ret = -EINVAL;
            goto err2;
        }
#endif
        pr_err("%s: target does not use pinctrl\n", __func__);
        cypd4225_pinctrl = NULL;
	}

    if (cypd4225_pinctrl) {
        cypd4225_pinctrl_state = pinctrl_lookup_state(cypd4225_pinctrl , "cypd4225_CCG4_XRES");
        if (IS_ERR(cypd4225_pinctrl_state))
            pr_err("%s: cypd4225_CCG4_XRES pinctrl lookup fail\n", __func__);

        ret = pinctrl_select_state(cypd4225_pinctrl , cypd4225_pinctrl_state);
        if (ret < 0)
            pr_err("%s: cypd4225_CCG4_XRES pinctrl select fail\n", __func__);

        cypd4225_pinctrl_state = pinctrl_lookup_state(cypd4225_pinctrl , "cypd4225_CCG4_INTz");
        if (IS_ERR(cypd4225_pinctrl_state))
            pr_err("%s: cypd4225_CCG4_INTz pinctrl lookup fail\n", __func__);

        ret = pinctrl_select_state(cypd4225_pinctrl , cypd4225_pinctrl_state);
        if (ret < 0)
            pr_err("%s: cypd4225_CCG4_INTz pinctrl select fail\n", __func__);
    }

    client->irq = gpio_to_irq(platform->pdata->gpio_CCG4_INTz);
    if (client->irq < 0) {
        pr_err("%s : failed to get gpio irq\n", __func__);
        goto err2;
    }

    ret = request_threaded_irq(client->irq, NULL, cypd4225_CCG4_INTz_isr, IRQF_TRIGGER_LOW | IRQF_ONESHOT, "cypd4225-CCG4-INTz", platform);
	if (ret < 0) {
		pr_err("%s : failed to request irq\n", __func__);
		goto err2;
	}

    ret = irq_set_irq_wake(client->irq, 1);
    if (ret < 0) {
        pr_err("%s : Request irq for CCG4_INTz", __func__);
        pr_err("interrupt wake set fail\n");
        goto err3;
    }

    ret = enable_irq_wake(client->irq);
    if (ret < 0) {
        pr_err("%s : Enable irq for CCG4_INTz", __func__);
        pr_err("interrupt wake enable fail\n");
        goto err3;
    }

#if 0
    gpio_set_value(pdata->gpio_CCG4_XRES, 0);
    udelay(10);
    gpio_set_value(pdata->gpio_CCG4_XRES, 1);
#endif

    pr_info("cypd4225_i2c_probe successfully end\n");
	goto exit;

err3:
	free_irq(client->irq, platform);
err2:
	cypd4225_free_gpio(platform);
    destroy_workqueue(platform->workqueue);
err1:
    usb_typec_fw_update_deinit(platform->usbc_fwu_notifier);
err0:
    cypd4225_client = NULL;
    g_data = NULL;
    g_pdata = NULL;
    kfree(platform);
exit:
    return ret;
}

static int cypd4225_i2c_remove(struct i2c_client *client)
{
    struct cypd4225_data *platform = i2c_get_clientdata(client);
    pr_info("cypd4225_i2c_remove\n");
    free_irq(client->irq, platform);
    cypd4225_free_gpio(platform);
    destroy_workqueue(platform->workqueue);
    usb_typec_fw_update_deinit(platform->usbc_fwu_notifier);
    kfree(platform);
    return 0;
}

static int cypd4225_i2c_suspend(struct i2c_client *client, pm_message_t state)
{
	return 0;
}

static int cypd4225_i2c_resume(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id cypd4225_id[] = {
	{"cypd4225", 0},
	{}
};

#ifdef CONFIG_OF
static struct of_device_id cypress_match_table[] = {
	{.compatible = "cypress,cypd4225",},
	{},
};
#endif

static struct i2c_driver cypd4225_driver = {
	.driver = {
		.name = "cypd4225",
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = cypress_match_table,
#endif
	},
	.probe = cypd4225_i2c_probe,
	.remove = cypd4225_i2c_remove,
	.suspend = cypd4225_i2c_suspend,
	.resume = cypd4225_i2c_resume,
	.id_table = cypd4225_id,
};

static void __init cypd4225_init_async(void *data, async_cookie_t cookie)
{
	int ret = 0;

	ret = i2c_add_driver(&cypd4225_driver);
	if (ret < 0)
		pr_err("%s: failed to register cypd4225 i2c driver", __func__);
}

static int __init cypd4225_init(void)
{
    pr_info("cypd4225_init\n");
	async_schedule(cypd4225_init_async, NULL);
	return 0;
}

static void __exit cypd4225_exit(void)
{
	i2c_del_driver(&cypd4225_driver);
}

module_init(cypd4225_init);
module_exit(cypd4225_exit);