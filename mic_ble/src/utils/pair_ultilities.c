#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/device.h>
#include <string.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/fs/nvs.h>
#include <zephyr/logging/log.h>
#include <ctype.h>
#include "pair_ultilities.h"
#include "led.h"
//#include "es8311.h"
#include "hw_codec.h"
/******************************************************************************
 * @file    	Utilities.c
 * @author  	Phinht
 * @version 	V1.0.0
 * @date    	15/01/2014
 * @brief   	
 ******************************************************************************/

LOG_MODULE_REGISTER(pair_ultilities, CONFIG_AUDIO_SERVICES_LOG_LEVEL);

/******************************************************************************
                                   INCLUDES					    			 
 ******************************************************************************/
#include <zephyr/bluetooth/bluetooth.h>
/******************************************************************************
                                   GLOBAL VARIABLES					    			 
 ******************************************************************************/
 
static struct nvs_fs fs;



#define NVS_PARTITION		storage_partition
#define NVS_PARTITION_DEVICE	FIXED_PARTITION_DEVICE(NVS_PARTITION)
#define NVS_PARTITION_OFFSET	FIXED_PARTITION_OFFSET(NVS_PARTITION)

#define NVS_GATEWAY_PAIR_INFO 0x02
#define NVS_RESET_HANDLE 0x03 

#define NVS_TOTAL_GATEWAY_INFO 0x04

#define SHUTDOWN_VALUE  0x12345678
#define INVALID_VALUE 0xFFFFFFFF

#define MAX_DEVICE_SUPPORT  10

 /******************************************************************************
                                   GLOBAL FUNCTIONS					    			 
 ******************************************************************************/


/******************************************************************************
                                   DATA TYPE DEFINE					    			 
 ******************************************************************************/

/******************************************************************************
                                   PRIVATE VARIABLES					    			 
 ******************************************************************************/
static uint8_t m_mac_address[6];
static uint8_t m_gateway_info[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

static gateway_flash_data_t m_gateway_paired;
/******************************************************************************
                                   LOCAL FUNCTIONS					    			 
 ******************************************************************************/


/*****************************************************************************/
/**
 * @brief	:  	Get Hexa number from string
 * @param	:   
 * @retval	:
 * @author	:	Phinht
 * @created	:	15/01/2014
 * @version	:
 * @reviewer:	
 */
uint32_t GetHexNumberFromString(uint16_t BeginAddress, char* Buffer , uint32_t len)
{
    uint32_t Value = 0;
    uint16_t tmpCount = 0;
    uint32_t current_len = 0;
    tmpCount = BeginAddress;
    Value = 0;
    while(Buffer[tmpCount] && tmpCount < 1024)
    {
        if((Buffer[tmpCount] >= '0' && Buffer[tmpCount] <= '9') || (Buffer[tmpCount] >= 'A' && Buffer[tmpCount] <= 'F') || (Buffer[tmpCount] >= 'a' && Buffer[tmpCount] <= 'f'))
        {
            Value *= 16;

            if(Buffer[tmpCount] == '1') Value += 1;
            if(Buffer[tmpCount] == '2') Value += 2;
            if(Buffer[tmpCount] == '3') Value += 3;
            if(Buffer[tmpCount] == '4') Value += 4;
            if(Buffer[tmpCount] == '5') Value += 5;
            if(Buffer[tmpCount] == '6') Value += 6;
            if(Buffer[tmpCount] == '7') Value += 7;
            if(Buffer[tmpCount] == '8') Value += 8;
            if(Buffer[tmpCount] == '9') Value += 9;

            if(Buffer[tmpCount] == 'A' || Buffer[tmpCount] == 'a') Value += 10;
            if(Buffer[tmpCount] == 'B' || Buffer[tmpCount] == 'b') Value += 11;
            if(Buffer[tmpCount] == 'C' || Buffer[tmpCount] == 'c') Value += 12;
            if(Buffer[tmpCount] == 'D' || Buffer[tmpCount] == 'd') Value += 13;
            if(Buffer[tmpCount] == 'E' || Buffer[tmpCount] == 'e') Value += 14;
            if(Buffer[tmpCount] == 'F' || Buffer[tmpCount] == 'f') Value += 15;
            if(len)
            {
                if(++current_len >= len)
                {
                    break;
                }
            }
        }
        else break;

        tmpCount++;
    }

    return Value;
}
/*****************************************************************************/
/**
 * @brief	:  	Tinh check sum trong mot mang buffer
 * @param	:   
 * @retval	:
 * @author	:	Phinht
 * @created	:	15/01/2014
 * @version	:
 * @reviewer:	
 */
uint16_t CalculateCheckSum(uint8_t* Buffer, uint16_t BeginAddress, uint16_t Length)
{
    uint32_t tempChecksum = 0;
    uint16_t i = 0;

    for(i = BeginAddress; i < BeginAddress + Length; i++)
        tempChecksum += Buffer[i];
	
    return(uint16_t) (tempChecksum);
}

/***************************************************************************************************************************/
/*
 * 	Tinh check sum CRC 16 
 *
 */
#define ISO15693_PRELOADCRC16	0xFFFF 
#define ISO15693_POLYCRC16      0x8408 
#define ISO15693_MASKCRC16      0x0001
#define ISO15693_RESIDUECRC16   0xF0B8
 
uint16_t CalculateCRC16(uint8_t *DataIn, uint8_t NbByte)
{
	int16_t   i,j; 
	int32_t ResCrc = ISO15693_PRELOADCRC16;
            
	for (i = 0; i < NbByte; i++) 
	{ 
		ResCrc = ResCrc ^ DataIn[i];
		for (j = 8; j > 0; j--) 
		{
			ResCrc = (ResCrc & ISO15693_MASKCRC16) ? (ResCrc>>1) ^ ISO15693_POLYCRC16 : (ResCrc>>1); 
		}
	} 
 
	return ((~ResCrc) & 0xFFFF);
}


/*****************************************************************************/
/**
 * @brief	:  	
 * @param	:   
 * @retval	:
 * @author	:	Phinht
 * @created	:	15/01/2014
 * @version	:
 * @reviewer:	
 */
int16_t FindIndexOfChar(char CharToFind, char *BufferToFind)
{
    uint8_t tmpCount = 0;
    uint16_t DoDai = 0;

    /* Do dai du lieu */
    DoDai = strlen(BufferToFind);

    for(tmpCount = 0; tmpCount < DoDai; tmpCount++)
    {
        if(BufferToFind[tmpCount] == CharToFind) return tmpCount;
    }
    return -1;
}
/*****************************************************************************/
/**
 * @brief	:  	Copy parameters
 * @param	:   
 * @retval	:
 * @author	:	Phinht
 * @created	:	15/01/2014
 * @version	:
 * @reviewer:	
 */

uint8_t CopyParameter(char* BufferSource, char* BufferDes, char FindCharBegin, char FindCharEnd)
{
    int16_t ViTriBatDau = FindIndexOfChar(FindCharBegin, BufferSource);
    int16_t ViTriKetThuc = FindIndexOfChar(FindCharEnd, BufferSource);
    int16_t tmpCount, i = 0;

    /* Kiem tra dinh dang du lieu */
    if(ViTriBatDau == -1 || ViTriKetThuc == -1) return 0;
    if(ViTriKetThuc - ViTriBatDau <= 1) return 0;

    for(tmpCount = ViTriBatDau + 1; tmpCount < ViTriKetThuc; tmpCount++)
    {
        BufferDes[i++] = BufferSource[tmpCount];
    }

    BufferDes[i] = 0;

    return 1;
}
/*****************************************************************************/
/**
 * @brief	:  	Ham doc mot so trong chuoi bat dau tu dia chi nao do
 *				Buffer = abc124mff thi GetNumberFromString(3,Buffer) = 123
 * @param	:   
 * @retval	:
 * @author	:	Phinht
 * @created	:	15/01/2014
 * @version	:
 * @reviewer:	
 */

uint32_t GetNumberFromString(uint16_t BeginAddress, char* Buffer)
{
    uint32_t Value = 0;
    uint16_t tmpCount = 0;

    tmpCount = BeginAddress;
    Value = 0;
    while(Buffer[tmpCount] && tmpCount < 1024)
    {
        if(Buffer[tmpCount] >= '0' && Buffer[tmpCount] <= '9')
        {
            Value *= 10;
            Value += Buffer[tmpCount] - 48;
        }
        else break;

        tmpCount++;
    }

    return Value;
}


bool pair_utilities_get_mac_from_name(char *p_name, uint8_t *p_mac_address)
{
    
    char *p_temp = strstr(p_name, "MICBLE");
    if(p_temp == NULL)
    {
        return false;
    }
    p_temp = strstr(p_name, " ");
    if(p_temp == NULL)
    {
        return false;
    }
    p_temp = p_temp + 1;
    //Device name: LavalierMicro-12:34:56:78:9A:BC
    p_mac_address[0] = GetHexNumberFromString(0, p_temp, 2);
    p_temp = p_temp + 2;
    p_mac_address[1] = GetHexNumberFromString(0, p_temp, 2);
    p_temp = p_temp + 2;
    p_mac_address[2] = GetHexNumberFromString(0, p_temp, 2);
    p_temp = p_temp + 2;
    p_mac_address[3] = GetHexNumberFromString(0, p_temp, 2);
    p_temp = p_temp + 2;
    p_mac_address[4] = GetHexNumberFromString(0, p_temp, 2);
    p_temp = p_temp + 2;
    p_mac_address[5] = GetHexNumberFromString(0, p_temp, 2);
    //p_temp = p_temp + 3;
    return true;
}

int ultilities_load_mac()
{
	int ret = 0;
	bt_addr_le_t mac_addr;

	size_t mac_size;
	/*step1: Load the mac address of device*/
	bt_id_get(&mac_addr, &mac_size);
    if(ret)
    {
        LOG_ERR("Cannot load MAC address\r\n");
        return ret;
    }
    memcpy(m_mac_address, mac_addr.a.val, 6);
    return 0;
}
/*Make sure we always load mac first :D*/
uint8_t *ultilities_get_mac()
{
    return m_mac_address;
}
uint8_t pair_ultilities_flash_save_gateway_info(uint8_t *gateway_mac)
{
    int ret = 0;
    uint8_t read_buffer[6];
    ret = pair_ultilities_flash_read_gateway_info(read_buffer);
    if(ret == -1)
    {
        goto write;
    }
    else if(ret == 0)
    {
        if(memcmp(gateway_mac, read_buffer, 6) == 0)
        {
            LOG_INF("Duplicate gateway info");
            //app_led_indicator_pair_success();
            return 0;
        }
        else
        {
            goto write;
        }
    }
write:
    ret = nvs_write(&fs, NVS_GATEWAY_PAIR_INFO, gateway_mac, 6);
    if(ret >= 6)
    {
        memcpy(m_gateway_info, gateway_mac, 6);
        LOG_INF("Save gateway data to flash success\r\n");
    }
    else
    {
        LOG_WRN("Failed ti save gateway data to flash\r\n");
    } 
    return ret;  
}

uint8_t pair_ultilities_flash_read_gateway_info(uint8_t *gateway_mac)
{
    static const uint8_t invalid_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    int rc = 0;
    rc = nvs_read(&fs, NVS_GATEWAY_PAIR_INFO, gateway_mac, 6);
    if(rc > 0)
    {
        LOG_HEXDUMP_INF(gateway_mac, 6, "Found gateway data");
        if(memcmp(gateway_mac, invalid_mac, 6) == 0)
        {
            LOG_WRN("Invalid gateway mac\r\n");
            return -1;
        }
        memcpy(m_gateway_info, gateway_mac, 6);
        return 0;
    }
    else
    {
        LOG_ERR("Cannot found gateway data");
        return -1;
    }
}

uint8_t pair_ultilities_change_pair_gateway(uint8_t *target_mac)
{
    if(target_mac == NULL)
    {
        return -EINVAL;
    }
    memcpy(m_gateway_info, target_mac, 6);
}
/**/
uint8_t* pair_ultitities_get_gateway_info()
{
    return m_gateway_info;
}
bool pair_ultilities_compare_mac(uint8_t *p_in_1, uint8_t *p_in_2)
{
    for(uint8_t i = 0; i < 6; i++)
    {
        if(p_in_1[i] != p_in_2[i])
        {
            return false;
        }
    }
    return true;
}

uint8_t pair_ultilities_flash_read_volume_cfg(uint8_t *volume_config)
{
    int rc = 0;
    return 0;
}
uint8_t pair_ultilities_flash_save_volume_cfg(uint8_t *volume_config)
{
    int ret = 0;
    uint8_t read_config;
    return ret;  
}
uint8_t pair_ultilities_flash_init()
{
    struct flash_pages_info info;
    fs.flash_device = NVS_PARTITION_DEVICE;
	if (!device_is_ready(fs.flash_device)) {
		LOG_ERR("Flash device %s is not ready\n", fs.flash_device->name);
		return -1;
	}
    uint8_t rc = 0;
    fs.offset = NVS_PARTITION_OFFSET;
    rc = flash_get_page_info_by_offs(fs.flash_device, fs.offset, &info);
	if (rc) {
		LOG_ERR("Unable to get page info\n");
		return -1;
	}
	fs.sector_size = info.size;
	fs.sector_count = 3U;
 	rc = nvs_mount(&fs);
	if (rc) {
		LOG_ERR("Flash Init failed err (err %d)\n", rc);
		return -1;
	}
    return rc;
}

uint8_t ultilities_flash_set_reset_val(bool shutdown)
{
    uint32_t var = 0;
    if(shutdown)
    {
        var = SHUTDOWN_VALUE;
    }
    else
    {
        var = 0;
    }
    int ret = 0;
    uint8_t temp[sizeof(var)];
    memcpy(temp, &var, sizeof(var));
    ret = nvs_write(&fs, NVS_RESET_HANDLE, (void*)temp, sizeof(temp));
    if(ret)
    {
        
    }
    else
    {
        LOG_INF("Save reset value: %x", var);
        return ret;
    }
    return ret;
}

uint8_t ultilities_flash_get_reset(void)
{
    uint8_t var[4] = {0};
    uint32_t temp = 0;
    int rc = 0;
    rc = nvs_read(&fs, NVS_RESET_HANDLE, (void*)var, sizeof(var));
    memcpy(&temp, var, sizeof(var));
    if(rc > 0)
    {
        if(temp == SHUTDOWN_VALUE)
        {
            LOG_WRN("Last state shutdown: Read val: %x - expect val: %x", temp, SHUTDOWN_VALUE);
            return true;
        }
        else 
        {
            /* code */
            LOG_ERR("Read val: %x - expect val: %x", temp, SHUTDOWN_VALUE);
            return false;
        }
        
    }
    else
    {
        LOG_ERR("Cannot read => ");
        /*First time turn on device -> Write ramdom val*/
        LOG_ERR("Cannot save reset value");
        ultilities_flash_set_reset_val(false);
        return true;
    }
}


void pair_ultitlities_set_device_name(char* P_pre_name)
{
    if(P_pre_name == NULL)
    {
        return;
    }
    uint8_t name[48];
    memset(name, 0, sizeof(name));
    sprintf(name, "%s %02x%02x%02x%02x%02x%02x" , P_pre_name
                                   ,ultilities_get_mac()[0]
                                   ,ultilities_get_mac()[1]
                                   ,ultilities_get_mac()[2]
                                   ,ultilities_get_mac()[3]
                                   ,ultilities_get_mac()[4]
                                   ,ultilities_get_mac()[5]);
    for(uint8_t i = 0; i < strlen(name); i++)
    {
        name[i] = toupper(name[i]);
    }
    LOG_INF("Change device name to %s", name);
    bt_set_name(name);
}


uint8_t pair_ultilities_gateway_pair_get(void)
{
    uint8_t ret = 0;
    ret = pair_ultilities_gateway_pair_read(&m_gateway_paired);
    if(ret == 0)
    {
        return ret;
    }
    else
    {
        memset(m_gateway_paired.raw, 0, sizeof(m_gateway_info));
        return ret;
    }
}
gateway_flash_data_t* pair_ultilities_gateway_pair_load(void)
{
    return &m_gateway_paired;
}

uint8_t pair_ultilities_gateway_pair_read(gateway_flash_data_t *gateway_mac)
{
    int rc = 0;
    rc = nvs_read(&fs, NVS_TOTAL_GATEWAY_INFO, gateway_mac->raw, sizeof(gateway_flash_data_t));
    if(rc > 0)
    {
        LOG_INF("[NVS] Found gateway paired area");
        memcpy(m_gateway_info, gateway_mac, 6);
        return 0;
    }
    else
    {
        LOG_ERR("Cannot found gateway paired data");
        return -1;
    }
}

int pair_ultilities_gateway_pair_write(uint8_t *new_gateway , uint8_t device_type)
{
    int ret = 0;
    //gateway_flash_data_t l_read;
    ret = pair_ultilities_gateway_pair_read(&m_gateway_paired);

    if(ret == -1)
    {
        //memset()
        goto write;
    }
    else if(ret == 0)
    {
        for(uint8_t i = 0; i < MAX_DEVICE_SUPPORT; i++)
        {
            if(memcmp(m_gateway_paired.refined.gateway_data[i].gw_refined.device_mac, new_gateway, 6) == 0)
            {
                LOG_INF("Duplicate gateway info");
                return 0;
            }
        }
        /*Determine the write index*/
        if(m_gateway_paired.refined.selected_device++ >= MAX_DEVICE_SUPPORT)
        {
            m_gateway_paired.refined.selected_device = 0;
        }
        if(m_gateway_paired.refined.total_device++ >= MAX_DEVICE_SUPPORT)
        {
            m_gateway_paired.refined.total_device = MAX_DEVICE_SUPPORT;
        }
        
        goto write;
        
    }
write:
    /*Must be careful in this copy or we will be suck :v*/
    memcpy(m_gateway_paired.refined.gateway_data[m_gateway_paired.refined.selected_device - 1].gw_refined.device_mac, new_gateway, 6);
    m_gateway_paired.refined.gateway_data[m_gateway_paired.refined.selected_device - 1].gw_refined.device_type = device_type;
    ret = nvs_write(&fs, NVS_TOTAL_GATEWAY_INFO, m_gateway_paired.raw, sizeof(m_gateway_paired));
    if(ret >= sizeof(m_gateway_paired))
    {
        memcpy(m_gateway_paired.raw, m_gateway_paired.raw, sizeof(gateway_flash_data_t));
        LOG_INF("Save gateway data to index: %d\r\n", m_gateway_paired.refined.selected_device);
    }
    else
    {
        LOG_WRN("Failed ti save gateway data to flash\r\n");
    } 
    return ret; 
}