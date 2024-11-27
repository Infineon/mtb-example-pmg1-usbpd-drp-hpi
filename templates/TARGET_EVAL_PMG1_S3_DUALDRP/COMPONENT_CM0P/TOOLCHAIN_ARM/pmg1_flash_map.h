/*******************************************************************************
* \file pmg1_flash_map.h
* \version 1.0
*
* This file provides project configuration macro definitions. They are used
* in the scatter files and source code files.
*
********************************************************************************
* \copyright
* Copyright 2024, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#ifndef PMG1_FLASH_MAP_H_
#define PMG1_FLASH_MAP_H_

/* Expand expression to the string */
#define CY_STR_EXPAND(foo)         #foo
#define CY_STR(foo)                CY_STR_EXPAND(foo)


/* DFU SDK parameters */
/* The user application may either update them or leave the defaults if they fit */
#define CY_PRODUCT_ID               0x01020304
#define CY_CHECKSUM_TYPE            0

/*
* The size of the section .cy_app_signature.
* 1, 2 or 4 for a checksum
* CRC-32 size: 4 bytes
* SHA1 size:   20 byte
* SHA256 size: 32 byte
* RSASSA-PKCS1-v1.5 with the 2048 bit RSA key: 256 bytes
*
* SHA1 must be used.
*/
#define CY_BOOT_SIGNATURE_SIZE      4

/* Memory region for the bootloader. */
#define CY_BOOTLOADER_ADDR          0x00000000
#define CY_BOOTLOADER_LENGTH        0x00001C00

/* Memory region ranges per applications */
/******* Single Image *********************/
/* Single image flash address and length. */
#define CY_APP_FLASH_ADDR           0x00001C00
#define CY_APP_FLASH_LENGTH         0x0003E300

/* Single Image metadata address range in flash */
#define CY_BOOT_META_FLASH_ADDR    0x0003FF00
#define CY_BOOT_META_FLASH_LENGTH  0x00000100

/************ Dual Image ********************/
/* App0 flash address and length. */
#define CY_APP0_FLASH_ADDR           0x00001C00
#define CY_APP0_FLASH_LENGTH         0x0001F0FF

/* App1 flash address and length. */
#define CY_APP1_FLASH_ADDR           0x00020D00
#define CY_APP1_FLASH_LENGTH         0x0001F0FF

/* App1 metadata address range in flash */
#define CY_BOOT_META1_FLASH_ADDR    0x0003FE00
#define CY_BOOT_META1_FLASH_LENGTH  0x00000100

/* App0 metadata address range in flash */
#define CY_BOOT_META0_FLASH_ADDR    0x0003FF00
#define CY_BOOT_META0_FLASH_LENGTH  0x00000100




#endif /* PMG1_FLASH_MAP_H_ */
