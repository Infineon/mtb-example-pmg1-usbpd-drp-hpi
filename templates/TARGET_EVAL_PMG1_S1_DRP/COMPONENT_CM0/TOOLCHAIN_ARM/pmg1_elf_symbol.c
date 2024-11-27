/*******************************************************************************
* \file pmg1_elf_symbol.c
* \version 1.0
*
* This file provides inline assembly to add symbols in the an ELF file required
* by CyMCUElfTool to generate correct CYACD2 image.
*
* \note
* This file requires modifications of DFU specific symbols for the
* application #1.
*
********************************************************************************
* \copyright
* Copyright 2024, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/
#include "pmg1_flash_map.h"

/* Symbols that are added to the ELF file */
__asm
(
    /* DFU specific symbols */
    ".global __cy_app_id \n"
    ".global __cy_product_id \n"
    ".global __cy_boot_signature_size \n"

    ".global  __cy_checksum_type \n"
    ".global  __cy_app_verify_start \n"
    ".global  __cy_app_verify_length \n"

    ".equ __cy_app_id,               1 \n"
    ".equ __cy_boot_signature_size,  4 \n"
    ".equ __cy_product_id,           " CY_STR(CY_PRODUCT_ID) "\n"
    ".equ __cy_checksum_type,        " CY_STR(CY_CHECKSUM_TYPE) "\n"
    ".equ __cy_app_verify_start,     " CY_STR(CY_APP_FLASH_ADDR) "\n"
    ".equ __cy_app_verify_length,    " CY_STR(CY_APP_FLASH_LENGTH) "\n"

    /* flash */
    ".global __cy_memory_0_start    \n"
    ".global __cy_memory_0_length   \n"
    ".global __cy_memory_0_row_size \n"

    ".global __cy_memory_1_start    \n"
    ".global __cy_memory_1_length   \n"
    ".global __cy_memory_1_row_size \n"

    /* flash */
    ".equ __cy_memory_0_start,    " CY_STR(CY_APP_FLASH_ADDR) "\n"
    ".equ __cy_memory_0_length,   " CY_STR(CY_APP_FLASH_LENGTH) "\n"
    ".equ __cy_memory_0_row_size, 0x100 \n"

    ".equ __cy_memory_1_start,    " CY_STR(CY_BOOT_META_FLASH_ADDR) "\n"
    ".equ __cy_memory_1_length,   " CY_STR(CY_BOOT_META_FLASH_LENGTH) "\n"
    ".equ __cy_memory_1_row_size, 0x100 \n"

);
