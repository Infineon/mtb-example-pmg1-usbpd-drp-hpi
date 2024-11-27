/******************************************************************************
* File Name:   hpi.c
*
* Description: This is the source code for the PMG1 MCU: USBPD DRP HPI Example
*              for ModusToolbox.
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2024, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

#if CY_HPI_ENABLED
#include "cy_pdl.h"
#include "cybsp.h"
#include "config.h"
#include "hpi.h"

#include "cy_pdstack_common.h"
#include "cy_pdstack_dpm.h"
#include "cy_pdutils.h"
#include "cy_app_instrumentation.h"
#include "cy_app.h"
#include "cy_app_pdo.h"
#include "cy_app_sink.h"
#include "cy_app_source.h"
#include "cy_app_swap.h"
#include "cy_app_vdm.h"
#include "pmg1_version.h"
#include "cy_app_fault_handlers.h"
#include "mtbcfg_ezpd.h"

#include "cy_hpi.h"
#include "cy_pdl.h"
#include "cy_app_flash_config.h"
#include "cy_app_hpi.h"

#include "cy_app_system.h"
#include "cy_app_boot.h"
#include "cy_app_flash.h"

/*******************************************************************************
* Global variables.
*******************************************************************************/

/*------------------------------------------------------------------------------
 * 31-FW_ARCH        | 30-SECOND_EXT     | 29:28-BOOT_MODE                     |
 * -----------------------------------------------------------------------------
 * 27:26-BOOT_TYPE                       | 25-RESERVED      | 24-RESERVED      |
 * -----------------------------------------------------------------------------
 * 23-RESERVED       | 22-RESERVED       | 21-EPR_EN        | 20-TUNNEL_EN     |
 * -----------------------------------------------------------------------------
 * 19-AMD_EN         | 18-POWER_SNK      | 17-ICL_EN        | 16-UCSI_EN       |
 * -----------------------------------------------------------------------------
 * 15-RESERVED       | 14-RESERVED       |13-BOOT_PRI       | 12-ALTMODE_QUERY |
 * -----------------------------------------------------------------------------
 * 11-INTR_STATUS    | 10-COMMAND_QUEUE  | 9-PD_CMD         | 8-USER_REG       |
 * -----------------------------------------------------------------------------
 * 7:4-MAJOR_VERSION                                                           |
 * -----------------------------------------------------------------------------
 * 3:0-MINOR_VERSION                                                           |
 * --------------------------------------------------------------------------- */
#define HPI_VERSION         ((0 << 31) | (0 << 30) | (0 << 28) | (1 << 26) | \
                            (0 << 21)  | (0 << 20) | (0 << 19) | (0 << 18) | \
                            (0 << 17)  | (0 << 16) | (0 << 13) | (0 << 12) | \
                            (0 << 11)  | (0 << 10) | (1 << 9)  | (1 << 8)  | \
                            (2 << 4)   | (4 << 0))

/*------------------------------------------------------------------------------
 * 31:5-RESERVED                                            | 4:0-HPI_VARIANT  |
 * --------------------------------------------------------------------------- */
#define HPI_VERSION_EXT     ((4 << 0))

/* These variables are defined in the linker scripts, the values of their
 * addresses define corresponding applications start address and length.
*/
#if defined(__GNUC__) || defined(__ICCARM__)
extern uint8_t __cy_app_verify_start;
extern uint8_t __cy_app_verify_length;

#define CY_APP_VERIFY_START           ((uint32_t)&__cy_app_verify_start)
#define CY_APP_VERIFY_LENGTH          ((uint32_t)&__cy_app_verify_length)
#else
    #error "Not implemented for this compiler"
#endif  /* defined(__GNUC__) || defined(__ICCARM__) */

#define CY_METADATA_VERSION           ((uint32_t)0x01)
#define CY_METADATA_VALID_SIG         (0x4946u)
#define BL_APP_DATA_VALID_SIG         (0x4946u)

/* For the App Image-1 the bootLastRow should be the bootloader last row
 * and for the App Image-2 the bootLastRow value should be Image-1 last row. */
#if (CY_APP_TYPE == CY_APP_FW_IMAGE1)
#define CY_APP_PREV_APP_LAST_ROW      CY_APP_BOOT_LOADER_LAST_ROW
#else
#define CY_APP_PREV_APP_LAST_ROW      CY_APP_IMG1_LAST_FLASH_ROW_NUM
#endif /* (CY_APP_TYPE == CY_APP_FW_IMAGE1) */

/*******************************************************************************
* Data used by the bootloader and CyMCUElfTool.
*******************************************************************************/
/* First 128 bytes of the meta-data row of PMG1-S1 and PMG1-S3 devices are padded.*/
#if (defined(CY_DEVICE_PMG1S3) || defined(CY_DEVICE_CCG6))
CY_SECTION(".meta_padding") __USED
static const uint8_t appMetadataPadding[0x80];
#endif /* (defined(CY_DEVICE_PMG1S3) || defined(CY_DEVICE_CCG6)) */

/* Application meta-data value, it is used by the bootloader to validate FW. */
CY_SECTION(".cy_metadata") __USED
static const uint32_t appMetadata[] =
{
    CY_APP_VERIFY_START,
    CY_APP_VERIFY_LENGTH,
    (CY_APP_SYS_FWMETA_APPID_WAIT_DEF | (CY_APP_PREV_APP_LAST_ROW << 16)),
    CY_APP_VERIFY_START,
    CY_APP_VERIFY_LENGTH,
    CY_APP_TYPE,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00,
    (CY_METADATA_VERSION | (CY_METADATA_VALID_SIG << 16))
    /* CRC32 is populated by CyMcuelftool and other symbols are reserved */
};

/* Secure Image Digital signature (Populated by cymcuelftool) */
CY_SECTION(".cy_app_signature") __USED CY_ALIGN(4)
static const uint32_t appSignature = {0u};

/* This section data is used by the CyMCUElfTool. */
CY_SECTION(".cymeta") __USED
const uint8_t cy_metadata[] = {
    0x00u, 0x02u,
    (CY_SILICON_ID>>24)& 0xFF,
    (CY_SILICON_ID>>16)& 0xFF,
    ((CY_SILICON_ID>>8)& 0xFF),
    (CY_SILICON_ID)& 0xFF,
    0x00u, 0x00u,
    0x3Au, 0x0Bu, 0x14u, 0xAEu
};

/*******************************************************************************
* Flash fixed offset (0xE0) to store the firmware information.
*******************************************************************************/
/*
 * Reserve 32 bytes of space for Customer related info.
 * This will be placed at an offset of 0xC0 from the start of FW Image.
 */
CY_SECTION(".customer_region") __USED
const uint32_t gl_customer_info[8] = {0u};

/* Composite firmware stack version value.*/
CY_SECTION(".cy_base_version") __USED
const uint32_t glBaseVersion = PMG1_CE_BASE_VERSION;

/* Custom application version. */
CY_SECTION(".cy_app_version") __USED
const uint32_t glAppVersion = APP_VERSION;

/* To store the silicon ID.*/
CY_SECTION(".cy_dev_siliconid") __USED
const uint32_t glPmg1SiliconId = CY_SILICON_ID;

/* Reserved bytes for future use. */
CY_SECTION(".cy_fw_reserved") __USED
const uint32_t glReservedBuf[5] = {0};

/*******************************************************************************
* RAM fixed address offset. Used as shared memory between bootloader and application.
*******************************************************************************/
/* Stores the current boot mode status. */
#if defined(__ARMCC_VERSION)
CY_SECTION(".bss.cy_boot_run_type") __USED
#else
CY_SECTION(".cy_boot_run_type") __USED
#endif /* defined(__ARMCC_VERSION) */
volatile uint32_t cyBtldrRunType;

/* Store the validity signature for the RAM data. */
#if defined(__ARMCC_VERSION)
CY_SECTION(".bss.cy_boot_data_sig") __USED
#else
CY_SECTION(".cy_boot_data_sig") __USED
#endif /* defined(__ARMCC_VERSION) */
volatile uint16_t glBootDataSignature;

/* Variable stores the HPI slave I2C address. */
#if defined(__ARMCC_VERSION)
CY_SECTION(".bss.cy_boot_i2c_addr") __USED
#else
CY_SECTION(".cy_boot_i2c_addr") __USED
#endif /* defined(__ARMCC_VERSION) */
volatile uint8_t glHpiSlaveAddr;

cy_stc_hpi_context_t gl_HpiContext;

cy_stc_hpi_hw_config_t gl_HpiHwConfig =
{
    .scbBase = HPI_I2C_HW,
    .scbPort = HPI_I2C_SCL_PORT,
    .slaveAddr = CY_HPI_ADDR_I2C_CFG_FLOAT,
    .ecIntPort = HPI_EC_INT_PORT,
    .ecIntPin = HPI_EC_INT_PIN
};

/* CYBSP_I2C_SCB_IRQ */
const cy_stc_sysint_t HPI_SCB_IRQ_config = {
         .intrSrc = (IRQn_Type) HPI_I2C_IRQ,
         .intrPriority = 3u
 };

 void scb_0_interrupt_IRQHandler(void)
 {
     /* ISR implementation for I2C */
     Cy_Hpi_I2cInterruptHandler(&gl_HpiContext);
 }

#if (DFP_ALT_MODE_SUPP || UFP_ALT_MODE_SUPP)
uint8_t alt_mode_get_status(cy_stc_pdstack_context_t *context)
{
    CY_UNUSED_PARAMETER(context);
    return 0;
}

bool set_custom_svid(cy_stc_pdstack_context_t *context, uint16_t svid)
{
    CY_UNUSED_PARAMETER(context);
    CY_UNUSED_PARAMETER(svid);
    return true;
}

void set_alt_mode_mask(cy_stc_pdstack_context_t *context, uint16_t mask)
{
    CY_UNUSED_PARAMETER(context);
    CY_UNUSED_PARAMETER(mask);
}
#endif /* (DFP_ALT_MODE_SUPP || UFP_ALT_MODE_SUPP) */

bool i2cm_gen_i2c_tunnel_cmd(uint8_t *write_req_data, uint8_t *read_req_data)
{
    CY_UNUSED_PARAMETER(write_req_data);
    CY_UNUSED_PARAMETER(read_req_data);

    return true;
}

bool hpi_is_event_enabled(uint8_t port, uint8_t evCode, uint32_t evMask)
{
    CY_UNUSED_PARAMETER(port);
    CY_UNUSED_PARAMETER(evCode);
    CY_UNUSED_PARAMETER(evMask);

    /* Assuming all the events are enabled for now */
    return true;
}

bool eval_app_alt_mode_cmd(cy_stc_pdstack_context_t *ptrPdStackContext, uint8_t *cmd, uint8_t *data)
{
    CY_UNUSED_PARAMETER(ptrPdStackContext);
    CY_UNUSED_PARAMETER(cmd);
    CY_UNUSED_PARAMETER(data);

    return true;
}
bool eval_app_alt_hw_cmd(cy_stc_pdstack_context_t *ptrPdStackContext, uint8_t *cmd_param)
{
    CY_UNUSED_PARAMETER(ptrPdStackContext);
    CY_UNUSED_PARAMETER(cmd_param);

    return true;
}

#if (CY_HPI_VDM_QUERY_SUPPORTED)
uint8_t* vdm_get_disc_id_resp(cy_stc_pdstack_context_t *context, uint8_t *resp_len_p)
{
    CY_UNUSED_PARAMETER(context);
    CY_UNUSED_PARAMETER(resp_len_p);
    return NULL;
}

uint8_t* vdm_get_disc_svid_resp( cy_stc_pdstack_context_t *context, uint8_t *resp_len_p)
{
    CY_UNUSED_PARAMETER(context);
    CY_UNUSED_PARAMETER(resp_len_p);
    return NULL;
}
#endif /* (CY_HPI_VDM_QUERY_SUPPORTED) */

void set_bootloader_run_type(uint32_t runType)
{
    cyBtldrRunType = runType;
}

void hpi_ec_intr_write(bool value)
{
    Cy_GPIO_Write(gl_HpiHwConfig.ecIntPort, gl_HpiHwConfig.ecIntPin, value);
}

cy_en_pdstack_status_t hpi_boot_validate_fw(void *fw_addr)
{
    return (cy_en_pdstack_status_t)Cy_App_Boot_ValidateFw((cy_stc_sys_fw_metadata_t *)fw_addr);
}

cy_en_pdstack_status_t hpi_boot_validate_fw_cmd(uint8_t fw_mode)
{
    return (cy_en_pdstack_status_t)Cy_App_Boot_HandleValidateFwCmd((cy_en_sys_fw_mode_t)fw_mode);
}

uint8_t hpi_sys_get_device_mode(void)
{
    return (uint8_t)Cy_App_Sys_GetDeviceMode();
}

cy_en_pdstack_status_t hpi_flash_row_write(uint16_t row_num, uint8_t *data, void *cbk)
{
#if (CY_APP_FIRMWARE_ARCH == CY_APP_FW_ARCH_DUAL)
    return (cy_en_pdstack_status_t)Cy_App_Flash_RowWrite(row_num, data, (cy_app_flash_cbk_t)cbk);
#else
    return (cy_en_pdstack_status_t)CY_APP_STAT_INVALID_ARGUMENT;
#endif /* (CY_APP_FIRMWARE_ARCH == CY_APP_FW_ARCH_DUAL) */
}

void hpi_flash_enter_mode( bool is_enable, uint8_t mode, bool data_in_place )
{
    Cy_App_Flash_EnterMode(is_enable, (cy_en_flash_interface_t)mode, data_in_place);
}

uint16_t vbus_get_live_current(cy_stc_pdstack_context_t *ptrPdStackContext)
{
    CY_UNUSED_PARAMETER(ptrPdStackContext);
    return 0;
}

#if CCG_UCSI_ENABLE
void ucsi_notify( struct cy_stc_hpi_context *context, uint8_t port, uint16_t notification)
{
    CY_UNUSED_PARAMETER(context);
    CY_UNUSED_PARAMETER(port);
    CY_UNUSED_PARAMETER(notification);
}

void ucsi_reg_space_write_handler( struct cy_stc_hpi_context *context,
                                    uint8_t *hpi_buffer, uint16_t hpi_wr_count)
{
    CY_UNUSED_PARAMETER(context);
    CY_UNUSED_PARAMETER(hpi_buffer);
    CY_UNUSED_PARAMETER(hpi_wr_count);
}

uint8_t ucsi_handle_hpi_commands(struct cy_stc_hpi_context *context, uint8_t *cmd_param )
{
    CY_UNUSED_PARAMETER(context);
    CY_UNUSED_PARAMETER(cmd_param);
    return 0;
}

void hpi_update_ucsi_reg_space(struct cy_stc_hpi_context *context, uint8_t offset )
{
    CY_UNUSED_PARAMETER(context);
    CY_UNUSED_PARAMETER(offset);
}
#endif /* CCG_UCSI_ENABLE */

bool hpi_dev_wr_handler_ext(
        struct cy_stc_hpi_context *context,             /**< HPI context pointer. */
        uint8_t  cmdOpcode,                             /**< Offset address of the HPI device-specific register.*/
        uint8_t *cmdParam,                              /**< Pointer to HPI command buffer. */
        uint8_t  cmdLength,                             /**< Write data count in bytes. */
        cy_en_hpi_status_t *stat,                       /**< Parameter to store HPI status code. */
        cy_en_hpi_response_t *code                      /**< Parameter to store HPI response and event codes. */
        )                                              /**< Function handles the writes to the HPI Device register space. */
{
    bool isHandled = false;
    if ((cy_en_hpi_dev_reg_address_t)cmdOpcode == CY_HPI_DEV_REG_READ_DIE_INFO)
    {
        if (cmdParam[0] == (uint8_t)'I')
        {
            uint8_t *ptr = &context->regSpace.flashMem[0];
            uint64_t dieInfo = 0;

            /* Clear the flash memory buffer. */
            (void)memset (ptr, 0, 32);
            /* Get 8-byte die related Information. */
            dieInfo = CALL_MAP(Cy_SysLib_GetUniqueId)();
            /* Copy dieInfo data from HPI flash memory ptr[0] to ptr[7] */
            Cy_PdUtils_MemCopy(ptr, (const  uint8_t *)(&dieInfo), sizeof(dieInfo));
            /* ((PID3 from ROMTABLE & 0xF0) >> 4) */
            ptr[8] = (uint8_t)((ROMTABLE->pid3 & 0xF0U) >> 4);
            /*  ((PID2 from ROMTABLE & 0xF0) >> 4) */
            ptr[9] = (uint8_t)((ROMTABLE->pid2 & 0xF0U) >> 4);
            /* Silicon Id[7:0] */
            ptr[10] = (uint8_t)(SFLASH->SILICON_ID & 0xFFU);
            /* Silicon Id[15:8] */
            ptr[11] = (uint8_t)((SFLASH->SILICON_ID >> 8U) & 0xFFU);

            *code = CY_HPI_RESPONSE_SUCCESS;
        }
        else
        {
            *code = CY_HPI_RESPONSE_INVALID_ARGUMENT;
        }

        isHandled = true;
    }

    return isHandled;
}

static cy_stc_hpi_app_cbk_t hpiAppCbk =
{
    .ec_intr_write = hpi_ec_intr_write,
    .sys_get_custom_info_addr = Cy_App_Sys_GetCustomInfoAddr,
#if (CY_HPI_PD_ENABLE)
    .hpi_is_event_enabled = hpi_is_event_enabled,
    .app_disable_pd_port = Cy_App_DisablePdPort,
#if (DFP_ALT_MODE_SUPP || UFP_ALT_MODE_SUPP)
    .alt_mode_get_status = alt_mode_get_status,
#endif /* (DFP_ALT_MODE_SUPP || UFP_ALT_MODE_SUPP) */
    .app_update_sys_pwr_state = NULL,
#if (CY_HPI_PD_CMD_ENABLE)
#if (DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP)
    .set_custom_svid = set_custom_svid,
    .set_alt_mode_mask = set_alt_mode_mask,
    .app_vdm_layer_reset = NULL,
    .eval_app_alt_mode_cmd = eval_app_alt_mode_cmd,
    .eval_app_alt_hw_cmd = eval_app_alt_hw_cmd,
#endif /* ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP)) */
    .set_custom_host_cap_control = NULL,
    .app_set_custom_pid = NULL,
    .i2cm_gen_i2c_tunnel_cmd = NULL,
    .switch_vddd_supply = NULL,
#if (CY_HPI_VDM_QUERY_SUPPORTED)
    .vdm_get_disc_id_resp = vdm_get_disc_id_resp,
    .vdm_get_disc_svid_resp = vdm_get_disc_svid_resp,
#endif /* (CY_HPI_VDM_QUERY_SUPPORTED) */
    .app_update_bc_src_support = NULL,
#if (CY_HPI_VBUS_C_CTRL_ENABLE)
    .psnk_set_vbus_cfet_on_ctrl = Cy_App_Sink_VbusCFetOnCtrl,
#endif /* (CY_HPI_VBUS_C_CTRL_ENABLE) */
    .hpi_vconn_enable = Cy_App_VconnEnable,
    .hpi_vconn_disable = Cy_App_VconnDisable,
#if (CY_HPI_RW_PD_RESP_MSG_DATA)
    .hpi_rw_pd_resp_data = Cy_App_Hpi_HandlePdRespDataRw,
#endif /* (CY_HPI_RW_PD_RESP_MSG_DATA) */
#endif /* (CY_HPI_PD_CMD_ENABLE) */
    .vbus_get_live_current = vbus_get_live_current,
#endif /* (CY_HPI_PD_ENABLE) */
    .set_bootloader_run_type = set_bootloader_run_type,
#if (!CY_HPI_BOOT_ENABLE)
    .hpi_boot_validate_fw = hpi_boot_validate_fw,
#endif /* (!CY_HPI_BOOT_ENABLE) */
    .hpi_boot_validate_fw_cmd = hpi_boot_validate_fw_cmd,
#if (!CY_HPI_BOOT_ENABLE)
    .hpi_sys_get_device_mode = hpi_sys_get_device_mode,
#endif /* (!CY_HPI_BOOT_ENABLE)) */
#if (CY_HPI_FLASH_RW_ENABLE)
    .hpi_flash_row_write = hpi_flash_row_write,
    .hpi_flash_row_read = Cy_App_Flash_RowRead,
    .hpi_flash_access_get_status = Cy_App_Flash_AccessGetStatus,
    .hpi_flash_enter_mode = hpi_flash_enter_mode,
#endif /* (CY_HPI_FLASH_RW_ENABLE) */
#if (CCG_UCSI_ENABLE)
    .ucsi_notify = ucsi_notify,
    .ucsi_reg_space_write_handler = ucsi_reg_space_write_handler,
    .ucsi_handle_hpi_commands = ucsi_handle_hpi_commands,
    .hpi_update_ucsi_reg_space = hpi_update_ucsi_reg_space,
#endif /* (CCG_UCSI_ENABLE) */
    .hpi_dev_wr_handler_ext = hpi_dev_wr_handler_ext,
    .hpi_port_wr_handler_ext = NULL,
};


/** Callback function to read/write the user defined registers. */
uint8_t hpi_userdef_reg_write_cbk( cy_stc_hpi_context_t *context,
                                   uint16_t  reg_addr,
                                   uint8_t   wr_size,
                                   uint8_t  *wr_data )
{
    return Cy_Hpi_InitUserdefRegs(context, reg_addr, wr_size, wr_data) ? \
            CY_HPI_RESPONSE_SUCCESS : \
            CY_HPI_RESPONSE_INVALID_ARGUMENT;
}

/*
 * This function checks the I2C_CFG pin strap status to identify the I2C
 * slave address to be used for the HPI interface.
 */
static void get_hpi_slave_addr(void)
{
    uint8_t slaveAddr = CY_HPI_ADDR_I2C_CFG_FLOAT;
    if (glBootDataSignature == BL_APP_DATA_VALID_SIG)
    {
        slaveAddr = glHpiSlaveAddr;
    }
#if !DISABLE_I2C_ADDR_CONFIG    
    else
    {
        /* Check if IO is driven low. */
        Cy_GPIO_SetDrivemode(I2C_CFG_PORT, I2C_CFG_PIN, CY_GPIO_DM_PULLUP);
        Cy_GPIO_Write(I2C_CFG_PORT, I2C_CFG_PIN, 1);
        Cy_SysLib_DelayUs(5);
        if (Cy_GPIO_Read(I2C_CFG_PORT, I2C_CFG_PIN) == 0)
        {
            slaveAddr = CY_HPI_ADDR_I2C_CFG_LOW;
        }
        else
        {
            /* Check if IO is driven high. */
            Cy_GPIO_SetDrivemode(I2C_CFG_PORT, I2C_CFG_PIN, CY_GPIO_DM_PULLDOWN);
            Cy_GPIO_Write(I2C_CFG_PORT, I2C_CFG_PIN, 0);
            Cy_SysLib_DelayUs(5);
            if (Cy_GPIO_Read(I2C_CFG_PORT, I2C_CFG_PIN) != 0)
            {
                slaveAddr = CY_HPI_ADDR_I2C_CFG_HIGH;
            }
        }
        /* Disable the pull up/pull down on IO. */
        Cy_GPIO_SetDrivemode(I2C_CFG_PORT, I2C_CFG_PIN, CY_GPIO_DM_HIGHZ);
    }
#endif /* DISABLE_I2C_ADDR_CONFIG */

    gl_HpiHwConfig.slaveAddr = slaveAddr;
}

static void update_hpi_regs ( void )
{
#if CY_APP_FIRMWARE_APP_ONLY
    uint8_t mode, reason;
    uint16_t fw1_loc=0x0000, fw2_loc=0x0000;

    /* Flash access is not allowed. */
    Cy_App_Flash_SetAccessLimits (CY_APP_SYS_IMG1_METADATA_ROW_NUM, CY_APP_SYS_IMG1_METADATA_ROW_NUM, CY_APP_SYS_IMG1_METADATA_ROW_NUM,
            CY_APP_BOOT_LOADER_LAST_ROW);

    /* Update HPI registers with default values. */
    mode   = 0x95;              /* Dual boot, 256 byte flash, 2 ports, FW1 running. */
    reason = 0x08;              /* FW2 is not valid. */
    Cy_Hpi_SetModeRegs(&gl_HpiContext, mode, reason);
    Cy_Hpi_SetHpiVersion(&gl_HpiContext, HPI_VERSION);
    Cy_Hpi_SetHpiVersionExt(&gl_HpiContext, HPI_VERSION_EXT);
    Cy_Hpi_UpdateFwLocations(&gl_HpiContext, fw1_loc, fw2_loc);

#else /* !CY_APP_FIRMWARE_APP_ONLY */

    uint8_t mode, reason = 0x00;
    uint32_t fw1_ver, fw2_ver;
    uint16_t fw1_loc, fw2_loc;
    cy_stc_sys_fw_metadata_t *fw1_md, *fw2_md;
    uint8_t ver_invalid[8] = {0};

    /* Set mode variables and flash access limits based on the active firmware. */
#if (CY_APP_TYPE == CY_APP_FW_IMAGE1)
    mode = 0x81 | ((NO_OF_TYPEC_PORTS - 1) << 2) | ((CCG_FLASH_ROW_SHIFT_NUM - 7) << 4);

    /* Get the firmware validity information from the boot-loader shared data otherwise
     * validate the alternate firmware image. */
    if (glBootDataSignature == BL_APP_DATA_VALID_SIG)
    {
        reason = Cy_App_Boot_GetBootModeReason().val;
    }
    else
    {
        /* Check if FW2 is valid. */
        if (Cy_App_Boot_ValidateFw((cy_stc_sys_fw_metadata_t *)CY_APP_SYS_IMG2_FW_METADATA_ADDR) != CY_APP_STAT_SUCCESS)
            reason = 0x08;
    }

    /* Set the legal flash access range.
       Note: This needs to be updated whenever the FW1/FW2 reserved memory size is updated.
     */
    Cy_App_Flash_SetAccessLimits (CY_APP_IMG1_LAST_FLASH_ROW_NUM + 1,
                                    CY_APP_IMG2_LAST_FLASH_ROW_NUM,
                                    CY_APP_SYS_IMG2_METADATA_ROW_NUM,
                                    CY_APP_BOOT_LOADER_LAST_ROW);

#else /* (CY_APP_TYPE == CY_APP_FW_IMAGE2) */

    mode = 0x82 | ((NO_OF_TYPEC_PORTS - 1) << 2)| ((CCG_FLASH_ROW_SHIFT_NUM - 7) << 4);

    if (glBootDataSignature == BL_APP_DATA_VALID_SIG)
    {
        reason = Cy_App_Boot_GetBootModeReason().val;
    }
    else
    {
        /* Check if FW1 is valid. */
        if (Cy_App_Boot_ValidateFw((cy_stc_sys_fw_metadata_t *)CY_APP_SYS_IMG1_FW_METADATA_ADDR) != CY_APP_STAT_SUCCESS)
            reason = 0x04;
    }

    /* Set the legal flash access range.
       Note: This needs to be updated whenever the FW1/FW2 reserved memory size is updated.
     */
    Cy_App_Flash_SetAccessLimits (CY_APP_BOOT_LOADER_LAST_ROW + 1,
                                    CY_APP_IMG1_LAST_FLASH_ROW_NUM,
                                    CY_APP_SYS_IMG1_METADATA_ROW_NUM,
                                    CY_APP_BOOT_LOADER_LAST_ROW);
#endif /* (CY_APP_TYPE == CY_APP_FW_IMAGE1) */

    /* Set HPI version register info. */
    Cy_Hpi_SetHpiVersion (&gl_HpiContext, HPI_VERSION);

    /* Set HPI version register info. */
    Cy_Hpi_SetHpiVersionExt(&gl_HpiContext, HPI_VERSION_EXT);

    /* Set HPI mode register info. */
    Cy_Hpi_SetModeRegs(&gl_HpiContext, mode, reason);

    /* Set HPI silicon Id register value. */
    Cy_Hpi_UpdateRegs(&gl_HpiContext,
                      (uint8_t)CY_HPI_REG_SECTION_DEV, 0x02,
                      ((uint8_t *) &glPmg1SiliconId) + 2, 0x02);

#if (CY_APP_TYPE == CY_APP_FW_IMAGE2)
    /* Calculate the version address from the firmware metadata. */
    if ((reason & 0x04) == 0)
#endif /* (CY_APP_TYPE == CY_APP_FW_IMAGE2) */
    {
        fw1_md  = (cy_stc_sys_fw_metadata_t *)CY_APP_SYS_IMG1_FW_METADATA_ADDR;
        fw1_ver = ((uint32_t)fw1_md->fw_start) + CY_APP_SYS_FW_VERSION_OFFSET;
        fw1_loc = fw1_md->fw_start >> CCG_FLASH_ROW_SHIFT_NUM;

    }
#if (CY_APP_TYPE == CY_APP_FW_IMAGE2)
    else
    {
        fw1_ver = (uint32_t)ver_invalid;
        fw1_loc = CY_APP_SYS_FLASH_LAST_ROW_NUM + 1;
    }
#endif /* (CY_APP_TYPE == CY_APP_FW_IMAGE2) */

#if (CY_APP_TYPE == CY_APP_FW_IMAGE1)
    if ((reason & 0x08) == 0)
#endif /* (CY_APP_TYPE == CY_APP_FW_IMAGE1) */
    {
        fw2_md  = (cy_stc_sys_fw_metadata_t *)CY_APP_SYS_IMG2_FW_METADATA_ADDR;
        fw2_ver = ((uint32_t)fw2_md->fw_start) + CY_APP_SYS_FW_VERSION_OFFSET;
        fw2_loc = fw2_md->fw_start >> CCG_FLASH_ROW_SHIFT_NUM;
    }
#if (CY_APP_TYPE == CY_APP_FW_IMAGE1)
    else
    {
        fw2_ver = (uint32_t)ver_invalid;
        fw2_loc = CY_APP_SYS_FLASH_LAST_ROW_NUM + 1;
    }
#endif /* (CY_APP_TYPE == CY_APP_FW_IMAGE1) */

    /* Update version information in the HPI registers. */
    Cy_Hpi_UpdateVersions(&gl_HpiContext,
                        (uint8_t *)CY_APP_SYS_BOOT_VERSION_ADDRESS,
                        (uint8_t *)fw1_ver,
                        (uint8_t *)fw2_ver
                        );

    /* Update firmware location registers. */
    Cy_Hpi_UpdateFwLocations(&gl_HpiContext, fw1_loc, fw2_loc);

#endif /* CY_APP_FIRMWARE_APP_ONLY */

    /* update the watch dog reset count value to dev specific register */
    Cy_Hpi_SetResetCount(&gl_HpiContext, (uint8_t)Cy_App_Instrumentation_GetWdtResetCount());
}

void hpi_set_device_mode (void)
{
#if CY_APP_FIRMWARE_APP_ONLY
    CALL_MAP(Cy_App_Sys_SetDeviceMode)(CY_APP_SYS_FW_MODE_FWIMAGE_1);
#else /* !CY_APP_FIRMWARE_APP_ONLY */

#if (!CY_APP_TYPE)
    CALL_MAP(Cy_App_Sys_SetDeviceMode)(CY_APP_SYS_FW_MODE_FWIMAGE_1);
#else
    CALL_MAP(Cy_App_Sys_SetDeviceMode)(CY_APP_SYS_FW_MODE_FWIMAGE_2);
#endif /* !CY_APP_TYPE */
#endif /* CY_APP_FIRMWARE_APP_ONLY */
}


extern cy_stc_pdstack_context_t gl_PdStackPort0Ctx;
#if PMG1_PD_DUALPORT_ENABLE
extern cy_stc_pdstack_context_t gl_PdStackPort1Ctx;
#endif /* PMG1_PD_DUALPORT_ENABLE */

void hpi_init (void)
{
    uint32_t count = 0;

    /* Set HPI flash parameters values. */
    Cy_Hpi_SetFlashParams(&gl_HpiContext, CY_APP_SYS_FLASH_SIZE, CY_APP_SYS_FLASH_ROW_SIZE,
            CY_APP_SYS_FLASH_LAST_ROW_NUM + 1, CY_APP_BOOT_LOADER_LAST_ROW);

    /* Get the HPI slave address from the boot-loader. */
    get_hpi_slave_addr();

    /* Initialize the HPI interface. */
    Cy_Hpi_Init(&gl_HpiContext,
                &gl_HpiHwConfig,
                &hpiAppCbk,
                &gl_PdStackPort0Ctx,
#if PMG1_PD_DUALPORT_ENABLE
                &gl_PdStackPort1Ctx,
#else
                NULL,
#endif
                NO_OF_TYPEC_PORTS
            );

    /* Set user defined register read/write callback. */
    Cy_Hpi_SetUserdefWriteHandler(&gl_HpiContext, hpi_userdef_reg_write_cbk);

    /* Initialize and enable HPI I2C SCB interrupt. */
    Cy_SysInt_Init(&HPI_SCB_IRQ_config, &scb_0_interrupt_IRQHandler);
    NVIC_EnableIRQ((IRQn_Type) HPI_SCB_IRQ_config.intrSrc);

    /* Register the HPI context to the PdStack. */
    Cy_PdStack_Dpm_HpiInitContext(&gl_PdStackPort0Ctx, &gl_HpiContext);
#if PMG1_PD_DUALPORT_ENABLE
    Cy_PdStack_Dpm_HpiInitContext(&gl_PdStackPort1Ctx, &gl_HpiContext);
#endif /* PMG1_PD_DUALPORT_ENABLE */

    update_hpi_regs();

#if (CY_HPI_RW_PD_RESP_MSG_DATA)
#if CY_PD_BAT_CAPS_HANDLER_ENABLE
    {
        /* Store the default PD responses for battery capabilities before sending
         * reset complete event to EC, so that if required EC can delete them.
         * Note that this applies to all the ports. Hence not port specific.
         */
        /* Valid Battery reference for slot 0.*/
        uint8_t slot0BatCap[] = {0x02, 0x04, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x00,
                                 0xFF, 0xFF, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0x00};
        Cy_App_Hpi_HandlePdRespDataRw(&gl_HpiContext, &gl_PdStackPort0Ctx, slot0BatCap);

        /* Invalid Battery reference for slots except 0.*/
        uint8_t invalidBatCap[] = {0x02, 0x04, 0x0D, 0x00, 0x00, 0x01, 0x00, 0x00,
                                   0xFF, 0xFF, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0x01};
        Cy_App_Hpi_HandlePdRespDataRw(&gl_HpiContext, &gl_PdStackPort0Ctx, invalidBatCap);
    }
#endif /* CY_PD_BAT_CAPS_HANDLER_ENABLE */
#if CY_PD_BAT_STATUS_HANDLER_ENABLE
    {
        /* Store the default PD responses for battery Status
         * before sending reset complete event to EC.
         * so that if required EC can delete them.
         * Note that this applies to all the ports. Hence not port specific.
         */
        /* Valid Battery reference for slot 0.*/
        uint8_t slot0BatStat[] = {0x01, 0x04, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xFF, 0xFF};
        Cy_App_Hpi_HandlePdRespDataRw(&gl_HpiContext, &gl_PdStackPort0Ctx, slot0BatStat);

        /* Invalid Battery reference for slots except 0.*/
        uint8_t invalidBatStat[] = {0x01, 0x04, 0x08, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0xFF, 0xFF};
        Cy_App_Hpi_HandlePdRespDataRw(&gl_HpiContext, &gl_PdStackPort0Ctx, invalidBatStat);
    }
#endif /* CY_PD_BAT_STATUS_HANDLER_ENABLE */

#if CY_PD_SRC_INFO_HANDLER_ENABLE
    /* Store the default PD responses for source info before sending reset complete event to EC.
     * so that if required EC can delete them.
     */
    {
        /* source info for port0.*/
        uint32_t srcInfo = mtb_usbpd_port0_pdstack_config.srcInfo;
        uint8_t sourceInfoDefaultP0[] = {CY_APP_PD_RESP_ID_SRC_INFO, CY_APP_PD_RESP_DATA_CMD_WRITE,
                                         CY_APP_PD_RESP_CMD_SRC_INFO_LEN, 0x00,
                                         (uint8_t) srcInfo, (uint8_t) (srcInfo>>8),
                                         (uint8_t) (srcInfo>>16), (uint8_t) (srcInfo>>24)};
        Cy_App_Hpi_HandlePdRespDataRw(&gl_HpiContext, &gl_PdStackPort0Ctx, sourceInfoDefaultP0);
#if PMG1_PD_DUALPORT_ENABLE
        /* source info for port1.*/
        srcInfo = mtb_usbpd_port1_pdstack_config.srcInfo;
        uint8_t sourceInfoDefaultP1[] = {CY_APP_PD_RESP_ID_SRC_INFO, CY_APP_PD_RESP_DATA_CMD_WRITE,
                                         CY_APP_PD_RESP_CMD_SRC_INFO_LEN, 0x00,
                                         (uint8_t) srcInfo, (uint8_t) (srcInfo>>8),
                                         (uint8_t) (srcInfo>>16), (uint8_t) (srcInfo>>24)};
        Cy_App_Hpi_HandlePdRespDataRw(&gl_HpiContext, &gl_PdStackPort1Ctx, sourceInfoDefaultP1);
#endif /*PMG1_PD_DUALPORT_ENABLE*/
    }
#endif /* CY_PD_SRC_INFO_HANDLER_ENABLE */
#if CY_PD_REVISION_MESSAGE_ENABLE
    /* Store the default PD responses for PD Revision message
     * before sending reset complete event to EC.
     * so that if required EC can delete them.
     * Note that this applies to all the ports. Hence not port specific.
     */
    {
        /* USBPD Revision message. */
        cy_stc_pdstack_dpm_ext_status_t* ptrDpmExtStat = &(gl_PdStackPort0Ctx.dpmExtStat);
        uint32_t pdRevison = ptrDpmExtStat->revision.val;
        /* Write the USBPD Revision message to PD Response message data buffer */
        uint8_t revision_msg[] = {CY_APP_PD_RESP_ID_PD_REV_MSG, 0x04, CY_APP_PD_RESP_CMD_REV_MSG_LEN, 0x00,
                                  (uint8_t) (pdRevison), (uint8_t) (pdRevison>>8u),
                                  (uint8_t) (pdRevison>>16u), (uint8_t) (pdRevison>>24u)};
        Cy_App_Hpi_HandlePdRespDataRw(&gl_HpiContext, &gl_PdStackPort0Ctx, revision_msg);
    }
#endif /* CY_PD_REVISION_MESSAGE_ENABLE */
#endif /* (CY_HPI_RW_PD_RESP_MSG_DATA) */

    /* Send a reset complete event to the EC. */
    Cy_Hpi_SendFwReadyEvent(&gl_HpiContext);

    /* Wait until EC ready event has been received or 100 ms has elapsed. */
    for (count = 0; count < 100; count++)
    {
        Cy_Hpi_Task(&gl_HpiContext);
        if (Cy_Hpi_IsEcReady(&gl_HpiContext))
        {
            break;
        }

#if (CY_CFG_SYSCLK_IMO_FREQ_MHZ == 48)
        /*
         * Using CyDelayCycles as CyDelay is leading to a 25% error.
         * The parameter used here assumes that CPU clock is running at 48 MHz.
         */
        Cy_SysLib_DelayCycles(38400);
#else
        /*
         * Using CyDelayCycles as CyDelay is leading to a 25% error.
         * The parameter used here assumes that CPU clock is running at 24 MHz.
         */
        Cy_SysLib_DelayCycles(19200);
#endif /* (CY_CFG_SYSCLK_IMO_FREQ_MHZ == 48) */
    }

    /* Start the device policy manager operation. This will initialize the
     * USB-PD block and enable connect detection. */
    /* Start the DPM for the port only if it is enabled at the HPI level. It is
     * possible that the port got disabled before we got here. */
    if((Cy_Hpi_GetPortEnable(&gl_HpiContext) & (1 << TYPEC_PORT_0_IDX)) != 0)
    {
        Cy_PdStack_Dpm_Start(&gl_PdStackPort0Ctx);
    }
#if PMG1_PD_DUALPORT_ENABLE
    if((Cy_Hpi_GetPortEnable(&gl_HpiContext) & (1 << TYPEC_PORT_1_IDX)) != 0)
    {
        Cy_PdStack_Dpm_Start(&gl_PdStackPort1Ctx);
    }
#endif /* PMG1_PD_DUALPORT_ENABLE */
}
#endif /* CY_HPI_ENABLED */

/* [] END OF FILE */
