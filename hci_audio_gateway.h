/*
 * Copyright 2016-2020, Cypress Semiconductor Corporation or a subsidiary of
 * Cypress Semiconductor Corporation. All Rights Reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software"), is owned by Cypress Semiconductor Corporation
 * or one of its subsidiaries ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products. Any reproduction, modification, translation,
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
 */

/** @file
 *
 * This file provides the private interface definitions for hci_control app
 *
 */
#ifndef HCI_AUDIO_GATEWAY_H
#define HCI_AUDIO_GATEWAY_H

#include "wiced_bt_dev.h"
#include "hci_control_api.h"
#include "wiced_bt_hfp_ag.h"
#include "wiced_bt_cfg.h"
#include "wiced_bt_audio.h"

/*****************************************************************************
**  Constants that define the capabilities and configuration
*****************************************************************************/
#define HDLR_HANDS_FREE_UNIT            0x10001
#define HDLR_HEADSET_UNIT               0x10002
#define TRANS_UART_BUFFER_SIZE          1024

#define HCI_CONTROL_AG_NUM_SCB          2           /* Max simultaneous connections to HFs */

#if (BTM_WBS_INCLUDED == TRUE )
#define BT_AUDIO_HFP_SUPPORTED_FEATURES     (HFP_AG_FEAT_VREC | HFP_AG_FEAT_CODEC | HFP_AG_FEAT_ESCO)
#else
#define BT_AUDIO_HFP_SUPPORTED_FEATURES     (HFP_AG_FEAT_VREC | HFP_AG_FEAT_ESCO)
#endif

#define HFP_VGM_VGS_DEFAULT             7

/*****************************************************************************
**  Data types
*****************************************************************************/

/* The main application control block */
typedef struct
{
    hfp_ag_session_cb_t  ag_scb[HCI_CONTROL_AG_NUM_SCB];       /* service control blocks */
    uint8_t pairing_allowed;
} hci_control_cb_t;


/*****************************************************************************
**  Global data
*****************************************************************************/

/* control block declaration */
#if BTA_DYNAMIC_MEMORY == FALSE
extern hci_control_cb_t hci_control_cb;
#else
extern hci_control_cb_t *hci_control_cb_ptr;
#define hci_control_cb( *hci_control_cb_ptr )
#endif

/*****************************************************************************
**  Function prototypes
*****************************************************************************/

/* main functions */
extern void     hci_control_send_command_status_evt( uint16_t code, uint8_t status );

extern int      hci_control_write_nvram( int nvram_id, int data_len, void *p_data, BOOLEAN from_host );
extern int      hci_control_read_nvram( int nvram_id, void *p_data, int data_len );
extern int      hci_control_find_nvram_id( uint8_t *p_data, int len);
extern void     hci_control_delete_nvram( int nvram_id , wiced_bool_t from_host);
extern int      hci_control_alloc_nvram_id( );

extern const wiced_bt_cfg_settings_t hci_ag_cfg_settings;
extern const wiced_bt_cfg_buf_pool_t hci_ag_cfg_buf_pools[];
extern const wiced_bt_audio_config_buffer_t hci_ag_audio_buf_config;

#endif /* HCI_AUDIO_GATEWAY_H */
