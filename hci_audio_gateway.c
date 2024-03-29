/*
 * Copyright 2016-2023, Cypress Semiconductor Corporation (an Infineon company) or
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
 */

/** @file
 *
 * AG Plus Device Sample Application for 20xxx devices.
 *
 * This file implements 20xxx embedded application controlled over UART.
 * Current version of the application exposes Handsfree Audio Gateway
 * MCU connected over UART can send commands to execute certain functionality
 * while configuration is local in the application including SDP.
 * Current version of the application does not support full HF AG functionality
 * but only enough to create service level connection to hands-free device
 * and establishment of the voice connection which can be used for example for
 * the voice recognition feature.
 *
 * To demonstrate the app, work through the following steps.
 * 1. Plug the WICED Bluetooth ( 20xxx ) evaluation board into your computer
 * 2. Build and download the application ( to the 20xxx board )
 * 3. Use ClientControl application to send various commands
 *
 * The sample app performs as a Bluetooth HF AG.
 *
 * The sample Windows ClientControl application is provided to show sample
 * MCU implementation on Windows platform.
 *
 * Features demonstrated
 *  - AIROC Bluetooth Handsfree (Audio Gateway) APIs
 *  - Handling of the UART WICED protocol
 *  - Setting of the Local Bluetooth Device address from the host MCU
 *
 * On startup this demo:
 *  - Initializes the Bluetooth sub system
 *  - Receive NVRAM information from the host
 *
 * Application Instructions
 *  - Connect a PC terminal to WICED Eval board.
 *  - Build and download the application to the board.
 *  - Run the ClientControl application and open the WICED HCI port
 *
 * BR/EDR
 * - To find BR/EDR devices: Click on "Start BR/EDR Discovery"
 *
 *
 * AG Connection
 * - To create audio gateway connection to remote handsfree controller, choose the bluetooth
 *   address of the remote device from the BR/EDR combo box
 * - Click "Connect" button under AG
 * - To establish SCO connection, click on Audio connect
 * - Check SCO audio quality
 * - NOTE : Default WBS is disabled, update hf_control_esco_params structure to enabled it.
 */

#include "wiced_bt_dev.h"
#include "sparcommon.h"
#include "wiced_bt_cfg.h"
#include "wiced_memory.h"
#include "wiced_platform.h"
#include "hci_audio_gateway.h"
#include "hci_control_api.h"
#include "wiced_transport.h"
#include "wiced_hal_puart.h"
#include "wiced_bt_stack.h"
#include "string.h"
#include "wiced_transport.h"
#include "wiced_hal_wdog.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_rfcomm.h"
#include "wiced_timer.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_sco.h"
#if defined(CYW55572A1)
#include "wiced_audio_manager.h"
#endif

/*****************************************************************************
**  Constants
*****************************************************************************/
#define WICED_HS_EIR_BUF_MAX_SIZE               264
#define HCI_CONTROL_FIRST_VALID_NVRAM_ID        0x10
#define HCI_CONTROL_INVALID_NVRAM_ID            0x00

#define READ_LITTLE_ENDIAN_TO_UINT16(into, m,dl)      \
        (into) = ((m)[0] | ((m)[1]<<8));\
        (m) +=2; (dl)-=2;

#define KEY_INFO_POOL_BUFFER_SIZE               145 //Size of the buffer used for holding the peer device key info
#define KEY_INFO_POOL_BUFFER_COUNT              10  //Correspond's to the number of peer devices

#ifdef CYW55572
#define AG_BUTTON_VOLUME_GAIN                   WICED_BUTTON1
#else
#define AG_BUTTON_VOLUME_GAIN                   WICED_GPIO_PIN_BUTTON
#endif

/*****************************************************************************
**  Structures
*****************************************************************************/
typedef struct
{
    void    *p_next;
    uint16_t nvram_id;
    uint8_t  chunk_len;
    uint8_t  data[1];
} hci_control_nvram_chunk_t;

#if BTSTACK_VER >= 0x03000001
#ifndef PACKED
#define PACKED
#endif
#pragma pack(1)

typedef PACKED struct
{
    uint8_t                           br_edr_key_type;
    wiced_bt_link_key_t               br_edr_key;
    wiced_bt_dev_le_key_type_t        le_keys_available_mask;
    wiced_bt_ble_address_type_t       ble_addr_type;
    wiced_bt_ble_address_type_t       static_addr_type; //55572A1 does not have but 20721 does.
    wiced_bt_device_address_t         static_addr; //55572A1 does not have but 20721 does.
    wiced_bt_ble_keys_t               le_keys;
} wiced_bt_device_sec_keys_t_20721;

typedef PACKED struct
{
    wiced_bt_device_address_t   bd_addr;
    wiced_bt_device_sec_keys_t_20721  key_data;
} wiced_bt_device_link_keys_t_20721;
#endif

/******************************************************
 *               Variables Definitions
 ******************************************************/
/*
 * This is the SDP database for the whole hci_control application
 */
const uint8_t hci_control_sdp_db[] =
{
    SDP_ATTR_SEQUENCE_1( 62 ),                                             // length is the sum of all records

    // SDP record for HF ( total length of record: 51 )
    SDP_ATTR_SEQUENCE_1( 60 ),                                              // 2 bytes, length of the record
        SDP_ATTR_RECORD_HANDLE( 0x10001 ),                                  // 8 byte ( handle=0x10001 )
        SDP_ATTR_ID( ATTR_ID_SERVICE_CLASS_ID_LIST ),                       // 3 bytes
        SDP_ATTR_SEQUENCE_1( 6 ),                                           // 2 bytes
        SDP_ATTR_UUID16( 0x111F ),                                          // 3 bytes ServiceClass0 UUID_SERVCLASS_HF_HANDSFREE
        SDP_ATTR_UUID16( 0X1203 ),                                          // 3 bytes ServiceClass1 UUID_SERVCLASS_GENERIC_AUDIO
        SDP_ATTR_RFCOMM_PROTOCOL_DESC_LIST( 1 ),                            // 17 bytes ( SCN=1 )
        SDP_ATTR_PROFILE_DESC_LIST( UUID_SERVCLASS_AG_HANDSFREE, 0x0108 ),  // 13 bytes UUID_SERVCLASS_HF_HANDSFREE, version 0x0108
        SDP_ATTR_UINT1(ATTR_ID_NETWORK, 0x00),                                  // 5 byte
        SDP_ATTR_UINT2(ATTR_ID_SUPPORTED_FEATURES,  AG_SUPPORTED_FEATURES_ATT), //6 bytes
};

hci_control_nvram_chunk_t *p_nvram_first = NULL;

/* HS control block */
#if BTA_DYNAMIC_MEMORY == FALSE
hci_control_cb_t  hci_control_cb;
#endif

wiced_bt_buffer_pool_t* p_key_info_pool;//Pool for storing the  key info

uint8_t pincode[4]  = {0x30,0x30,0x30,0x30};

static uint32_t hci_control_proc_rx_cmd( uint8_t *p_data, uint32_t length );
#if BTSTACK_VER >= 0x03000001
static void      hci_control_tx_complete(void);
#else
static void      hci_control_tx_complete( wiced_transport_buffer_pool_t* p_pool );
#endif

const wiced_transport_cfg_t  transport_cfg =
{
    WICED_TRANSPORT_UART,
    {{ WICED_TRANSPORT_UART_HCI_MODE, HCI_UART_DEFAULT_BAUD}},
#if BTSTACK_VER >= 0x03000001
    .heap_config =
    {
        .data_heap_size = 1024 * 4 + 1500 * 2,
        .hci_trace_heap_size = 1024 * 2,
        .debug_trace_heap_size = 1024,
    },
#else
    { TRANS_UART_BUFFER_SIZE, 1},
#endif
    NULL,
    hci_control_proc_rx_cmd,
    hci_control_tx_complete
};

#if BTSTACK_VER >= 0x03000001
#define BT_STACK_HEAP_SIZE          1024 * 7
wiced_bt_heap_t *p_default_heap = NULL;
#endif

wiced_bt_voice_path_setup_t audiogateway_sco_path = {
#ifdef CYW20706A2
    .path = WICED_BT_SCO_OVER_I2SPCM,
#else
    .path = WICED_BT_SCO_OVER_PCM,
#endif
#if defined(CYW20721B2) || defined (CYW43012C0) || defined(CYW55572A1)
    .p_sco_data_cb = NULL
#endif
};

#if defined(CYW55572A1)
static int32_t stream_id = WICED_AUDIO_MANAGER_STREAM_ID_INVALID;
static wiced_bool_t app_ag_use_wbs = WICED_TRUE;
static audio_config_t audio_config =
    {
#if (WICED_BT_HFP_HF_WBS_INCLUDED == TRUE)
        .sr = AM_PLAYBACK_SR_16K,
#else
        .sr = AM_PLAYBACK_SR_8K,
#endif
       .channels = 1,
       .bits_per_sample = DEFAULT_BITSPSAM,
       .volume = AM_VOL_LEVEL_HIGH-2,
       .mic_gain = AM_VOL_LEVEL_HIGH-2,
       .sink = AM_HEADPHONES,
    };
#endif

static uint32_t app_ag_button_gpio = AG_BUTTON_VOLUME_GAIN;

/******************************************************
 *               Function Declarations
 ******************************************************/
static void     hci_control_handle_reset_cmd( void );
static void     hci_control_handle_trace_enable( uint8_t *p_data );
static void     hci_control_device_handle_command( uint16_t cmd_opcode, uint8_t* p_data, uint32_t data_len );
static void     hci_control_ag_handle_command(uint16_t opcode, uint8_t* p_data, uint32_t length);
static void     hci_control_handle_set_local_bda( uint8_t *p_bda );
static void     hci_control_inquiry( uint8_t enable );
static void     hci_control_handle_set_visibility( uint8_t discoverability, uint8_t connectability );
static void     hci_control_send_device_started_evt( void );
static void     hci_control_send_pairing_completed_evt( uint8_t status , wiced_bt_device_address_t bdaddr );
static void     hci_control_send_encryption_changed_evt( uint8_t encrypted , wiced_bt_device_address_t bdaddr );
static void     hci_control_misc_handle_command( uint16_t cmd_opcode, uint8_t* p_data, uint32_t data_len );
static void     hci_control_misc_handle_get_version( void );
static wiced_result_t hci_control_management_callback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data );
static void     hci_control_handle_set_pairability ( uint8_t pairing_allowed );

extern uint8_t  avrc_is_abs_volume_capable( void );

/******************************************************
 *               Function Definitions
 ******************************************************/

/*
 *  Application Start, ie, entry point to the application.
 */
APPLICATION_START( )
{
    wiced_result_t ret = WICED_BT_ERROR;

    wiced_transport_init( &transport_cfg );

#ifdef WICED_BT_TRACE_ENABLE
    // Set the debug uart as WICED_ROUTE_DEBUG_NONE to get rid of prints
    // wiced_set_debug_uart(WICED_ROUTE_DEBUG_NONE);

#ifdef CYW55572
    // Default PUART baudrate is 115200, update it to 3M before calling wiced_set_debug_uart(WICED_ROUTE_DEBUG_TO_PUART);
    wiced_set_debug_uart_baudrate(3000000);
#else
    // Set to PUART to see traces on peripheral uart(puart)
    wiced_hal_puart_init();
    wiced_hal_puart_configuration( 3000000, PARITY_NONE, STOP_BIT_2 );
#endif
    wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_PUART );
#if ( defined(CYW20706A2) || defined(CYW43012C0) )
    wiced_hal_puart_select_uart_pads( WICED_PUART_RXD, WICED_PUART_TXD, 0, 0);
#endif

    // Set to HCI to see traces on HCI uart - default if no call to wiced_set_debug_uart()
    // wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_HCI_UART );

    // Use WICED_ROUTE_DEBUG_TO_WICED_UART to send formatted debug strings over the WICED
    // HCI debug interface to be parsed by ClientControl/BtSpy
    // Note: WICED HCI must be configured to use this - see wiced_trasnport_init(), must
    // be called with wiced_transport_cfg_t.wiced_tranport_data_handler_t callback present
    // wiced_set_debug_uart(WICED_ROUTE_DEBUG_TO_WICED_UART);
#endif

    WICED_BT_TRACE( "#######################\n" );
    WICED_BT_TRACE( "Audio Gateway APP START\n" );
    WICED_BT_TRACE( "#######################\n" );


#if BTSTACK_VER >= 0x03000001
    /* Create default heap */
    p_default_heap = wiced_bt_create_heap("default_heap", NULL, BT_STACK_HEAP_SIZE, NULL,
            WICED_TRUE);
    if (p_default_heap == NULL)
    {
        WICED_BT_TRACE("create default heap error: size %d\n", BT_STACK_HEAP_SIZE);
        return;
    }
#endif

#if BTSTACK_VER >= 0x03000001
    /* Register the dynamic configurations */
    ret = wiced_bt_stack_init( hci_control_management_callback, &hci_ag_cfg_settings);
#else
    /* Register the dynamic configurations */
    ret = wiced_bt_stack_init( hci_control_management_callback, &hci_ag_cfg_settings, hci_ag_cfg_buf_pools);
#endif

    if( ret != WICED_BT_SUCCESS )
        return;

    /* Configure Audio buffer */
    ret = wiced_audio_buffer_initialize (hci_ag_audio_buf_config);
    if( ret != WICED_BT_SUCCESS )
        return;
}

#define DEBOUNCE_TIME           100     /* unit: ms */
#define BUTTON_LONG_TIMEOUT     10000   /* unit: ms */
#define BUTTON_PERIODIC_TIME    50      /* unit: ms */
uint8_t vg_button = 0, vg_increase = 0, vg_press = 0;
uint16_t pressed_duration = 0;
wiced_timer_t button_timer;
static void button_gpio_interrupt_handler(void *data, uint8_t pin)
{
    hfp_ag_session_cb_t *p_scb = &hci_control_cb.ag_scb[0];
    uint8_t temp_vg = vg_button;
    uint8_t gpio_status;
    uint32_t* p_button_gpio = (uint32_t *)data;

    // Callback pin is incorrect in 55572A1, so replaced it by using user data now.
    gpio_status = wiced_hal_gpio_get_pin_input_status( *p_button_gpio );

    /* only handle the valid release-button event (including press and release) */
    if (gpio_status == WICED_FALSE)
    {
        if (vg_press == 0)
        {
            vg_press = 1;
            wiced_start_timer(&button_timer, BUTTON_PERIODIC_TIME);
            pressed_duration = 0;
        }
        return;
    }
    else if (gpio_status == WICED_TRUE)
    {
        /* discard repeated release interrupt */
        if (vg_press == 0)
            return;

        vg_press = 0;
        wiced_stop_timer(&button_timer);
        if (pressed_duration < DEBOUNCE_TIME)
            return;
    }
    else
        return;

    if (vg_increase == TRUE)
    {
        if (vg_button < HFP_VGM_VGS_MAX)
            vg_button++;
    }
    else
    {
        if (vg_button > HFP_VGM_VGS_MIN)
            vg_button--;
    }

    if ( (hfp_ag_send_VGM_to_hf(p_scb, vg_button) == FALSE) ||
            (hfp_ag_send_VGS_to_hf(p_scb, vg_button) == FALSE) )
    {
        vg_button = temp_vg;
        return;
    }

    if (vg_button == HFP_VGM_VGS_MAX)
        vg_increase = FALSE;
    else if (vg_button == HFP_VGM_VGS_MIN)
        vg_increase = TRUE;
}

static void timeout_cb( TIMER_PARAM_TYPE cb_params )
{
    pressed_duration += BUTTON_PERIODIC_TIME;
    if (pressed_duration >= BUTTON_LONG_TIMEOUT)
    {
        wiced_stop_timer(&button_timer);
        vg_press = 0;
        pressed_duration = 0;
    }
}

void volume_gain_button_init( void )
{
    /* Configure GPIO PIN# as input, pull up and interrupt on rising edge and output value as high
     *  (pin should be configured before registering interrupt handler ) */
    wiced_hal_gpio_configure_pin( app_ag_button_gpio, WICED_GPIO_BUTTON_SETTINGS( GPIO_EN_INT_BOTH_EDGE ), GPIO_PIN_OUTPUT_LOW );
    wiced_hal_gpio_register_pin_for_interrupt( app_ag_button_gpio, button_gpio_interrupt_handler, &app_ag_button_gpio );
    vg_button = HFP_VGM_VGS_DEFAULT;
    vg_increase = TRUE;
    wiced_init_timer(&button_timer, timeout_cb, (TIMER_PARAM_TYPE)NULL, WICED_MILLI_SECONDS_PERIODIC_TIMER);
}

/*
 *  Prepare extended inquiry response data.  Current version publishes headset,
 *  handsfree and generic audio services.
 */
void hci_control_write_eir( void )
{
    uint8_t *pBuf;
    uint8_t *p;
    uint8_t length;

    pBuf = (uint8_t*)wiced_bt_get_buffer( WICED_HS_EIR_BUF_MAX_SIZE );
    WICED_BT_TRACE( "hci_control_write_eir %x\n", pBuf );

    if ( !pBuf )
    {
        return;
    }
    p = pBuf;

    //p = ( uint8_t * )( pBuf + 1 );
    //p += 4;

    length = strlen( (char *)hci_ag_cfg_settings.device_name );

    *p++ = length + 1;
    *p++ = 0x09;            // EIR type full name
    memcpy( p, hci_ag_cfg_settings.device_name, length );
    p += length;
    *p++ = ( 3 * 2 ) + 1;     // length of services + 1
    *p++ =   0x02;            // EIR type full list of 16 bit service UUIDs
    *p++ =   UUID_SERVCLASS_HEADSET         & 0xff;
    *p++ = ( UUID_SERVCLASS_HEADSET >> 8 ) & 0xff;
    *p++ =   UUID_SERVCLASS_HF_HANDSFREE        & 0xff;
    *p++ = ( UUID_SERVCLASS_HF_HANDSFREE >> 8 ) & 0xff;
    *p++ =   UUID_SERVCLASS_GENERIC_AUDIO        & 0xff;
    *p++ = ( UUID_SERVCLASS_GENERIC_AUDIO >> 8 ) & 0xff;
    *p++ = 0;

    // print EIR data
    WICED_BT_TRACE_ARRAY( ( uint8_t* )( pBuf+1 ), MIN( p-( uint8_t* )pBuf,100 ), "EIR :" );
    wiced_bt_dev_write_eir( pBuf, (uint16_t)(p - pBuf) );

    return;
}

#if BTSTACK_VER >= 0x03000001
/*
 * HF event callback. Format the data to be sent over the UART
 */
void hfp_ag_event_callback(uint16_t evt, uint16_t handle,  hfp_ag_event_t *p_data)
{
    uint8_t tx_buf[300];
    uint8_t *p = tx_buf;
    int i;

    WICED_BT_TRACE("[%u]hfp_ag_hci_send_ag_event: Sending Event: %u  to UART\n", handle, evt);

    *p++ = (uint8_t)(handle);
    *p++ = (uint8_t)(handle >> 8);

    switch (evt)
    {
        case HCI_CONTROL_AG_EVENT_OPEN: /* HS connection opened or connection attempt failed  */
            for (i = 0; i < BD_ADDR_LEN; i++)
                *p++ = p_data->open.bd_addr[BD_ADDR_LEN - 1 - i];
            *p++ = p_data->open.status;
            break;

        case HCI_CONTROL_AG_EVENT_CONNECTED: /* HS Service Level Connection is UP */
            *p++ = (uint8_t)(p_data->conn.peer_features);
            *p++ = (uint8_t)(p_data->conn.peer_features >> 8);
            break;
        case HCI_CONTROL_AG_EVENT_AT_CMD:
            memcpy(p, p_data->at_cmd.cmd_ptr, p_data->at_cmd.cmd_len);
            p += p_data->at_cmd.cmd_len;
            break;
        case HCI_CONTROL_AG_EVENT_AUDIO_OPEN:
            *p++ = (uint8_t)(p_data->audio_open.wbs_supported);
            *p++ = (uint8_t)(p_data->audio_open.wbs_used);
            app_ag_use_wbs = (uint8_t)(p_data->audio_open.wbs_used);
            break;
        default: /* Rest have no parameters */
            break;
    }

    wiced_transport_send_data( evt, tx_buf, ( int ) ( p - tx_buf ) );
}
#endif

void hci_control_ag_init( void )
{
    hfp_ag_session_cb_t *p_scb = &hci_control_cb.ag_scb[0];
    wiced_bt_dev_status_t result;
    int i;

    memset( &hci_control_cb, 0, sizeof( hci_control_cb ) );

    for ( i = 0; i < HCI_CONTROL_AG_NUM_SCB; i++, p_scb++ )
    {
        p_scb->app_handle = ( uint16_t ) ( i + 1 );

        if(i == 0)
            p_scb->hf_profile_uuid = UUID_SERVCLASS_HF_HANDSFREE;
        else
            p_scb->hf_profile_uuid = UUID_SERVCLASS_HEADSET;
    }

#if BTSTACK_VER >= 0x03000001
    hfp_ag_startup( &hci_control_cb.ag_scb[0], HCI_CONTROL_AG_NUM_SCB, BT_AUDIO_HFP_SUPPORTED_FEATURES, hfp_ag_event_callback);
#else
    hfp_ag_startup( &hci_control_cb.ag_scb[0], HCI_CONTROL_AG_NUM_SCB, BT_AUDIO_HFP_SUPPORTED_FEATURES );
#endif
}

/*
 *  Pass protocol traces up through the UART
 */
void hci_control_hci_trace_cback( wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t* p_data )
{
#if BTSTACK_VER >= 0x03000001
    //send the trace
    wiced_transport_send_hci_trace( type, p_data, length  );
#else
    //send the trace
    wiced_transport_send_hci_trace( NULL, type, length, p_data  );
#endif
}

/*
 *  Management callback receives various notifications from the stack
 */
wiced_result_t hci_control_management_callback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data )
{
    wiced_result_t                     result = WICED_BT_SUCCESS;
    wiced_bt_dev_status_t              dev_status;
    wiced_bt_dev_ble_pairing_info_t   *p_pairing_info;
    wiced_bt_dev_encryption_status_t  *p_encryption_status;
    int                                bytes_written, bytes_read;
    int                                nvram_id;
    wiced_bt_dev_pairing_cplt_t        *p_pairing_cmpl;
    uint8_t                             pairing_result;

    WICED_BT_TRACE( "hci_control_management_callback 0x%02x\n", event );

    switch( event )
    {
        /* Bluetooth  stack enabled */
        case BTM_ENABLED_EVT:
            hci_control_write_eir( );

            /* initialize everything */
            memset( &hci_control_cb, 0, sizeof( hci_control_cb ) );

            /* create SDP records */
            wiced_bt_sdp_db_init( ( uint8_t * )hci_control_sdp_db, sizeof( hci_control_sdp_db ) );

            /* Enable HCI traces by default */
            wiced_bt_dev_register_hci_trace( hci_control_hci_trace_cback );

            /* Perform the rfcomm init before hf start up */
            if( (wiced_bt_rfcomm_result_t)wiced_bt_rfcomm_init( 200, 5 ) != WICED_BT_RFCOMM_SUCCESS )
            {
                result = WICED_BT_ERROR;
                WICED_BT_TRACE("Error Initializing RFCOMM failed\n");
                break;
            }

            hci_control_ag_init( );

#if BTSTACK_VER >= 0x03000001
            p_key_info_pool = wiced_bt_create_pool( "key_info", KEY_INFO_POOL_BUFFER_SIZE, KEY_INFO_POOL_BUFFER_COUNT, NULL );
#else
            p_key_info_pool = wiced_bt_create_pool( KEY_INFO_POOL_BUFFER_SIZE, KEY_INFO_POOL_BUFFER_COUNT );
#endif
            WICED_BT_TRACE( "wiced_bt_create_pool %x\n", p_key_info_pool );

            volume_gain_button_init(); //Use WICED_GPIO_BUTTON to send "+VGM(S)" to HS

            hci_control_send_device_started_evt( );

            wiced_bt_sco_setup_voice_path(&audiogateway_sco_path);
#if defined(CYW55572A1)
            wiced_am_init();
            //Open external codec first to prevent DSP download delay later
            stream_id = wiced_am_stream_open(HFP);
            if (stream_id == WICED_AUDIO_MANAGER_STREAM_ID_INVALID)
            {
                WICED_BT_TRACE("wiced_am_stream_open failed\n");
            }
            else
            {
                if (wiced_am_stream_close(stream_id) != WICED_SUCCESS)
                {
                    WICED_BT_TRACE("Err: wiced_am_stream_close\n");
                }
                else
                {
                    WICED_BT_TRACE("Init external codec done\n");
                }
                stream_id = WICED_AUDIO_MANAGER_STREAM_ID_INVALID;
            }
#endif
            break;

        case BTM_DISABLED_EVT:
            break;

        case BTM_PIN_REQUEST_EVT:
            WICED_BT_TRACE("remote address= %B\n", p_event_data->pin_request.bd_addr);
            wiced_bt_dev_pin_code_reply(*p_event_data->pin_request.bd_addr,result/*WICED_BT_SUCCESS*/,4, &pincode[0]);
        break;


        case BTM_USER_CONFIRMATION_REQUEST_EVT:
            /* User confirmation request for pairing (sample app always accepts) */
            wiced_bt_dev_confirm_req_reply (WICED_BT_SUCCESS, p_event_data->user_confirmation_request.bd_addr);
            break;

        case BTM_PAIRING_IO_CAPABILITIES_BR_EDR_REQUEST_EVT:
            /* Use the default security for BR/EDR*/
            WICED_BT_TRACE("BTM_PAIRING_IO_CAPABILITIES_REQUEST_EVT bda %B\n",
                                            p_event_data->pairing_io_capabilities_br_edr_request.bd_addr);
            p_event_data->pairing_io_capabilities_br_edr_request.local_io_cap = BTM_IO_CAPABILITIES_NONE;
            p_event_data->pairing_io_capabilities_br_edr_request.auth_req = BTM_AUTH_SINGLE_PROFILE_GENERAL_BONDING_NO;
            p_event_data->pairing_io_capabilities_br_edr_request.oob_data     = WICED_FALSE;
            break;

        case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT:
            p_event_data->pairing_io_capabilities_ble_request.local_io_cap          = BTM_IO_CAPABILITIES_NONE;
            p_event_data->pairing_io_capabilities_ble_request.oob_data   = BTM_OOB_NONE;
            p_event_data->pairing_io_capabilities_ble_request.auth_req   = BTM_LE_AUTH_REQ_BOND | BTM_LE_AUTH_REQ_MITM;
            p_event_data->pairing_io_capabilities_ble_request.max_key_size  = 0x10;
            p_event_data->pairing_io_capabilities_ble_request.init_keys     = BTM_LE_KEY_PENC|BTM_LE_KEY_PID|BTM_LE_KEY_PCSRK|BTM_LE_KEY_LENC;
            p_event_data->pairing_io_capabilities_ble_request.resp_keys     = BTM_LE_KEY_PENC|BTM_LE_KEY_PID|BTM_LE_KEY_PCSRK|BTM_LE_KEY_LENC;
            break;


        case BTM_PAIRING_COMPLETE_EVT:
            p_pairing_cmpl = &p_event_data->pairing_complete;

            if(p_pairing_cmpl->transport == BT_TRANSPORT_BR_EDR)
            {
                pairing_result = p_pairing_cmpl->pairing_complete_info.br_edr.status;
            }
            else
            {
                pairing_result = p_pairing_cmpl->pairing_complete_info.ble.reason;
            }

            hci_control_send_pairing_completed_evt( pairing_result, p_event_data->pairing_complete.bd_addr );
            break;

        case BTM_ENCRYPTION_STATUS_EVT:
            p_encryption_status = &p_event_data->encryption_status;

            WICED_BT_TRACE( "Encryption Status Event: bd ( %B ) res %d\n", p_encryption_status->bd_addr, p_encryption_status->result );

            hci_control_send_encryption_changed_evt ( p_encryption_status->result, p_encryption_status->bd_addr);
            break;

        case BTM_SECURITY_REQUEST_EVT:
            wiced_bt_ble_security_grant( p_event_data->security_request.bd_addr, WICED_BT_SUCCESS );
            break;

        case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT:
            /* Check if we already have information saved for this bd_addr */
            if ( ( nvram_id = hci_control_find_nvram_id( p_event_data->paired_device_link_keys_update.bd_addr, BD_ADDR_LEN ) ) == 0)
            {
                /* This is the first time, allocate id for the new memory chunk */
                nvram_id = hci_control_alloc_nvram_id( );
                WICED_BT_TRACE( "Allocated NVRAM ID:%d\n", nvram_id );
            }
            bytes_written = hci_control_write_nvram( nvram_id, sizeof( wiced_bt_device_link_keys_t ), &p_event_data->paired_device_link_keys_update, FALSE );

            WICED_BT_TRACE("NVRAM write:id:%d bytes:%d dev: [%B]\n", nvram_id, bytes_written, p_event_data->paired_device_link_keys_update.bd_addr);
//            WICED_BT_TRACE_ARRAY(((uint8_t *)&p_event_data->paired_device_link_keys_update.key_data), sizeof(p_event_data->paired_device_link_keys_update.key_data), "key: ");
//            WICED_BT_TRACE("\n");
            break;

        case  BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
            /* read existing key from the NVRAM  */

            WICED_BT_TRACE("\t\tfind device %B\n", p_event_data->paired_device_link_keys_request.bd_addr);

            if ( ( nvram_id = hci_control_find_nvram_id( p_event_data->paired_device_link_keys_request.bd_addr, BD_ADDR_LEN ) ) != 0)
            {
                 bytes_read = hci_control_read_nvram( nvram_id, &p_event_data->paired_device_link_keys_request, sizeof( wiced_bt_device_link_keys_t ) );

                 result = WICED_BT_SUCCESS;
                 WICED_BT_TRACE("Read:nvram_id:%d bytes:%d\n", nvram_id, bytes_read);
            }
            else
            {
                result = WICED_BT_ERROR;
                WICED_BT_TRACE("Key retrieval failure\n");
            }
            break;

        case BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT:
            /* Request to store newly generated local identity keys to NVRAM */
            /* (sample app does not store keys to NVRAM) */

            break;


        case BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT:
            /*
             * Request to restore local identity keys from NVRAM
             * (requested during Bluetooth start up)
             * */
            /* (sample app does not store keys to NVRAM)
             * New local identity keys will be generated
             * */
            result = WICED_BT_NO_RESOURCES;
            break;

        case BTM_SCO_CONNECTED_EVT:
            hfp_ag_sco_management_callback( event, p_event_data );
#if defined(CYW55572A1)
            /* setup audio path */
            if (stream_id == WICED_AUDIO_MANAGER_STREAM_ID_INVALID)
            {
                stream_id = wiced_am_stream_open(HFP);
                WICED_BT_TRACE("wiced_am_stream_open completed stream_id: %d\n", stream_id);
            }

            /* Set sample rate. */
            /* app_ag_use_wbs will be updated after executing hfp_ag_sco_management_callback */
            if (app_ag_use_wbs == WICED_TRUE)
            {
                audio_config.sr = AM_PLAYBACK_SR_16K;
            }
            else
            {
                audio_config.sr = AM_PLAYBACK_SR_8K;
            }

            audio_config.volume = AM_VOL_LEVEL_HIGH - 2;
            audio_config.mic_gain = AM_VOL_LEVEL_HIGH - 2;

            if( WICED_SUCCESS != wiced_am_stream_set_param(stream_id, AM_AUDIO_CONFIG, &audio_config))
                WICED_BT_TRACE("wiced_am_set_param failed\n");

            if( WICED_SUCCESS != wiced_am_stream_start(stream_id))
                WICED_BT_TRACE("wiced_am_stream_start failed stream_id : %d \n", stream_id);

            /* Set speaker volume and MIC gain to make the volume consistency between call
             * sessions. */
            if (WICED_SUCCESS != wiced_am_stream_set_param(stream_id, AM_SPEAKER_VOL_LEVEL, (void *) &audio_config.volume))
                WICED_BT_TRACE("wiced_am_set_param failed\n");

            if (WICED_SUCCESS != wiced_am_stream_set_param(stream_id, AM_MIC_GAIN_LEVEL, (void *) &audio_config.mic_gain))
                WICED_BT_TRACE("wiced_am_set_param failed\n");
#endif
            break;

        case BTM_SCO_DISCONNECTED_EVT:
#if defined(CYW55572A1)
            if (stream_id != WICED_AUDIO_MANAGER_STREAM_ID_INVALID)
            {
                if( WICED_SUCCESS != wiced_am_stream_stop(stream_id))
                    WICED_BT_TRACE("wiced_am_stream_stop failed stream_id : %d \n", stream_id);

                if( WICED_SUCCESS != wiced_am_stream_close(stream_id))
                    WICED_BT_TRACE("wiced_am_stream_close failed stream_id : %d \n", stream_id);

                stream_id = WICED_AUDIO_MANAGER_STREAM_ID_INVALID;
            }
#endif
            hfp_ag_sco_management_callback( event, p_event_data );
            break;

        case BTM_SCO_CONNECTION_REQUEST_EVT:
        case BTM_SCO_CONNECTION_CHANGE_EVT:
            hfp_ag_sco_management_callback( event, p_event_data );
            break;

        default:
            result = WICED_BT_USE_DEFAULT_SECURITY;
            break;
    }
    return result;
}

/*
 * Handle received command over UART. Please refer to the WICED HCI Control Protocol
 * Software User Manual (WICED-SWUM10x-R) for details on the
 * HCI UART control protocol.
*/
static uint32_t hci_control_proc_rx_cmd( uint8_t *p_data, uint32_t length )
{
    uint16_t opcode;
    uint16_t payload_len;
    uint8_t status = HCI_CONTROL_STATUS_SUCCESS;
    uint8_t* p_rx_buf = p_data;

    WICED_BT_TRACE( "hci_control_proc_rx_cmd:%d, %x \n", length ,p_rx_buf);

    if ( !p_rx_buf )
    {
        return HCI_CONTROL_STATUS_INVALID_ARGS;
    }
    //Expected minimum 4 byte as the wiced header
    if( length < 4 )
    {
        WICED_BT_TRACE("invalid params\n");
#ifndef BTSTACK_VER
        wiced_transport_free_buffer( p_rx_buf );
#endif
        return HCI_CONTROL_STATUS_INVALID_ARGS;
    }


    READ_LITTLE_ENDIAN_TO_UINT16(opcode, p_data, length);     // Get opcode
    READ_LITTLE_ENDIAN_TO_UINT16(payload_len, p_data, length); // Get len

    WICED_BT_TRACE("cmd_opcode 0x%02x\n", opcode);

    switch((opcode >> 8) & 0xff)
    {
    case HCI_CONTROL_GROUP_DEVICE:
        hci_control_device_handle_command( opcode, p_data, payload_len );
        break;

    case HCI_CONTROL_GROUP_AG:
        hci_control_ag_handle_command( opcode, p_data, payload_len );
        break;

    case HCI_CONTROL_GROUP_MISC:
        hci_control_misc_handle_command(opcode, p_data, payload_len);
        break;

    default:
        WICED_BT_TRACE( "unknown class code\n");
        break;
    }

#ifndef BTSTACK_VER
    //Freeing the buffer in which data is received
    wiced_transport_free_buffer( p_rx_buf );
#endif
    return status;
}

void hci_control_device_handle_command( uint16_t cmd_opcode, uint8_t* p_data, uint32_t data_len )
{
    uint8_t bytes_written;
    switch( cmd_opcode )
    {
    case HCI_CONTROL_COMMAND_RESET:
        hci_control_handle_reset_cmd( );
        break;

    case HCI_CONTROL_COMMAND_TRACE_ENABLE:
        hci_control_handle_trace_enable( p_data );
        break;

    case HCI_CONTROL_COMMAND_SET_LOCAL_BDA:
        hci_control_handle_set_local_bda( p_data );
        break;

    case HCI_CONTROL_COMMAND_PUSH_NVRAM_DATA:
        bytes_written = hci_control_write_nvram( p_data[0] | ( p_data[1] << 8 ), data_len - 2, &p_data[2], TRUE );
        WICED_BT_TRACE( "NVRAM write: %d dev: [%B]\n", bytes_written , &p_data[2]);
        break;

    case HCI_CONTROL_COMMAND_DELETE_NVRAM_DATA:
        hci_control_delete_nvram( p_data[0] | ( p_data[1] << 8 ), WICED_TRUE);
        WICED_BT_TRACE( "NVRAM delete: %d\n", p_data[0] | ( p_data[1] << 8 ) );
        break;

    case HCI_CONTROL_COMMAND_INQUIRY:
        hci_control_inquiry( *p_data );
        break;

    case HCI_CONTROL_COMMAND_SET_VISIBILITY:
        hci_control_handle_set_visibility( p_data[0], p_data[1] );
        break;

    case HCI_CONTROL_COMMAND_SET_PAIRING_MODE:
        hci_control_handle_set_pairability( p_data[0] );
        break;

    default:
        WICED_BT_TRACE( "??? Unknown command code\n" );
        break;
    }
}

/*
 * Handle Handsfree commands received over UART.
 */
void hci_control_ag_handle_command( uint16_t opcode, uint8_t* p_data, uint32_t length )
{
    uint16_t handle;
    uint8_t  hs_cmd;
    uint8_t  *p = ( uint8_t * ) p_data;

    switch ( opcode )
    {
    case HCI_CONTROL_AG_COMMAND_CONNECT:
        hfp_ag_connect( p );
        break;

    case HCI_CONTROL_AG_COMMAND_DISCONNECT:
        handle = p[0] | ( p[1] << 8 );
        hfp_ag_disconnect( handle );
        break;

    case HCI_CONTROL_AG_COMMAND_OPEN_AUDIO:
        handle = p[0] | ( p[1] << 8 );
        hfp_ag_audio_open( handle );
        break;

    case HCI_CONTROL_AG_COMMAND_CLOSE_AUDIO:
        handle = p[0] | ( p[1] << 8 );
        hfp_ag_audio_close( handle );
        break;

    case HCI_CONTROL_AG_COMMAND_SET_CIND:
        hfp_ag_set_cind((char *)&p[0], length);
        break;

    case HCI_CONTROL_AG_COMMAND_STR:
        handle = p[0] | ( p[1] << 8 );
        hfp_ag_send_cmd_str(handle, &p[2], length-2);
        break;

    default:
        WICED_BT_TRACE ( "hci_control_ag_handle_command - unkn own opcode: %u %u\n", opcode);
        break;
    }
}

/*
 * handle reset command from UART
 */
void hci_control_handle_reset_cmd( void )
{
    // trip watch dog now.
    wiced_hal_wdog_reset_system( );
}

/*
 * handle command from UART to configure traces
 */
void hci_control_handle_trace_enable( uint8_t *p_data )
{
    uint8_t hci_trace_enable = *p_data++;
    wiced_debug_uart_types_t route_debug = (wiced_debug_uart_types_t)*p_data;

    if ( hci_trace_enable )
    {
        /* Register callback for receiving hci traces */
        // Disable while streaming audio over the uart.
        wiced_bt_dev_register_hci_trace( hci_control_hci_trace_cback );
    }
    else
    {
        wiced_bt_dev_register_hci_trace( NULL);
    }

    wiced_set_debug_uart( route_debug );
}

/*
 * handle command to set local Bluetooth device address
 */
void hci_control_handle_set_local_bda( uint8_t *p_bda)
{
    BD_ADDR bd_addr;
    STREAM_TO_BDADDR(bd_addr,p_bda);
    wiced_bt_set_local_bdaddr( bd_addr, BLE_ADDR_PUBLIC );

    hci_control_send_command_status_evt( HCI_CONTROL_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_SUCCESS );
}
/*
 *  Handle Set Pairability command received over UART
 */
static void hci_control_handle_set_pairability ( uint8_t pairing_allowed )
{
    uint8_t                   status = HCI_CONTROL_STATUS_SUCCESS;
    hci_control_nvram_chunk_t *p1;

    if ( hci_control_cb.pairing_allowed != pairing_allowed )
    {
        if ( pairing_allowed )
        {
            // Check if key buffer pool has buffer available. If not, cannot enable pairing until nvram entries are deleted
            if ( ( p1 = ( hci_control_nvram_chunk_t * )wiced_bt_get_buffer_from_pool( p_key_info_pool ) ) == NULL)
            {
                pairing_allowed = 0; //The key buffer pool is full therefore we cannot allow pairing to be enabled
                status = HCI_CONTROL_STATUS_OUT_OF_MEMORY;
            }
            else
            {
                wiced_bt_free_buffer( p1 );
            }
        }

        hci_control_cb.pairing_allowed = pairing_allowed;
        wiced_bt_set_pairable_mode( hci_control_cb.pairing_allowed, 0 );
        WICED_BT_TRACE( " Set the pairing allowed to %d \n", hci_control_cb.pairing_allowed );
    }

    hci_control_send_command_status_evt( HCI_CONTROL_EVENT_COMMAND_STATUS, status );
}

/*
 *  Handle Inquiry result callback from teh stack, format and send event over UART
 */
void hci_control_inquiry_result_cback( wiced_bt_dev_inquiry_scan_result_t *p_inquiry_result, uint8_t *p_eir_data )
{
    int       i;
    uint8_t   len;
    uint8_t   tx_buf[300];
    uint16_t  code;
    uint8_t   *p = tx_buf;

    if ( p_inquiry_result == NULL )
    {
        code = HCI_CONTROL_EVENT_INQUIRY_COMPLETE;
        WICED_BT_TRACE( "inquiry complete \n");
    }
    else
    {
        code = HCI_CONTROL_EVENT_INQUIRY_RESULT;
        WICED_BT_TRACE( "inquiry result %B\n", p_inquiry_result->remote_bd_addr );
        for ( i = 0; i < 6; i++ )
            *p++ = p_inquiry_result->remote_bd_addr[5 - i];
        for ( i = 0; i < 3; i++ )
            *p++ = p_inquiry_result->dev_class[2 - i];
        *p++ = p_inquiry_result->rssi;

        // currently callback does not pass the data of the adv data, need to go through the data
        // zero len in the LTV means that there is no more data
        while (p_eir_data && ( len = *p_eir_data ) != 0 )
        {
            // In the HCI event all parameters should fit into 255 bytes
            if ( p + len + 1 > tx_buf + 255 )
            {
                WICED_BT_TRACE( "Bad data\n" );
                break;
            }
            for ( i = 0; i < len + 1; i++ )
                *p++ = *p_eir_data++;
        }
    }
    wiced_transport_send_data( code, tx_buf, ( int )( p - tx_buf ) );
}

/*
 *  Handle Inquiry command received over UART
 */
void hci_control_inquiry( uint8_t enable )
{
    wiced_result_t           result;
    wiced_bt_dev_inq_parms_t params;

    if ( enable )
    {

        memset( &params, 0, sizeof( params ) );

        params.mode             = BTM_GENERAL_INQUIRY;
        params.duration         = 5;
        params.filter_cond_type = BTM_CLR_INQUIRY_FILTER;

        result = wiced_bt_start_inquiry( &params, &hci_control_inquiry_result_cback );
        WICED_BT_TRACE( "inquiry started:%d\n", result );
    }
    else
    {
        result = wiced_bt_cancel_inquiry( );
        WICED_BT_TRACE( "cancel inquiry:%d\n", result );
    }
    hci_control_send_command_status_evt( HCI_CONTROL_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_SUCCESS );
}

/*
 *  Handle Set Visibility command received over UART
 */
void hci_control_handle_set_visibility( uint8_t discoverability, uint8_t connectability )
{
    wiced_bt_dev_status_t dev_status;
    uint8_t               visibility;
    uint8_t               status = HCI_CONTROL_STATUS_SUCCESS;

    // we cannot be discoverable and not connectable
    if ( ( ( discoverability != 0 ) && ( connectability == 0 ) ) ||
           ( discoverability > 1 ) ||
           ( connectability > 1 ) )
    {
        hci_control_send_command_status_evt( HCI_CONTROL_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_INVALID_ARGS );
    }
    else
    {
        wiced_bt_dev_set_discoverability( ( discoverability != 0 ) ? BTM_GENERAL_DISCOVERABLE : BTM_NON_DISCOVERABLE ,
                                            BTM_DEFAULT_DISC_WINDOW,
                                            BTM_DEFAULT_DISC_INTERVAL);

        wiced_bt_dev_set_connectability( ( connectability != 0 ) ? WICED_TRUE : WICED_FALSE ,
                                            BTM_DEFAULT_CONN_WINDOW,
                                            BTM_DEFAULT_CONN_INTERVAL);

        hci_control_send_command_status_evt( HCI_CONTROL_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_SUCCESS );
    }
}


/*
 *  Send Device Started event through UART
 */
void hci_control_send_device_started_evt( void )
{
    wiced_transport_send_data( HCI_CONTROL_EVENT_DEVICE_STARTED, NULL, 0 );

#if BTSTACK_VER >= 0x03000001
    WICED_BT_TRACE( "maxChannels:%d maxpsm:%d rfcom max links%d, rfcom max ports:%d\n",
            hci_ag_cfg_settings.p_l2cap_app_cfg->max_app_l2cap_channels,
            hci_ag_cfg_settings.p_l2cap_app_cfg->max_app_l2cap_psms,
            hci_ag_cfg_settings.p_br_cfg->rfcomm_cfg.max_links,
            hci_ag_cfg_settings.p_br_cfg->rfcomm_cfg.max_ports );
#else
    WICED_BT_TRACE( "maxLinks:%d maxChannels:%d maxpsm:%d rfcom max links%d, rfcom max ports:%d\n",
            hci_ag_cfg_settings.l2cap_application.max_links,
            hci_ag_cfg_settings.l2cap_application.max_channels,
            hci_ag_cfg_settings.l2cap_application.max_psm,
            hci_ag_cfg_settings.rfcomm_cfg.max_links,
            hci_ag_cfg_settings.rfcomm_cfg.max_ports );
#endif

}

/*
* transfer command status event to UART
*/
void hci_control_send_command_status_evt( uint16_t code, uint8_t status )
{
    wiced_transport_send_data( code, &status, 1 );
}

/*
 *  Send Pairing Completed event through UART
 */
void hci_control_send_pairing_completed_evt( uint8_t status , wiced_bt_device_address_t bdaddr )
{
    const int cmd_size = BD_ADDR_LEN + sizeof(uint8_t);
    uint8_t event_data[cmd_size];
    int     cmd_bytes = 0;

    event_data[cmd_bytes++] = status;

    {
        int i;
        for ( i = 0 ; i < BD_ADDR_LEN; i++ )                     // bd address
            event_data[cmd_bytes++] = bdaddr[BD_ADDR_LEN - 1 - i];
    }

    WICED_BT_TRACE(" sending the pairing complete evt: %B as %B status %d\n", bdaddr, &event_data[1], status);

    wiced_transport_send_data(HCI_CONTROL_EVENT_PAIRING_COMPLETE, event_data, cmd_bytes );
}

/*
 *  Send Encryption Changed event through UART
 */
void hci_control_send_encryption_changed_evt( uint8_t encrypted ,  wiced_bt_device_address_t bdaddr )
{
    const int cmd_size = BD_ADDR_LEN + sizeof(uint8_t);
    uint8_t event_data[cmd_size];
    int     cmd_bytes = 0;

    event_data[cmd_bytes++] = encrypted;

    {
        int i;
        for ( i = 0 ; i < BD_ADDR_LEN; i++ )                     // bd address
            event_data[cmd_bytes++] = bdaddr[BD_ADDR_LEN - 1 - i];
    }

    wiced_transport_send_data( HCI_CONTROL_EVENT_ENCRYPTION_CHANGED, event_data, cmd_bytes );
}

/*
 *  send audio connect complete event to UART
 */
wiced_result_t hci_control_audio_send_connect_complete( wiced_bt_device_address_t bd_addr, uint8_t status, uint16_t handle )
{
    int i;
    const int cmd_size = BD_ADDR_LEN + sizeof(handle) + sizeof(uint8_t);
    uint8_t event_data[cmd_size];

    WICED_BT_TRACE( "[%s] %B status %x handle %x\n", __FUNCTION__, bd_addr, status, handle );

    //Build event payload
    if ( status == WICED_SUCCESS )
    {
        for ( i = 0; i < BD_ADDR_LEN; i++ )                     // bd address
            event_data[i] = bd_addr[BD_ADDR_LEN - 1 - i];

        event_data[i++] = handle & 0xff;                        //handle
        event_data[i++]   = ( handle >> 8 ) & 0xff;

        event_data[i] = avrc_is_abs_volume_capable();

        return wiced_transport_send_data( HCI_CONTROL_AUDIO_EVENT_CONNECTED, event_data, cmd_size );
    }
    else
    {
        return wiced_transport_send_data( HCI_CONTROL_AUDIO_EVENT_CONNECTION_FAILED, NULL, 0 );
    }
}

/*
 *  send audio disconnect complete event to UART
 */
wiced_result_t hci_control_audio_send_disconnect_complete( uint16_t handle, uint8_t status, uint8_t reason )
{
    int i;
    uint8_t event_data[4];

    WICED_BT_TRACE( "[%s] %04x status %d reason %d\n", __FUNCTION__, handle, status, reason );

    //Build event payload
    event_data[0] = handle & 0xff;                          //handle
    event_data[1] = ( handle >> 8 ) & 0xff;
    event_data[2] = status;                                 // status
    event_data[3] = reason;                                 // reason(1 byte)

    return wiced_transport_send_data( HCI_CONTROL_AUDIO_EVENT_DISCONNECTED, event_data, 4 );
}

/*
 *  send audio connect complete event to UART
 */
wiced_result_t hci_control_audio_send_started_stopped( uint16_t handle, wiced_bool_t started )
{
    uint8_t event_data[2];

    WICED_BT_TRACE( "[%s] handle %04x\n", __FUNCTION__, handle );

    //Build event payload
    event_data[0] = handle & 0xff;                          //handle
    event_data[1] = ( handle >> 8 ) & 0xff;

    return wiced_transport_send_data(started ? HCI_CONTROL_AUDIO_EVENT_STARTED : HCI_CONTROL_AUDIO_EVENT_STOPPED, event_data, 2);
}

/*
 * Write NVRAM function is called to store information in the RAM.  This can be called when
 * stack requires persistent storage, for example to save link keys.  In this case
 * data is also formatted and send to the host for real NVRAM storage.  The same function is
 * called when host pushes NVRAM chunks during the startup.  Parameter from_host in this
 * case is set to FALSE indicating that data does not need to be forwarded.
 */
int hci_control_write_nvram( int nvram_id, int data_len, void *p_data, BOOLEAN from_host )
{
    uint8_t                    tx_buf[257];
    uint8_t                   *p = tx_buf;
    hci_control_nvram_chunk_t *p1;
    wiced_result_t            result;

#if BTSTACK_VER >= 0x03000001
    wiced_bt_device_link_keys_t data;
    wiced_bt_device_link_keys_t_20721 * p_data_from_host;
    wiced_bt_device_link_keys_t_20721 device_link_key_data;

    if (from_host)
    {
        p_data_from_host = (wiced_bt_device_link_keys_t_20721 *)p_data;
        memset(&data, 0, sizeof(wiced_bt_device_link_keys_t));
        memcpy(&data.bd_addr, &p_data_from_host->bd_addr, sizeof(wiced_bt_device_address_t));
        data.key_data.br_edr_key_type = p_data_from_host->key_data.br_edr_key_type;
        memcpy(&data.key_data.br_edr_key, &p_data_from_host->key_data.br_edr_key, sizeof(wiced_bt_link_key_t));
        data.key_data.le_keys_available_mask = p_data_from_host->key_data.le_keys_available_mask;
        data.key_data.ble_addr_type = p_data_from_host->key_data.ble_addr_type;
        memcpy(&data.key_data.le_keys, &p_data_from_host->key_data.le_keys, sizeof(wiced_bt_ble_keys_t));
        data_len = sizeof(wiced_bt_device_link_keys_t);
        p_data = &data;
}
#endif

    /* first check if this ID is being reused and release the memory chunk */
    hci_control_delete_nvram( nvram_id ,WICED_FALSE);

    if ( ( p1 = ( hci_control_nvram_chunk_t * )wiced_bt_get_buffer_from_pool( p_key_info_pool ) ) == NULL )
    {
        WICED_BT_TRACE( "Failed to alloc:%d\n", data_len );
        return ( 0 );
    }
    if ( wiced_bt_get_buffer_size( p1 ) < ( sizeof( hci_control_nvram_chunk_t ) + data_len - 1 ) )
    {
        WICED_BT_TRACE( "Insufficient buffer size, Buff Size %d, Len %d  \n",
                        wiced_bt_get_buffer_size( p1 ), ( sizeof( hci_control_nvram_chunk_t ) + data_len - 1 ) );
        wiced_bt_free_buffer( p1 );
        return ( 0 );
    }
    p1->p_next    = p_nvram_first;
    p1->nvram_id  = nvram_id;
    p1->chunk_len = data_len;
    memcpy( p1->data, p_data, data_len );

    p_nvram_first = p1;

    wiced_bt_device_link_keys_t * p_keys = ( wiced_bt_device_link_keys_t *) p_data;
#ifdef CYW20706A2
    result = wiced_bt_dev_add_device_to_address_resolution_db( p_keys ,
            p_keys->key_data.ble_addr_type );
#else
    result = wiced_bt_dev_add_device_to_address_resolution_db( p_keys );
#endif

    WICED_BT_TRACE("Updated Addr Resolution DB:%d\n", result );

    /* If NVRAM chunk arrived from host, no need to send it back, otherwise send over transport */
    if (!from_host)
    {
        *p++ = nvram_id & 0xff;
        *p++ = (nvram_id >> 8) & 0xff;
#if BTSTACK_VER >= 0x03000001
        memset(&device_link_key_data, 0, sizeof(wiced_bt_device_link_keys_t_20721));
        memcpy(&device_link_key_data.bd_addr, &p_keys->bd_addr, sizeof(wiced_bt_device_address_t));

        device_link_key_data.key_data.br_edr_key_type = p_keys->key_data.br_edr_key_type;
        memcpy(&device_link_key_data.key_data.br_edr_key, &p_keys->key_data.br_edr_key, sizeof(wiced_bt_link_key_t));
        device_link_key_data.key_data.le_keys_available_mask = p_keys->key_data.le_keys_available_mask;
        device_link_key_data.key_data.ble_addr_type = p_keys->key_data.ble_addr_type;
        memcpy(&device_link_key_data.key_data.le_keys, &p_keys->key_data.le_keys, sizeof(wiced_bt_ble_keys_t));
        memcpy(p, &device_link_key_data, sizeof(wiced_bt_device_link_keys_t_20721));
        data_len = sizeof(wiced_bt_device_link_keys_t_20721);

#else
        memcpy(p, p_data, data_len);
#endif
        wiced_transport_send_data( HCI_CONTROL_EVENT_NVRAM_DATA, tx_buf, ( int )( data_len + 2 ) );
    }
    return (data_len);
}

/*
 * Find nvram_id of the NVRAM chunk with first bytes matching specified byte array
 */
int hci_control_find_nvram_id(uint8_t *p_data, int len)
{
    hci_control_nvram_chunk_t *p1;

    /* Go through the linked list of chunks */
    for (p1 = p_nvram_first; p1 != NULL; p1 = (hci_control_nvram_chunk_t *)p1->p_next)
    {
        WICED_BT_TRACE( "find %B %B len:%d\n", p1->data, p_data, len );
        if ( memcmp( p1->data, p_data, len ) == 0 )
        {
            return ( p1->nvram_id );
        }
    }
    return HCI_CONTROL_INVALID_NVRAM_ID;
}

/*
 * Delete NVRAM function is called when host deletes NVRAM chunk from the persistent storage.
 */
void hci_control_delete_nvram( int nvram_id ,wiced_bool_t from_host)
{
    hci_control_nvram_chunk_t *p1, *p2;

    if ( p_nvram_first == NULL )
        return;

    /* Special case when need to remove the first chunk */
    if ( ( p_nvram_first != NULL ) && ( p_nvram_first->nvram_id == nvram_id ) )
    {
        p1 = p_nvram_first;

        if ( from_host && ( wiced_bt_dev_delete_bonded_device (p1->data) == WICED_ERROR ) )
        {
            WICED_BT_TRACE("ERROR: while Unbonding device \n");
        }
        else
        {
            p_nvram_first = (hci_control_nvram_chunk_t *)p_nvram_first->p_next;
            wiced_bt_free_buffer( p1 );
        }
        return;
    }

    /* Go through the linked list of chunks */
    for ( p1 = p_nvram_first; p1 != NULL; p1 = (hci_control_nvram_chunk_t *)p1->p_next )
    {
        p2 = (hci_control_nvram_chunk_t *)p1->p_next;
        if ( ( p2 != NULL ) && ( p2->nvram_id == nvram_id ) )
        {
            if ( from_host && ( wiced_bt_dev_delete_bonded_device (p2->data) == WICED_ERROR ) )
            {
                WICED_BT_TRACE("ERROR: while Unbonding device \n");
            }
            else
            {
                p1->p_next = p2->p_next;
                wiced_bt_free_buffer( p2 );
            }
            return;
        }
    }
}

/*
 * Read NVRAM actually finds the memory chunk in the RAM
 */
int hci_control_read_nvram( int nvram_id, void *p_data, int data_len )
{
    hci_control_nvram_chunk_t *p1;
    int                        data_read = 0;

    /* Go through the linked list of chunks */
    for ( p1 = p_nvram_first; p1 != NULL; p1 = p1->p_next )
    {
        if ( p1->nvram_id == nvram_id )
        {
            data_read = ( data_len < p1->chunk_len ) ? data_len : p1->chunk_len;
            memcpy( p_data, p1->data, data_read );
            break;
        }
    }
    return ( data_read );
}

/*
 * Allocate nvram_id to save new NVRAM chunk
 */
int hci_control_alloc_nvram_id( )
{
    hci_control_nvram_chunk_t *p1 = p_nvram_first;
    int                        nvram_id;

    /* Go through the linked list of chunks */
    WICED_BT_TRACE ( "hci_control_alloc_nvram_id\n" );
    for ( nvram_id = HCI_CONTROL_FIRST_VALID_NVRAM_ID; p1 != NULL; nvram_id++ )
    {
        for ( p1 = p_nvram_first; p1 != NULL; p1 = (hci_control_nvram_chunk_t *)p1->p_next )
        {
            if ( p1->nvram_id == nvram_id )
            {
                /* this nvram_id is already used */
                break;
            }
        }
        if ( p1 == NULL )
        {
            break;
        }
    }
    WICED_BT_TRACE ( "hci_control_alloc_nvram_id:%d\n", nvram_id );
    return ( nvram_id );
}

#if BTSTACK_VER >= 0x03000001
static void hci_control_tx_complete(void)
{
    return;
}
#else
static void hci_control_tx_complete( wiced_transport_buffer_pool_t* p_pool )
{
    WICED_BT_TRACE ( "hci_control_tx_complete :%x \n", p_pool );
}
#endif

/* Handle misc command group */
void hci_control_misc_handle_command( uint16_t cmd_opcode, uint8_t* p_data, uint32_t data_len )
{
    switch( cmd_opcode )
    {
    case HCI_CONTROL_MISC_COMMAND_GET_VERSION:
        hci_control_misc_handle_get_version();
        break;
    }
}


/* Handle get version command */
void hci_control_misc_handle_get_version( void )
{
    uint8_t   tx_buf[15];
    uint8_t   cmd = 0;
// If this is 20819 or 20820, we do detect the device from hardware
#define RADIO_ID    0x006007c0
#define RADIO_20820 0x80
#define CHIP_20820  20820
#define CHIP_20819  20819
#if (CHIP==CHIP_20819) || (CHIP==CHIP_20820)
    uint32_t chip = CHIP_20819;
    if (*(UINT32*) RADIO_ID & RADIO_20820)
    {
        chip = CHIP_20820;
    }
#else
    uint32_t  chip = CHIP;
#endif

    tx_buf[cmd++] = WICED_SDK_MAJOR_VER;
    tx_buf[cmd++] = WICED_SDK_MINOR_VER;
    tx_buf[cmd++] = WICED_SDK_REV_NUMBER;
    tx_buf[cmd++] = WICED_SDK_BUILD_NUMBER & 0xFF;
    tx_buf[cmd++] = (WICED_SDK_BUILD_NUMBER>>8) & 0xFF;
    tx_buf[cmd++] = chip & 0xFF;
    tx_buf[cmd++] = (chip>>8) & 0xFF;
    tx_buf[cmd++] = (chip>>24) & 0xFF;
    tx_buf[cmd++] = 0; // not used

    /* Send MCU app the supported features */
    tx_buf[cmd++] = HCI_CONTROL_GROUP_AG;

    wiced_transport_send_data( HCI_CONTROL_MISC_EVENT_VERSION, tx_buf, cmd );
}
