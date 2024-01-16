/******************************************************************************
* File Name: main.c
*
* Description: This is the source code for the LE Findme Example
*              for ModusToolbox.
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2020-2024, Cypress Semiconductor Corporation (an Infineon company) or
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

/*******************************************************************************
* Header Files
*******************************************************************************/
#include "wiced_bt_stack.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <string.h>
#include "cybt_platform_trace.h"
#include "wiced_memory.h"
#include "cyhal.h"
#include "stdio.h"
#include "GeneratedSource/cycfg_gatt_db.h"
#include "GeneratedSource/cycfg_bt_settings.h"
#include "GeneratedSource/cycfg_gap.h"
#include "wiced_bt_dev.h"
#include "app_bt_utils.h"
#include "cybsp_bt_config.h"


/*******************************************************************************
* Macros
********************************************************************************/

/* PWM frequency of LED's in Hertz when blinking */
#define IAS_LED_PWM_FREQUENCY          (1)
#define ADV_LED_PWM_FREQUENCY          (1)

/* PWM Duty Cycle of LED's for different states */
enum
{
    LED_ON_DUTY_CYCLE = 0,
    LED_BLINKING_DUTY_CYCLE= 50,
    LED_OFF_DUTY_CYCLE = 100
} led_duty_cycles;

/* IAS Alert Levels */
#define IAS_ALERT_LEVEL_LOW             (0u)
#define IAS_ALERT_LEVEL_MID             (1u)
#define IAS_ALERT_LEVEL_HIGH            (2u)

/* This enumeration combines the advertising, connection states from two different
 * callbacks to maintain the status in a single state variable */
typedef enum
{
    APP_BT_ADV_OFF_CONN_OFF,
    APP_BT_ADV_ON_CONN_OFF,
    APP_BT_ADV_OFF_CONN_ON
} app_bt_adv_conn_mode_t;

/*******************************************************************************
* Variable Definitions
*******************************************************************************/

/* create PWM object for Alert LED */
static cyhal_pwm_t ias_led_pwm;

/* CYBSP_USER_LED2 is only present on some kits. For those kits,it is used to indicate advertising/connection status */
/* create PWM object for second LED only if it exists */
#ifdef CYBSP_USER_LED2
static cyhal_pwm_t                adv_led_pwm;
#endif

static uint16_t                  bt_connection_id = 0;
static app_bt_adv_conn_mode_t    app_bt_adv_conn_state = APP_BT_ADV_OFF_CONN_OFF;

/* This enables RTOS aware debugging. */
volatile int uxTopUsedPriority;

/*******************************************************************************
* Function Prototypes
********************************************************************************/
static void                   ias_led_update                 (void);
static void                   adv_led_update                 (void);
static void                   le_app_init                    (void);
static void*                  app_alloc_buffer               (int len);

static void                   app_free_buffer                (uint8_t *p_event_data);

typedef void                 (*pfn_free_buffer_t)            (uint8_t *);
static gatt_db_lookup_table_t *le_app_find_by_handle         (uint16_t handle);

/* GATT Event Callback Functions */
static wiced_bt_gatt_status_t le_app_write_handler          (uint16_t conn_id,
                                                              wiced_bt_gatt_opcode_t opcode,
                                                              wiced_bt_gatt_write_req_t *p_write_req,
                                                              uint16_t len_req, 
                                                              uint16_t *p_error_handle);
static wiced_bt_gatt_status_t le_app_read_handler           (uint16_t conn_id,
                                                              wiced_bt_gatt_opcode_t opcode,
                                                              wiced_bt_gatt_read_t *p_read_req,
                                                              uint16_t len_req, 
                                                              uint16_t *p_error_handle);
static wiced_bt_gatt_status_t le_app_connect_handler        (wiced_bt_gatt_connection_status_t *p_conn_status);
static wiced_bt_gatt_status_t le_app_server_handler         (wiced_bt_gatt_attribute_request_t *p_attr_req, 
                                                              uint16_t *p_error_handle);
static wiced_bt_gatt_status_t le_app_gatt_event_callback    (wiced_bt_gatt_evt_t  event,
                                                              wiced_bt_gatt_event_data_t *p_event_data);

/* Callback function for Bluetooth stack management type events */
static wiced_bt_dev_status_t  app_bt_management_callback     (wiced_bt_management_evt_t event,
                                                              wiced_bt_management_evt_data_t *p_event_data);
static wiced_bt_gatt_status_t app_bt_gatt_req_read_by_type_handler (uint16_t conn_id,
                                                                    wiced_bt_gatt_opcode_t opcode,
                                                                    wiced_bt_gatt_read_by_type_t *p_read_req, 
                                                                    uint16_t len_requested, 
                                                                    uint16_t *p_error_handle);

/******************************************************************************
 * Function Definitions
 ******************************************************************************/
/*
 *  Entry point to the application. Set device configuration and start BT
 *  stack initialization.  The actual application initialization will happen
 *  when stack reports that BT device is ready.
 */
int main()
{
    cy_rslt_t cy_result;
    wiced_result_t wiced_result;

    /* This enables RTOS aware debugging in OpenOCD. */
    uxTopUsedPriority = configMAX_PRIORITIES - 1;

    /* Initialize the board support package */
    cy_result = cybsp_init();

    if (CY_RSLT_SUCCESS != cy_result)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

    /* Initialize retarget-io to use the debug UART port */
    cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, CY_RETARGET_IO_BAUDRATE);

    printf("************* Find Me Profile Application Start ************************\n");

   /* Configure platform specific settings for the BT device */
   cybt_platform_config_init(&cybsp_bt_platform_cfg);

   /* Register call back and configuration with stack */
   wiced_result = wiced_bt_stack_init (app_bt_management_callback, &wiced_bt_cfg_settings);

   /* Check if stack initialization was successful */
   if( WICED_BT_SUCCESS == wiced_result)
   {
       printf("Bluetooth Stack Initialization Successful \n");
   }
   else
   {
       printf("Bluetooth Stack Initialization failed!! \n");
       CY_ASSERT(0);
   }

    /* Start the FreeRTOS scheduler */
    vTaskStartScheduler();

    /* Should never get here */
    CY_ASSERT(0) ;
}
/**************************************************************************************************
* Function Name: app_bt_management_callback
***************************************************************************************************
* Summary:
*   This is a Bluetooth stack event handler function to receive management events from
*   the LE stack and process as per the application.
*
* Parameters:
*   wiced_bt_management_evt_t event             : LE event code of one byte length
*   wiced_bt_management_evt_data_t *p_event_data: Pointer to LE management event structures
*
* Return:
*  wiced_result_t: Error code from WICED_RESULT_LIST or BT_RESULT_LIST
*
*************************************************************************************************/
wiced_result_t app_bt_management_callback(wiced_bt_management_evt_t event,
                                          wiced_bt_management_evt_data_t *p_event_data)
{
    wiced_result_t wiced_result = WICED_BT_SUCCESS;
    wiced_bt_device_address_t bda = { 0 };
    wiced_bt_ble_advert_mode_t *p_adv_mode = NULL;

    switch (event)
    {
        case BTM_ENABLED_EVT:
            /* Bluetooth Controller and Host Stack Enabled */
            if (WICED_BT_SUCCESS == p_event_data->enabled.status)
            {
                wiced_bt_set_local_bdaddr((uint8_t *)cy_bt_device_address, BLE_ADDR_PUBLIC);
                wiced_bt_dev_read_local_addr(bda);
                printf("Local Bluetooth Address: ");
                print_bd_address(bda);

                /* Perform application-specific initialization */
                le_app_init();
            }
            else
            {
                printf( "Bluetooth Disabled \n" );
            }

            break;

        case BTM_BLE_ADVERT_STATE_CHANGED_EVT:

            /* Advertisement State Changed */
            p_adv_mode = &p_event_data->ble_advert_state_changed;
            printf("Advertisement State Change: %s\n", get_bt_advert_mode_name(*p_adv_mode));

            if (BTM_BLE_ADVERT_OFF == *p_adv_mode)
            {
                /* Advertisement Stopped */
                printf("Advertisement stopped\n");

                /* Check connection status after advertisement stops */
                if(0 == bt_connection_id)
                {
                    app_bt_adv_conn_state = APP_BT_ADV_OFF_CONN_OFF;
                }
                else
                {
                    app_bt_adv_conn_state = APP_BT_ADV_OFF_CONN_ON;
                }
            }
            else
            {
                /* Advertisement Started */
                printf("Advertisement started\n");
                app_bt_adv_conn_state = APP_BT_ADV_ON_CONN_OFF;
            }

            /* Update Advertisement LED to reflect the updated state */
            adv_led_update();
            break;

        case BTM_BLE_CONNECTION_PARAM_UPDATE:
            printf("Connection parameter update status:%d, Connection Interval: %d, Connection Latency: %d, Connection Timeout: %d\n",
                                           p_event_data->ble_connection_param_update.status,
                                           p_event_data->ble_connection_param_update.conn_interval,
                                           p_event_data->ble_connection_param_update.conn_latency,
                                           p_event_data->ble_connection_param_update.supervision_timeout);
            break;

        default:
            printf("Unhandled Bluetooth Management Event: 0x%x %s\n", event, get_btm_event_name(event));
            break;
    }

    return wiced_result;
}

/**************************************************************************************************
* Function Name: le_app_init
***************************************************************************************************
* Summary:
*   This function handles application level initialization tasks and is called from the BT
*   management callback once the LE stack enabled event (BTM_ENABLED_EVT) is triggered
*   This function is executed in the BTM_ENABLED_EVT management callback.
*
* Parameters:
*   None
*
* Return:
*  None
*
*************************************************************************************************/
static void le_app_init(void)
{
    cy_rslt_t cy_result = CY_RSLT_SUCCESS;
    wiced_result_t wiced_result = WICED_BT_SUCCESS;
    wiced_bt_gatt_status_t gatt_status = WICED_BT_GATT_SUCCESS;

    printf("\n***********************************************\n");
    printf("**Discover device with \"Find Me Target\" name*\n");
    printf("***********************************************\n\n");

    /* Initialize the PWM used for IAS alert level LED */
    cy_result = cyhal_pwm_init_adv(&ias_led_pwm, CYBSP_USER_LED1 , NC, CYHAL_PWM_RIGHT_ALIGN, true, 0u, false, NULL);

    /* PWM init failed. Stop program execution */
    if (CY_RSLT_SUCCESS != cy_result  )
    {
        printf("IAS LED PWM Initialization has failed! \n");
        CY_ASSERT(0);
    }
    /* CYBSP_USER_LED2 is only present on some kits. For those kits,it is used to indicate advertising/connection status */
#ifdef CYBSP_USER_LED2
    /* Initialize the PWM used for Advertising LED */
    cy_result = cyhal_pwm_init_adv(&adv_led_pwm, CYBSP_USER_LED2 , NC, CYHAL_PWM_RIGHT_ALIGN, true, 0u, false, NULL);

    /* PWM init failed. Stop program execution */
    if (CY_RSLT_SUCCESS != cy_result)
    {
        printf("Advertisement LED PWM Initialization has failed! \n");
        CY_ASSERT(0);
    }
#endif

    wiced_bt_set_pairable_mode(FALSE, FALSE);

    /* Set Advertisement Data */
    wiced_bt_ble_set_raw_advertisement_data(CY_BT_ADV_PACKET_DATA_SIZE, cy_bt_adv_packet_data);

    /* Register with BT stack to receive GATT callback */
    gatt_status = wiced_bt_gatt_register(le_app_gatt_event_callback);
    printf("GATT event Handler registration status: %s \n",get_bt_gatt_status_name(gatt_status));

    /* Initialize GATT Database */
    gatt_status = wiced_bt_gatt_db_init(gatt_database, gatt_database_len, NULL);
    printf("GATT database initialization status: %s \n",get_bt_gatt_status_name(gatt_status));

    /* Start Undirected LE Advertisements on device startup.
     * The corresponding parameters are contained in 'app_bt_cfg.c' */
    wiced_result = wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL);

    /* Failed to start advertisement. Stop program execution */
    if (WICED_BT_SUCCESS != wiced_result)
    {
        printf("failed to start advertisement! \n");
        CY_ASSERT(0);
    }
}

/**************************************************************************************************
* Function Name: le_app_gatt_event_callback
***************************************************************************************************
* Summary:
*   This function handles GATT events from the BT stack.
*
* Parameters:
*   wiced_bt_gatt_evt_t event                   : LE GATT event code of one byte length
*   wiced_bt_gatt_event_data_t *p_event_data    : Pointer to LE GATT event structures
*
* Return:
*  wiced_bt_gatt_status_t: See possible status codes in wiced_bt_gatt_status_e in wiced_bt_gatt.h
*
**************************************************************************************************/
static wiced_bt_gatt_status_t le_app_gatt_event_callback(wiced_bt_gatt_evt_t event,
                                                         wiced_bt_gatt_event_data_t *p_event_data)
{
    wiced_bt_gatt_status_t gatt_status = WICED_BT_GATT_SUCCESS;
    wiced_bt_gatt_attribute_request_t *p_attr_req = &p_event_data->attribute_request;

    uint16_t error_handle = 0;
    /* Call the appropriate callback function based on the GATT event type, and pass the relevant event
     * parameters to the callback function */
    switch ( event )
    {
        case GATT_CONNECTION_STATUS_EVT:
            gatt_status = le_app_connect_handler( &p_event_data->connection_status );
            break;

        case GATT_ATTRIBUTE_REQUEST_EVT:
            gatt_status = le_app_server_handler(p_attr_req, 
                                                &error_handle );
            if(gatt_status != WICED_BT_GATT_SUCCESS)
            {
               wiced_bt_gatt_server_send_error_rsp(p_attr_req->conn_id, 
                                                   p_attr_req->opcode, 
                                                   error_handle, 
                                                   gatt_status);
            }
            break;

        case GATT_GET_RESPONSE_BUFFER_EVT: /* GATT buffer request, typically sized to max of bearer mtu - 1 */
            p_event_data->buffer_request.buffer.p_app_rsp_buffer =
            app_alloc_buffer(p_event_data->buffer_request.len_requested);
            p_event_data->buffer_request.buffer.p_app_ctxt = (void *)app_free_buffer;
            gatt_status = WICED_BT_GATT_SUCCESS;
            break;

        case GATT_APP_BUFFER_TRANSMITTED_EVT: /* GATT buffer transmitted event,  check \ref wiced_bt_gatt_buffer_transmitted_t*/
        {
            pfn_free_buffer_t pfn_free = (pfn_free_buffer_t)p_event_data->buffer_xmitted.p_app_ctxt;

            /* If the buffer is dynamic, the context will point to a function to free it. */
            if (pfn_free)
                pfn_free(p_event_data->buffer_xmitted.p_app_data);

            gatt_status = WICED_BT_GATT_SUCCESS;
        }
            break;


        default:
            gatt_status = WICED_BT_GATT_ERROR;
               break;
    }

    return gatt_status;
}

/**************************************************************************************************
* Function Name: le_app_set_value
***************************************************************************************************
* Summary:
*   This function handles writing to the attribute handle in the GATT database using the
*   data passed from the BT stack. The value to write is stored in a buffer
*   whose starting address is passed as one of the function parameters
*
* Parameters:
* @param attr_handle  GATT attribute handle
* @param p_val        Pointer to LE GATT write request value
* @param len          length of GATT write request
*
*
* Return:
*   wiced_bt_gatt_status_t: See possible status codes in wiced_bt_gatt_status_e in wiced_bt_gatt.h
*
**************************************************************************************************/
static wiced_bt_gatt_status_t le_app_set_value(uint16_t attr_handle,
                                                uint8_t *p_val,
                                                uint16_t len)
{
    wiced_bool_t isHandleInTable = WICED_FALSE;
    wiced_bool_t validLen = WICED_FALSE;
    wiced_bt_gatt_status_t gatt_status = WICED_BT_GATT_INVALID_HANDLE;
    /* Check for a matching handle entry */
    for (int i = 0; i < app_gatt_db_ext_attr_tbl_size; i++)
    {
        if (app_gatt_db_ext_attr_tbl[i].handle == attr_handle)
        {
            /* Detected a matching handle in external lookup table */
            isHandleInTable = WICED_TRUE;

            /* Check if the buffer has space to store the data */
            validLen = (app_gatt_db_ext_attr_tbl[i].max_len >= len);

            if (validLen)
            {
                /* Value fits within the supplied buffer; copy over the value */
                app_gatt_db_ext_attr_tbl[i].cur_len = len;
                memcpy(app_gatt_db_ext_attr_tbl[i].p_data, p_val, len);
                gatt_status = WICED_BT_GATT_SUCCESS;

                /* Add code for any action required when this attribute is written.
                 * In this case, we update the IAS led based on the IAS alert
                 * level characteristic value */

                switch ( attr_handle )
                {
                    case HDLC_IAS_ALERT_LEVEL_VALUE:
                        printf("Alert Level = %d\n", app_ias_alert_level[0]);
                        ias_led_update();
                        break;

                    /* The application is not going to change its GATT DB,
                     * So this case is not handled */
                    case HDLD_GATT_SERVICE_CHANGED_CLIENT_CHAR_CONFIG:
                        gatt_status = WICED_BT_GATT_SUCCESS;
                        break;
                }
            }
            else
            {
                /* Value to write does not meet size constraints */
                gatt_status = WICED_BT_GATT_INVALID_ATTR_LEN;
            }
            break;
        }
    }

    if (!isHandleInTable)
    {
        /* TODO: Add code to read value for handles not contained within generated lookup table.
         * This is a custom logic that depends on the application, and is not used in the
         * current application. If the value for the current handle is successfully written in the
         * below code snippet, then set the result using:
         * res = WICED_BT_GATT_SUCCESS; */
        switch ( attr_handle )
        {
            default:
                /* The write operation was not performed for the indicated handle */
                printf("Write Request to Invalid Handle: 0x%x\n", attr_handle);
                gatt_status = WICED_BT_GATT_WRITE_NOT_PERMIT;
                break;
        }
    }

    return gatt_status;
}

/**************************************************************************************************
* Function Name: le_app_write_handler
***************************************************************************************************
* Summary:
*   This function handles Write Requests received from the client device
*
* Parameters:
*  @param conn_id       Connection ID
*  @param opcode        LE GATT request type opcode
*  @param p_write_req   Pointer to LE GATT write request
*  @param len_req       length of data requested
*
* Return:
*  wiced_bt_gatt_status_t: See possible status codes in wiced_bt_gatt_status_e in wiced_bt_gatt.h
*
**************************************************************************************************/
static wiced_bt_gatt_status_t le_app_write_handler(uint16_t conn_id,
                                                    wiced_bt_gatt_opcode_t opcode,
                                                    wiced_bt_gatt_write_req_t *p_write_req,
                                                    uint16_t len_req, 
                                                    uint16_t *p_error_handle)
{
    wiced_bt_gatt_status_t gatt_status = WICED_BT_GATT_INVALID_HANDLE;

    *p_error_handle = p_write_req->handle;

    /* Attempt to perform the Write Request */
    gatt_status = le_app_set_value(p_write_req->handle,
                                p_write_req->p_val,
                               p_write_req->val_len);

    if( WICED_BT_GATT_SUCCESS != gatt_status )
    {
        printf("WARNING: GATT set attr status 0x%x\n", gatt_status);
    }
    else
    {
        if(GATT_REQ_WRITE == opcode)
        wiced_bt_gatt_server_send_write_rsp(conn_id, opcode, p_write_req->handle);
    }
    return (gatt_status);
}

/**************************************************************************************************
* Function Name: le_app_read_handler
***************************************************************************************************
* Summary:
*   This function handles Read Requests received from the client device
*
* Parameters:
* @param conn_id       Connection ID
* @param opcode        LE GATT request type opcode
* @param p_read_req    Pointer to read request containing the handle to read
* @param len_req       length of data requested
*
* Return:
*  wiced_bt_gatt_status_t: See possible status codes in wiced_bt_gatt_status_e in wiced_bt_gatt.h
*
**************************************************************************************************/
static wiced_bt_gatt_status_t le_app_read_handler( uint16_t conn_id,
                                                    wiced_bt_gatt_opcode_t opcode,
                                                    wiced_bt_gatt_read_t *p_read_req,
                                                    uint16_t len_req, 
                                                    uint16_t *p_error_handle)
{

    gatt_db_lookup_table_t  *puAttribute;
    int          attr_len_to_copy;
    uint8_t     *from;
    int          to_send;

    *p_error_handle = p_read_req->handle;

    puAttribute = le_app_find_by_handle(p_read_req->handle);
    if ( NULL == puAttribute )
    {
        return WICED_BT_GATT_INVALID_HANDLE;
    }

    attr_len_to_copy = puAttribute->cur_len;
    if (p_read_req->offset >= puAttribute->cur_len)
    {
        return WICED_BT_GATT_INVALID_OFFSET;
    }

    to_send = MIN(len_req, attr_len_to_copy - p_read_req->offset);
    from = ((uint8_t *)puAttribute->p_data) + p_read_req->offset;

    return wiced_bt_gatt_server_send_read_handle_rsp(conn_id, opcode, to_send, from, NULL); /* No need for context, as buff not allocated */;
}

/**************************************************************************************************
* Function Name: le_app_connect_handler
***************************************************************************************************
* Summary:
*   This callback function handles connection status changes.
*
* Parameters:
*   wiced_bt_gatt_connection_status_t *p_conn_status  : Pointer to data that has connection details
*
* Return:
*  wiced_bt_gatt_status_t: See possible status codes in wiced_bt_gatt_status_e in wiced_bt_gatt.h
*
**************************************************************************************************/
static wiced_bt_gatt_status_t le_app_connect_handler(wiced_bt_gatt_connection_status_t *p_conn_status)
{
    wiced_bt_gatt_status_t gatt_status = WICED_BT_GATT_SUCCESS ;

    if ( NULL != p_conn_status )
    {
        if ( p_conn_status->connected )
        {
            /* Device has connected */
            printf("Connected : BDA " );
            print_bd_address(p_conn_status->bd_addr);
            printf("Connection ID '%d' \n", p_conn_status->conn_id );

            /* Store the connection ID */
            bt_connection_id = p_conn_status->conn_id;

            /* Update the adv/conn state */
            app_bt_adv_conn_state = APP_BT_ADV_OFF_CONN_ON;
        }
        else
        {
            /* Device has disconnected */
            printf("Disconnected : BDA " );
            print_bd_address(p_conn_status->bd_addr);
            printf("Connection ID '%d', Reason '%s'\n", p_conn_status->conn_id, get_bt_gatt_disconn_reason_name(p_conn_status->reason) );

            /* Set the connection id to zero to indicate disconnected state */
            bt_connection_id = 0;

            /* Restart the advertisements */
            wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL);

            /* Update the adv/conn state */
            app_bt_adv_conn_state = APP_BT_ADV_ON_CONN_OFF;

            /* Turn Off the IAS LED on a disconnection */
            ias_led_update();
        }

        /* Update Advertisement LED to reflect the updated state */
        adv_led_update();

        gatt_status = WICED_BT_GATT_ERROR;
    }

    return gatt_status;
}

/**************************************************************************************************
* Function Name: le_app_server_handler
***************************************************************************************************
* Summary:
*   This function handles GATT server events from the BT stack.
*
* Parameters:
*  p_attr_req     Pointer to LE GATT connection status
*
* Return:
*  wiced_bt_gatt_status_t: See possible status codes in wiced_bt_gatt_status_e in wiced_bt_gatt.h
*
**************************************************************************************************/
static wiced_bt_gatt_status_t le_app_server_handler (wiced_bt_gatt_attribute_request_t *p_attr_req, 
                                                      uint16_t *p_error_handle)
{
    wiced_bt_gatt_status_t gatt_status = WICED_BT_GATT_SUCCESS;
    switch ( p_attr_req->opcode )
    {
        case GATT_REQ_READ:
        case GATT_REQ_READ_BLOB:
             /* Attribute read request */
            gatt_status = le_app_read_handler(p_attr_req->conn_id,p_attr_req->opcode,
                                              &p_attr_req->data.read_req,
                                              p_attr_req->len_requested,
                                              p_error_handle);
             break;
        case GATT_REQ_WRITE:
        case GATT_CMD_WRITE:
             /* Attribute write request */
            gatt_status = le_app_write_handler(p_attr_req->conn_id, 
                                               p_attr_req->opcode,
                                               &p_attr_req->data.write_req,
                                               p_attr_req->len_requested, 
                                               p_error_handle );

             break;
        case GATT_REQ_MTU:
            gatt_status = wiced_bt_gatt_server_send_mtu_rsp(p_attr_req->conn_id,
                                                       p_attr_req->data.remote_mtu,
                                                       CY_BT_MTU_SIZE);
             break;
        case GATT_HANDLE_VALUE_NOTIF:
                    printf("Notfication send complete\n");
             break;
        case GATT_REQ_READ_BY_TYPE:
            gatt_status = app_bt_gatt_req_read_by_type_handler(p_attr_req->conn_id, 
                                                               p_attr_req->opcode,
                                                               &p_attr_req->data.read_by_type, 
                                                               p_attr_req->len_requested, 
                                                               p_error_handle);
             break;

        default:
                printf("ERROR: Unhandled GATT Connection Request case: %d\n", p_attr_req->opcode);
                gatt_status = WICED_BT_GATT_ERROR;
                break;
    }

    return gatt_status;
}

/*******************************************************************************
* Function Name: adv_led_update
********************************************************************************
*
* Summary:
*   This function updates the advertising LED state based on LE advertising/
*   connection state
*
* Parameters:
*   None
*
* Return:
*   None
*
*******************************************************************************/
void adv_led_update(void)
{
    /* CYBSP_USER_LED2 is only present on some kits. For those kits,it is used to indicate advertising/connection status */
#ifdef CYBSP_USER_LED2
    cy_rslt_t cy_result = CY_RSLT_SUCCESS;
    /* Stop the advertising led pwm */
    cyhal_pwm_stop(&adv_led_pwm);

    /* Update LED state based on LE advertising/connection state.
     * LED OFF for no advertisement/connection, LED blinking for advertisement
     * state, and LED ON for connected state  */
    switch(app_bt_adv_conn_state)
    {
        case APP_BT_ADV_OFF_CONN_OFF:
            cy_result = cyhal_pwm_set_duty_cycle(&adv_led_pwm, LED_OFF_DUTY_CYCLE, ADV_LED_PWM_FREQUENCY);
            break;

        case APP_BT_ADV_ON_CONN_OFF:
            cy_result = cyhal_pwm_set_duty_cycle(&adv_led_pwm, LED_BLINKING_DUTY_CYCLE, ADV_LED_PWM_FREQUENCY);
            break;

        case APP_BT_ADV_OFF_CONN_ON:
            cy_result = cyhal_pwm_set_duty_cycle(&adv_led_pwm, LED_ON_DUTY_CYCLE, ADV_LED_PWM_FREQUENCY);
            break;

        default:
            /* LED OFF for unexpected states */
            cy_result = cyhal_pwm_set_duty_cycle(&adv_led_pwm, LED_OFF_DUTY_CYCLE, ADV_LED_PWM_FREQUENCY);
            break;
    }
    /* Check if update to PWM parameters is successful*/
    if (CY_RSLT_SUCCESS != cy_result)
    {
         printf("Failed to set duty cycle parameters!!");
    }

    cy_result = cyhal_pwm_start(&adv_led_pwm);
    /* Check if PWM started successfully */
    if (CY_RSLT_SUCCESS != cy_result)
    {
         printf("Failed to start PWM !!");
    }
#endif
}

/*******************************************************************************
* Function Name: ias_led_update
********************************************************************************
*
* Summary:
*   This function updates the IAS alert level LED state based on LE
*   advertising/connection state
*
* Parameters:
*   None
*
* Return:
*   None
*
*******************************************************************************/
static void ias_led_update(void)
{
    cy_rslt_t cy_result = CY_RSLT_SUCCESS;
    /* Stop the IAS led pwm */
    cyhal_pwm_stop(&ias_led_pwm);

    /* Update LED based on IAS alert level only when the device is connected */
    if(APP_BT_ADV_OFF_CONN_ON == app_bt_adv_conn_state)
    {
        /* Update LED state based on IAS alert level. LED OFF for low level,
         * LED blinking for mid level, and LED ON for high level  */
        switch(app_ias_alert_level[0])
        {
            case IAS_ALERT_LEVEL_LOW:
                cy_result = cyhal_pwm_set_duty_cycle(&ias_led_pwm, LED_OFF_DUTY_CYCLE, IAS_LED_PWM_FREQUENCY);
                break;

            case IAS_ALERT_LEVEL_MID:
                cy_result = cyhal_pwm_set_duty_cycle(&ias_led_pwm, LED_BLINKING_DUTY_CYCLE, IAS_LED_PWM_FREQUENCY);
                break;

            case IAS_ALERT_LEVEL_HIGH:
                cy_result = cyhal_pwm_set_duty_cycle(&ias_led_pwm, LED_ON_DUTY_CYCLE, IAS_LED_PWM_FREQUENCY);
                break;

            default:
                /* Consider any other level as High alert level */
                cy_result = cyhal_pwm_set_duty_cycle(&ias_led_pwm, LED_ON_DUTY_CYCLE, IAS_LED_PWM_FREQUENCY);
                break;
        }
    }
    else
    {
        /* In case of disconnection, turn off the IAS LED */
        cy_result = cyhal_pwm_set_duty_cycle(&ias_led_pwm, LED_OFF_DUTY_CYCLE, IAS_LED_PWM_FREQUENCY);
    }

    /* Check if update to PWM parameters is successful*/
    if (CY_RSLT_SUCCESS != cy_result)
    {
         printf("Failed to set duty cycle parameters!!");
    }

    cy_result = cyhal_pwm_start(&ias_led_pwm);
    /* Check if PWM started successfully */
    if (CY_RSLT_SUCCESS != cy_result)
    {
         printf("Failed to start PWM !!");
    }
}
/*******************************************************************************
 * Function Name: app_free_buffer
 *******************************************************************************
 * Summary:
 *  This function frees up the memory buffer
 *
 *
 * Parameters:
 *  uint8_t *p_data: Pointer to the buffer to be free
 *
 ******************************************************************************/
static void app_free_buffer(uint8_t *p_buf)
{
    vPortFree(p_buf);
}

/*******************************************************************************
 * Function Name: app_alloc_buffer
 *******************************************************************************
 * Summary:
 *  This function allocates a memory buffer.
 *
 *
 * Parameters:
 *  int len: Length to allocate
 *
 ******************************************************************************/
static void* app_alloc_buffer(int len)
{
    return pvPortMalloc(len);
}
/*******************************************************************************
 * Function Name : le_app_find_by_handle
 * *****************************************************************************
 * Summary : @brief  Find attribute description by handle
 *
 * @param handle    handle to look up
 *
 * @return gatt_db_lookup_table_t   pointer containing handle data
 ******************************************************************************/
static gatt_db_lookup_table_t  *le_app_find_by_handle(uint16_t handle)
{
    int i;
    for (i = 0; i < app_gatt_db_ext_attr_tbl_size; i++)
    {
        if (handle == app_gatt_db_ext_attr_tbl[i].handle )
        {
            return (&app_gatt_db_ext_attr_tbl[i]);
        }
    }
    return NULL;
}
/**
 * Function Name:
 * app_bt_gatt_req_read_by_type_handler
 *
 * Function Description:
 * @brief  Process read-by-type request from peer device
 *
 * @param conn_id       Connection ID
 * @param opcode        LE GATT request type opcode
 * @param p_read_req    Pointer to read request containing the handle to read
 * @param len_requested length of data requested
 *
 * @return wiced_bt_gatt_status_t  LE GATT status
 */
static wiced_bt_gatt_status_t app_bt_gatt_req_read_by_type_handler(uint16_t conn_id,
                                                                   wiced_bt_gatt_opcode_t opcode,
                                                                   wiced_bt_gatt_read_by_type_t *p_read_req,
                                                                   uint16_t len_requested,
                                                                   uint16_t *p_error_handle)
{
    gatt_db_lookup_table_t *puAttribute;
    uint16_t last_handle = 0;
    uint16_t attr_handle = p_read_req->s_handle;
    uint8_t *p_rsp = app_alloc_buffer(len_requested);
    uint8_t pair_len = 0;
    int used_len = 0;

    if (NULL == p_rsp)
    {
        printf("No memory, len_requested: %d!!\r\n",len_requested);
        return WICED_BT_GATT_INSUF_RESOURCE;
    }

    /* Read by type returns all attributes of the specified type, between the start and end handles */
    while (WICED_TRUE)
    {
        *p_error_handle = attr_handle;
        last_handle = attr_handle;
        attr_handle = wiced_bt_gatt_find_handle_by_type(attr_handle, p_read_req->e_handle,
                                                        &p_read_req->uuid);
        if (0 == attr_handle )
            break;

        if ( NULL == (puAttribute = le_app_find_by_handle(attr_handle)))
        {
            printf("found type but no attribute for %d \r\n",last_handle);
            app_free_buffer(p_rsp);
            return WICED_BT_GATT_INVALID_HANDLE;
        }

        {
            int filled = wiced_bt_gatt_put_read_by_type_rsp_in_stream(p_rsp + used_len, len_requested - used_len, &pair_len,
                                                                attr_handle, puAttribute->cur_len, puAttribute->p_data);
            if (0 == filled)
            {
                break;
            }
            used_len += filled;
        }

        /* Increment starting handle for next search to one past current */
        attr_handle++;
    }

    if (0 == used_len)
    {
       printf("attr not found  start_handle: 0x%04x  end_handle: 0x%04x  Type: 0x%04x\r\n",
               p_read_req->s_handle, p_read_req->e_handle, p_read_req->uuid.uu.uuid16);
        app_free_buffer(p_rsp);
        return WICED_BT_GATT_INVALID_HANDLE;
    }

    /* Send the response */

    return wiced_bt_gatt_server_send_read_by_type_rsp(conn_id, opcode, pair_len, used_len, p_rsp, (void *)app_free_buffer);
}

/* END OF FILE [] */
