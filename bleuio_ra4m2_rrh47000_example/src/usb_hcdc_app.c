/***********************************************************************************************************************
 * File Name    : usb_hcdc_app.c
 * Description  : Contains data structures and functions used in usb_hcdc_app.c.
 **********************************************************************************************************************/
/***********************************************************************************************************************
 * DISCLAIMER
 * This software is supplied by Renesas Electronics Corporation and is only intended for use with Renesas products. No
 * other uses are authorized. This software is owned by Renesas Electronics Corporation and is protected under all
 * applicable laws, including copyright laws.
 * THIS SOFTWARE IS PROVIDED "AS IS" AND RENESAS MAKES NO WARRANTIES REGARDING
 * THIS SOFTWARE, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. ALL SUCH WARRANTIES ARE EXPRESSLY DISCLAIMED. TO THE MAXIMUM
 * EXTENT PERMITTED NOT PROHIBITED BY LAW, NEITHER RENESAS ELECTRONICS CORPORATION NOR ANY OF ITS AFFILIATED COMPANIES
 * SHALL BE LIABLE FOR ANY DIRECT, INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR ANY REASON RELATED TO THIS
 * SOFTWARE, EVEN IF RENESAS OR ITS AFFILIATES HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
 * Renesas reserves the right, without notice, to make changes to this software and to discontinue the availability of
 * this software. By using this software, you agree to the additional terms and conditions found by accessing the
 * following link:
 * http://www.renesas.com/disclaimer
 *
 * Copyright (C) 2020 Renesas Electronics Corporation. All rights reserved.
 ***********************************************************************************************************************/

#include <usb_hcdc_app.h>
#include <r_usb_cstd_rtos.h>
#include <common_utils.h>
#include <usb_thread.h>
#include "r_usb_basic.h"
#include "r_usb_hcdc.h"
#include "rrh7000_drv.h"

/*******************************************************************************************************************//**
 * @addtogroup usb_hcdc_ep
 * @{
 **********************************************************************************************************************/
/** Private variables **/
static usb_hcdc_linecoding_t g_serial_state;
static usb_hcdc_linecoding_t g_com_parm;
static usb_hcdc_device_info_t dev_info;
static uint8_t  g_rcv_buf[CDC_READ_DATA_LEN];               //Receive buffer
static uint8_t g_snd_buf[CDC_WRITE_DATA_LEN];               //Send buffer
static uint8_t  g_usb_dummy = RESET_VALUE;                  //dummy variable to send
static volatile bool g_err_flag = false;                    //flag bit

/** Private functions**/
static void set_line_coding (usb_instance_ctrl_t * p_ctrl, uint8_t device_address);
static void set_control_line_state (usb_instance_ctrl_t * p_ctrl, uint8_t device_address);
static void get_line_coding (usb_instance_ctrl_t * p_ctrl, uint8_t device_address);
static void usb_data_process(usb_event_info_t *event_info);
static void handle_error(fsp_err_t err, char *err_str);
uint32_t hex2int(char *hex);

#define BLEUIO_CMD_ADV_START "AT+ADVSTART\r"
#define BLEUIO_CMD_ADVDATA "AT+ADVDATA="

bool advstart_set = false;
bool scan_started = false;
bool adv_started = false;
bool dongle_booted = false;
bool update_adv_data = false;

static uint16_t co2;
static float temp;
static float hum;
static int temp_wh;
static int temp_fr;
static int hum_wh;
static int hum_fr;
uint8_t dev_addr = 0;

static uint8_t CO2_adv_1;
static uint8_t CO2_adv_2;
static uint8_t Temp_adv_1;
static uint8_t Temp_adv_2;
static uint8_t Hum_adv_1;
static uint8_t Hum_adv_2;

/******************************************************************************************************//**
* @brief This function process the data from the host to device and prints the received data on RTT terminal
* @param[IN]   None
* @retval      None
*******************************************************************************************************/
void usb_hcdc_task(void)
{
    fsp_err_t err = FSP_SUCCESS;
    uint32_t gpt_desired_period_ms          = RESET_VALUE;
    uint64_t period_counts                  = RESET_VALUE;
    uint32_t pclkd_freq_hz                  = RESET_VALUE;
    static usb_event_info_t *event_info = NULL;
    BaseType_t err_queue = pdFALSE;
    memset(&g_serial_state, RESET_VALUE, sizeof(g_serial_state));
    memset(&g_com_parm, RESET_VALUE, sizeof(g_com_parm));
    /*Prepare first BleuIO AT command*/
    sprintf((char *)g_snd_buf, "%s", (char *)BLEUIO_CMD_ADV_START);

    /* Example Project information printed on the Console */
    APP_PRINT(BANNER_INFO);
    APP_PRINT(EP_INFO);


    err = R_USB_Open (&g_basic_ctrl, &g_basic_cfg);
    /* Error Handle */
    if (FSP_SUCCESS != err)
    {
        APP_ERR_PRINT ("\r\nError in initializing USB Host CDC\r\n");
        APP_ERR_TRAP (err);
    }


    TURN_BLUE_ON;
    TURN_GREEN_ON;
    TURN_RED_ON;

    /* Init I2C Master */
    err = init_i2c_driver();
    /* Error Handle */
    if (FSP_SUCCESS != err)
    {
        APP_ERR_PRINT ("\r\nError in initializing I2C Master\r\n");
        APP_ERR_TRAP (err);
    } else
    {
        APP_PRINT ("\r\nInitializing I2C Master OK\r\n");
    }

    err = rrh47000_get_measurements(&co2, &temp, &hum);
    if (FSP_SUCCESS != err)
    {
        APP_ERR_PRINT ("\r\nError in getting rrh47000 measurements\r\n");
        APP_ERR_TRAP (err);
    } else
    {

        APP_PRINT ("\r\nRRH47000 CO2: %u", co2);
        ftof(temp, &temp_wh, &temp_fr);
        APP_PRINT ("\r\nRRH47000 Temp: %u.%u", temp_wh, temp_fr);
        ftof(hum, &hum_wh, &hum_fr);
        APP_PRINT ("\r\nRRH47000 Hum: %u.%u", hum_wh, hum_fr);
    }

    /*Initialize Periodic Timer */
    err = R_GPT_Open(&g_timer_periodic_ctrl, &g_timer_periodic_cfg);
    if(FSP_SUCCESS != err)
    {
        APP_ERR_PRINT("** GPT TIMER INIT FAILED ** \r\n");
        APP_ERR_TRAP(err);
    }

    /* Set timer period to 8000ms */
    gpt_desired_period_ms = 8000;

    /* Get the source clock frequency (in Hz) */
    pclkd_freq_hz = R_FSP_SystemClockHzGet(FSP_PRIV_CLOCK_PCLKD);
    pclkd_freq_hz >>= (uint32_t)(g_timer_periodic_cfg.source_div);

    /* Convert period to PCLK counts so it can be set in hardware. */
    period_counts = (uint64_t)((gpt_desired_period_ms * (pclkd_freq_hz * CLOCK_TYPE_SPECIFIER))  / TIMER_UNITS_MILLISECONDS);

    /* Period Set API set the desired period counts*/
    err = R_GPT_PeriodSet(&g_timer_periodic_ctrl, (uint32_t)period_counts);
    if (FSP_SUCCESS != err)
    {
        /* GPT Timer PeriodSet Failure message */
        APP_ERR_PRINT ("\r\n ** R_GPT_PeriodSet API failed ** \r\n");
        /*Close Periodic Timer instance */
        R_GPT_Close(&g_timer_periodic_ctrl);
        APP_ERR_TRAP(err);
    }

    /* Start Periodic Timer*/
    err = R_GPT_Start(&g_timer_periodic_ctrl);
    if(FSP_SUCCESS != err)
    {
        APP_ERR_PRINT("** GPT TIMER START FAILED ** \r\n");
        /*Close Periodic Timer instance */
        R_GPT_Close(&g_timer_periodic_ctrl);
        APP_ERR_TRAP(err);
    }

    APP_PRINT("\r\nStarting Example.\r\nPlease wait for the dongle to exit booloader mode...");
    vTaskDelay (1000);
    TURN_BLUE_OFF;
    TURN_GREEN_OFF;
    while (true)
    {
        /* Handle error if queue send fails*/
        if (true == g_err_flag)
        {
            handle_error(g_err_flag, "Error in sending usb event through queue");
        }

        /* Receive message from queue and analyzing the received message*/
         err_queue= xQueueReceive(g_usb_queue, &event_info,(portMAX_DELAY));
         /* Handle error */
         if(pdTRUE != err_queue)
         {
             handle_error (err_queue, "Error in receiving USB event message through queue");
         }

        switch (event_info->event)
        {
            case USB_STATUS_CONFIGURED:
            {
                /* CDC Class request "SetLineCoding" */
                set_line_coding (&g_basic_ctrl, event_info->device_address);
                APP_PRINT("\r\nUSB_STATUS_CONFIGURED");
            }
            break;

            case USB_STATUS_READ_COMPLETE:
            {
                /* CDC class communication data process */
                usb_data_process(event_info);
            }
            break;

            case USB_STATUS_WRITE_COMPLETE:
            {
//                APP_PRINT("\r\nUSB_STATUS_WRITE_COMPLETE");
            }
            break;

            case USB_STATUS_REQUEST_COMPLETE:
            {
                /* Check Complete request "SetLineCoding" */
                if (USB_CDC_SET_LINE_CODING == (event_info->setup.request_type & USB_BREQUEST))
                {
                    APP_PRINT("\r\nUSB_CDC_SET_LINE_CODING REQ");
                    /* Class notification "SerialState" receive start */
                    set_control_line_state (&g_basic_ctrl, event_info->device_address);
                }
                /* Check Complete request "SetControlLineState" */
                else if (USB_CDC_SET_CONTROL_LINE_STATE == (event_info->setup.request_type & USB_BREQUEST))
                {
                    APP_PRINT("\r\nUSB_CDC_SET_CONTROL_LINE_STATE REQ");
                    /* CDC Class request "SetLineCoding" */
                    get_line_coding (&g_basic_ctrl, event_info->device_address);
                }
                else if (USB_CDC_GET_LINE_CODING == (event_info->setup.request_type & USB_BREQUEST))
                {
                    /*Read device info*/
                    err = R_USB_HCDC_DeviceInfoGet(&g_basic_ctrl, &dev_info, event_info->device_address);
                    APP_PRINT("\r\nUSB_CDC_GET_LINE_CODING REQ");
                    if (FSP_SUCCESS != err)
                    {
                        handle_error (err, "**R_USB_HCDC_DeviceInfoGet API FAILED**");
                    }
                    APP_PRINT("\r\n\r\n PID %04X", dev_info.product_id);
                    APP_PRINT("\r\n SUBCLASS %02X", dev_info.subclass);
                    APP_PRINT("\r\n VID %04X\r\n", dev_info.vendor_id);
                    TURN_RED_OFF;
                    /*Check that BleuIO is out of bootloader mode*/
                    if((dev_info.product_id == 0x6002) && (dev_info.vendor_id == 0x2DCF))
                    {
                        /* Report receive start */
                        R_USB_Read (&g_basic_ctrl, g_rcv_buf, CDC_READ_DATA_LEN,
                                              event_info->device_address);
                        /* Handle Error */
                        if (FSP_SUCCESS != err)
                        {
                            handle_error (err,"**R_USB_Read API FAILED**");
                        }
                        dev_addr = event_info->device_address;
                        err = R_USB_Write (&g_basic_ctrl, g_snd_buf, strlen((char *)BLEUIO_CMD_ADV_START),
                                     event_info->device_address);
                        if (FSP_SUCCESS != err)
                        {
                            handle_error (err, "**R_USB_Write API FAILED**");
                        }
                        dongle_booted = true;
                    }
                }
                else
                {
                    /* Not support request */
                }
            }
            break;

            default:
            {
                APP_PRINT("\r\nUnknown operation: %02X", event_info->event);
                /* No operation to do*/
            }
            break;
        }
    }
}
 /* End of function main_task() */

/*******************************************************************************************************************//**
 * @brief In this function initializes to set control line state information for host control transfer.
 * @param[IN]   p_ctrl         Pointer to control structure
 * @param[IN]   device_address         USB device address
 * @retval      None.
 ***********************************************************************************************************************/
static void set_control_line_state (usb_instance_ctrl_t * p_ctrl, uint8_t device_address)
{
    usb_setup_t setup;
    fsp_err_t err = FSP_SUCCESS;


    setup.request_type      = SET_CONTROL_LINE_STATE;   /* bRequestCode:SET_CONTROL_LINE_STATE, bmRequestType */
    setup.request_value     = 3U;                           /* wValue:3U (Enables DTR and RTS)*/
    setup.request_index     = VALUE_ZERO;                   /* wIndex:Interface */
    setup.request_length    = VALUE_ZERO;                   /* wLength:Zero */

    err = R_USB_HostControlTransfer(p_ctrl, &setup, &g_usb_dummy, device_address);
    if (FSP_SUCCESS != err)
    {
        handle_error (err, "**R_USB_HOSTControlTransfer API FAILED**");
    }

} /* End of function cdc_set_control_line_state */
/*******************************************************************************************************************//**
 * @brief In this function initializes to set line coding information for host control transfer.
 * @param[IN]   p_ctrl         Pointer to control structure
 * @param[IN]   device_address  USB device address
 * @retval      None.
 ***********************************************************************************************************************/
 static void set_line_coding (usb_instance_ctrl_t * p_ctrl, uint8_t device_address)
{
     usb_setup_t setup;
     fsp_err_t err = FSP_SUCCESS;
     g_com_parm.dwdte_rate   = (uint32_t)USB_HCDC_SPEED_9600;
     g_com_parm.bdata_bits   = USB_HCDC_DATA_BIT_8;
     g_com_parm.bchar_format = USB_HCDC_STOP_BIT_1;
     g_com_parm.bparity_type = USB_HCDC_PARITY_BIT_NONE;


     setup.request_type      = SET_LINE_CODING;    /* bRequestCode:SET_LINE_CODING, bmRequestType */
     setup.request_value     = VALUE_ZERO;             /* wValue:Zero */
     setup.request_index     = VALUE_ZERO;             /* wIndex:Interface */
     setup.request_length    = LINE_CODING_LENGTH; /* Data:Line Coding Structure */

     /* Request Control transfer */
     err = R_USB_HostControlTransfer(p_ctrl, &setup, (uint8_t *)&g_com_parm, device_address);
     if (FSP_SUCCESS != err)
     {
         handle_error (err, "**R_USB_HostControlTransfer API FAILED**");
     }
} /* End of function cdc_set_line_coding */

 /*******************************************************************************************************************//**
  * @brief In this function initializes to get line coding information for host control transfer.
  * @param[IN]   p_ctrl                pointer to control structure
  * @param[IN]   device_address         USB device address
  * @retval      None.
  ***********************************************************************************************************************/
static void get_line_coding (usb_instance_ctrl_t * p_ctrl, uint8_t device_address)
{
    usb_setup_t setup;
    fsp_err_t err = FSP_SUCCESS;
    setup.request_type      = GET_LINE_CODING;      /* bRequestCode:GET_LINE_CODING, bmRequestType */
    setup.request_value     = VALUE_ZERO;               /* wValue:Zero */
    setup.request_index     = VALUE_ZERO;               /* wIndex:Interface */
    setup.request_length    = LINE_CODING_LENGTH;   /* Data:Line Coding Structure */

    /* Request Control transfer */
    err = R_USB_HostControlTransfer(p_ctrl, &setup, (uint8_t *)&g_com_parm, device_address);
    if (FSP_SUCCESS != err)
    {
        handle_error (err, "**R_USB_HostControlTransfer API FAILED**");
    }

} /* End of function cdc_get_line_coding */

/*******************************************************************************************************************//**
 * @brief  This function is called to do closing of usb module using its HAL level API and handles the error trap.
 *  Handle the Error internally with Proper Message. Application handles the rest.
 * @param[IN] err       Return values of APIs
 * @param[IN] err_str   Print message from the failed API call
 * @retval    None
 **********************************************************************************************************************/
static void handle_error(fsp_err_t err, char *err_str)
{
    fsp_err_t error = FSP_SUCCESS;

    /* close opened USB module */
    error = R_USB_Close(&g_basic_ctrl);
    /* Handle error */
    if(FSP_SUCCESS != error)
    {
        APP_ERR_PRINT ("**\r\n R_USB_Close API FAILED **\r\n");
    }
    APP_ERR_PRINT(err_str);
    APP_ERR_TRAP(err);
} /* End of function handle_error() */

/*******************************************************************************************************************//**
 * @brief     This function is callback for FreeRTOS+HCDC and triggers when USB event occurs from the device.
 * @param[IN]   p_event_info    Triggered event
 * @param[IN]   cur_task        current task handle
 * @param[IN]   usb_state       State of USB
 * @retval      None.
 ***********************************************************************************************************************/
void usb_rtos_callback (usb_event_info_t *p_event_info, usb_hdl_t cur_task, usb_onoff_t usb_state)
{
    FSP_PARAMETER_NOT_USED (cur_task);
    FSP_PARAMETER_NOT_USED (usb_state);

    /* Send event received to queue */
    if (pdTRUE != (xQueueSend(g_usb_queue, (const void *)&p_event_info, (TickType_t)(NO_WAIT_TIME))))
    {
        g_err_flag = true;
    }
} /* End of function usb_rtos_callback */
/*******************************************************************************************************************//**
  * @brief This function is to do data process with peripheral device
  * @param[IN]   event_info             data process in HCDC event type
  * @retval      None.
  ***********************************************************************************************************************/
void usb_data_process(usb_event_info_t *event_info)
{
    fsp_err_t err = FSP_SUCCESS;
    char * OK_STR = "OK";
    char * ADV_START_RESP_STR = "ADVERTISING...";
    size_t msg_len = 0;

    if (USB_CLASS_HCDC == event_info->type)
    {
        if (RESET_VALUE < event_info->data_size)
        {

            if(event_info->data_size > 2)
            {
                APP_PRINT(">%s\r\n", g_rcv_buf);
            }

            /*Check if we received 'OK' response */
            if(strstr((char *)&g_rcv_buf, OK_STR) != NULL)
            {
                /*Clear send buffer*/
                memset(g_snd_buf, 0, sizeof(g_snd_buf));

                /*Check what command to send next*/
                if(!advstart_set)
                {
                    /*Set flag*/
                    advstart_set = true;
                    /*Prepare command and set command length */
                    sprintf((char *)g_snd_buf, "%s", (char *)BLEUIO_CMD_ADV_START);
                    msg_len = strlen((char *)g_snd_buf);

                    err = R_USB_Write (&g_basic_ctrl, g_snd_buf, msg_len,
                                 event_info->device_address);
                    if (FSP_SUCCESS != err)
                    {
                        handle_error (err, "**R_USB_Write API FAILED**");
                    }
                }
            }
            else if(strstr((char *)&g_rcv_buf, ADV_START_RESP_STR) != NULL)
            {
                /*Set flag*/
                adv_started = true;
                TURN_GREEN_ON;
                TURN_RED_OFF;
            }
            else if (update_adv_data)
            {
                update_adv_data = false;
//                /* Update Advertising*/
                sprintf((char *)g_snd_buf, "%s09:FF:36:00:%02X:%02X:%02X:%02X:%02X:%02X\r",
                        (char *)BLEUIO_CMD_ADVDATA,
                        CO2_adv_1, CO2_adv_2,
                        Temp_adv_1, Temp_adv_2,
                        Hum_adv_1, Hum_adv_2);
                msg_len = strlen((char *)g_snd_buf);
                err = R_USB_Write (&g_basic_ctrl, g_snd_buf, msg_len, event_info->device_address);
                if (FSP_SUCCESS != err)
                {
                    handle_error (err, "**R_USB_Write API FAILED**");
                }
                APP_PRINT ("\r\n\r\nRRH47000 CO2: %u", co2);
                APP_PRINT ("\r\nRRH47000 Temp: %u.%u", temp_wh, temp_fr);
                APP_PRINT ("\r\nRRH47000 Hum: %u.%u\r\n", hum_wh, hum_fr);
            }
            else
            {
                sprintf((char *)g_snd_buf, "\r\n");
                msg_len = strlen((char *)g_snd_buf);

                err = R_USB_Write (&g_basic_ctrl, g_snd_buf, msg_len,
                             event_info->device_address);
                if (FSP_SUCCESS != err)
                {
                    handle_error (err, "**R_USB_Write API FAILED**");
                }
            }

            /*Clear receive buffer*/
            memset(g_rcv_buf, 0, sizeof(g_rcv_buf));
            /* Report receive start */
            R_USB_Read (&g_basic_ctrl, g_rcv_buf, CDC_READ_DATA_LEN,
                                  event_info->device_address);


            /* Handle Error */
            if (FSP_SUCCESS != err)
            {
                handle_error (err,"**R_USB_Read API FAILED**");
            }
        }
        else
        {
            /* Send the data reception request when the zero-length packet is received. */
            err = R_USB_Read (&g_basic_ctrl,g_rcv_buf, event_info->data_size,
                        event_info->device_address);
            if (FSP_SUCCESS != err)
            {
                handle_error (err, "**R_USB_Read API FAILED**");
            }
        }
    }
    else
    {
        /* Class notification "SerialState" receive start */
        err = R_USB_Read (&g_basic_ctrl, (uint8_t *) &g_serial_state,
        USB_HCDC_SERIAL_STATE_MSG_LEN,event_info->device_address);
        /* Error Handle */
        if (FSP_SUCCESS != err)
        {
            handle_error (err, "**R_USB_Read API FAILED**");
        }
    }

} /* End of function usb_data_process */

/**
 * hex2int
 * take a hex string and convert it to a 32bit number (max 8 hex digits)
 */
uint32_t hex2int(char *hex) {
    uint32_t val = 0;
    while (*hex) {
        // get current character then increment
        char byte = *hex++;
        // transform hex character to the 4bit equivalent number, using the ascii table indexes
        if (byte >= '0' && byte <= '9') byte = byte - '0';
        else if (byte >= 'a' && byte <='f') byte = byte - 'a' + 10;
        else if (byte >= 'A' && byte <='F') byte = byte - 'A' + 10;
        // shift 4 to make space for new digit, and add the 4 bits of the new digit
        val = (val << 4) | (byte & 0xF);
    }
    return val;
}

void g_timer_periodic_callback(timer_callback_args_t *p_args)
{
    fsp_err_t err = FSP_SUCCESS;
    if(NULL != p_args)
    {
        if(dongle_booted)
        {
            err = rrh47000_get_measurements(&co2, &temp, &hum);
            if (FSP_SUCCESS != err)
            {
                APP_ERR_PRINT ("\r\nError in getting rrh47000 measurements\r\n");
                APP_ERR_TRAP (err);
            } else
            {
                ftof(temp, &temp_wh, &temp_fr);
                ftof(hum, &hum_wh, &hum_fr);
                CO2_adv_1 = (uint8_t)(((uint16_t)co2) / 256);        // high
                CO2_adv_2 = (uint8_t)((uint16_t)co2) & 0x00ff;       // low

                Temp_adv_1 = (uint8_t)temp_wh;
                Temp_adv_2 = (uint8_t)temp_fr;
                Hum_adv_1 = (uint8_t)hum_wh;
                Hum_adv_2 = (uint8_t)hum_fr;
                update_adv_data = true;
            }
        }
    }
}

/*******************************************************************************************************************//**
 * @} (end addtogroup usb_hcdc_ep)
 **********************************************************************************************************************/
