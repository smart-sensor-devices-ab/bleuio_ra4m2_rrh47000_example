/*
 * rrh47000_drv.c
 *
 *  Created on: 15 Oct 2024
 *      Author: emil
 */


#include "rrh7000_drv.h"
#include "common_utils.h"
#include <math.h>



/* Private Global Variables */

/* capture callback event for master module */
static volatile i2c_master_event_t g_master_event = (i2c_master_event_t)RESET_VALUE;

/*
 * private functions
 */
static fsp_err_t validate_i2c_event(void);

/*******************************************************************************************************************//**
 * @brief     Initializes SCI_I2C_MASTER driver.
 * @param[IN] None
 * @retval    FSP_SUCCESS       SCI_I2C_MASTER driver opened successfully.
 * @retval    err               Any Other Error code apart from FSP_SUCCESS like Unsuccessful Open.
 **********************************************************************************************************************/
fsp_err_t init_i2c_driver(void)
{
    fsp_err_t err = FSP_SUCCESS;

    /* Open sci_i2c master channel */
    err = R_SCI_I2C_Open(&g_i2c0_ctrl, &g_i2c0_cfg);
    if (FSP_SUCCESS != err)
    {
        /* Display failure message in RTT */
        APP_ERR_PRINT ("R_SCI_I2C_Open API FAILED \r\n");
        return err;
    }
    return err;
}


/*******************************************************************************************************************//**
 *  @brief        User defined master callback function
 *  @param[IN]    p_args
 *  @retval       None
 **********************************************************************************************************************/
void sci_i2c_master_callback(i2c_master_callback_args_t * p_args)
{
    if (NULL != p_args)
    {
        g_master_event = p_args->event;
    }
}

/*******************************************************************************************************************//**
 * This function is called to do closing of sci_i2c master using its HAL level API.
 * @brief     Close the SCI_I2C Master. Handle the Error internally with Proper RTT Message.
 *            Application handles the rest.
 * @param[IN] None
 * @retval    None
 **********************************************************************************************************************/
void deinit_i2c_driver(void)
{
    fsp_err_t err = FSP_SUCCESS;

    /* close opened SCI_I2C master module */
    err = R_SCI_I2C_Close(&g_i2c0_ctrl);
    if (FSP_SUCCESS != err)
    {
        APP_ERR_PRINT(" **R_SCI_I2C_MASTER_Close API FAILED ** \r\n");
    }
}


/*******************************************************************************************************************//**
 *  @brief     Read Device FW Ver from sensor
 *
 *  @param[in] serial                       retrieve RRH47000 FW Ver
 *
 *  @retval    FSP_SUCCESS                  on successful I2C transaction
 *             FSP_ERR_INVALID_POINTER      if invalid parameters passed
 *             err                          whichever occurs on either API or on I2C event failure
 **********************************************************************************************************************/
fsp_err_t rrh47000_get_fw_ver(uint8_t *fw_ver)
{
    fsp_err_t err         = FSP_SUCCESS;
//    uint8_t reg_address[4];
//    reg_address[0]   = FW_VER_REG;
    uint8_t reg_address = FW_VER_REG;
    uint8_t fw_input[16];

    /* Parameter checking */
    if (NULL == fw_ver)
    {
        err = FSP_ERR_INVALID_POINTER;
        APP_ERR_PRINT("** NULL Pointer check fail ** \r\n");
        return err;
    }

    /* Send register address to sensor */
    err = R_SCI_I2C_Write(&g_i2c0_ctrl, &reg_address, 1, true);
    /* handle error */
    if (FSP_SUCCESS != err)
    {
        APP_ERR_PRINT("** R_SCI_I2C_MASTER_Write API failed ** \r\n");
    }
    else
    {
        err = validate_i2c_event();
        /* handle error */
        if(FSP_ERR_TRANSFER_ABORTED == err)
        {
            APP_ERR_PRINT("** Serial Number reg, I2C write failed ** \r\n");
        }
        else
        {
            /* Read only when I2C write and its callback event is successful */
            err  = R_SCI_I2C_Read(&g_i2c0_ctrl, fw_input, 15, false);
            /* handle error */
            if (FSP_SUCCESS != err)
            {
                APP_ERR_PRINT("** R_SCI_I2C_Read API failed ** \r\n");
                //  Do nothing, the error is returned in the end
            }
            else
            {
                err = validate_i2c_event();
                /* handle error */
                if(FSP_ERR_TRANSFER_ABORTED == err)
                {
                    APP_ERR_PRINT("** Serial Number read,  I2C read failed ** \r\n");
                }
                else
                {
                    APP_PRINT ("\r\nDebug rrh47000 FW ver: ");
                    for(int i = 0; i < (int)sizeof(fw_input); i++)
                    {
                        APP_PRINT ("%02X ", fw_input[i]);
                    }
                    APP_PRINT ("\r\n");
                    memcpy(fw_ver, fw_input + 1, 12);
                }
            }
        }
    }
    /* On successful I2C transaction return from here */
    return err;
}


/*******************************************************************************************************************//**
 *  @brief     Read measurements from sensor
 *
 *  @param[in] serial                       retrieve RRH47000 Serial Number
 *
 *  @retval    FSP_SUCCESS                  on successful I2C transaction
 *             FSP_ERR_INVALID_POINTER      if invalid parameters passed
 *             err                          whichever occurs on either API or on I2C event failure
 **********************************************************************************************************************/
fsp_err_t rrh47000_get_measurements(uint16_t * CO2, float * temperature, float * humidity)
{
    fsp_err_t err         = FSP_SUCCESS;
    uint8_t reg_address   = GET_MEASUREMENTS_REG;
    uint8_t input[10];

    /* Parameter checking */
    if (NULL == CO2 || NULL == temperature || NULL == humidity)
    {
        err = FSP_ERR_INVALID_POINTER;
        APP_ERR_PRINT("** NULL Pointer check fail ** \r\n");
        return err;
    }

    /* Send register address to sensor */
    err = R_SCI_I2C_Write(&g_i2c0_ctrl, &reg_address, ONE_BYTE, true);
    /* handle error */
    if (FSP_SUCCESS != err)
    {
        APP_ERR_PRINT("** R_SCI_I2C_MASTER_Write API failed ** \r\n");
    }
    else
    {
        err = validate_i2c_event();
        /* handle error */
        if(FSP_ERR_TRANSFER_ABORTED == err)
        {
            APP_ERR_PRINT("** Serial Number reg, I2C write failed ** \r\n");
        }
        else
        {
            /* Read only when I2C write and its callback event is successful */
            err  = R_SCI_I2C_Read(&g_i2c0_ctrl, input, 10, false);
            /* handle error */
            if (FSP_SUCCESS != err)
            {
                APP_ERR_PRINT("** R_SCI_I2C_Read API failed ** \r\n");
                //  Do nothing, the error is returned in the end
            }
            else
            {
                err = validate_i2c_event();
                /* handle error */
                if(FSP_ERR_TRANSFER_ABORTED == err)
                {
                    APP_ERR_PRINT("** Get Measurements,  I2C read failed ** \r\n");
                }
                else
                {
//                    APP_PRINT ("\r\nDebug rrh47000 raw measurement data: ");
//                    for(int i = 0; i < (int)sizeof(input); i++)
//                    {
//                        APP_PRINT ("%02X ", input[i]);
//                    }
//                    APP_PRINT ("\r\n");
                    uint16_t temp_co2_val;
                    float temp_temp_val;
                    float temp_hum_val;

                    temp_co2_val = (uint16_t)(input[1] * 256 + input[2]);
                    if(input[5] * 256 + input[6] < 32768)
                    {
                        temp_temp_val = (float) (input[5] * 256 + input[6])/100;
                    } else
                    {
                        temp_temp_val = (float) (input[5] * 256 + input[6] - 65536)/100;
                    }
                    temp_hum_val = (float) (input[7] * 256 + input[8])/100;
                    memcpy(CO2, &temp_co2_val, sizeof(temp_co2_val));
                    memcpy(temperature, &temp_temp_val, sizeof(temp_temp_val));
                    memcpy(humidity, &temp_hum_val, sizeof(temp_hum_val));

                }
            }
        }
    }
    /* On successful I2C transaction return from here */
    return err;
}

/*******************************************************************************************************************//**
 *  @brief       Validate i2c receive/transmit  based on required write read operation
 *
 *  @param[in]   None
 *
 *  @retval      FSP_SUCCESS                       successful event receiving returns FSP_SUCCESS
 *               FSP_ERR_TRANSFER_ABORTED          Either on timeout elapsed or received callback event is
 *                                                 I2C_MASTER_EVENT_ABORTED
 **********************************************************************************************************************/
static fsp_err_t validate_i2c_event(void)
{
    uint16_t local_time_out = UINT16_MAX;

    /* resetting call back event capture variable */
    g_master_event = (i2c_master_event_t)RESET_VALUE;

    do
    {
        /* This is to avoid infinite loop */
        --local_time_out;

        if(RESET_VALUE == local_time_out)
        {
            return FSP_ERR_TRANSFER_ABORTED;
        }

    }while(g_master_event == RESET_VALUE);

    if(g_master_event != I2C_MASTER_EVENT_ABORTED)
    {
        g_master_event = (i2c_master_event_t)RESET_VALUE;  // Make sure this is always Reset before return
        return FSP_SUCCESS;
    }

    g_master_event = (i2c_master_event_t)RESET_VALUE; // Make sure this is always Reset before return
    return FSP_ERR_TRANSFER_ABORTED;
}

void ftof(float input, int * whole, int * frac)
{
    double floating = input, fractional, integer;

    fractional = modf(floating, &integer);
    //printf ("\r\nFloating: %g\nInteger: %g\nFractional: %g", floating, integer, fractional); // when using printf, there are no floats
    int out_whole = integer;
    int out_frac = (float)fractional * 100;
    memcpy(whole, &out_whole, sizeof(out_whole));
    memcpy(frac, &out_frac, sizeof(out_frac));
}
