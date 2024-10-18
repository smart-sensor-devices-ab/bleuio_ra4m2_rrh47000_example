/*
 * rrh7000_drv.h
 *
 *  Created on: 15 Oct 2024
 *      Author: emil
 */

#ifndef RRH7000_DRV_H_
#define RRH7000_DRV_H_

#include "usb_thread.h"


/* for on board LED */
#if defined (BOARD_RA4W1_EK) || defined (BOARD_RA6T1_RSSK)
#define LED_ON             (bool)BSP_IO_LEVEL_LOW
#define LED_OFF            (bool)BSP_IO_LEVEL_HIGH
#else
#define LED_ON             (bool)BSP_IO_LEVEL_HIGH
#define LED_OFF            (bool)BSP_IO_LEVEL_LOW
#endif

/* MACRO for checking if two buffers are equal */
#define BUFF_EQUAL (0U)

/* buffer size for slave and master data */
#define BUF_LEN            0x06

/* Led toggle delay */
#define TOGGLE_DELAY       0x15E

/*Delay added to recognise LED toggling after wrie/read operation */
#define DELAY_OPERATION  (1U)

/* enumerators to identify Master event to be processed */
typedef enum e_master
{
    MASTER_READ  = 1U,
    MASTER_WRITE = 2U
}master_transfer_mode_t;

#define ONE_BYTE                0x01
#define TWO_BYTE                0x02
#define NINE_BYTE               0x09

#define MEASURE_PAYLOAD_SIZE    0x03        //measurement enable data length
#define SENSOR_READ_DELAY       0x03
#define ENABLE_BIT              0x08
#define DATA_REGISTERS          0x06

#define FW_VER_REG              0x1E
#define GET_MEASUREMENTS_REG    0x31

/*
 *  Global functions
 */
fsp_err_t init_i2c_driver(void);
fsp_err_t process_master_WriteRead(void);
void deinit_i2c_driver(void);
fsp_err_t rrh47000_get_fw_ver(uint8_t *fw_ver);
fsp_err_t rrh47000_get_measurements(uint16_t * CO2, float * temperature, float * humidity);
void ftof(float input, int * whole, int * frac);

#endif /* RRH7000_DRV_H_ */
