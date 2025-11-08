/**
*******************************************************************************
* @file           : bsp.h
* @brief          : Description of header file
* @author         : Gonzalo Rivera
* @date           : 02/11/2025
*******************************************************************************
* @attention
*
* Copyright (c) <date> grivera. All rights reserved.
*
*/
#ifndef __BSP_H__
#define __BSP_H__
/******************************************************************************
        Includes
 ******************************************************************************/
#include <stdbool.h>
#include <stdint.h>
/******************************************************************************
        Constants
 ******************************************************************************/
#define BSP_DEBUG					(1)

/******************************************************************************
        Data types
 ******************************************************************************/
typedef enum
{
	BSP_OK = 0,
	BSP_ERROR
	
} bsp_ret_t;

/**
* @brief [TBD]... 
*/
typedef void (*keypad_event_hdr_t)(uint8_t);

/**
* @brief [TBD]... 
*/
typedef void (*gptimer_event_cb_t)(void);
/******************************************************************************
        Public function prototypes
 ******************************************************************************/
#ifdef __cplusplus
extern "C" {
#endif

bsp_ret_t bsp_init(void);
bsp_ret_t bsp_gpio_expander_init(void);
bsp_ret_t bsp_keypad_scan_device_init(void);
void bsp_set_keypad_event_cb(keypad_event_hdr_t keypad_event_cb);

#ifdef __cplusplus
} // extern "C"
#endif

#endif // EOF __BSP_H__