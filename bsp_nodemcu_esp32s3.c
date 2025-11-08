/**
*******************************************************************************
* @file           : bsp_nodemcu_esp32s3.c
* @brief          : Description of C implementation module
* @author         : Gonzalo Rivera
* @date           : 02/11/2025
*******************************************************************************
* @attention
*
* Copyright (c) <date> grivera. All rights reserved.
*
*/
/******************************************************************************
    Includes
******************************************************************************/
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include "esp_err.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_intr_alloc.h"
#include "driver/gpio_filter.h"
#include "freertos/projdefs.h"
#include "portmacro.h"
#define LOG_LOCAL_LEVEL ESP_LOG_INFO	/*ESP_LOG_WARN*/ /*ESP_LOG_DEBUG*/ /*ESP_LOG_ERROR*/ /*ESP_LOG_INFO*/
#include "esp_log.h"

#include "bsp.h"
#include "hal/gpio_types.h"
#include "../tca9554_drv/tca9554_drv.h"
#include "../tca8418_drv/tca8418_drv.h"
/******************************************************************************
    Defines and constants
******************************************************************************/
#define I2C_MASTER_SCL_IO 				(39)    	/*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 				(38)     	/*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM 					(0)			/*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 				(400000)	/*!< I2C master clock frequency */

#define BSP_TCA8418_GPIO_RST			(36)
#define BSP_TCA8418_GPIO_INT			(37)
/******************************************************************************
    Data types
******************************************************************************/

/******************************************************************************
    Local variables
******************************************************************************/
static const char *MODULE_NAME = "[BSP]";
/* I2C Context handler */
static struct 
{
    i2c_master_bus_handle_t bus;
    i2c_master_dev_handle_t dev_tca9554;
    i2c_master_dev_handle_t dev_tca8418;
    SemaphoreHandle_t mutex;
} bsp_i2c_ctx;
/* GPIO Expander */
static tca9554_t tca9554 = {0};
/* Keypad scan device */
static tca8418_t tca8418 = {0};
/* GPIO queue event */
static QueueHandle_t gpio_evt_queue = NULL;
/******************************************************************************
    Local function prototypes
******************************************************************************/
static int i2c_master_init(void);
static int i2c_master_search(i2c_master_bus_handle_t bus_handle, uint8_t address);
static int mock_i2c_write(uint8_t dev_address, uint8_t reg_address, uint8_t reg_value);
static int mock_i2c_read(uint8_t dev_address, uint8_t reg_address, uint8_t *reg_value);
static i2c_master_dev_handle_t get_i2c_device_by_addr(uint8_t address);
static void gpio_interrupt_handler(void *arg);
static void keyboard_interrupt_handler_task(void* arg);
static void tca8418_gpio_set_int(bool en);
static void tca8418_gpio_rst(bool value);
/******************************************************************************
    Local function definitions
******************************************************************************/
static int i2c_master_init(void)
{
    i2c_master_bus_config_t bus_cfg = 
    {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bsp_i2c_ctx.bus));
    bsp_i2c_ctx.mutex = xSemaphoreCreateMutex();

    i2c_device_config_t dev_cfg_9554 = {
        .device_address = TCA9554_ADDRESS_20H,
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };

    i2c_device_config_t dev_cfg_8418 = {
        .device_address = TCA8418_DEFAULT_ADDRESS,
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };

    if (ESP_OK != i2c_master_bus_add_device(bsp_i2c_ctx.bus, &dev_cfg_9554, &bsp_i2c_ctx.dev_tca9554))
    {
		return BSP_ERROR;
	}
	
	if (ESP_OK != i2c_master_bus_add_device(bsp_i2c_ctx.bus, &dev_cfg_8418, &bsp_i2c_ctx.dev_tca8418))
	{
		return BSP_ERROR;
	}
	
	return BSP_OK;
}

static int i2c_master_search(i2c_master_bus_handle_t bus_i2c, uint8_t address)
{
	return (ESP_OK == i2c_master_probe(bus_i2c, (uint16_t) address, -1)) ? 0 : 1;
}

/**
 * @brief This configuration is to TCA9554 gpio expander.
 *
 * @note M = Master S = Slave 
 * _______________________________________________________________________________________________
 * | start | slave_addr + wr_bit + | ack | register_addr + | ack  | register_value  | ack  | stop |
 * ----M---|------------M----------|--S--|--------M--------|--S---|--------M--------|--S---|--M---|
 */
static int mock_i2c_write(uint8_t dev_address, uint8_t reg_address, uint8_t reg_value)
{
    uint8_t txbuf[2] = { reg_address, reg_value };
    int ret = 1;

	i2c_master_dev_handle_t dev = get_i2c_device_by_addr(dev_address);
    if (!dev) return 1;
	
	if (xSemaphoreTake(bsp_i2c_ctx.mutex, portMAX_DELAY) == pdTRUE)
	{	    
		ret = (i2c_master_transmit(dev, txbuf, sizeof(txbuf), -1) == ESP_OK) ? 0 : 1;	
		xSemaphoreGive(bsp_i2c_ctx.mutex);	
	}
	
	return ret;
}

/**
 * @brief This configuration is to TCA9554 gpio expander.
 *
 * @note M = Master S = Slave 
 * ________________________________________________________________________________________________________________________________
 * | start | slave_addr + wr_bit | ack | register_addr | ack  | re_start | slave_addr + rd_bit| ack | register_value | nack | stop |
 * ----M---|------------M--------|--S--|--------M------|--S---|----M-----|----------M---------|--S--|--------S-------|---M--|--M---|
 */
static int mock_i2c_read(uint8_t dev_address, uint8_t reg_address, uint8_t *reg_value)
{
	uint8_t txbuf[1] = {reg_address};
	uint8_t rxbuf[1];
	esp_err_t esp_ret = ESP_FAIL;

	i2c_master_dev_handle_t dev = get_i2c_device_by_addr(dev_address);
    if (!dev) return 1;
    
	if (xSemaphoreTake(bsp_i2c_ctx.mutex, portMAX_DELAY) == pdTRUE)
	{
		esp_ret = i2c_master_transmit_receive(dev, txbuf, 1, rxbuf, 1, -1);
		*reg_value = (ESP_OK == esp_ret) ? rxbuf[0] : 0;
		xSemaphoreGive(bsp_i2c_ctx.mutex);		
	}
	
	return esp_ret;
}

static i2c_master_dev_handle_t get_i2c_device_by_addr(uint8_t address)
{
    switch (address) 
    {
        case TCA9554_ADDRESS_20H: return bsp_i2c_ctx.dev_tca9554;
        case TCA8418_DEFAULT_ADDRESS: return bsp_i2c_ctx.dev_tca8418;
        default: return NULL;
    }
}

static void IRAM_ATTR gpio_interrupt_handler(void *arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void keyboard_interrupt_handler_task(void* arg)
{
	uint32_t io_num = 0;
    ESP_LOGI(MODULE_NAME, "KeyBoard interrupt handler task running");
    
    while(1) 
    {
        if (pdPASS == xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY))
        {
			if (BSP_TCA8418_GPIO_INT == io_num)
			{
				io_num = 0; 
				tca8418_interrupt_handler(&tca8418);
			} 
		}
    }
}

static void tca8418_gpio_set_int(bool en)
{
    gpio_pin_glitch_filter_config_t glitch_filter_cfg = 
    {
        .clk_src = GLITCH_FILTER_CLK_SRC_DEFAULT,
        .gpio_num = BSP_TCA8418_GPIO_INT,
    };
	
	gpio_glitch_filter_handle_t glitch_filter_handle;
	gpio_new_pin_glitch_filter(&glitch_filter_cfg, &glitch_filter_handle);
		
	if (true == en)
	{
		gpio_install_isr_service(ESP_INTR_FLAG_LEVEL2);
		
		gpio_config_t gpio_cfg = 
		{
			.mode = GPIO_MODE_INPUT,
			.pin_bit_mask = (1ULL << BSP_TCA8418_GPIO_INT),
			.pull_up_en = GPIO_PULLUP_ENABLE,
			.pull_down_en = GPIO_PULLDOWN_DISABLE,
			.intr_type = (true == en) ? GPIO_INTR_NEGEDGE : GPIO_INTR_DISABLE,
		};
		
		gpio_config(&gpio_cfg);
		gpio_isr_handler_add(BSP_TCA8418_GPIO_INT, gpio_interrupt_handler, (void *) BSP_TCA8418_GPIO_INT);
		gpio_glitch_filter_enable(glitch_filter_handle);	
	}
	else
	{
		gpio_uninstall_isr_service();
		gpio_isr_handler_remove(BSP_TCA8418_GPIO_INT);
		gpio_glitch_filter_disable(glitch_filter_handle);
	}	
}

// TODO: gpio set int tca9554.

static void tca8418_gpio_rst(bool value)
{
	gpio_set_level(BSP_TCA8418_GPIO_RST, (uint32_t) value);
}
/******************************************************************************
    Public function definitions
******************************************************************************/
bsp_ret_t bsp_init(void)
{
	/* I2C master Initialization */
	if (ESP_OK != i2c_master_init())
	{
		ESP_LOGE(MODULE_NAME, "I2C init failed");	
		return BSP_ERROR;
	}

	ESP_LOGI(MODULE_NAME, "I2C init ok");	

	/* GPIO´s Initializations */
	gpio_config_t gpio_cfg = 
	{
		.mode = GPIO_MODE_OUTPUT,
		.pin_bit_mask = (1ULL << BSP_TCA8418_GPIO_RST),
		.pull_up_en = GPIO_PULLUP_ENABLE,
		.intr_type = GPIO_INTR_DISABLE,
	};
	
	if (ESP_OK != gpio_config(&gpio_cfg))
	{
		ESP_LOGE(MODULE_NAME, "GPIO TCA8418 RST init failed");
	}
	
	gpio_set_level(BSP_TCA8418_GPIO_RST, 1);
	ESP_LOGI(MODULE_NAME, "GPIO TCA8418 RST init ok");	
	
	tca8418_gpio_set_int(false);
	ESP_LOGI(MODULE_NAME, "GPIO TCA8418 INT disable");
	
	/* Queue for enqueue TCA8418 interrupts */
	gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
	
	/* KeyBoard interrupt handler task*/
	xTaskCreate(keyboard_interrupt_handler_task, "keyboard_interrupt_handler_task", 
														configMINIMAL_STACK_SIZE * 2, 
														NULL, 
														tskIDLE_PRIORITY + 15, 
														NULL);
	
	return BSP_OK;
}

bsp_ret_t bsp_gpio_expander_init(void)
{
	/* Get the bus handle */
	i2c_master_bus_handle_t current_i2c_bus;
	ESP_ERROR_CHECK(i2c_master_get_bus_handle(0, &current_i2c_bus));

	if (0 != i2c_master_search(current_i2c_bus, TCA9554_ADDRESS_20H))
	{
		ESP_LOGE(MODULE_NAME, "GPIO Expander device not found on I2C bus");
		return BSP_ERROR;
	}
	
	ESP_LOGI(MODULE_NAME, "Device GPIO Expander found on I2C bus: 0x%02X", TCA9554_ADDRESS_20H);
	
	/* GPIO Expander port Initialization */
	tca9554.i2c_master_xmit = mock_i2c_write;
	tca9554.i2c_master_recv = mock_i2c_read;
	
	if (TCA9554_OK != tca9554_init(&tca9554, TCA9554_ADDRESS_20H, true))
	{
		ESP_LOGE(MODULE_NAME, "GPIO Expander init error");
		return BSP_ERROR;
	}
	
	ESP_LOGI(MODULE_NAME, "GPIO Expander init ok");
	
	/* All port as output */
	tca9554_write_reg(&tca9554, TCA9554_CONFIG_REG, 0x00);
	
	/* All port = 0 */
	tca9554_write_reg(&tca9554, TCA9554_OUTPUT_REG, 0x00);	
	
	return BSP_OK;
}

bsp_ret_t bsp_keypad_scan_device_init(void)
{
	/* Get the bus handle */
	i2c_master_bus_handle_t current_i2c_bus;
	ESP_ERROR_CHECK(i2c_master_get_bus_handle(0, &current_i2c_bus));
	
	if (0 != i2c_master_search(current_i2c_bus, TCA8418_DEFAULT_ADDRESS))
	{
		ESP_LOGE(MODULE_NAME, "Keypad scan device not found on I2C bus");
		return BSP_ERROR;
	}
	
	ESP_LOGI(MODULE_NAME, "Keypad scan device found on I2C bus 0x%02X", TCA8418_DEFAULT_ADDRESS);
	
	tca8418.hw.i2c_master_xmit = mock_i2c_write;
	tca8418.hw.i2c_master_recv = mock_i2c_read;
	tca8418.hw.gpio_set_interrupt = tca8418_gpio_set_int;
	tca8418.hw.gpio_rst = tca8418_gpio_rst;
	tca8418.address = TCA8418_DEFAULT_ADDRESS;
	tca8418.key_event_callback = NULL;
	
	if (TCA8418_OK != tca8418_init(&tca8418))
	{
		ESP_LOGE(MODULE_NAME, "TCA8418 init error");
		return BSP_ERROR;
	}		
	
	ESP_LOGI(MODULE_NAME, "TCA8418 init ok");

	tca8418_set_kb_matrix(&tca8418, TCA8418_MAX_ROWS, TCA8418_MAX_COLS);
	tca8418_pin_reset(&tca8418, true);
	tca8418_enable_interrupts(&tca8418, CFG_KE_IEN);
	
	return BSP_OK;
}

void bsp_set_keypad_event_cb(keypad_event_hdr_t keypad_event_cb)
{
	tca8418.key_event_callback = keypad_event_cb;
}
/******************************************************************************
    Weak function definitions
******************************************************************************/

