# BSP NodeMCU ESP32-S3

This repo contains the Board Support Package (BSP) implementation for NodeMCU ESP32-S3 platform, providing hardware abstraction layer for I2C peripherals and GPIO management.

## Overview
This BSP module implements hardware-specific functionality for the ESP32-S3 platform, including:
- I2C master interface configuration
- GPIO expander (TCA9554) initialization and control
- Keypad scan device (TCA8418) initialization and interrupt handling
- Hardware abstraction for portable driver integration

## Hardware Configuration

### I2C Interface
- **SCL Pin**: GPIO 39
- **SDA Pin**: GPIO 38
- **Bus**: I2C_NUM_0
- **Frequency**: 400 kHz

### TCA8418 Keypad Controller
- **Reset Pin**: GPIO 36
- **Interrupt Pin**: GPIO 37
- **I2C Address**: 0x34 (default)

### TCA9554 GPIO Expander
- **I2C Address**: 0x20

## API Reference

### Initialization Functions

#### `bsp_ret_t bsp_init(void)`
Initializes the BSP module including I2C master, GPIO pins, interrupt queue and keyboard handler task.

**Returns:**
- `BSP_OK` - Initialization successful
- `BSP_ERROR` - Initialization failed

**Example:**
```c
if (BSP_OK != bsp_init())
{
    printf("BSP initialization failed\n");
    while(1); // handle error
}
```

#### `bsp_ret_t bsp_gpio_expander_init(void)`
Initializes the TCA9554 GPIO expander device on I2C bus. Configures all pins as outputs and sets them to LOW.

**Returns:**
- `BSP_OK` - GPIO expander initialized successfully
- `BSP_ERROR` - Device not found or initialization failed

#### `bsp_ret_t bsp_keypad_scan_device_init(void)`
Initializes the TCA8418 keypad scan device with interrupt support. Configures the keypad matrix and enables key events.

**Returns:**
- `BSP_OK` - Keypad device initialized successfully
- `BSP_ERROR` - Device not found or initialization failed

### Configuration Functions

#### `void bsp_set_keypad_event_cb(keypad_event_hdr_t keypad_event_cb)`
Registers a callback function to handle keypad events.

**Parameters:**
- `keypad_event_cb` - Callback function pointer that receives key event data

**Callback typedef:**
```c
typedef void (*keypad_event_hdr_t)(uint8_t);
```

## Usage

### Basic Initialization

```c
#include "bsp.h"

int main(void)
{
    // Initialize BSP
    if (BSP_OK != bsp_init())
    {
        printf("BSP init error\n");
        while(1);
    }
    
    // Initialize GPIO Expander
    if (BSP_OK != bsp_gpio_expander_init())
    {
        printf("GPIO Expander init error\n");
        while(1);
    }
    
    // Initialize Keypad Scan Device
    if (BSP_OK != bsp_keypad_scan_device_init())
    {
        printf("Keypad scan init error\n");
        while(1);
    }
    
    printf("BSP initialization complete\n");
    
    while(1)
    {
        // Your application code
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
```

### Keypad Event Handling

```c
#include "bsp.h"

// Keypad event callback
void keypad_event_handler(uint8_t event)
{
    printf("Key event received: 0x%02X\n", event);
    
    // Extract key information
    uint8_t key_code = event & 0x7F;
    uint8_t key_state = (event & 0x80) ? 1 : 0; // 1=pressed, 0=released
    
    printf("Key %d %s\n", key_code, key_state ? "pressed" : "released");
}

int main(void)
{
    // Initialize BSP modules
    bsp_init();
    bsp_gpio_expander_init();
    bsp_keypad_scan_device_init();
    
    // Register keypad event callback
    bsp_set_keypad_event_cb(keypad_event_handler);
    
    printf("Keypad ready, waiting for events...\n");
    
    while(1)
    {
        // Main application loop
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
```

## Features

### I2C Master Interface
- Automatic device handle management per slave address
- Mutex-protected bus access for thread safety
- Support for multiple I2C devices on the same bus
- Configurable clock frequency and glitch filtering

### Interrupt Management
- Hardware interrupt handling for TCA8418 events
- FreeRTOS queue-based interrupt processing
- Dedicated task for keyboard event handling
- Pin glitch filtering for noise immunity

### GPIO Control
- Hardware reset control for TCA8418
- Configurable interrupt enable/disable
- Pull-up/pull-down configuration support

## Internal Implementation

### I2C Proxy Functions
The BSP implements the hardware proxy pattern required by the TCA8418 and TCA9554 drivers:

```c
static int mock_i2c_write(uint8_t dev_address, uint8_t reg_address, uint8_t reg_value);
static int mock_i2c_read(uint8_t dev_address, uint8_t reg_address, uint8_t *reg_value);
```

### GPIO Control Functions
```c
static void tca8418_gpio_set_int(bool en);  // Enable/disable interrupt
static void tca8418_gpio_rst(bool value);   // Control reset pin
```

### Interrupt Processing
The BSP creates a dedicated FreeRTOS task (`keyboard_interrupt_handler_task`) that processes keyboard interrupts from a queue, ensuring interrupt handlers remain short and efficient.

## Dependencies

### ESP-IDF Components
- `driver/i2c_master`
- `driver/gpio`
- `driver/gpio_filter`
- `freertos/FreeRTOS`
- `freertos/task`
- `freertos/queue`

### External Drivers
- `tca9554_drv` - GPIO expander driver
- `tca8418_drv` - Keypad scan driver

## Debug Configuration

Enable verbose logging by setting:
```c
#define BSP_DEBUG (1)
```

Set ESP-IDF log level in source file:
```c
#define LOG_LOCAL_LEVEL ESP_LOG_INFO
```

## Authors

- [Gonzalo Rivera](mailto:gonzaloriveras90@gmail.com)

## License

© 2025 grivera. All rights reserved.