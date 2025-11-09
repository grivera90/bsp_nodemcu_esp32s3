# BSP NodeMCU ESP32-S3

This repo contains the Board Support Package (BSP) implementation for NodeMCU ESP32-S3 platform, providing hardware abstraction layer for I2C peripherals and GPIO management.

## Overview
This BSP module implements hardware-specific functionality for the ESP32-S3 platform, including:
- I2C master interface configuration with mutex-protected access
- GPIO expander (TCA9554) initialization and control
- Keypad scan device (TCA8418) initialization and interrupt handling
- Hardware abstraction for portable driver integration
- FreeRTOS-based interrupt processing with event queue

## Hardware Configuration

### I2C Interface
- **SCL Pin**: GPIO 39
- **SDA Pin**: GPIO 38
- **Bus**: I2C_NUM_0
- **Frequency**: 400 kHz (Fast Mode)
- **Internal Pull-ups**: Enabled
- **Glitch Filter**: 7 clock cycles

### TCA8418 Keypad Controller
- **Reset Pin**: GPIO 36
- **Interrupt Pin**: GPIO 37 (with glitch filter, NEGEDGE trigger)
- **I2C Address**: 0x34 (default)
- **Matrix Configuration**: 8 rows x 10 columns (max)

### TCA9554 GPIO Expander
- **I2C Address**: 0x20
- **Pin Mapping**:
  - GPIO0: LED_OK
  - GPIO1: LED_ERROR
  - GPIO2: LED_ALARM
  - GPIO3: RELE_1
  - GPIO4: RELE_2
  - GPIO5: SW_1 (input)
  - GPIO6: SW_2 (input)
  - GPIO7: SW_3 (input)

## API Reference

### Initialization Functions

#### `bsp_ret_t bsp_init(void)`
Initializes the BSP module including I2C master, GPIO pins, interrupt queue and keyboard handler task.

**Returns:**
- `BSP_OK` - Initialization successful
- `BSP_ERROR` - Initialization failed

**Details:**
- Initializes I2C bus with 400 kHz frequency
- Creates mutex for thread-safe I2C access
- Adds TCA9554 and TCA8418 devices to I2C bus
- Configures TCA8418 reset GPIO (GPIO 36)
- Creates interrupt event queue (10 elements)
- Launches keyboard interrupt handler task (priority: `tskIDLE_PRIORITY + 15`)

**Example:**
```c
if (BSP_OK != bsp_init())
{
    printf("BSP initialization failed\n");
    while(1); // handle error
}
```

#### `bsp_ret_t bsp_gpio_expander_init(void)`
Initializes the TCA9554 GPIO expander device on I2C bus. Configures pins 0-4 as outputs and pins 5-7 as inputs.

**Returns:**
- `BSP_OK` - GPIO expander initialized successfully
- `BSP_ERROR` - Device not found or initialization failed

**Configuration:**
- Searches for device on I2C bus (address 0x20)
- Injects I2C functions into TCA9554 driver
- Configures GPIO0-4 as outputs (LEDs and relays)
- Configures GPIO5-7 as inputs (switches)
- Sets all outputs to LOW (0x00)

#### `bsp_ret_t bsp_keypad_scan_device_init(void)`
Initializes the TCA8418 keypad scan device with interrupt support. Configures the keypad matrix and enables key events.

**Returns:**
- `BSP_OK` - Keypad device initialized successfully
- `BSP_ERROR` - Device not found or initialization failed

**Configuration:**
- Searches for device on I2C bus (address 0x34)
- Injects I2C and GPIO control functions into TCA8418 driver
- Configures maximum keypad matrix (8x10)
- Performs hardware reset
- Enables key event interrupts (`CFG_KE_IEN`)

### Control Functions

#### LED Control
```c
void bsp_led_ok_set(bool value);
void bsp_led_error_set(bool value);
void bsp_led_alarm_set(bool value);
```
Controls the state of status LEDs connected to TCA9554 GPIO expander.

**Parameters:**
- `value` - `true` to turn LED ON, `false` to turn OFF

#### Relay Control
```c
void bsp_rele_1_set(bool value);
void bsp_rele_2_set(bool value);
```
Controls the state of relays connected to TCA9554 GPIO expander.

**Parameters:**
- `value` - `true` to activate relay, `false` to deactivate

#### Switch Reading
```c
bool bsp_read_sw_1(void);
bool bsp_read_sw_2(void);
bool bsp_read_sw_3(void);
```
Reads the state of switches connected to TCA9554 GPIO expander.

**Returns:**
- `true` - Switch is pressed/active
- `false` - Switch is released/inactive

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

### GPIO Expander Control

```c
#include "bsp.h"

void control_example(void)
{
    // Control LEDs
    bsp_led_ok_set(true);      // Turn ON OK LED
    bsp_led_error_set(false);  // Turn OFF Error LED
    bsp_led_alarm_set(true);   // Turn ON Alarm LED
    
    // Control Relays
    bsp_rele_1_set(true);      // Activate Relay 1
    bsp_rele_2_set(false);     // Deactivate Relay 2
    
    // Read Switches
    if (bsp_read_sw_1())
    {
        printf("Switch 1 is pressed\n");
    }
    
    if (bsp_read_sw_2())
    {
        printf("Switch 2 is pressed\n");
    }
    
    if (bsp_read_sw_3())
    {
        printf("Switch 3 is pressed\n");
    }
}
```

## Features

### I2C Master Interface
- Automatic device handle management per slave address
- Mutex-protected bus access for thread safety (FreeRTOS Semaphore)
- Support for multiple I2C devices on the same bus
- Configurable clock frequency (400 kHz Fast Mode)
- Glitch filtering (7 clock cycles)
- Internal pull-ups enabled
- Device probing for connection verification

### Interrupt Management
- Hardware interrupt handling for TCA8418 events
- FreeRTOS queue-based interrupt processing (10 event capacity)
- Dedicated task for keyboard event handling
- Pin glitch filtering for noise immunity
- ISR-safe queue operations (`xQueueSendFromISR`)
- High-priority interrupt handler task (`tskIDLE_PRIORITY + 15`)
- Negative edge triggered interrupts (NEGEDGE)

### GPIO Control
- Hardware reset control for TCA8418
- Configurable interrupt enable/disable
- Pull-up/pull-down configuration support
- Individual pin control through TCA9554 expander
- High-level abstraction for LEDs, relays and switches

### Resource Management
- Shared I2C bus with mutex synchronization
- Thread-safe access to peripherals
- Efficient interrupt-to-task communication
- Minimal stack usage in ISR context
- Timeout support with `portMAX_DELAY`

## Internal Implementation

### I2C Proxy Functions
The BSP implements the hardware proxy pattern required by the TCA8418 and TCA9554 drivers:

```c
static int mock_i2c_write(uint8_t dev_address, uint8_t reg_address, uint8_t reg_value);
static int mock_i2c_read(uint8_t dev_address, uint8_t reg_address, uint8_t *reg_value);
```

These functions automatically select the correct I2C device handle based on the device address and provide mutex-protected access to the shared I2C bus.

### Device Handle Management
```c
static i2c_master_dev_handle_t get_i2c_device_by_addr(uint8_t address);
```
Returns the appropriate device handle for TCA9554 (0x20) or TCA8418 (0x34).

### GPIO Control Functions
```c
static void tca8418_gpio_set_int(bool en);  // Enable/disable interrupt with glitch filter
static void tca8418_gpio_rst(bool value);   // Control reset pin
```

### Interrupt Processing
The BSP creates a dedicated FreeRTOS task (`keyboard_interrupt_handler_task`) that:
1. Waits for interrupt events from the queue
2. Identifies the interrupt source (GPIO 37 for TCA8418)
3. Calls the appropriate driver interrupt handler
4. Runs with high priority to ensure responsive key detection

**Task Configuration:**
- Stack size: `configMINIMAL_STACK_SIZE * 2`
- Priority: `tskIDLE_PRIORITY + 15`
- Queue depth: 10 events

### I2C Bus Architecture
```c
typedef struct {
    i2c_master_bus_handle_t bus;
    i2c_master_dev_handle_t dev_tca9554;
    i2c_master_dev_handle_t dev_tca8418;
    SemaphoreHandle_t mutex;
} bsp_i2c_ctx;
```

## Architecture

```
┌─────────────────────────────────────────────┐
│         Application Layer                   │
└────────────────┬────────────────────────────┘
                 │
┌────────────────▼────────────────────────────┐
│              BSP Layer                      │
│  ┌─────────────────────────────────────┐    │
│  │  I2C Bus Manager (Mutex Protected)  │    │
│  │  • mock_i2c_write()                 │    │
│  │  • mock_i2c_read()                  │    │
│  │  • get_i2c_device_by_addr()         │    │
│  └─────────────────────────────────────┘    │
│  ┌─────────────────────────────────────┐    │
│  │  GPIO Manager                       │    │
│  │  • Reset control (GPIO 36)          │    │
│  │  • Interrupt handling (GPIO 37)     │    │
│  │  • Glitch filtering                 │    │
│  └─────────────────────────────────────┘    │
│  ┌─────────────────────────────────────┐    │
│  │  Event Queue (FreeRTOS)             │    │
│  │  • Keyboard interrupts              │    │
│  │  • Handler task (high priority)     │    │
│  └─────────────────────────────────────┘    │
└────────────────┬────────────────────────────┘
                 │
┌────────────────▼────────────────────────────┐
│       TCA9554 Driver  |  TCA8418 Driver     │
└────────────────┬────────────────────────────┘
                 │
┌────────────────▼────────────────────────────┐
│           Hardware Layer                    │
│  I2C Bus (0x20, 0x34) | GPIO 36, 37         │
└─────────────────────────────────────────────┘
```

## Dependencies

### ESP-IDF Components
- `driver/i2c_master` - I2C master driver
- `driver/gpio` - GPIO control
- `driver/gpio_filter` - Glitch filtering
- `esp_intr_alloc` - Interrupt allocation
- `freertos/FreeRTOS` - Real-time OS
- `freertos/task` - Task management
- `freertos/queue` - Queue implementation
- `esp_log` - Logging system

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
#define LOG_LOCAL_LEVEL ESP_LOG_INFO  // or ESP_LOG_DEBUG for verbose output
```

Available log levels:
- `ESP_LOG_ERROR` - Only errors
- `ESP_LOG_WARN` - Warnings and errors
- `ESP_LOG_INFO` - Informational messages (default)
- `ESP_LOG_DEBUG` - Detailed debug information

## Platform Requirements

- **ESP-IDF**: v5.4.2
- **MCU**: ESP32-S3
- **FreeRTOS**: Included with ESP-IDF
- **Python**: 3.8+ (for build tools)
- **CMake**: 3.16+

## Thread Safety

All BSP functions are thread-safe when called from different FreeRTOS tasks:
- I2C operations are protected by mutex
- ISR uses queue for safe communication with task context
- GPIO operations are atomic at hardware level

## Authors

- [Gonzalo Rivera](mailto:gonzaloriveras90@gmail.com)

## License

© 2025 grivera. All rights reserved.