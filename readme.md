# Clone the Repository  
If you're interested in exploring the code, you can clone this repository to your STM32CubeIDE workspace using the following command:  
```bash
git clone https://github.com/RayenBof97/STM32F4xx-Drivers-From-Scratch
```

# STM32F4xx GPIO Driver

This repository contains custom drivers libraries for STM32F4xx microcontrollers, written entirely from scratch as part of my learning journey. It provides basic and modular functions to configure and interact with peripherals, helping me deepen my understanding of embedded systems and microcontroller programming.

**NOTE :** This project is intended solely for personal learning and to deepen my understanding of STM32 architectures. It is not meant for practical use, especially considering the existence of the HAL library, which is much more efficient than my implementation.

For this, I'm using my Nucleo-F401RE board, and you can find its datasheet on the  [official STM32 website](https://www.st.com/en/evaluation-tools/nucleo-f401re.html) 

## APIs References

### GPIO Initialization    
**Definition :** initialize a specific GPIO Pin.
```c
void RB_GPIO_Init(GPIOx_Handler_t *pGPIOHandle)
```

| Parameter            | Type             | Description                                         |
| :------------------- | :--------------- | :-------------------------------------------------- |
| `pGPIOHandle`        | `GPIOx_Handler_t*` | **Required**. Pointer to the GPIO handler structure containing the configuration for the GPIO pin. |


### GPIO Deinitialization
**Definition :** Deinitialize a specific GPIO port (reset all registers).
```c
void RB_GPIO_DeInit(GPIO_t *pGPIOx)
```

| Parameter       | Type         | Description |
| :-------------- | :----------- | :-------------------------------------------------- |
| pGPIOx          | GPIO_t*      | **Required**. Pointer to the GPIO structure. |


### GPIO Clock Control
**Definition :** This function enable or disable the GPIO peripheral clock.
```c
void RB_GPIO_PeriClockControl(GPIOx_t *pGPIOx, uint8_t State)
```

| Parameter            | Type             | Description                                         |
| :------------------- | :--------------- | :-------------------------------------------------- |
| `pGPIOx`             | `GPIOx_t*`       | **Required**. Pointer to the GPIO port (e.g., `GPIOA`). |
| `State`              | `uint8_t`        | **Required**. State to enable (`SET`) or disable (`RESET`) the GPIO peripheral clock. |


### GPIO Pin Data Read
**Definition :** Read the state of a GPIO Pin.
```c
uint8_t RB_GPIO_ReadInputPin(GPIOx_t *pGPIOx, uint8_t PinNumber)
```

| Parameter            | Type             | Description                                         |
| :------------------- | :--------------- | :-------------------------------------------------- |
| `pGPIOx`             | `GPIOx_t*`       | **Required**. Pointer to the GPIO port (e.g., `GPIOA`). |
| `PinNumber`          | `uint8_t`        | **Required**. GPIO pin number to read. |

**Return :** The state of the pin (0,1)


### GPIO Port Data Read
**Definition :** Read the state of a GPIO Port , returned in a 16bit Word (16pins).
```c
uint16_t RB_GPIO_ReadInputPort(GPIO_t *pGPIOx)
```

| Parameter       | Type         | Description |
| :-------------- | :----------- | :-------------------------------------------------- |
| pGPIOx          | GPIO_t*      | **Required**. Pointer to the GPIO structure. |

**Return**: The state of all pins in portX (16Bits Word)


### GPIO Pin Data Write
**Definition :** Write the state into a GPIO Pin.
```c
void RB_GPIO_WriteOutputPin(GPIOx_t *pGPIOx, uint8_t PinNumber, uint8_t value)
```

| Parameter            | Type             | Description                                         |
| :------------------- | :--------------- | :-------------------------------------------------- |
| `pGPIOx`             | `GPIOx_t*`       | **Required**. Pointer to the GPIO port (e.g., `GPIOA`). |
| `PinNumber`          | `uint8_t`        | **Required**. GPIO pin number to write to. |
| `value`              | `uint8_t`        | **Required**. Value to write (either `GPIO_PIN_SET` or `GPIO_PIN_RESET`). |

### GPIO Port Data Write
**Definition :** Write the state of all pins in port (16bit Word).
```c
void RB_GPIO_WriteOutputPort(GPIO_t *pGPIOx, uint16_t value)
```

| Parameter       | Type         | Description |
| :-------------- | :----------- | :-------------------------------------------------- |
| pGPIOx          | GPIO_t*      | **Required**. Pointer to the GPIO structure. |
| value           | uint16_t     | **Required**. Value to write to the GPIO port (ranging from 0x0000 to 0xFFFF). |


### GPIO Pin Toggle
**Definition :** Toggle a pin (Invert the current state of the GPIO Pin).
```c
void RB_GPIO_TogglePin(GPIOx_t *pGPIOx, uint8_t PinNumber)
```

| Parameter            | Type             | Description                                         |
| :------------------- | :--------------- | :-------------------------------------------------- |
| `pGPIOx`             | `GPIOx_t*`       | **Required**. Pointer to the GPIO port (e.g., `GPIOA`). |
| `PinNumber`          | `uint8_t`        | **Required**. GPIO pin number to toggle. |


### GPIO Interrupt Request Enabling
**Definition :** IRQ Enable on CPU (Refers to NVIC Part in Arm CortexM4 RM)
```c
void RB_GPIO_IRQITConfig(uint8_t IRQNumber, uint8_t state)
```

| Parameter            | Type             | Description                                         |
| :------------------- | :--------------- | :-------------------------------------------------- |
| `IRQNumber`          | `uint8_t`        | **Required**. Interrupt request line (e.g., 6 for EXTI Line 0). |
| `state`              | `uint8_t`        | **Required**. Enable (`ENABLE`) or Disable (`DISABLE`) the interrupt. |


### GPIO Interrupt Priority Configuration
**Definition :** Configure the priority of the IRQ
```c
void RB_GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t Priority)
```

| Parameter            | Type             | Description                                         |
| :------------------- | :--------------- | :-------------------------------------------------- |
| `IRQNumber`          | `uint8_t`        | **Required**. Interrupt request line (e.g., 6 for EXTI Line 0). |
| `Priority`           | `uint32_t`       | **Required**. Interrupt priority (value between 0 and 15). |


### GPIO Interrupt Request (IRQ) Handling
**Definition :** Handles the IRQ on GPIO Peripherals
```c
void RB_GPIO_IRQHandling(uint8_t PinNumber)
```

| Parameter       | Type         | Description |
| :-------------- | :----------- | :-------------------------------------------------- |
| PinNumber       | uint8_t      | **Required**. The GPIO pin number for which the IRQ is being handled. |



## Usage Example

### GPIO Initialization Example

```c
#include "stm32f401xx.h"

int main() {
    GPIOx_Handler_t gpioHandler;
    gpioHandler.pGPIOx = GPIOA;  // Select GPIOA port
    gpioHandler.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_5;  // Pin number 5
    gpioHandler.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT;  // Output mode
    gpioHandler.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;  // Speed: Fast
    gpioHandler.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPD_NO_PULL;  // No pull-up or pull-down

    RB_GPIO_Init(&gpioHandler);  // Initialize the GPIO pin
    
    while(1) {
        RB_GPIO_WriteOutputPin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);  // Set the pin to high
        delay();  // Simple delay
        RB_GPIO_WriteOutputPin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);  // Set the pin to low
        delay();  // Simple delay
    }
}
```

### Interrupt Example

```c
#include "stm32f401xx.h"

void EXTI0_IRQHandler(void) {
    // Interrupt handling code
    RB_GPIO_TogglePin(GPIOA, GPIO_PIN_0);  // Toggle the state of pin 0
}

int main() {
    GPIOx_Handler_t gpioHandler;
    gpioHandler.pGPIOx = GPIOA;
    gpioHandler.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_0;
    gpioHandler.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;  // Falling edge interrupt

    RB_GPIO_Init(&gpioHandler);
    RB_GPIO_IRQITConfig(6, ENABLE);  // Enable IRQ for EXTI Line 0

    while(1) {
        // Main loop
    }
}
```
---
# STM32F4xx UART Driver
## API Reference

### USART Initialization
**Definition :** initialize an USART peripheral.
```c
void RB_USART_Init(USARTx_Handler_t *pUSARTHandle)
```

| Parameter            | Type             | Description                                         |
| :------------------- | :--------------- | :-------------------------------------------------- |
| `pUSARTHandle`        | `USARTx_Handler_t*` | **Required**. Pointer to the USART handler structure containing the configuration for the USART peripheral. |


### USART Deinitialization
**Definition :** Deinitialize an USART peripheral. (Reset all USART Registers)
```c
void RB_USART_DeInit(USART_Handle_t *pUSARTHandle)
```

| Parameter       | Type           | Description |
| :-------------- | :------------- | :-------------------------------------------------- |
| pUSARTHandle    | USART_Handle_t* | **Required**. Pointer to the USART handle structure. |


### USART Clock Control
**Definition :** Enable or disable the clock on the USART peripheral
```c
void RB_USART_PeriClockControl(USARTx_t *pUSARTx, uint8_t State)
```

| Parameter            | Type             | Description                                         |
| :------------------- | :--------------- | :-------------------------------------------------- |
| `pUSARTx`             | `USARTx_t*`       | **Required**. Pointer to the USART peripheral (e.g., `USART1`). |
| `State`              | `uint8_t`        | **Required**. State to enable (`SET`) or disable (`RESET`) the USART peripheral clock. |


### USART Data Transmission (Poll Mode)
**Definition :** Transmit the data (Stored in a TxBuffer) (Blocking API)
```c
void RB_USART_Data_TX(USART_t *pUSARTx, uint8_t *pTxBuffer, uint16_t length)
```
| Parameter   | Type         | Description |
| :---------- | :----------- | :-------------------------------------------------- |
| pUSARTx    | USART_t*    | **Required**. Pointer to the USART structure. |
| pTxBuffer  | uint8_t*    | **Required**. Buffer containing the data to be transmitted. |
| length     | uint16_t    | **Required**. Length of the message to transmit. |

**Note**: This API is a blocking call (polling mode).  


### USART Data Reception (Poll Mode)
**Definition :** Receive the data and store it in RxBuffer (Blocking API)
```c
void RB_USART_Data_RX(USART_t *pUSARTx, uint8_t *pRxBuffer, uint16_t length)
```

| Parameter   | Type         | Description |
| :---------- | :----------- | :-------------------------------------------------- |
| pUSARTx     | USART_t*     | **Required**. Pointer to the USART structure. |
| pRxBuffer   | uint8_t*     | **Required**. Buffer to store the received data. |
| length      | uint16_t     | **Required**. Length of the message to be received. |

**Note**: This API is also a blocking call (polling mode).


### USART Data Transmission (Interruption mode)
**Definition :** Transmit the data (Stored in a TxBuffer) (Non-Blocking API)
```c
void RB_USART_Data_TXIT(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint16_t length)
```

| Parameter       | Type           | Description |
| :-------------- | :------------- | :-------------------------------------------------- |
| pUSARTHandle    | USART_Handle_t* | **Required**. Pointer to the USART handle structure. |
| pTxBuffer       | uint8_t*       | **Required**. Buffer containing the data to be transferred. |
| length          | uint16_t       | **Required**. Length of the message to be transferred. |

**Return**: The state of Tx (Check the states in stm32f401xx_usart.h header file).
**Note**: This API is a non-blocking call (interrupt mode).

### USART Data Reception (Interruption mode)
**Definition :** Receive the data and store it in RxBuffer (Non-Blocking API)
```c
void RB_USART_Data_RXIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint16_t length)
```

| Parameter       | Type           | Description |
| :-------------- | :------------- | :-------------------------------------------------- |
| pUSARTHandle    | USART_Handle_t* | **Required**. Pointer to the USART handle structure. |
| pRxBuffer       | uint8_t*       | **Required**. Buffer to store the received data. |
| length          | uint16_t       | **Required**. Length of the message to be received. |

**Return**: The state of Rx (Check the states in stm32f401xx_usart.h header file).
**Note**: This API is also a non-blocking call (interrupt mode).


### USART Interrupt Request Enabling
**Definition :** IRQ Enable on CPU (Refers to NVIC Part in Arm CortexM4 RM)
```c
void RB_USART_IRQITConfig(uint8_t IRQNumber, uint8_t state)
```

| Parameter            | Type             | Description                                         |
| :------------------- | :--------------- | :-------------------------------------------------- |
| `IRQNumber`          | `uint8_t`        | **Required**. Interrupt request line (e.g., 6 for EXTI Line 0). |
| `state`              | `uint8_t`        | **Required**. Enable (`ENABLE`) or Disable (`DISABLE`) the interrupt. |


### USART Interrupt Priority Configuration
**Definition :** Configure the priority of the IRQ
```c
void RB_USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t Priority)
```

| Parameter            | Type             | Description                                         |
| :------------------- | :--------------- | :-------------------------------------------------- |
| `IRQNumber`          | `uint8_t`        | **Required**. Interrupt request line (e.g., 6 for EXTI Line 0). |
| `Priority`           | `uint32_t`       | **Required**. Interrupt priority (value between 0 and 15). |


### USART Interrupt Request (IRQ) Handling
**Definition :** Handles the IRQ on USART Peripherals
```c
void RB_USART_IRQHandling(USART_Handle_t *pUSARTHandle)
```

| Parameter       | Type           | Description |
| :-------------- | :------------- | :-------------------------------------------------- |
| pUSARTHandle    | USART_Handle_t* | **Required**. Pointer to a USART handle structure containing the USART peripheral and its configuration. |


### USART Peripheral Enable
**Definition :** Enable/Disable the USART Peripheral
```c
void USART_PeriphControl(USART_t *pUSARTx, uint8_t state)
```

| Parameter   | Type      | Description |
| :---------- | :-------- | :-------------------------------------------------- |
| pUSARTx     | USART_t*  | **Required**. Pointer to the USART peripheral. |
| state       | uint8_t   | **Required**. State of the peripheral (`ENABLE` or `DISABLE`). |

**Note**: The USART peripheral is disabled by default to allow control configuration before enabling it.


### USART Flag Status
**Definition :** Return the status of specific USART flag. (Check USART_SR Register for flags)
```c
uint8_t RB_USART_GetFlagStatus(USART_t *pUSARTx, uint32_t flag)
```

| Parameter   | Type      | Description |
| :---------- | :-------- | :-------------------------------------------------- |
| pUSARTx     | USART_t*  | **Required**. Pointer to the USART peripheral. |
| flag        | uint32_t  | **Required**. Desired flag (use the bit position in the `SPI_SR` register). |

**Return**: Status of the flag (FLAG_SET or FLAG_RESET).  


### USART Clear Flag
**Definition :** Clear a specific USART Flag
```c
void RB_USART_ClearFlag(USART_t *pUSARTx, uint32_t flag)
```

| Parameter   | Type      | Description |
| :---------- | :-------- | :-------------------------------------------------- |
| pUSARTx     | USART_t*  | **Required**. Pointer to the USART peripheral. |
| flag        | uint32_t  | **Required**. Desired flag (use the bit position in the `SPI_SR` register). |

### USART Event Callback Function 
**Definition :** This is a weakly defined callback function that the user can override to handle USART application events. The function is called when a specific USART event occurs, such as transmission complete, reception complete, or error detection.
```c
__weak void USART_ApplicationEventCallback(USARTx_Handler_t *pUSARTHandle, uint8_t AppEv)
```

| Parameter      | Type                 | Description |
| :------------ | :------------------- | :-------------------------------------------------- |
| pUSARTHandle  | USARTx_Handler_t*     | **Required**. Pointer to the USART handle structure. |
| AppEv         | uint8_t               | **Required**. Application event identifier. |

---
# SPI Driver

# Contributing
Contributions are welcome! If you'd like to report bugs, suggest features, or submit improvements, please open an issue or create a pull request.

# License
This project is licensed under the MIT License. See the LICENSE file for details.
