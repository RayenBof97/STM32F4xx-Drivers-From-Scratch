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
void RB_GPIO_DeInit(GPIOx_t *pGPIOx)
```

| Parameter       | Type         | Description |
| :-------------- | :----------- | :-------------------------------------------------- |
| `pGPIOx`          | `GPIOx_t*`      | **Required**. Pointer to the GPIO structure. |


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
uint16_t RB_GPIO_ReadInputPort(GPIOx_t *pGPIOx)
```

| Parameter       | Type         | Description |
| :-------------- | :----------- | :-------------------------------------------------- |
| `pGPIOx`          | `GPIOx_t*`      | **Required**. Pointer to the GPIO structure. |

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
void RB_GPIO_WriteOutputPort(GPIOx_t *pGPIOx, uint16_t value)
```

| Parameter       | Type         | Description |
| :-------------- | :----------- | :-------------------------------------------------- |
| `pGPIOx`          | `PIOx_t*`      | **Required**. Pointer to the GPIO structure. |
| `value`           | `uint16_t`     | **Required**. Value to write to the GPIO port (ranging from 0x0000 to 0xFFFF). |


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
| `PinNumber`       | `uint8_t`      | **Required**. The GPIO pin number for which the IRQ is being handled. |



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
## APIs References

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
void RB_USART_DeInit(USARTx_Handler_t *pUSARTHandle)
```

| Parameter       | Type           | Description |
| :-------------- | :------------- | :-------------------------------------------------- |
| `pUSARTHandle`    | `USARTx_Handler_t*` | **Required**. Pointer to the USART handle structure. |


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
void RB_USART_Data_TX(USARTx_t *pUSARTx, uint8_t *pTxBuffer, uint16_t length)
```
| Parameter   | Type         | Description |
| :---------- | :----------- | :-------------------------------------------------- |
| `pUSARTx`    | `USARTx_t*`    | **Required**. Pointer to the USART structure. |
| `pTxBuffer`  | `uint8_t*`    | **Required**. Buffer containing the data to be transmitted. |
| `length`     | `uint16_t`    | **Required**. Length of the message to transmit. |

**Note**: This API is a blocking call (polling mode).  


### USART Data Reception (Poll Mode)
**Definition :** Receive the data and store it in RxBuffer (Blocking API)
```c
void RB_USART_Data_RX(USARTx_t *pUSARTx, uint8_t *pRxBuffer, uint16_t length)
```

| Parameter   | Type         | Description |
| :---------- | :----------- | :-------------------------------------------------- |
| `pUSARTx`     | `USARTx_t*`     | **Required**. Pointer to the USART structure. |
| `pRxBuffer`   | `uint8_t*`     | **Required**. Buffer to store the received data. |
| `length`      | `uint16_t`     | **Required**. Length of the message to be received. |

**Note**: This API is also a blocking call (polling mode).


### USART Data Transmission (Interruption mode)
**Definition :** Transmit the data (Stored in a TxBuffer) (Non-Blocking API)
```c
void RB_USART_Data_TXIT(USARTx_Handler_t *pUSARTHandle, uint8_t *pTxBuffer, uint16_t length)
```

| Parameter       | Type           | Description |
| :-------------- | :------------- | :-------------------------------------------------- |
| `pUSARTHandle`    | `USARTx_Handler_t*` | **Required**. Pointer to the USART handle structure. |
| `pTxBuffer`       | `uint8_t*`       | **Required**. Buffer containing the data to be transferred. |
| `length`          | `uint16_t`       | **Required**. Length of the message to be transferred. |

**Return**: The state of Tx (Check the states in stm32f401xx_usart.h header file).
**Note**: This API is a non-blocking call (interrupt mode).


### USART Data Reception (Interruption mode)
**Definition :** Receive the data and store it in RxBuffer (Non-Blocking API)
```c
void RB_USART_Data_RXIT(USARTx_Handler_t *pUSARTHandle, uint8_t *pRxBuffer, uint16_t length)
```

| Parameter       | Type           | Description |
| :-------------- | :------------- | :-------------------------------------------------- |
| `pUSARTHandle`    | `USARTx_Handler_t*` | **Required**. Pointer to the USART handle structure. |
| `pRxBuffer`       | `uint8_t*`       | **Required**. Buffer to store the received data. |
| `length`          | `uint16_t`       | **Required**. Length of the message to be received. |

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
void RB_USART_IRQHandling(USARTx_Handler_t *pUSARTHandle)
```

| Parameter       | Type           | Description |
| :-------------- | :------------- | :-------------------------------------------------- |
| `pUSARTHandle`    | `USARTx_Handler_t*` | **Required**. Pointer to a USART handle structure containing the USART peripheral and its configuration. |


### USART Peripheral Enable
**Definition :** Enable/Disable the USART Peripheral
```c
void USART_PeriphControl(USARTx_t *pUSARTx, uint8_t state)
```

| Parameter   | Type      | Description |
| :---------- | :-------- | :-------------------------------------------------- |
| `pUSARTx`     | `USARTx_t*`  | **Required**. Pointer to the USART peripheral. |
| `state`       | `uint8_t`   | **Required**. State of the peripheral (`ENABLE` or `DISABLE`). |

**Note**: The USART peripheral is disabled by default to allow control configuration before enabling it.


### USART Flag Status
**Definition :** Return the status of specific USART flag. (Check USART_SR Register for flags)
```c
uint8_t RB_USART_GetFlagStatus(USARTx_t *pUSARTx, uint32_t flag)
```

| Parameter   | Type      | Description |
| :---------- | :-------- | :-------------------------------------------------- |
| `pUSARTx`     | `USARTx_t*`  | **Required**. Pointer to the USART peripheral. |
| `flag`        | `uint32_t`  | **Required**. Desired flag (use the bit position in the `SPI_SR` register). |

**Return**: Status of the flag (FLAG_SET or FLAG_RESET).  


### USART Clear Flag
**Definition :** Clear a specific USART Flag
```c
void RB_USART_ClearFlag(USARTx_t *pUSARTx, uint32_t flag)
```

| Parameter   | Type      | Description |
| :---------- | :-------- | :-------------------------------------------------- |
| `pUSARTx`     | `USARTx_t*`  | **Required**. Pointer to the USART peripheral. |
| `flag`        | `uint32_t`  | **Required**. Desired flag (use the bit position in the `SPI_SR` register). |

### USART Event Callback Function 
**Definition :** This is a weakly defined callback function that the user can override to handle USART application events. The function is called when a specific USART event occurs, such as transmission complete, reception complete, or error detection.
```c
__weak void USART_ApplicationEventCallback(USARTx_Handler_t *pUSARTHandle, uint8_t AppEv)
```

| Parameter      | Type                 | Description |
| :------------ | :------------------- | :-------------------------------------------------- |
| `pUSARTHandle` | `USARTx_Handler_t*`     | **Required**. Pointer to the USART handle structure. |
| `AppEv`         | `uint8_t`              | **Required**. Application event identifier. |

---
# STM32F4xx SPI Driver
## APIs References

### SPI Initialization
**Definition :** initialize an SPI peripheral.
```c
void RB_SPI_Init(SPIx_Handler_t *pSPIHandle)
```

| Parameter            | Type             | Description                                         |
| :------------------- | :--------------- | :-------------------------------------------------- |
| `pSPIHandle`        | `SPIx_Handler_t*` | **Required**. Pointer to the SPI handler structure containing the configuration for the SPI peripheral. |


### SPI Deinitialization
**Definition :** Deinitialize an SPI peripheral. (Reset all SPI Registers)
```c
void RB_SPI_DeInit(SPIx_Handler_t *pSPIHandle)
```

| Parameter       | Type           | Description |
| :-------------- | :------------- | :-------------------------------------------------- |
| `pSPIHandle`    | `SPIx_Handler_t*` | **Required**. Pointer to the SPI handle structure. |


### SPI Clock Control
**Definition :** Enable or disable the clock on the SPI peripheral
```c
void RB_SPI_PeriClockControl(SPIx_t *pSPIx, uint8_t State)
```

| Parameter            | Type             | Description                                         |
| :------------------- | :--------------- | :-------------------------------------------------- |
| `pSPIx`             | `SPIx_t*`       | **Required**. Pointer to the SPI peripheral (e.g., `SPI1`). |
| `State`              | `uint8_t`        | **Required**. State to enable (`SET`) or disable (`RESET`) the SPI peripheral clock. |


### SPI Data Transmission (Poll Mode)
**Definition :** Transmit the data (Stored in a TxBuffer) (Blocking API)
```c
void RB_SPI_Data_TX(SPIx_t *pSPIx, uint8_t *pTxBuffer, uint16_t length)
```

| Parameter   | Type      | Description |
| :---------- | :-------- | :-------------------------------------------------- |
| `pSPIx`       | `SPIx_t*`    | **Required**. Pointer to the SPI peripheral. |
| `pTxBuffer`   | `uint8_t*`  | **Required**. Buffer containing the data to be transmitted. |
| `length`      | `uint16_t`  | **Required**. Length of the message to transmit. |

**Note**: This API is a blocking call (polling mode).


### SPI Data Reception (Poll Mode)
**Definition :** Receive the data and store it in  RxBuffer (Blocking API)
```c
void RB_SPI_Data_RX(SPIx_t *pSPIx, uint8_t *pRxBuffer, uint16_t length)
```

| Parameter   | Type      | Description |
| :---------- | :-------- | :-------------------------------------------------- |
| `pSPIx`       | `SPIx_t*`    | **Required**. Pointer to the SPI peripheral. |
| `pRxBuffer`   | `uint8_t*`  | **Required**. Buffer to store the received data. |
| `length`      | `uint16_t`  | **Required**. Length of the message to receive. |

**Note**: This API is a blocking call (polling mode).


### SPI Data Transmission (Interruption mode)
**Definition :** Transmit the data (Stored in a TxBuffer) (Non-Blocking API)
```c
uint8_t RB_SPI_Data_TXIT(SPIx_Handler_t *pSPIHandle, uint8_t *pTxBuffer, uint16_t length)
```

| Parameter    | Type            | Description |
| :----------- | :-------------- | :-------------------------------------------------- |
| `pSPIHandle`  | `SPIx_Handler_t*`  | **Required**. Pointer to the SPI handle structure. |
|`pTxBuffer`   | `uint8_t*`        | **Required**. Buffer containing the data to be transmitted. |
| `length`      | `uint16_t`        | **Required**. Length of the message to transmit. |

**Return**: Returns the state of the transmission. (Refer to stm32f401xx_spi.h header file for the states)

**Note**: This API is a non-blocking call (interrupt mode).


### SPI Data Reception (Interruption Mode)
**Definition :** Receive the data and store it in  RxBuffer (Non-Blocking API)
```c
uint8_t RB_SPI_Data_RXIT(SPIx_Handler_t *pSPIHandle, uint8_t *pRxBuffer, uint16_t length)
```

| Parameter    | Type            | Description |
| :----------- | :-------------- | :-------------------------------------------------- |
| `pSPIHandle`  | `SPIx_Handler_t*`  | **Required**. Pointer to the SPI handle structure. |
| `pRxBuffer`   | `uint8_t*`        | **Required**. Buffer to store the received data. |
| `length`      | `uint16_t`        | **Required**. Length of the message to receive. |

**Return**: Returns the state of the reception.  

**Note**: This API is a non-blocking call (interrupt mode).


### SPI Interrupt Request Enabling
**Definition :** IRQ Enable on CPU (Refers to NVIC Part in Arm CortexM4 RM)
```c
void RB_SPI_IRQITConfig(uint8_t IRQNumber, uint8_t state)
```

| Parameter            | Type             | Description                                         |
| :------------------- | :--------------- | :-------------------------------------------------- |
| `IRQNumber`          | `uint8_t`        | **Required**. Interrupt request line (e.g., 6 for EXTI Line 0). |
| `state`              | `uint8_t`        | **Required**. Enable (`ENABLE`) or Disable (`DISABLE`) the interrupt. |


### SPI Interrupt Priority Configuration
**Definition :** Configure the priority of the IRQ
```c
void RB_SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t Priority)
```

| Parameter            | Type             | Description                                         |
| :------------------- | :--------------- | :-------------------------------------------------- |
| `IRQNumber`          | `uint8_t`        | **Required**. Interrupt request line (e.g., 6 for EXTI Line 0). |
| `Priority`           | `uint32_t`       | **Required**. Interrupt priority (value between 0 and 15). |


### SPI Interrupt Request (IRQ) Handling
**Definition :** Handles the IRQ on SPI Peripherals
```c
void RB_SPI_IRQHandling(SPIx_Handler_t *pSPIHandle)
```

| Parameter       | Type           | Description |
| :-------------- | :------------- | :-------------------------------------------------- |
| `pSPIHandle`    | `SPIx_Handler_t*` | **Required**. Pointer to a SPI handle structure containing the SPI peripheral and its configuration. |


### USART Peripheral Enable
**Definition :** Enable/Disable the SPI Peripheral
```c
void SPI_PeriphControl(SPIx_t *pSPIx, uint8_t state)
```

| Parameter   | Type      | Description |
| :---------- | :-------- | :-------------------------------------------------- |
| `pSPIx`     | `SPIx_t*`  | **Required**. Pointer to the SPI peripheral. |
| `state`       | `uint8_t`   | **Required**. State of the peripheral (`ENABLE` or `DISABLE`). |

**Note**: The SPI peripheral is disabled by default to allow control configuration before enabling it.


### Software Slave Management
**Definition :** Enable/Disable the SSI in NSS Software mode
```c
void SPI_SSI_Config(SPIx_t *pSPIx, uint8_t state)
```

| Parameter  | Type      | Description |
| :--------- | :-------- | :-------------------------------------------------- |
| `pSPIx`     | `SPIx_t*`  | **Required**. Pointer to the SPI peripheral. |
| `state`     | `uint8_t`  | **Required**. State (ENABLE or DISABLE). |

**Note**: This configuration is necessary when using the SPI peripheral in master mode (Set SSI to 1).


### SPI Flag Status
**Definition :** Return the status of specific SPI flag. (Check SPI_SR Register for flags)
```c
uint8_t RB_SPI_GetFlagStatus(SPIx_t *pSPIx, uint8_t flag)
```

| Parameter  | Type      | Description |
| :--------- | :-------- | :-------------------------------------------------- |
| `pSPIx`     | `SPIx_t*`  | **Required**. Pointer to the SPI peripheral. |
| `flag`      | `uint8_t`  | **Required**. Desired flag (Use the bit position in the SPI_SR register). |

**Return**: Status of the flag (FLAG_SET or FLAG_RESET).  


### Close SPI Transmission
**Definition :** This API close the SPI Data Transmission
```c
void RB_SPI_CloseTx(SPIx_Handler_t *pSPIHandle)
```

| Parameter   | Type             | Description |
| :---------- | :--------------- | :-------------------------------------------------- |
| `pSPIHandle` | `SPIx_Handler_t*` | **Required**. Pointer to the SPI handler. |


### Close SPI Reception
**Definition :** This API close the SPI Data Reception
```c
void RB_SPI_CloseRx(SPIx_Handler_t *pSPIHandle)
```

| Parameter   | Type             | Description |
| :---------- | :--------------- | :-------------------------------------------------- |
| `pSPIHandle` | `SPIx_Handler_t*` | **Required**. Pointer to the SPI handler. |


---
# STM32F4xx I²C Driver
## APIs References


### I²C Initialization
**Definition :** initialize an I²C peripheral.
```c
void RB_I2C_Init(I2Cx_Handler_t *pI2CHandle)
```

| Parameter            | Type             | Description                                         |
| :------------------- | :--------------- | :-------------------------------------------------- |
| `pI2CHandle`        | `I2Cx_Handler_t*` | **Required**. Pointer to the I²C handler structure containing the configuration for the I²C peripheral. |


### I²C Deinitialization
**Definition :** Deinitialize an I²C peripheral. (Reset all I²C Registers)
```c
void RB_I2C_DeInit(I2Cx_Handler_t *pI2CHandle)
```

| Parameter       | Type           | Description |
| :-------------- | :------------- | :-------------------------------------------------- |
| `pI2CHandle`    | `I2Cx_Handler_t*` | **Required**. Pointer to the I²C handle structure. |


### I²C Clock Control
**Definition :** Enable or disable the clock on the I²C peripheral
```c
void RB_I2C_PeriClockControl(I2Cx_t *pI2Cx, uint8_t State)
```

| Parameter            | Type             | Description                                         |
| :------------------- | :--------------- | :-------------------------------------------------- |
| `pI2Cx`             | `I2Cx_t*`       | **Required**. Pointer to the I²C peripheral (e.g., `I2C3`). |
| `State`              | `uint8_t`        | **Required**. State to enable (`SET`) or disable (`RESET`) the SPI peripheral clock. |


### I²C Master Data Transmission (Poll mode)
**Definition :** Master Transmit Data (Stored in TxBuffer) (Blocking API)
```c
void RB_I2C_MasterTX(I2Cx_Handler_t *pI2CHandle, uint8_t* pTxBuffer, uint32_t length, uint8_t SlaveAddr, uint8_t Sr)
```

| Parameter   | Type              | Description |
| :---------- | :---------------- | :-------------------------------------------------- |
| `pI2CHandle` | `I2Cx_Handler_t*`   | **Required**. Pointer to the I2C handler. |
| `pTxBuffer`  | `uint8_t*`          | **Required**. Buffer containing the data to be transmitted. |
| `length`     | `uint32_t`          | **Required**. Length of the message to transmit. |
| `SlaveAddr`  | `uint8_t`           | **Required**. Address of the slave device. |
| `Sr`         | `uint8_t`           | **Required**. Specifies whether to send a repeated start condition. |
 
**Note**: This API is a blocking call.


### I²C Master Data Reception (Poll mode)
**Definition :** Master Receive Data and store it in RxBuffer (Blocking API)
```c
void RB_I2C_MasterRX(I2Cx_Handler_t *pI2CHandle, uint8_t* pRxBuffer, uint32_t length, uint8_t SlaveAddr, uint8_t Sr)
```

| Parameter   | Type              | Description |
| :---------- | :---------------- | :-------------------------------------------------- |
| `pI2CHandle` | `I2Cx_Handler_t*`   | **Required**. Pointer to the I2C handler. |
| `pRxBuffer`  | `uint8_t*`          | **Required**. Buffer where the received data will be stored. |
| `length`     | `uint32_t`          | **Required**. Length of the message to receive. |
| `SlaveAddr`  | `uint8_t`           | **Required**. Address of the slave device. |
| `Sr`         | `uint8_t`           | **Required**. Specifies whether to send a repeated start condition. |

**Note**: This API is a blocking call.


### I²C Master Data Transmission (Interruption mode)
**Definition :** Master Transmit Data (Stored in TxBuffer) (Non-Blocking API)
```c
uint8_t RB_I2C_MasterTX_IT(I2Cx_Handler_t *pI2CHandle, uint8_t* pTxBuffer, uint32_t length, uint8_t SlaveAddr, uint8_t Sr)
```

| Parameter   | Type              | Description |
| :---------- | :---------------- | :-------------------------------------------------- |
| `pI2CHandle` | `I2Cx_Handler_t*`   | **Required**. Pointer to the I2C handler. |
| `pTxBuffer`  | `uint8_t*`          | **Required**. Buffer containing the data to be transmitted. |
| `length`     | `uint32_t`          | **Required**. Length of the message to transmit. |
| `SlaveAddr`  | `uint8_t`           | **Required**. Address of the slave device. |
| `Sr`         | `uint8_t`           | **Required**. Specifies whether to send a repeated start condition. |

**Return**: `uint8_t` - State of the transmission.  (Refers to stm32f401xx_i2c.h header file for Tx States)

**Note**: This API is a non-blocking call.


### I²C Master Data Reception (Interruption mode)
**Definition :** Master Receive Data and store it in RxBuffer (Non-Blocking API)
```c
uint8_t RB_I2C_MasterRX_IT(I2Cx_Handler_t *pI2CHandle, uint8_t* pRxBuffer, uint32_t length, uint8_t SlaveAddr, uint8_t Sr)
```

| Parameter   | Type              | Description |
| :---------- | :---------------- | :-------------------------------------------------- |
| `pI2CHandle` | `I2Cx_Handler_t*`   | **Required**. Pointer to the I2C handler. |
| `pRxBuffer`  | `uint8_t*`          | **Required**. Buffer to store the received data. |
| `length`     | `uint32_t`          | **Required**. Length of the message to receive. |
| `SlaveAddr`  | `uint8_t`           | **Required**. Address of the slave device. |
| `Sr`         | `uint8_t`           | **Required**. Specifies whether to send a repeated start condition. |

**Return**: `uint8_t` - State of the reception.  (Refers to stm32f401xx_i2c.h header file for Rx States)

**Note**: This API is also a non-blocking call.


### I²C Slave Data Transmission (Poll mode)
**Definition :** Slave Transmit data in the parameter.
```c
void RB_I2C_SlaveTX(I2Cx_t *pI2C, uint8_t data)
```

| Parameter   | Type           | Description |
| :---------- | :------------- | :------------------------------------------------- |
| `pI2C`      | `I2Cx_t*`       | **Required**. Pointer to the I2C peripheral structure. |
| `data`      | `uint8_t`       | **Required**. Data to transmit. |


### I²C Slave Data Reception (Poll mode)
**Definition :** Slave Receive data as a function's return.
```c
uint8_t RB_I2C_SlaveRX(I2Cx_t *pI2C)
```

| Parameter   | Type           | Description |
| :---------- | :------------- | :------------------------------------------------- |
| `pI2C`      | `I2Cx_t*`       | **Required**. Pointer to the I2C peripheral structure. |

**Return**: `uint8_t`  : Returns the received data (1 byte).


### I²C Interrupt Request Enabling
**Definition :** IRQ Enable on CPU (Refers to NVIC Part in Arm CortexM4 RM)
```c
void RB_I2C_IRQITConfig(uint8_t IRQNumber, uint8_t state)
```

| Parameter            | Type             | Description                                         |
| :------------------- | :--------------- | :-------------------------------------------------- |
| `IRQNumber`          | `uint8_t`        | **Required**. Interrupt request line (e.g., 6 for EXTI Line 0). |
| `state`              | `uint8_t`        | **Required**. Enable (`ENABLE`) or Disable (`DISABLE`) the interrupt. |


### I²C Interrupt Priority Configuration
**Definition :** Configure the priority of the IRQ
```c
void RB_I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t Priority)
```

| Parameter            | Type             | Description                                         |
| :------------------- | :--------------- | :-------------------------------------------------- |
| `IRQNumber`          | `uint8_t`        | **Required**. Interrupt request line (e.g., 6 for EXTI Line 0). |
| `Priority`           | `uint32_t`       | **Required**. Interrupt priority (value between 0 and 15). |


### I²C Events Handling
**Definition :** Handle IRQ for I²C Events
```c
void RB_I2C_EV_IRQHandling(I2Cx_Handler_t *pI2CHandle)
```

| Parameter   | Type                  | Description                             |
| :---------- | :-------------------- | :-------------------------------------- |
| `pI2CHandle`| `I2Cx_Handler_t*`      | **Required**. Pointer to the I2C handler structure. |

**Note**: Interrupt handling for I2C events.


### I²C Errors Handling
**Definition :** Handle IRQ for I²C Errors
```c
void RB_I2C_ER_IRQHandling(I2Cx_Handler_t *pI2CHandle)
```

| Parameter   | Type                  | Description                             |
| :---------- | :-------------------- | :-------------------------------------- |
| `pI2CHandle`| `I2Cx_Handler_t*`      | **Required**. Pointer to the I2C handler structure. |

**Note**: Interrupt handling for I2C errors.


### I²C Peripheral Enable
**Definition :** Enable/Disable the I²C Peripheral
```c
void RB_I2C_PeriphControl(I2Cx_t *pI2Cx, uint8_t state)
```

| Parameter   | Type                 | Description                              |
| :---------- | :------------------- | :--------------------------------------- |
| `pI2Cx`     | `I2Cx_t*`             | **Required**. Pointer to the I2C peripheral. |
| `state`     | `uint8_t`             | **Required**. State to enable (`ENABLE`) or disable (`DISABLE`) the I2C peripheral. |

**Note**: The I2C Peripheral is disabled by default to allow configuration of the I2C peripheral before enabling.


### I²C Flag Status
**Definition :** Return the status of specific I²C flag. (Check I2C_SR Register for flags)
```c
uint8_t RB_I2C_GetFlagStatus(I2Cx_t *pI2Cx, uint8_t flag)
```

| Parameter   | Type                 | Description                                                       |
| :---------- | :------------------- | :---------------------------------------------------------------- |
| `pI2Cx`     | `I2Cx_t*`             | **Required**. Pointer to the I2C peripheral.                      |
| `flag`      | `uint8_t`             | **Required**. Desired flag (uses macros in `@I2C_Flags`).         |

**Return**: `uint8_t` : Returns the status of the flag: `FLAG_SET` or `FLAG_RESET`.


### Acknowledgment Setting
**Definition :** Enable or Disable the Acknowledgment
```c
void RB_I2C_ManageAcking(I2Cx_t *pI2Cx, uint8_t status)
```

| Parameter   | Type                 | Description                                                       |
| :---------- | :------------------- | :---------------------------------------------------------------- |
| `pI2Cx`     | `I2Cx_t*`             | **Required**. Pointer to the I2C peripheral.                      |
| `status`    | `uint8_t`             | **Required**. Desired status (`ENABLE` or `DISABLE`).             |



# Contributing
Contributions are welcome! If you'd like to report bugs, suggest features, or submit improvements, please open an issue or create a pull request.

# License
This project is licensed under the MIT License. See the LICENSE file for details.
