
# STM32F4xx GPIO Driver

This repository contains a custom driver library for STM32F4xx microcontrollers, written entirely from scratch as part of my learning journey. It provides basic and modular functions to configure and interact with peripherals, helping me deepen my understanding of embedded systems and microcontroller programming

For this, I'm using my Nucleo-F401RE board, and you can find its datasheet on the  [official STM32 website](https://www.st.com/en/evaluation-tools/nucleo-f401re.html) 


## Clone the Repository  
If you're interested in exploring the code, you can clone this repository to your STM32CubeIDE workspace using the following command:  
```bash
git clone https://github.com/your-username/stm32f4xx-driver.git  
```


## API Reference


#### GPIO Initialization

```c
void RB_GPIO_Init(GPIOx_Handler_t *pGPIOHandle)
```

| Parameter            | Type             | Description                                         |
| :------------------- | :--------------- | :-------------------------------------------------- |
| `pGPIOHandle`        | `GPIOx_Handler_t*` | **Required**. Pointer to the GPIO handler structure containing the configuration for the GPIO pin. |

#### GPIO Clock Control

```c
void RB_GPIO_PeriClockControl(GPIOx_t *pGPIOx, uint8_t State)
```

| Parameter            | Type             | Description                                         |
| :------------------- | :--------------- | :-------------------------------------------------- |
| `pGPIOx`             | `GPIOx_t*`       | **Required**. Pointer to the GPIO port (e.g., `GPIOA`). |
| `State`              | `uint8_t`        | **Required**. State to enable (`SET`) or disable (`RESET`) the GPIO peripheral clock. |

#### GPIO Pin Data Read

```c
uint8_t RB_GPIO_ReadInputPin(GPIOx_t *pGPIOx, uint8_t PinNumber)
```

| Parameter            | Type             | Description                                         |
| :------------------- | :--------------- | :-------------------------------------------------- |
| `pGPIOx`             | `GPIOx_t*`       | **Required**. Pointer to the GPIO port (e.g., `GPIOA`). |
| `PinNumber`          | `uint8_t`        | **Required**. GPIO pin number to read. |

#### GPIO Pin Data Write

```c
void RB_GPIO_WriteOutputPin(GPIOx_t *pGPIOx, uint8_t PinNumber, uint8_t value)
```

| Parameter            | Type             | Description                                         |
| :------------------- | :--------------- | :-------------------------------------------------- |
| `pGPIOx`             | `GPIOx_t*`       | **Required**. Pointer to the GPIO port (e.g., `GPIOA`). |
| `PinNumber`          | `uint8_t`        | **Required**. GPIO pin number to write to. |
| `value`              | `uint8_t`        | **Required**. Value to write (either `GPIO_PIN_SET` or `GPIO_PIN_RESET`). |

#### GPIO Pin Toggle

```c
void RB_GPIO_TogglePin(GPIOx_t *pGPIOx, uint8_t PinNumber)
```

| Parameter            | Type             | Description                                         |
| :------------------- | :--------------- | :-------------------------------------------------- |
| `pGPIOx`             | `GPIOx_t*`       | **Required**. Pointer to the GPIO port (e.g., `GPIOA`). |
| `PinNumber`          | `uint8_t`        | **Required**. GPIO pin number to toggle. |

#### GPIO Interrupt Configuration

```c
void RB_GPIO_IRQITConfig(uint8_t IRQNumber, uint8_t state)
```

| Parameter            | Type             | Description                                         |
| :------------------- | :--------------- | :-------------------------------------------------- |
| `IRQNumber`          | `uint8_t`        | **Required**. Interrupt request line (e.g., 6 for EXTI Line 0). |
| `state`              | `uint8_t`        | **Required**. Enable (`ENABLE`) or Disable (`DISABLE`) the interrupt. |

#### GPIO Interrupt Priority Configuration

```c
void RB_GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t Priority)
```

| Parameter            | Type             | Description                                         |
| :------------------- | :--------------- | :-------------------------------------------------- |
| `IRQNumber`          | `uint8_t`        | **Required**. Interrupt request line (e.g., 6 for EXTI Line 0). |
| `Priority`           | `uint32_t`       | **Required**. Interrupt priority (value between 0 and 15). |

---

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

## Contributing
Contributions are welcome! If you'd like to report bugs, suggest features, or submit improvements, please open an issue or create a pull request.

## License
This project is licensed under the MIT License. See the LICENSE file for details.