# STM32 Bare-Metal Custom Headers (No CMSIS / No HAL)

This repository contains **fully custom header files** for STM32 microcontroller families.  
It is written from scratch for **pure register-level programming** — without using STM32 HAL or CMSIS.

**Supported Boards:**
- STM32F103 (Original implementation)
- **STM32F446RE Nucleo Board** (Current implementation)

This project is ideal for:
- Learning bare-metal embedded systems
- Understanding how microcontroller registers work internally
- Building your own driver stack from zero
- Educational and hobby projects

---

# 🚀 Features

### ✔ Fully handwritten register mapping  
GPIO, RCC, and all memory-mapped registers are defined manually.

### ✔ No CMSIS  
Zero dependency — ideal for understanding ARM Cortex-M at low level.

### ✔ Supports multiple GPIO ports  
GPIOA through GPIOH (depending on MCU variant).

### ✔ Clock enable macros  
Simple one-line macros to enable peripheral clocks.

### ✔ Example main program  
LED blinking on STM32F446RE Nucleo board (PA5 - LD2) using direct register access.

### ✔ Multiple MCU support
- `stm32f103xx.h` - For STM32F103 series (Cortex-M3)
- `stm32f446xx.h` - For STM32F446 series (Cortex-M4)

---

# 📂 Folder Structure

```
STM32F103-BareMetal-Custom-Header/
│
├── src/
│   └── main.c              (Currently configured for STM32F446RE)
│
├── inc/
│   ├── stm32f103xx.h       (Header for STM32F103)
│   └── stm32f446xx.h       (Header for STM32F446RE)
│
└── README.md
```

---

# 📄 Custom Headers

## stm32f103xx.h (STM32F103)
Located at: `inc/stm32f103xx.h`

For STM32F103 (Cortex-M3) with CRL/CRH GPIO configuration style.

## stm32f446xx.h (STM32F446RE - Current)
Located at: `inc/stm32f446xx.h`

For STM32F446 (Cortex-M4) with MODER/OTYPER/OSPEEDR GPIO configuration style.

Both files include:
- GPIO register struct  
- RCC register struct  
- Base addresses  
- Register pointer definitions  
- Clock enable macros  

The goal is **clean, readable bare-metal code**.

---

# 💡 Example: LED Blink on STM32F446RE Nucleo

Located in: `src/main.c`

**Onboard LED (LD2)** on Nucleo-F446RE is connected to **PA5**.

```c
#include "stm32f446xx.h"

void delay(int t){
    for (int i = t * 8000; i > 0; i--);
}

int main(void)
{
    // Enable GPIOA clock
    GPIOA_CLK_EN();
    
    // Configure PA5 as output (onboard LED LD2)
    GPIOA->MODER &= ~(0x3 << (5 * 2));  // Clear mode bits
    GPIOA->MODER |= (0x1 << (5 * 2));   // Set as output
    
    GPIOA->OTYPER &= ~(1 << 5);         // Push-pull
    GPIOA->OSPEEDR |= (0x1 << (5 * 2)); // Medium speed
    GPIOA->PUPDR &= ~(0x3 << (5 * 2));  // No pull-up/down
    
    while(1){
        GPIOA->ODR ^= (1 << 5);         // Toggle LED
        delay(200);
    }
}
```

---

# 🛠 How to Build

Use any ARM GCC toolchain.

**For STM32F446RE (Cortex-M4):**
```bash
arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 \
    src/main.c -Iinc -o main.elf

arm-none-eabi-objcopy -O binary main.elf main.bin
```

**For STM32F103 (Cortex-M3):**
```bash
arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb src/main.c -Iinc -o main.elf
arm-none-eabi-objcopy -O binary main.elf main.bin
```

Flash with ST-Link:

```bash
st-flash write main.bin 0x08000000
```

---

# 📘 Why This Project?

This project helps you understand:

- How registers are mapped in memory  
- How GPIO, RCC, and clocks work internally  
- How to build your own microcontroller library  
- How hardware abstraction REALLY works  

It is the best starting point for embedded engineers moving from Arduino/HAL into **pure firmware development**.

---

# 📌 Future Updates

Planned additions:

- EXTI driver  
- SysTick delay driver  
- Timer register mapping  
- UART bare-metal driver  
- ADC bare-metal driver  
- LCD/7-segment examples using custom header  

---

# 📎 License

MIT License — free to use for learning, projects, or commercial applications.

---

# ⭐ Contribute

Pull requests and suggestions are welcome!  
This project aims to become the simplest learning repository for STM32 bare-metal programming.

