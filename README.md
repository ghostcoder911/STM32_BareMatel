# STM32F103 Bare-Metal Custom Header (No CMSIS / No HAL)

This repository contains a **fully custom header file** for the STM32F103 microcontroller family.  
It is written from scratch for **pure register-level programming** — without using STM32 HAL or CMSIS.

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
GPIOA, GPIOB, GPIOC, GPIOD, GPIOE.

### ✔ Clock enable macros  
Simple one-line macros to enable peripheral clocks.

### ✔ Example main program  
LED blinking using direct register access.

---

# 📂 Folder Structure

```
STM32F103-BareMetal-Custom-Header/
│
├── src/
│   └── main.c
│
├── inc/
│   └── stm32f103xx.h
│
└── README.md
```

---

# 📄 stm32f103xx.h (Custom Header)

Located at: `inc/stm32f103xx.h`

This file includes:
- GPIO register struct  
- RCC register struct  
- Base addresses  
- Register pointer definitions  
- Clock enable macros  

The goal is **clean, readable bare-metal code**.

---

# 💡 Example: LED Toggle on PA1

Located in: `src/main.c`

```c
#include "stm32f103xx.h"

void delay_ms(int t)
{
    for (int i = 0; i < t * 8000; i++)
        __asm__("nop");
}

int main(void)
{
    GPIOA_CLK_EN();

    // PA1 -> Output push-pull (2 MHz)
    GPIOA->CRL &= ~(0xF << (1 * 4));
    GPIOA->CRL |=  (0x2 << (1 * 4));

    while (1)
    {
        GPIOA->BSRR = (1 << 1);     // Set PA1
        delay_ms(300);

        GPIOA->BRR = (1 << 1);      // Reset PA1
        delay_ms(300);
    }
}
```

---

# 🛠 How to Build

Use any ARM GCC toolchain.

```
arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb src/main.c -Iinc -o main.o
```

Flash with ST-Link:

```
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

