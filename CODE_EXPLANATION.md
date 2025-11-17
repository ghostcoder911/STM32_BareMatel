# 📖 Code Explanation - STM32F446RE LED Blink

This document provides a detailed explanation of how the GPIO configuration works in the STM32F446RE bare-metal LED blink example.

---

## 📂 File: `src/main.c`

```c
#include "stm32f446xx.h"

void delay(int t){
	for (int i = t * 8000; i > 0; i--);
}

int main(void)
{
	// Enable GPIOA clock
	GPIOA_CLK_EN();
	
	// Configure PA5 as output (onboard LED LD2 on Nucleo-F446RE)
	// Clear mode bits for pin 5 (bits 10-11)
	GPIOA->MODER &= ~(0x3 << (5 * 2));
	// Set mode to output (01)
	GPIOA->MODER |= (0x1 << (5 * 2));
	
	// Set output type to push-pull (default, bit 5 = 0)
	GPIOA->OTYPER &= ~(1 << 5);
	
	// Set output speed to medium (01)
	GPIOA->OSPEEDR &= ~(0x3 << (5 * 2));
	GPIOA->OSPEEDR |= (0x1 << (5 * 2));
	
	// No pull-up/pull-down (00)
	GPIOA->PUPDR &= ~(0x3 << (5 * 2));
	
	while(1){
		// Toggle PA5 (LED)
		GPIOA->ODR ^= (1 << 5);
		delay(200);
	}
}
```

---

## 🔍 Detailed Step-by-Step Explanation

### **Step 1: Enable GPIOA Clock**

```c
GPIOA_CLK_EN();
```

**Why is this needed?**

In STM32 microcontrollers, peripherals are connected to various buses (AHB1, AHB2, APB1, APB2). By default, these peripheral clocks are **disabled** to save power. Before using any peripheral (like GPIOA), you **must** enable its clock.

**What happens under the hood:**

This macro expands to:
```c
RCC->AHB1ENR |= (1 << 0);  // Set bit 0 to enable GPIOA clock
```

**Breakdown:**
- `RCC` = Reset and Clock Control peripheral (manages all system clocks)
- `AHB1ENR` = AHB1 peripheral clock Enable Register
- Bit 0 controls GPIOA clock
- `|=` performs a bitwise OR to set the bit without affecting other bits

**Memory Address:**
- RCC Base: `0x40023800`
- AHB1ENR Offset: `0x30`
- Full Address: `0x40023830`

---

### **Step 2: Configure Pin Mode (MODER Register)**

```c
// Clear mode bits for pin 5 (bits 10-11)
GPIOA->MODER &= ~(0x3 << (5 * 2));
// Set mode to output (01)
GPIOA->MODER |= (0x1 << (5 * 2));
```

**What is MODER?**

The **MODE Register** sets the pin's function. Each pin uses **2 bits**:

| Bits | Mode | Description |
|------|------|-------------|
| 00 | Input | Pin reads external signals |
| **01** | **Output** | **Pin drives external devices** |
| 10 | Alternate Function | Pin controlled by peripherals (UART, SPI, etc.) |
| 11 | Analog | Pin used for ADC/DAC |

**Detailed Breakdown:**

**Line 1: Clear existing bits**
```c
GPIOA->MODER &= ~(0x3 << (5 * 2));
```

Let's break this down piece by piece:
- `5 * 2 = 10` → Pin 5 starts at bit position 10
- `0x3` = binary `0b11` (two bits set to 1)
- `0x3 << 10` = `0b11 << 10` = `0b0000 1100 0000 0000` (bits 10-11 set)
- `~(0x3 << 10)` = `0b1111 0011 1111 1111` (bits 10-11 cleared, all others set)
- `&=` performs AND operation, clearing bits 10-11 while preserving all other bits

**Line 2: Set to output mode**
```c
GPIOA->MODER |= (0x1 << (5 * 2));
```

- `0x1` = binary `0b01` (our desired output mode)
- `0x1 << 10` = `0b0000 0100 0000 0000` (bit 10 set, bit 11 clear)
- `|=` performs OR operation, setting bit 10 = 1, bit 11 = 0

**Visual Representation:**

```
MODER register (32 bits, 16 pins × 2 bits each):

Bit position: 31 30|29 28|...|13 12|11 10|9 8|7 6|5 4|3 2|1 0
Pin number:   15   |14   |...| 6   | 5   |4  |3  |2  |1  |0
                                    ^^^^
                                    Pin 5 bits
                                    We set to: 01 (Output)

Before: MODER = 0xXXXX XXXX  (unknown state)
After:  MODER = 0xXXXX X4XX  (bits 10-11 = 01)
```

---

### **Step 3: Configure Output Type (OTYPER Register)**

```c
// Set output type to push-pull (bit 5 = 0)
GPIOA->OTYPER &= ~(1 << 5);
```

**What is OTYPER?**

The **Output TYPE Register** determines how the pin drives the output. Each pin uses **1 bit** (only the lower 16 bits are used):

| Bit Value | Type | Description |
|-----------|------|-------------|
| **0** | **Push-Pull** | Pin actively drives both HIGH (3.3V) and LOW (0V) |
| 1 | Open-Drain | Pin only actively drives LOW, HIGH is floating (needs external pull-up) |

**Push-Pull vs Open-Drain:**

**Push-Pull (Bit = 0):**
```
HIGH state: Pin connected to VDD (3.3V) through internal transistor
LOW state:  Pin connected to GND (0V) through internal transistor
Use case:   LEDs, most digital outputs
```

**Open-Drain (Bit = 1):**
```
HIGH state: Pin is floating (high impedance), needs external pull-up resistor
LOW state:  Pin connected to GND (0V) through internal transistor
Use case:   I2C communication, driving higher voltages with external pull-up
```

**For our LED:** We use push-pull because the LED needs both active HIGH and LOW states.

**Breakdown:**
- `(1 << 5)` creates a mask with bit 5 set: `0b0000 0000 0010 0000`
- `~(1 << 5)` inverts it: `0b1111 1111 1101 1111`
- `&=` clears bit 5, setting push-pull mode

---

### **Step 4: Configure Output Speed (OSPEEDR Register)**

```c
// Set output speed to medium (01)
GPIOA->OSPEEDR &= ~(0x3 << (5 * 2));
GPIOA->OSPEEDR |= (0x1 << (5 * 2));
```

**What is OSPEEDR?**

The **Output SPEED Register** controls how fast the pin can change states (rise/fall time). Each pin uses **2 bits**:

| Bits | Speed | Max Frequency | Use Case |
|------|-------|---------------|----------|
| 00 | Low | 2 MHz | Low-speed I/O, reduces EMI |
| **01** | **Medium** | **25 MHz** | **General purpose I/O** |
| 10 | High | 50 MHz | Fast communication protocols |
| 11 | Very High | 100 MHz | Very high-speed interfaces |

**Why does speed matter?**

1. **Electromagnetic Interference (EMI):**
   - Faster switching = sharper edges = more high-frequency components
   - More EMI/noise on power rails and surrounding circuits

2. **Power Consumption:**
   - Faster switching = higher dynamic power consumption

3. **Signal Integrity:**
   - For high-speed signals, you need faster slew rates
   - For LED blinking, speed doesn't matter

**For our LED:** Medium speed (25 MHz) is more than sufficient. The LED blinks at ~2.5 Hz (way below 25 MHz!).

**Breakdown:** (Same pattern as MODER)
- Clear bits 10-11: `&= ~(0x3 << 10)`
- Set to medium (01): `|= (0x1 << 10)`

---

### **Step 5: Configure Pull-Up/Pull-Down (PUPDR Register)**

```c
// No pull-up/pull-down (00)
GPIOA->PUPDR &= ~(0x3 << (5 * 2));
```

**What is PUPDR?**

The **Pull-Up/Pull-Down Register** adds internal resistors to the pin. Each pin uses **2 bits**:

| Bits | Configuration | Description |
|------|---------------|-------------|
| **00** | **No pull** | **Pin has no internal resistor** |
| 01 | Pull-up | Internal ~40kΩ resistor to VDD (3.3V) |
| 10 | Pull-down | Internal ~40kΩ resistor to GND (0V) |
| 11 | Reserved | Not used |

**When to use pull resistors?**

**Input Mode:**
```
Problem:  Floating input (not connected) can read random values
Solution: Pull-up/down ensures defined state when nothing is connected

Example - Button with pull-up:
  Button pressed:   Pin reads 0 (connected to GND)
  Button released:  Pin reads 1 (pulled to VDD by resistor)
```

**Output Mode:**
```
Not needed: Pin actively drives the voltage
Our case:   We're driving an LED in output mode, no pull needed
```

**Schematic Representation:**

```
Without pull (00):
         GPIO Pin -----[LED]----[Resistor]----GND
         (actively drives 3.3V or 0V)

With pull-up (01):
    VDD (3.3V)
         |
        [40kΩ]  <-- Internal pull-up resistor (not needed here)
         |
         GPIO Pin -----[LED]----[Resistor]----GND
```

**Breakdown:**
- `~(0x3 << 10)` clears bits 10-11
- `&=` operation sets both bits to 0 (no pull)

---

### **Step 6: Main Loop - Toggle LED**

```c
while(1){
    // Toggle PA5 (LED)
    GPIOA->ODR ^= (1 << 5);
    delay(200);
}
```

**What is ODR?**

The **Output Data Register** controls the actual output state of pins. Each bit corresponds to one pin:

| Bit Value | Pin State | LED State |
|-----------|-----------|-----------|
| 0 | LOW (0V) | OFF |
| 1 | HIGH (3.3V) | ON |

**Toggle Operation using XOR (`^=`):**

The XOR (exclusive OR) operator has a special property:
```
0 XOR 1 = 1  (bit flips from 0 to 1)
1 XOR 1 = 0  (bit flips from 1 to 0)
```

**Example Execution:**

```
Initial state:
ODR = 0x0000 0000 (all pins LOW)
LED = OFF

First toggle:
ODR ^= (1 << 5) = ODR ^= 0x0020
ODR = 0x0000 0020 (bit 5 = 1)
LED = ON

Second toggle:
ODR ^= (1 << 5) = ODR ^= 0x0020
ODR = 0x0000 0000 (bit 5 = 0)
LED = OFF

Third toggle:
ODR = 0x0000 0020 (bit 5 = 1)
LED = ON
...and so on
```

**Alternative methods (not used here):**

```c
// Method 1: Using BSRR (Bit Set/Reset Register) - More efficient!
GPIOA->BSRR = (1 << 5);      // Set bit 5 (LED ON)
GPIOA->BSRR = (1 << (5+16)); // Reset bit 5 (LED OFF)

// Method 2: Direct assignment
GPIOA->ODR |= (1 << 5);      // Set bit 5 (LED ON)
GPIOA->ODR &= ~(1 << 5);     // Clear bit 5 (LED OFF)

// Method 3: XOR toggle (what we use)
GPIOA->ODR ^= (1 << 5);      // Toggle bit 5
```

**Why we use XOR toggle:**
- Simple and concise
- One line does both ON and OFF
- Good for toggling/blinking applications

---

## 🆚 Key Difference: STM32F1 vs STM32F4

### **STM32F103 (F1 series) - Old Architecture:**

```c
// Single register (CRL/CRH) controls everything
GPIOA->CRL &= ~(0xF << (1 * 4));  // Clear 4 bits for pin 1
GPIOA->CRL |=  (0x2 << (1 * 4));  // Set mode+config
```

**Characteristics:**
- **One register** (CRL for pins 0-7, CRH for pins 8-15)
- **4 bits per pin** (mode + speed + type combined)
- Less flexible: changing one aspect requires bit manipulation

**Bit Layout (4 bits per pin):**
```
[3:2] CNF (Configuration)
[1:0] MODE (Mode and speed)

Example: 0x2 = 0b0010
  MODE[1:0] = 10 = Output 2 MHz
  CNF[3:2]  = 00 = Push-pull output
```

### **STM32F446 (F4 series) - Modern Architecture:**

```c
// Separate registers for each aspect
GPIOA->MODER   |= (0x1 << (5 * 2));  // Mode (2 bits)
GPIOA->OTYPER  &= ~(1 << 5);         // Type (1 bit)
GPIOA->OSPEEDR |= (0x1 << (5 * 2));  // Speed (2 bits)
GPIOA->PUPDR   &= ~(0x3 << (5 * 2)); // Pull (2 bits)
```

**Characteristics:**
- **Separate registers** for different aspects
- **More flexible:** Can modify one aspect without affecting others
- **Easier to read:** Each register has a clear purpose
- **More features:** Separate pull-up/pull-down control

---

## 📊 Quick Reference Card

### **Register Summary for Pin 5 (PA5)**

| Register | Bit Range | Bits Per Pin | Value | Meaning |
|----------|-----------|--------------|-------|---------|
| **MODER** | [11:10] | 2 | 01 | Output mode |
| **OTYPER** | [5] | 1 | 0 | Push-pull |
| **OSPEEDR** | [11:10] | 2 | 01 | Medium speed (25 MHz) |
| **PUPDR** | [11:10] | 2 | 00 | No pull |
| **ODR** | [5] | 1 | Toggle | LED state (0=OFF, 1=ON) |

### **Memory Addresses**

| Register | Base Address | Offset | Full Address |
|----------|--------------|--------|--------------|
| GPIOA Base | 0x40020000 | - | 0x40020000 |
| MODER | 0x40020000 | 0x00 | 0x40020000 |
| OTYPER | 0x40020000 | 0x04 | 0x40020004 |
| OSPEEDR | 0x40020000 | 0x08 | 0x40020008 |
| PUPDR | 0x40020000 | 0x0C | 0x4002000C |
| ODR | 0x40020000 | 0x14 | 0x40020014 |

### **Bit Manipulation Patterns**

```c
// Pattern for 2-bit fields (MODER, OSPEEDR, PUPDR)
REG &= ~(0x3 << (pin * 2));  // Clear 2 bits
REG |=  (val << (pin * 2));  // Set 2 bits to val

// Pattern for 1-bit fields (OTYPER)
REG &= ~(1 << pin);          // Clear 1 bit
REG |=  (val << pin);        // Set 1 bit to val

// Toggle pattern (ODR)
REG ^= (1 << pin);           // Flip 1 bit
```

---

## 🔧 Hardware Connection

### **STM32F446RE Nucleo Board**

```
Onboard LED (LD2):
  - Connected to PA5
  - Green LED
  - Active HIGH (LED turns ON when PA5 = 3.3V)
  
Physical connection:
  PA5 ----[LED]----[Resistor]----GND
  
  When PA5 = HIGH (3.3V): Current flows, LED ON
  When PA5 = LOW  (0V):   No current, LED OFF
```

---

## 💡 Learning Tips

1. **Use the Reference Manual:** STM32F446 Reference Manual (RM0390) contains detailed register descriptions

2. **Practice bit manipulation:** Understanding `<<`, `&`, `|`, `^`, `~` operators is crucial

3. **Read-Modify-Write:** Always clear bits before setting them to avoid unintended states

4. **Hardware before Software:** Enable clocks before accessing peripherals (or you'll get a hard fault!)

5. **Start simple:** Master GPIO before moving to complex peripherals like timers or UART

---

## 🚀 Next Steps

Once you understand GPIO, you can explore:
1. **BSRR Register** - Atomic bit set/reset (faster and safer than ODR)
2. **IDR Register** - Reading button inputs
3. **EXTI** - External interrupts for button press detection
4. **Timers** - Precise timing instead of busy-wait delays
5. **PWM** - LED brightness control
6. **Alternate Functions** - Using GPIO pins for UART, SPI, I2C, etc.

---

## 📚 References

- **STM32F446xx Reference Manual (RM0390)** - Sections:
  - Section 7: RCC (Reset and Clock Control)
  - Section 8: GPIO (General Purpose I/O)
  
- **STM32F446RE Datasheet** - Pin mappings and electrical characteristics

- **Nucleo-F446RE User Manual (UM1724)** - Board-specific information

---

**Happy Bare-Metal Programming! 🎉**

