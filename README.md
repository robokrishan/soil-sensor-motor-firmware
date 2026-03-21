# 🚁 Soil Sensor Deployment Mechanism Firmware (ESP32-C6)

This firmware controls a **stepper-driven linear actuator** used to deploy and retract a soil sensor on a drone platform.

It interfaces with:
- A **NEMA17 stepper motor**
- **A4988 stepper driver**
- **Limit switches** for motion boundaries
- **GPIO-triggered inputs** for extend/retract commands

---

## 📌 Features

- ✅ Extend / retract control via GPIO inputs  
- ✅ Hardware limit switch protection  
- ✅ Automatic motor stop at limits  
- ✅ Driver sleep/disable to eliminate idle noise  
- ✅ Watchdog-safe (FreeRTOS compliant)  
- ✅ Simple and reliable design (no blocking starvation)

---

## ⚙️ Hardware Overview

### 🧠 Microcontroller
- ESP32-C6

### ⚡ Motor Driver
- A4988 Stepper Driver

### 🔩 Actuation
- NEMA17 Stepper Motor
- Lead screw linear actuator

### 🔘 Inputs
- 2x limit switches (top & bottom)
- 2x GPIO trigger inputs (extend / retract)

---

## 🔌 Pin Configuration

### Stepper Driver

| Signal | ESP32 Pin |
|--------|----------|
| DIR    | GPIO8    |
| STEP   | GPIO9    |
| SLP/RST| GPIO14   |
| EN     | GPIO15   |
| VDD    | 3V3      |
| GND    | GND      |

### Limit Switches

| Function        | ESP32 Pin |
|----------------|----------|
| Bottom Limit   | GPIO6    |
| Top Limit      | GPIO7    |

### Control Inputs

| Function | ESP32 Pin |
|----------|----------|
| Extend   | GPIO1    |
| Retract  | GPIO2    |

---

## 🔧 Wiring Notes

### Limit Switches (Normally Open)

- **C → GND**
- **NO → GPIO (with internal pull-up enabled)**

Logic:
- Not pressed → HIGH  
- Pressed → LOW  

---

### Control Inputs

- Leave GPIO1 and GPIO2 **floating (internal pull-up enabled)**
- Trigger by **shorting to GND**

---

### Motor Power

- VMOT → External 12V supply  
- GND → Common ground with ESP32  

⚠️ **Important:** All grounds must be shared.

---

## 🚀 How It Works

### Extend
- Trigger: GPIO1 pulled LOW  
- Motor rotates in extend direction  
- Stops when:
  - Bottom limit switch is hit  
  - Or safety condition triggers  

---

### Retract
- Trigger: GPIO2 pulled LOW  
- Motor rotates in reverse  
- Stops when:
  - Top limit switch is hit  

---

### Driver Behavior

| State        | EN | SLP |
|--------------|----|-----|
| Active       | LOW | HIGH |
| Idle (silent)| HIGH | LOW  |

👉 This removes the **hissing noise** when idle.

---

## 🧠 Software Architecture

- Single FreeRTOS task (`button_task`)
- Blocking motion loops with periodic yielding:
  - `esp_rom_delay_us()` → precise stepping
  - `vTaskDelay()` → watchdog safety

---

## ⏱ Timing

- Step delay: ~1000 µs per half cycle  
- Periodic yield every ~100 steps  
- Prevents watchdog resets  

---

## 🛑 Safety Features

- Limit switch enforcement (hardware + software)
- Immediate stop on limit detection
- Driver disable after motion
- Watchdog-safe execution

---

## ⚠️ Important Notes

### 1. GPIO1 / GPIO2
Some ESP32-C6 boards use these pins for boot or USB.

👉 If unstable, reassign to other GPIOs.

---

### 2. Stepper Direction
If motion is reversed:
```c
gpio_set_level(STEPPER_DIR_GPIO, 1);