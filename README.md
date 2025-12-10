# **UnMutex – Real-Time Voice-Piloted Controller**
*A Raspberry Pi Pico + FreeRTOS + Keyword Spotting Project*

UnMutex is a **real-time embedded voice controller** designed to manage critical tasks by combining **audio acquisition on Raspberry Pi Pico** and **keyword-spotting AI on a host PC**.  
The system uses a MAX4466 microphone to stream raw audio from the microcontroller to the PC, where a CNN-based KWS model detects four voice commands: **on**, **off**, **up**, **down**.  
Each recognized command is then sent back to the Pico to control hardware peripherals (LED and buzzer) through synchronized FreeRTOS tasks.


---

## **Table of Contents**
- [Project Overview](#project-overview)
- [System Architecture](#system-architecture)
- [Hardware Components](#hardware-components)
- [Software Architecture](#software-architecture)
- [Commands & Real-Time Behavior](#commands--real-time-behavior)
- [Packet Format](#packet-format)
- [Keyword Spotting Model](#keyword-spotting-model)
- [How to Build & Run](#how-to-build--run)

---

## **Project Overview**

UnMutex is a distributed embedded system composed of:

### **1. Raspberry Pi Pico**
- Samples audio at **16 kHz** from a **MAX4466 microphone**
- Sends audio frames via USB serial
- Receives commands from the PC and controls hardware peripherals
- Uses **FreeRTOS** for real-time scheduling, mutexes, and queues

### **2. Host PC (Python)**
- Reconstructs audio packets
- Extracts **MFCC** features
- Runs a **CNN-based keyword spotting model**
- Sends detected commands (e.g., `CMD:on`) back to the Pico

The system is designed for **high responsiveness**, **deterministic execution**, and **safe concurrent control** of shared hardware resources.

---

## **System Architecture**

### **High-Level Workflow**
1. Microphone → ADC (16 kHz, 12-bit)  
2. Ping-pong buffers filled via ADC interrupt  
3. Full buffers queued to the USB streaming task  
4. PC reconstructs packets + runs keyword spotting  
5. PC sends command strings over serial  
6. Pico activates LED/buzzer tasks accordingly  
7. FreeRTOS enforces priority rules (e.g., **alarm > blink > off > on**)

---

## **Hardware Components**

| Component | Description |
|----------|-------------|
| **Raspberry Pi Pico** | Dual-core ARM Cortex-M0+, ADC 12-bit, USB |
| **MAX4466 Microphone** | Analog electret mic with low-noise amplifier |
| **LED (GPIO 25)** | Visual feedback |
| **Buzzer (GPIO 15, PWM)** | Real-time alarm output |
| **Breadboard + wiring** | For prototyping |

The MAX4466 output is connected to **ADC0 (GP26)**.

---

## **Software Architecture**

### **FreeRTOS Tasks**
| Task | Priority | Description |
|------|----------|-------------|
| **AudioStream** | 5 | Streams audio to PC (highest priority) |
| **Alarm (down)** | 4 | Buzzer + rapid LED blinking |
| **Blink (up)** | 3 | LED blinking at medium frequency |
| **Off** | 2 | Turns LED off |
| **On** | 1 | Turns LED on |
| **SerialReceiver** | 1 | Parses commands from PC |

### **Synchronization**
- **Mutexes** protect LED and buzzer  
- **Queues** transport filled audio buffers  
- **Task notifications** trigger command execution instantly  

---

## **Commands & Real-Time Behavior**

| Voice Command | PC Output | Pico Behavior |
|---------------|-----------|----------------|
| **on** | `CMD:on` | LED on |
| **off** | `CMD:off` | LED off |
| **up** | `CMD:blink` | LED blinking |
| **down** | `CMD:alarm` | **Buzzer + fast LED blink (highest priority)** |

The **alarm** task preempts all others, guaranteeing responsiveness for critical events.

---

## **Packet Format**

Audio packets contain **256 samples**.  
Each 16-bit transmitted word encodes: [ 4-bit marker | 12-bit ADC sample ]

Markers:
- `0xF000` — Start of frame  
- `0xE000` — End of frame  
- `0x0000` — Normal sample  

This ensures robust framing even with possible serial jitter.

---

## **Keyword Spotting Model**

The Python model uses:
- A **CNN classifier** with convolution, pooling, and fully connected layers
- Four output classes: *on, off, up, down*
- A confidence threshold of **0.85** to avoid false positives

The model runs in real time on the host PC.

---

## **How to Build & Run**

### **1. Firmware (C + FreeRTOS)**

**Requirements:**
- Pico SDK (```git clone https://github.com/RaspberryPi/pico-sdk --recurse-submodules```)
- FreeRTOS Kernel (```git clone -b smp https://github.com/FreeRTOS/FreeRTOS-Kernel --recurse-submodules```) 
- CMake  
- FreeRTOSConfig 

```bash
mkdir build
cd build
cmake ..
make
```
Flash the generated .uf2 to the Pico.

### **2. Keyword Spotter (Python)**

### **Dependencies**
- numpy  
- pytorch   
- pyserial  

### **Run**
```bash
python keyword_spotter.py
```
The script:
- Reads audio from USB
- Runs inference
- Sends commands (CMD:<cmd>) back to the Pico


