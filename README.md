# Sine Wave Generator with Adjustable Amplitude and Frequency

### 🎵 Welcome to the world of DIY waveforms! 🎵

This project is a Sine Wave Generator built around the STM32 Nucleo board (F446RE) that lets you:

  * Vary the amplitude of the sine wave using two buttons.
  * Adjust the frequency using an encoder.
    
Whether you're into audio experiments, signal generation, or simply exploring the power of embedded systems, this project is for you!


## 🔧 Hardware Used
1. Microcontroller: Nucleo F446RE:
   
    * DAC Output Pin: PA4 (STM32 pin) / A2 (Nucleo board pin).
   
3. Encoder: Operates in Timer 3's encoder mode.
4. Two Buttons: For increasing and decreasing the amplitude.
5. LED: Visual feedback for button presses.


## 💻 How It Works
### 1️⃣ DAC Output
The DAC output generates the sine wave, which is sampled over 100 discrete points to ensure smooth waveform generation. The PA4 pin outputs the analog signal.

### 2️⃣ Amplitude Control
* Button 1: Increases amplitude.
* Button 2: Decreases amplitude.
Amplitude values are capped between 0 and 1 for safe operation.

### 3️⃣ Frequency Control
* Encoder: Adjusts the sine wave frequency by changing the time delay between DAC updates.

## 📜 Features
* Smooth Sine Wave Generation: Uses a 12-bit DAC resolution for clean output.
* Dynamic Amplitude Adjustment: Real-time control with gain values.
* Real-Time Frequency Tuning: Encoder alters delay intervals.
* Error Handling: Prevents out-of-bound values for both amplitude and frequency.
  

## ⚙️ Software Overview
The project is written in C using the STM32 HAL library. Below are some core functions and concepts:

### 🌀 Sine Wave Table
```C
uint16_t sinewave[100] = { 0, 4, 16, 36, 64, 100, ... };
```
Precomputed values for the sine wave are stored in an array for efficient generation.

### ⏱️ Timer-Based Frequency Control
The encoder leverages Timer 3 in encoder mode to track rotational input, while Timer 2 provides precise microsecond delays for waveform timing.

### 🔄 Main Loop
```C
for (uint8_t i = 0; i < 100; i++) {
    int var = (int)(((gain)*(float)sinewave[i]));
    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, var);
    microDelay(Delayer);
}
```

