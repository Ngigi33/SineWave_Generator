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

###3️⃣ Frequency Control
*Encoder: Adjusts the sine wave frequency by changing the time delay between DAC updates.
