# Sine Wave Generator with Adjustable Amplitude and Frequency

### 🎵 Welcome to the world of DIY waveforms! 🎵

This project is a Sine Wave Generator built around the STM32 Nucleo board (F446RE) that lets you:

  * Vary the amplitude of the sine wave using two buttons.
  * Adjust the frequency using an encoder.
    
Whether you're into audio experiments, signal generation, or simply exploring the power of embedded systems, this project is for you!


## 🔧 Hardware Used
1. Microcontroller: Nucleo F446RE
     * DAC Output Pin: PA4 (STM32 pin) / A2 (Nucleo board pin).
2. Encoder: Operates in Timer 3's encoder mode.
3. Two Buttons: For increasing and decreasing the amplitude.
4. LED: Visual feedback for button presses.
