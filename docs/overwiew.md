# Oscilloscope — Quick Summary

## Hardware
- STM32F103C8T6: ADC acquisition (12-bit)
- ESP32: UART reception + UI rendering (TFT ILI9341)

## Key parameters
- Sampling rate: 20 kHz
- Buffer: 1024 samples
- UART: 230400 baud
- Total update time (acquire+process+send): ~140 ms

## Input conditioning
- Divider 1/19 + 1.65 V offset
- Estimated input range: ~±30 V (tested up to ±10 V)

## Measured performance
- Bandwidth: ~400–600 Hz
- Repeatability: 3.23%
