# STM32 + ESP32 Oscilloscope (2-Channel)

Dual-channel oscilloscope prototype where the **STM32 performs ADC data acquisition** and the **ESP32 handles processing + UI** on a TFT display.

## My role
Technical lead: electronics design, firmware architecture, and system integration.

## System architecture
- **STM32**: timer-based sampling (ADC) → UART streaming
- **ESP32**: UART reception → processing → waveform rendering + measurements
- **Display**: TFT (ILI9341)

## Key features
- Real-time waveform display
- Timebase & vertical scaling controls
- Grid overlay
- Measurements (e.g., Vpp, Vavg)
- Raw acquisition on STM32, full processing on ESP32

## Evidence
- Photos/videos in `/media`
- Reports/docs in `/docs`

## Repo structure (recommended)
- `/firmware-stm32`
- `/firmware-esp32`
- `/hardware` (schematics/PCB)
- `/docs`
- `/media`
