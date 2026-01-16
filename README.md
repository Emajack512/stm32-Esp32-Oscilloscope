# STM32 + ESP32 Oscilloscope (2-Channel Prototype)

Dual-channel oscilloscope prototype where the **STM32 handles ADC data acquisition** and the **ESP32 renders the UI** on an ILI9341 TFT display.  
This repository contains firmware, hardware captures (schematic/PCB), documentation, and media evidence.

## Quick links
- 📄 Summary: `docs/overview.md`
- 🧾 Report (PDF): `docs/`
- 🖼️ Photos: `media/`
- 🧠 STM32 firmware: `firmware-stm32/`
- 🧠 ESP32 firmware: `firmware-esp32/`
- 🔧 Hardware: `hardware/` (schematic + PCB layout + enclosure renders)

## My role
Technical lead: electronics design, firmware architecture, and system integration.

## System architecture
- **STM32 (Blue Pill / STM32F103)**: ADC sampling → UART streaming
- **ESP32**: UART reception → processing → waveform rendering + measurements
- **Display**: TFT ILI9341 (SPI)
- **Controls**: encoders + buttons (timebase, vertical gain, etc.)

## Communication protocol (current main version)
- **UART baudrate:** 230400
- **Frame:** `0xAA 0x55` + `1024 samples × uint16 (little-endian)`  
  (No length field, no footer)

> Note: earlier iterations used different framing. This repo documents the working “main” variant.

## Assumptions used by UI
- **Sampling interval:** 50 µs (20 kHz) for frequency/timebase calculations
- **Calibration:** applied on ESP32 UI (offset + attenuation factor).  
  Values depend on the analog front-end revision; see `docs/overview.md`.

## Key features
- Real-time waveform display with grid overlay
- Timebase & vertical scaling controls (encoders)
- Measurements: **Vp**, **Vpp**, **Vavg**, **Frequency**
- Modular repo: firmware + docs + hardware + media

## How to run
### STM32 (Arduino core)
- Open the STM32 sketch inside `firmware-stm32/`
- Board: **Generic STM32F103C8 / Blue Pill** (Arduino STM32 core)
- Wire STM32 TX (PA9) → ESP32 RX2 (GPIO16), and GND ↔ GND
- Flash and verify UART output

### ESP32 (Arduino core)
- Open the ESP32 sketch inside `firmware-esp32/`
- Board: **ESP32 Dev Module**
- UART2 RX2: GPIO16 (listening to STM32)
- Flash and verify the waveform appears on the ILI9341 display

## Evidence
- Photos/videos: `media/`
- Report + summary: `docs/`
- Schematic/PCB/enclosure: `hardware/`

## Repository structure
- `firmware-stm32/` — acquisition firmware (STM32)
- `firmware-esp32/` — UI + processing firmware (ESP32)
- `hardware/` — schematic/PCB layout, Proteus project files, enclosure renders
- `docs/` — overview + report
- `media/` — photos and demos

## License
MIT (see `LICENSE`).
