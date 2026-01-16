# Bill of Materials (BOM) — v1

> Values are based on the current schematic screenshots. Footprints/package sizes depend on the PCB library used.

## Core modules
- 1× STM32 “Blue Pill” (STM32F103C8T6)
- 1× ESP32 Dev board (30-pin)
- 1× ILI9341 TFT (SPI)

## ICs
- 1× MCP23017 (I2C 16-bit I/O expander)

## Analog front-end (per channel)
**CH1**
- R2: 190k
- R3: 10k
- R1: 10k
- R4: 10k
- R5: 10k
- C1: 1nF

**CH2**
- R7: 190k
- R8: 10k
- R6: 10k
- R9: 10k
- R10: 10k
- C2: 1nF

## Connectors / IO
- 2× CH input connectors (CH1, CH2)
- 1× Square-wave output connector (calibration)
- 1× TFT connector header
- 1× Encoders connector header
- 2× Push-buttons connector headers
- 1× Potentiometers connector header
- 1× 3.3V power terminal block

## Mechanical (optional)
- Enclosure (3D printed)
- Buttons, encoders, potentiometers, BNC connectors
