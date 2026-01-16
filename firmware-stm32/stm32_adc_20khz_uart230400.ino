/*
  STM32 oscilloscope transmitter (Arduino core)
  - Sampling: 20 kHz (50 us)
  - Samples: 1024
  - UART: 230400 (Serial1)
  - Frame: 0xAA 0x55 + 2048 bytes (LSB first)
  - Pre-filter: 3-sample moving average on STM32
*/

#include <Arduino.h>

#define NUM_MUESTRAS 1024

// Calibration constants are applied on ESP32 UI (kept here for documentation).
// ===== CONFIGURACIÓN DEL CIRCUITO ACONDICIONADOR =====
#define ADC_BITS         12
#define ADC_MAX_VALUE    4095.0f
#define ADC_REF_VOLTAGE  3.3f

const float CIRCUIT_OFFSET = 1.65f;     // Offset agregado por el circuito (V)
const float ATTENUATION_FACTOR = 19.0f; // Factor de atenuación 1/19

uint16_t buffer[NUM_MUESTRAS];

// === FUNCIÓN PARA SUAVIZAR EL BUFFER (MEDIA MÓVIL 3) ===
void suavizarBuffer() {
  uint16_t temp[NUM_MUESTRAS];
  for (int i = 0; i < NUM_MUESTRAS; i++) {
    uint32_t suma = 0;
    uint8_t cuenta = 0;
    // Toma la muestra anterior, actual y siguiente (ventana de 3)
    for (int j = -1; j <= 1; j++) {
      int idx = i + j;
      if (idx >= 0 && idx < NUM_MUESTRAS) {
        suma += buffer[idx];
        cuenta++;
      }
    }
    temp[i] = suma / cuenta;
  }
  // Copiar el resultado suavizado de vuelta al buffer original
  for (int i = 0; i < NUM_MUESTRAS; i++) {
    buffer[i] = temp[i];
  }
}

// ====== TU CÓDIGO DE SETUP Y DEMÁS FUNCIONES IGUAL =====

void setupFastADC() {
  pinMode(PA0, INPUT_ANALOG);
  analogReadResolution(12);

  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
  ADC1->SMPR2 &= ~(ADC_SMPR2_SMP0_2 | ADC_SMPR2_SMP0_1 | ADC_SMPR2_SMP0_0);
  ADC1->CR2 |= ADC_CR2_ADON;
  delay(1);
  ADC1->CR2 |= ADC_CR2_CAL;
  while (ADC1->CR2 & ADC_CR2_CAL);
  delay(10);
}

inline uint16_t fastAnalogRead() {
  return analogRead(PA0);
}

// ... (el resto de funciones de estadísticas y helpers igual)

void setup() {
  Serial.begin(115200);
  Serial1.begin(230400);

  setupFastADC();

  Serial.println("=== STM32 OSCILOSCOPIO - VERSIÓN CORREGIDA ===");
  // ... (tus prints de setup)
}

void loop() {
  uint32_t startTime = micros();

  // Llenar el buffer con datos del ADC con timing controlado
  for (int i = 0; i < NUM_MUESTRAS; i++) {
    buffer[i] = fastAnalogRead();
    delayMicroseconds(50); // 20kHz de muestreo (50us entre muestras)
  }

  // ======= SUAVIZADO ANTES DE ENVIAR =======
  suavizarBuffer();  // <<<--- AGREGA ESTA LÍNEA

  uint32_t samplingTime = micros() - startTime;
  float realSampleRate = 1000000.0f / (samplingTime / (float)NUM_MUESTRAS);

  // ===== ENVIAR HEADER CON INFORMACIÓN DE CALIBRACIÓN =====
  Serial1.write(0xAA); // Byte de sincronización 1
  Serial1.write(0x55); // Byte de sincronización 2


  // ENVIAR BUFFER AL ESP32
  uint8_t* bufferBytes = (uint8_t*)buffer;
  Serial1.write(bufferBytes, NUM_MUESTRAS * 2);

  // ... (tu código de debug, estadísticas, etc.)
  delay(20);
}

// ... (funciones de min/max adc, stats, etc.)
