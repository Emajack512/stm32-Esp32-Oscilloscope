/*
  ESP32 oscilloscope UI
  - Receives: 0xAA 0x55 + 1024 samples (uint16 LE) over UART @ 230400
  - Sampling assumed: 20 kHz (50 us between samples on STM32)
  - UI: ILI9341 + grid, Vpp/Vp, frequency, timebase & gain via encoders
*/


#include <HardwareSerial.h>
#include <Adafruit_ILI9341.h>
#include <SPI.h>

#define TFT_CS   5
#define TFT_DC   2
#define TFT_RST  4
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);

//Base de teimpo dependiente del encoder
#define ENCODER_CLK 32
#define ENCODER_DT  27

volatile int timebaseIndex = 1;
const uint16_t timebaseOptions[] = {1, 10, 20, 50, 100, 200, 500, 1000};  // en µs



#define RXD2 16
#define TXD2 17
#define NUM_MUESTRAS 1024

HardwareSerial STM32Serial(2);
uint16_t buffer[NUM_MUESTRAS];
uint16_t bufferFiltrado[NUM_MUESTRAS];


// === CALIBRACIÓN MANUAL ===
float CIRCUIT_OFFSET = 1.658f;       // En voltios
float ATTENUATION_FACTOR = 19.0f;   // Por ejemplo, 19 para 1/19

int lastEncoderState = HIGH;

//PARTE VERTICAL

#define ENCODER_VERT_CLK   14  // CLK vertical
#define ENCODER_VERT_DT    12  // DT vertical

volatile int gainIndex = 3;
const float verticalGainOptions[] = {0.1, 0.2, 0.5, 1.0, 2.0, 5.0};  // V/div
int lastVertState = HIGH;


void setup() {
  Serial.begin(115200);
  STM32Serial.begin(230400, SERIAL_8N1, RXD2, TXD2);  // UART2
 
  //Inicializacion de la pantalla
  tft.begin();
  tft.setRotation(3);  // Horizontal
  tft.fillScreen(ILI9341_BLACK);

  //configuracion pines de base de tiempo

  pinMode(ENCODER_CLK, INPUT_PULLUP);
  pinMode(ENCODER_DT, INPUT_PULLUP);
   lastEncoderState = digitalRead(ENCODER_CLK);

  //Inicializacion de pines

  pinMode(ENCODER_VERT_CLK, INPUT_PULLUP);
pinMode(ENCODER_VERT_DT, INPUT_PULLUP);
lastVertState = digitalRead(ENCODER_VERT_CLK);



  Serial.println("ESP32 listo. Escribí 'buffer' para ver los datos.");
  Serial.printf("Offset: %.3f V | Atenuación: %.2f\n", CIRCUIT_OFFSET, ATTENUATION_FACTOR);
}

void drawGrid() {
  const int ancho = 320;
  const int alto = 240;
  const int divHoriz = 10;
  const int divVert = 8;
  const int pxDivH = ancho / divHoriz; // 32 px
  const int pxDivV = alto / divVert;   // 30 px

  // Líneas verticales
  for (int x = 0; x <= ancho; x += pxDivH) {
    uint16_t color = (x % (pxDivH * 5) == 0) ? ILI9341_DARKGREY : ILI9341_NAVY;
    tft.drawLine(x, 0, x, alto, color);
  }

  // Líneas horizontales
  for (int y = 0; y <= alto; y += pxDivV) {
    uint16_t color = (y % (pxDivV * 4) == 0) ? ILI9341_DARKGREY : ILI9341_NAVY;
    tft.drawLine(0, y, ancho, y, color);
  }
}

void aplicarFiltroMediaMovil() {
  const int ventana = 5;  // Tamaño de la ventana
  const int margen = ventana / 2;

  for (int i = 0; i < NUM_MUESTRAS; i++) {
    int suma = 0;
    int conteo = 0;

    for (int j = -margen; j <= margen; j++) {
      int idx = i + j;
      if (idx >= 0 && idx < NUM_MUESTRAS) {
        suma += buffer[idx];
        conteo++;
      }
    }
    bufferFiltrado[i] = suma / conteo;
  }
}


void graficarBuffer() {
  tft.fillScreen(ILI9341_BLACK);
  drawGrid();  // Dibujar grilla de fondo

  const int ancho = 320;
  const int alto = 240;
  const int divisionesVerticales = 8;
  const int divisionesHorizontales = 10;

  float voltsPorDiv = verticalGainOptions[gainIndex];
  float voltajeMax = voltsPorDiv * divisionesVerticales / 2.0;
  float voltajeMin = -voltajeMax;

  int timebase_us_div = timebaseOptions[timebaseIndex];
  int tiempoTotal_us = timebase_us_div * divisionesHorizontales;
  const float tiempoEntreMuestras_us = 50.0f;  // 20 kHz
  int muestrasNecesarias = tiempoTotal_us / tiempoEntreMuestras_us;

  if (muestrasNecesarias < 2) muestrasNecesarias = 2;
  if (muestrasNecesarias > NUM_MUESTRAS) muestrasNecesarias = NUM_MUESTRAS;

  int centro = NUM_MUESTRAS / 2;
  int inicio = centro - muestrasNecesarias / 2;
  if (inicio < 1) inicio = 1;
  if (inicio + muestrasNecesarias >= NUM_MUESTRAS)
    inicio = NUM_MUESTRAS - muestrasNecesarias - 1;

  float escalaX = (float)ancho / (float)muestrasNecesarias;

  // Línea de 0 V
  int y0 = map(0, voltajeMin * 1000, voltajeMax * 1000, alto, 0);
  y0 = constrain(y0, 0, alto - 1);
  tft.drawLine(0, y0, ancho, y0, ILI9341_RED);

  // === Cálculo de parámetros ===
  float vmax = -1000.0f, vmin = 1000.0f, vsum = 0;
  for (int i = inicio; i < inicio + muestrasNecesarias; i++) {
    float v = ((bufferFiltrado[i] / 4095.0f) * 3.3f - CIRCUIT_OFFSET) * ATTENUATION_FACTOR;
    vsum += v;
    if (v > vmax) vmax = v;
    if (v < vmin) vmin = v;
  }

  float vpp = vmax - vmin;
  float vp = vmax;
  float vavg = vsum / muestrasNecesarias;
  float frecuencia = calcularFrecuencia();

  // === Graficar señal ===
  for (int i = inicio + 1; i < inicio + muestrasNecesarias; i++) {
    float v1 = ((bufferFiltrado[i - 1] / 4095.0f) * 3.3f - CIRCUIT_OFFSET) * ATTENUATION_FACTOR;
    float v2 = ((bufferFiltrado[i]     / 4095.0f) * 3.3f - CIRCUIT_OFFSET) * ATTENUATION_FACTOR;

    int x1 = (i - 1 - inicio) * escalaX;
    int x2 = (i - inicio) * escalaX;

    int y1 = map(v1 * 1000, voltajeMin * 1000, voltajeMax * 1000, alto, 0);
    int y2 = map(v2 * 1000, voltajeMin * 1000, voltajeMax * 1000, alto, 0);

    y1 = constrain(y1, 0, alto - 1);
    y2 = constrain(y2, 0, alto - 1);

    if (x1 < ancho && x2 < ancho)
      tft.drawLine(x1, y1, x2, y2, ILI9341_CYAN);
  }
float frecuenciareal;
frecuenciareal=frecuencia;
  // === TEXTO SUPERIOR: Frecuencia, Vp, Vpp, Vavg ===
  tft.setTextColor(ILI9341_GREEN, ILI9341_BLACK);
  tft.setTextSize(1);
  tft.setCursor(5, 5);
  tft.printf("Freq: %.1f Hz", frecuenciareal);

  tft.setCursor(120, 5);
  tft.printf("Vp: %.2f V", vp);

  tft.setCursor(210, 5);
  tft.printf("Vpp: %.2f V", vpp);

  // === TEXTO INFERIOR: Base, Ganancia, Ventana ===
  tft.setTextColor(ILI9341_YELLOW, ILI9341_BLACK);
  tft.setTextSize(1);
  tft.setCursor(5, alto - 18);
  tft.printf("Base: %d us/div", timebase_us_div);

  tft.setCursor(120, alto - 18);
  tft.printf("Gain: %.2f V/div", voltsPorDiv);

  tft.setCursor(230, alto - 18);
  tft.printf("Ventana: %.1f ms", (float)tiempoTotal_us / 1000.0f);
}



void loop() {
  static enum {ESPERANDO_SYNC1, ESPERANDO_SYNC2, LEYENDO_BUFFER} estado = ESPERANDO_SYNC1;
  static int byteIndex = 0;
  static uint8_t rawBuffer[NUM_MUESTRAS * 2];

  // === LECTURA UART DESDE STM32 ===
  while (STM32Serial.available()) {
    uint8_t b = STM32Serial.read();

    switch (estado) {
      case ESPERANDO_SYNC1:
        if (b == 0xAA) estado = ESPERANDO_SYNC2;
        break;

      case ESPERANDO_SYNC2:
        if (b == 0x55) {
          byteIndex = 0;
          estado = LEYENDO_BUFFER;
        } else {
          estado = ESPERANDO_SYNC1;
        }
        break;

      case LEYENDO_BUFFER:
        rawBuffer[byteIndex++] = b;

        if (byteIndex == NUM_MUESTRAS * 2) {
          for (int i = 0; i < NUM_MUESTRAS; i++) {
            buffer[i] = rawBuffer[i * 2] | (rawBuffer[i * 2 + 1] << 8);
          }

          aplicarFiltroMediaMovil();  // Filtra buffer[] → bufferFiltrado[]
          graficarBuffer();           // Grafica bufferFiltrado[]

          byteIndex = 0;
          estado = ESPERANDO_SYNC1;
        }
        break;
    }
  }

  // === ENCODER HORIZONTAL ===
  int currentState = digitalRead(ENCODER_CLK);
  if (currentState != lastEncoderState) {
    if (digitalRead(ENCODER_DT) != currentState) {
      timebaseIndex++;  // Sentido horario
    } else {
      timebaseIndex--;  // Sentido antihorario
    }

    if (timebaseIndex < 0) timebaseIndex = 0;
    if (timebaseIndex >= sizeof(timebaseOptions)/sizeof(timebaseOptions[0]))
      timebaseIndex = sizeof(timebaseOptions)/sizeof(timebaseOptions[0]) - 1;

    Serial.printf("Base de tiempo: %d us/div\n", timebaseOptions[timebaseIndex]);
  }
  lastEncoderState = currentState;

  // === ENCODER VERTICAL ===
  int currentVert = digitalRead(ENCODER_VERT_CLK);
  if (currentVert != lastVertState) {
    if (digitalRead(ENCODER_VERT_DT) != currentVert) {
      gainIndex++;
    } else {
      gainIndex--;
    }

    if (gainIndex < 0) gainIndex = 0;
    if (gainIndex >= sizeof(verticalGainOptions)/sizeof(verticalGainOptions[0]))
      gainIndex = sizeof(verticalGainOptions)/sizeof(verticalGainOptions[0]) - 1;

    Serial.printf("Ganancia vertical: %.2f V/div\n", verticalGainOptions[gainIndex]);
  }
  lastVertState = currentVert;

  // === COMANDO 'buffer' DESDE MONITOR SERIAL ===
  static String input = "";
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      input.trim();
      if (input.equalsIgnoreCase("buffer")) {
        Serial.println("\n===== DATOS DEL BUFFER =====");

        float vmin = 1000.0f, vmax = -1000.0f, vsum = 0;
        float resolution = (3.3f / 4096.0f) * ATTENUATION_FACTOR;

        for (int i = 0; i < NUM_MUESTRAS; i++) {
          float adcVoltage = (buffer[i] / 4095.0f) * 3.3f;
          float realVoltage = (adcVoltage - CIRCUIT_OFFSET) * ATTENUATION_FACTOR;
          vsum += realVoltage;
          if (realVoltage < vmin) vmin = realVoltage;
          if (realVoltage > vmax) vmax = realVoltage;

          if (i < 4096) {
            Serial.printf("[%04d] ADC: %4d -> %.3f V\n", i, buffer[i], realVoltage);
          }
        }

        float vavg = vsum / NUM_MUESTRAS;
        Serial.println("------------------------------------");
        Serial.printf("Offset aplicado:       %.3f V\n", CIRCUIT_OFFSET);
        Serial.printf("Factor de atenuación:  %.2f\n", ATTENUATION_FACTOR);
        Serial.printf("Resolución efectiva:   %.3f mV/paso\n", resolution * 1000.0f);
        Serial.printf("Voltaje promedio:      %.3f V\n", vavg);
        Serial.printf("Voltaje mínimo:        %.3f V\n", vmin);
        Serial.printf("Voltaje máximo:        %.3f V\n", vmax);
        Serial.printf("Amplitud pico a pico:  %.3f V\n", vmax - vmin);
        Serial.println("=====================================\n");
      }
      input = "";
    } else {
      input += c;
    }
  }
}
float calcularFrecuencia() {
  const float Tm_us = 50.0f;  // Tiempo entre muestras (20 kHz)
  const float umbral = CIRCUIT_OFFSET / 3.3f * 4095.0f;

  int primero = -1;
  int segundo = -1;

  // Buscar primer flanco ascendente
  for (int i = 1; i < NUM_MUESTRAS; i++) {
    if (bufferFiltrado[i - 1] < umbral && bufferFiltrado[i] >= umbral) {
      primero = i;
      break;
    }
  }

  // Buscar segundo flanco ascendente **después de un mínimo de separación**
  if (primero != -1) {
    const int minSeparacion = 20;  // evita flancos consecutivos (ajustar según forma de onda)
    for (int i = primero + minSeparacion; i < NUM_MUESTRAS; i++) {
      if (bufferFiltrado[i - 1] < umbral && bufferFiltrado[i] >= umbral) {
        segundo = i;
        break;
      }
    }
  }

  if (primero != -1 && segundo != -1) {
    int periodo_muestras = segundo - primero;
    float periodo_us = periodo_muestras * Tm_us;
    return 1e6f / periodo_us;  // Frecuencia en Hz
  } else {
    return 0.0f;  // No se pudo detectar frecuencia
  }
}

void calcularParametros(float &vp, float &vpp, float &vavg) {
  float vmax = -1000.0f;
  float vmin = 1000.0f;
  float suma = 0;

  for (int i = 0; i < NUM_MUESTRAS; i++) {
    float v = ((bufferFiltrado[i] / 4095.0f) * 3.3f - CIRCUIT_OFFSET) * ATTENUATION_FACTOR;
    suma += v;
    if (v > vmax) vmax = v;
    if (v < vmin) vmin = v;
  }

  vp   = vmax;
  vpp  = vmax - vmin;
  vavg = suma / NUM_MUESTRAS;
}
