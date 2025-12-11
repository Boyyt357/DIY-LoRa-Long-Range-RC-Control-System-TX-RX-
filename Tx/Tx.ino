#include <SPI.h>
#include <LoRa.h>

// -----------------------------------------------------------------------------
// LORA HARDWARE CONFIG (AI Thinker Ra-02)
// -----------------------------------------------------------------------------
#define SCK 5
#define MISO 19
#define MOSI 27
#define SS 18
#define RST 14
#define DIO0 26

// -----------------------------------------------------------------------------
// USER CONFIGURABLE LORA SETTINGS
// -----------------------------------------------------------------------------
#define LORA_FREQUENCY 433E6       // LoRa Frequency (e.g., 433E6, 868E6, 915E6)
#define LORA_SF 7                 // Spreading Factor (Range 6-12)
#define LORA_TX_POWER 18          // Transmit Power (Range 2-20 dBm)
#define LORA_BANDWIDTH 125E3      // LoRa Bandwidth (Fixed 125 KHz)
#define PACKET_SEND_INTERVAL 20   // TX interval in milliseconds (~50Hz)

// -----------------------------------------------------------------------------
// JOYSTICK AND SWITCH CONFIG
// -----------------------------------------------------------------------------
#define JOY1_X 34                 // Roll (ADC)
#define JOY1_Y 35                 // Pitch (ADC)
#define JOY2_X 32                 // Yaw (ADC)
#define JOY2_Y 33                 // Throttle (ADC)
#define SW1_UP_PIN 25
#define SW1_DOWN_PIN 13
#define SW2_UP_PIN 15
#define SW2_DOWN_PIN 17
#define DEADZONE 34               // Deadzone for analog stick readings

// -----------------------------------------------------------------------------
// DATA STRUCT (10 bytes total)
// -----------------------------------------------------------------------------
typedef struct {
  uint16_t joy1X;
  uint16_t joy1Y;
  uint16_t joy2X;
  uint16_t joy2Y;
  uint8_t aux1;   // 0=Mid, 1=Up, 2=Down
  uint8_t aux2;   // 0=Mid, 1=Up, 2=Down
} LoraData;
LoraData sendData;

// Calibration storage
uint16_t j1x_center, j1y_center, j2x_center, j2y_center;
uint16_t j1x_min, j1x_max, j1y_min, j1y_max;
uint16_t j2x_min, j2x_max, j2y_min, j2y_max;

// Timing
unsigned long lastSendTime = 0;
unsigned long lastDebugTime = 0;
unsigned long startTime = 0;
int packetCount = 0;

// Simple center calibration
void calibrateSticks() {
  Serial.println("Calibrating sticks...");
  
  j1x_center = analogRead(JOY1_X);
  j1y_center = analogRead(JOY1_Y);
  j2x_center = analogRead(JOY2_X);
  j2y_center = analogRead(JOY2_Y);
  
  // Initialize min/max ranges
  j1x_min = j1x_center; j1x_max = j1x_center;
  j1y_min = j1y_center; j1y_max = j1y_center;
  j2x_min = j2x_center; j2x_max = j2x_center;
  j2y_min = j2y_center; j2y_max = j2y_center;
  
  Serial.println("Calibration complete!");
}

// Apply deadzone and center the value around 2048
inline uint16_t processCenteredStick(uint16_t raw, uint16_t center, uint16_t &min_val, uint16_t &max_val) {
  // Update min/max for adaptive range
  if (raw < min_val) min_val = raw;
  if (raw > max_val) max_val = raw;
  
  // Apply deadzone
  int16_t diff = (int16_t)raw - (int16_t)center;
  if (abs(diff) < DEADZONE) {
    return 2048; // Return exact center
  }
  
  // Map to 0-4095 range centered at 2048
  if (raw < center) {
    return map(raw, min_val, center, 0, 2048);
  } else {
    return map(raw, center, max_val, 2048, 4095);
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("\n--- LoRa TX Controller ---");

  // Input pins
  pinMode(JOY1_X, INPUT);
  pinMode(JOY1_Y, INPUT);
  pinMode(JOY2_X, INPUT);
  pinMode(JOY2_Y, INPUT);
  pinMode(SW1_UP_PIN, INPUT);
  pinMode(SW1_DOWN_PIN, INPUT);
  pinMode(SW2_UP_PIN, INPUT);
  pinMode(SW2_DOWN_PIN, INPUT);

  // Calibrate sticks
  calibrateSticks();
  
  // LoRa setup
  SPI.begin(SCK, MISO, MOSI, SS);
  LoRa.setPins(SS, RST, DIO0);

  if (!LoRa.begin(LORA_FREQUENCY)) {
    Serial.println("LoRa init failed!");
    while (1);
  }
  
  LoRa.setSpreadingFactor(LORA_SF);
  LoRa.setSignalBandwidth(LORA_BANDWIDTH);
  LoRa.setTxPower(LORA_TX_POWER);
  
  Serial.printf("LoRa TX Ready - Freq: %.1f MHz, SF: %d\n", 
    (float)LORA_FREQUENCY/1E6, LORA_SF);
  
  startTime = millis();
}

void loop() {
  unsigned long now = millis();
  
  // Send at fixed interval
  if (now - lastSendTime >= PACKET_SEND_INTERVAL) {
    lastSendTime = now;
    
    // Read and process joysticks
    sendData.joy1X = processCenteredStick(analogRead(JOY1_X), j1x_center, j1x_min, j1x_max);
    sendData.joy1Y = processCenteredStick(analogRead(JOY1_Y), j1y_center, j1y_min, j1y_max);
    sendData.joy2X = processCenteredStick(analogRead(JOY2_X), j2x_center, j2x_min, j2x_max);
    sendData.joy2Y = processCenteredStick(analogRead(JOY2_Y), j2y_center, j2y_min, j2y_max);
    
    // Read switches
    if (digitalRead(SW1_UP_PIN) == HIGH)        sendData.aux1 = 1;  
    else if (digitalRead(SW1_DOWN_PIN) == HIGH) sendData.aux1 = 2;
    else                                        sendData.aux1 = 0; 

    if (digitalRead(SW2_UP_PIN) == HIGH)        sendData.aux2 = 1;
    else if (digitalRead(SW2_DOWN_PIN) == HIGH) sendData.aux2 = 2;
    else                                        sendData.aux2 = 0;
    
    // Send LoRa packet (non-blocking)
    LoRa.beginPacket();
    LoRa.write((uint8_t*)&sendData, sizeof(sendData));
    LoRa.endPacket(false);
    
    packetCount++;
    
    // Debug output every 100ms
    if (now - lastDebugTime >= 100) {
      lastDebugTime = now;
      Serial.printf("[%lu ms] TX #%d | J1X:%u J1Y:%u J2X:%u J2Y:%u | Aux1:%d Aux2:%d\n",
        now - startTime, packetCount, sendData.joy1X, sendData.joy1Y, 
        sendData.joy2X, sendData.joy2Y, sendData.aux1, sendData.aux2);
    }
  }
}
