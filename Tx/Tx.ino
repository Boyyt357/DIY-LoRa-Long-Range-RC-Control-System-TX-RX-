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
#define LORA_SF 7                 // Spreading Factor (Range 6-12). Higher SF = longer range, slower data rate.
#define LORA_TX_POWER 18          // Transmit Power (Range 2-20 dBm). Max is 20, but 18 is common for high power.
#define LORA_BANDWIDTH 125E3      // LoRa Bandwidth (Fixed 125 KHz as requested)
#define PACKET_SEND_INTERVAL 20   // Target TX interval in milliseconds (~50Hz)

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
// Uses uint16_t for 0-4095 ADC values for efficient packing.
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

// Timing variables for loop control and debugging
unsigned long lastSendTime = 0;
unsigned long startTime = 0;
unsigned long lastDebugTime = 0; // New timer for limiting debug output
int packetCount = 0;

// Apply deadzone function
uint16_t applyDeadzone(uint16_t raw, uint16_t center) {
  int16_t diff = (int16_t)raw - (int16_t)center;
  if (abs(diff) < DEADZONE) return center;
  return raw;
}

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("\n--- LoRa TX Controller Initializing ---");

  // Joystick inputs
  pinMode(JOY1_X, INPUT);
  pinMode(JOY1_Y, INPUT);
  pinMode(JOY2_X, INPUT);
  pinMode(JOY2_Y, INPUT);

  // Switch inputs (3.3V center wiring)
  pinMode(SW1_UP_PIN, INPUT);
  pinMode(SW1_DOWN_PIN, INPUT);
  pinMode(SW2_UP_PIN, INPUT);
  pinMode(SW2_DOWN_PIN, INPUT);

  // Auto calibration of joystick centers (takes the initial reading as center)
  j1x_center = analogRead(JOY1_X);
  j1y_center = analogRead(JOY1_Y);
  j2x_center = analogRead(JOY2_X);
  j2y_center = analogRead(JOY2_Y);
  Serial.printf("Calibration Centers: J1X=%u, J1Y=%u, J2X=%u, J2Y=%u\n", 
    j1x_center, j1y_center, j2x_center, j2y_center);
  
  // LoRa setup
  SPI.begin(SCK, MISO, MOSI, SS);
  LoRa.setPins(SS, RST, DIO0);

  if (!LoRa.begin(LORA_FREQUENCY)) {
    Serial.println("LoRa initialization failed. Check wiring!");
    while (1);
  }
  
  LoRa.setSpreadingFactor(LORA_SF);
  LoRa.setSignalBandwidth(LORA_BANDWIDTH);
  LoRa.setTxPower(LORA_TX_POWER);
  
  Serial.printf("LoRa TX Ready. Freq: %.1f MHz, SF: %d, TX Power: %d dBm\n", 
    (float)LORA_FREQUENCY/1E6, LORA_SF, LORA_TX_POWER);
  
  startTime = millis();
}

void loop() {
  unsigned long now = millis();
  
  // Throttle loop based on desired send rate
  if (now - lastSendTime > PACKET_SEND_INTERVAL) {
    lastSendTime = now;
    
    // 1. Read Inputs with Deadzone and Calibration
    sendData.joy1X = applyDeadzone(analogRead(JOY1_X), j1x_center);
    sendData.joy1Y = applyDeadzone(analogRead(JOY1_Y), j1y_center);
    sendData.joy2X = applyDeadzone(analogRead(JOY2_X), j2x_center);
    sendData.joy2Y = applyDeadzone(analogRead(JOY2_Y), j2y_center);
    
    // 2. Read Switches
    if (digitalRead(SW1_UP_PIN) == HIGH)        sendData.aux1 = 1;  
    else if (digitalRead(SW1_DOWN_PIN) == HIGH) sendData.aux1 = 2;
    else                                        sendData.aux1 = 0; 

    if (digitalRead(SW2_UP_PIN) == HIGH)        sendData.aux2 = 1;
    else if (digitalRead(SW2_DOWN_PIN) == HIGH) sendData.aux2 = 2;
    else                                        sendData.aux2 = 0;
    
    // 3. Send LoRa Packet (10 bytes)
    LoRa.beginPacket();
    LoRa.write((uint8_t*)&sendData, sizeof(sendData));
    int result = LoRa.endPacket(false); // Non-blocking send
    
    // 4. Debugging (Print status only every 100ms)
    if (now - lastDebugTime > 100) {
      lastDebugTime = now;
      if (result) {
        packetCount++;
        Serial.printf("[%lu ms] TX Success. J1X:%u, J1Y:%u, J2X:%u, J2Y:%u | Aux1:%d, Aux2:%d\n",
          now - startTime, sendData.joy1X, sendData.joy1Y, sendData.joy2X, sendData.joy2Y, 
          sendData.aux1, sendData.aux2);
      } else {
        Serial.printf("[%lu ms] TX Failed (LoRa busy or error).\n", now - startTime);
      }
    }
  }
}
