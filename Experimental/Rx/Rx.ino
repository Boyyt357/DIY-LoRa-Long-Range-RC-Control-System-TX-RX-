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
// USER CONFIGURABLE LORA SETTINGS (MUST MATCH TX)
// -----------------------------------------------------------------------------
#define LORA_FREQUENCY 433E6       // LoRa Frequency
#define LORA_SF 7                 // Spreading Factor 
#define LORA_BANDWIDTH 125E3      // LoRa Bandwidth (Fixed 125 KHz)

// -----------------------------------------------------------------------------
// JOYSTICK INVERSION CONFIG (1 = Normal, 2 = Inverted)
// -----------------------------------------------------------------------------
#define ROLL_DIRECTION 1          // 1 = Normal (1000-2000), 2 = Inverted (2000-1000)
#define PITCH_DIRECTION 2         // 1 = Normal (1000-2000), 2 = Inverted (2000-1000)
#define YAW_DIRECTION 1           // 1 = Normal (1000-2000), 2 = Inverted (2000-1000)
#define THROTTLE_DIRECTION 2      // 1 = Normal (1000-2000), 2 = Inverted (2000-1000)

// -----------------------------------------------------------------------------
// IBUS AND FAILSAFE CONFIG
// -----------------------------------------------------------------------------
#define IBUS_TX_PIN 17
#define IBUS_SERIAL Serial1
#define CHANNELS 14
#define FAILSAFE_TIMEOUT_MS 1000   // Failsafe timeout in milliseconds

// -----------------------------------------------------------------------------
// DATA STRUCT (MUST MATCH TX)
// -----------------------------------------------------------------------------
typedef struct {
  uint16_t joy1X;
  uint16_t joy1Y;
  uint16_t joy2X;
  uint16_t joy2Y;
  uint8_t aux1;
  uint8_t aux2;
} LoraData;

// Volatile variables for interrupt
volatile LoraData receivedData;
volatile bool newDataAvailable = false;
volatile unsigned long lastReceiveTime = 0;

// iBUS channel data
uint16_t channels[CHANNELS] = {
  1500,1500,1500,1000, // Roll, Pitch, Yaw, Throttle (Failsafe: throttle low)
  1000,1000,1000,1000, // AUX1–AUX4
  1500,1500,1500,1500,1500,1500
};

bool isFailsafeActive = true;
unsigned long startTime = 0;
unsigned long lastDebugTime = 0;
int packetCount = 0;
float controlHz = 0.0f;

// AUX translation
inline uint16_t auxValue(uint8_t pos) {
  return (pos == 1) ? 1300 : (pos == 2) ? 1700 : 1500;
}

// Map centered stick (2048=center) to iBUS (1500=center)
inline uint16_t mapCenteredStick(uint16_t value, uint8_t direction) {
  if (value > 4095) value = 4095;
  
  uint16_t result;
  if (value < 2048) {
    result = map(value, 0, 2048, 1000, 1500);
  } else if (value > 2048) {
    result = map(value, 2048, 4095, 1500, 2000);
  } else {
    result = 1500;
  }
  
  // Apply inversion if direction is 2
  return (direction == 2) ? (3000 - result) : result;
}

// Send iBUS frame
void sendIBUS(){
  uint8_t packet[32];
  packet[0] = 0x20; 
  packet[1] = 0x40;
  
  for(int i = 0; i < CHANNELS; i++){
    packet[2 + i*2] = channels[i] & 0xFF;
    packet[3 + i*2] = (channels[i] >> 8) & 0xFF;
  }
  
  uint16_t checksum = 0xFFFF;
  for(int i = 0; i < 30; i++) checksum -= packet[i];
  packet[30] = checksum & 0xFF;
  packet[31] = checksum >> 8;
  
  IBUS_SERIAL.write(packet, 32);
}

// LoRa receive callback (keep minimal)
void onReceive(int packetSize) {
  if (packetSize == sizeof(LoraData)) {
    LoRa.readBytes((uint8_t*)&receivedData, sizeof(receivedData));
    newDataAvailable = true;
    lastReceiveTime = millis();
  }
}

// Apply failsafe
void applyFailsafe() {
  if (!isFailsafeActive) {
    Serial.printf("[%lu ms] FAILSAFE ACTIVE - No signal for >%dms\n", 
      millis() - startTime, FAILSAFE_TIMEOUT_MS);
    channels[0] = 1500; // Roll center
    channels[1] = 1500; // Pitch center
    channels[2] = 1500; // Yaw center
    channels[3] = 1000; // Throttle minimum (CRITICAL)
    channels[4] = 1000; // AUX1 disarm
    channels[5] = 1500; // AUX2 mid
    isFailsafeActive = true;
  }
  sendIBUS();
}

// Process received data
void processNewData() {
  LoraData currentData;
  unsigned long now = millis();
  
  // Copy volatile data
  noInterrupts();
  memcpy(&currentData, (const void*)&receivedData, sizeof(LoraData)); 
  newDataAvailable = false;
  interrupts();
  
  isFailsafeActive = false;
  packetCount++;
  
  // Calculate Hz
  static unsigned long previousReceiveTime = 0;
  if (previousReceiveTime != 0) {
    unsigned long timeDelta = now - previousReceiveTime;
    if (timeDelta > 0) {
      controlHz = 1000.0f / timeDelta;
    }
  }
  previousReceiveTime = now;
  
  // Map joystick values to iBUS channels
  channels[0] = mapCenteredStick(currentData.joy1X, ROLL_DIRECTION);
  channels[1] = mapCenteredStick(currentData.joy1Y, PITCH_DIRECTION);
  channels[2] = mapCenteredStick(currentData.joy2X, YAW_DIRECTION);
  channels[3] = mapCenteredStick(currentData.joy2Y, THROTTLE_DIRECTION);
  channels[4] = auxValue(currentData.aux1);
  channels[5] = auxValue(currentData.aux2);
  
  // Send iBUS frame
  sendIBUS();
  
  // Debug output every 100ms
  if (now - lastDebugTime >= 100) {
    lastDebugTime = now;
    Serial.printf("[%lu ms] RX #%d | Hz: %.1f\n", now - startTime, packetCount, controlHz);
    Serial.printf("  Raw: J1X:%u J1Y:%u J2X:%u J2Y:%u | Aux1:%u Aux2:%u\n",
      currentData.joy1X, currentData.joy1Y, currentData.joy2X, currentData.joy2Y,
      currentData.aux1, currentData.aux2);
    Serial.printf("  iBUS: Roll:%u Pitch:%u Yaw:%u Thr:%u | AUX1:%u AUX2:%u\n",
      channels[0], channels[1], channels[2], channels[3], channels[4], channels[5]);
  }
}

void setup(){
  Serial.begin(115200);
  Serial.println("\n--- LoRa RX iBUS Controller ---");

  // iBUS serial
  IBUS_SERIAL.begin(115200, SERIAL_8N1, -1, IBUS_TX_PIN);

  // LoRa setup
  SPI.begin(SCK, MISO, MOSI, SS);
  LoRa.setPins(SS, RST, DIO0);

  if (!LoRa.begin(LORA_FREQUENCY)) {
    Serial.println("LoRa init failed!");
    while (1);
  }
  
  LoRa.setSpreadingFactor(LORA_SF);
  LoRa.setSignalBandwidth(LORA_BANDWIDTH);
  LoRa.onReceive(onReceive);
  LoRa.receive(); 
  
  Serial.printf("LoRa RX Ready - Freq: %.1f MHz, SF: %d\n", 
    (float)LORA_FREQUENCY/1E6, LORA_SF);
    
  startTime = millis();
  lastReceiveTime = startTime;
}

void loop(){
  // Process new data
  if (newDataAvailable) {
    processNewData();
  }
  
  // Failsafe check
  if (millis() - lastReceiveTime > FAILSAFE_TIMEOUT_MS) {
    applyFailsafe();
  }
  
  delay(1);
}