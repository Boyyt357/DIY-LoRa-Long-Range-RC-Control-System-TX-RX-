#include <SPI.h>
#include <LoRa.h>
#include <WiFi.h> // Keep for esp_now dependency functions, though WiFi is not used

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
// IBUS AND FAILSAFE CONFIG
// -----------------------------------------------------------------------------
#define IBUS_TX_PIN 17
#define IBUS_SERIAL Serial1
#define CHANNELS 14
#define FAILSAFE_TIMEOUT_MS 100   // Time (ms) without a packet before engaging failsafe

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

// Variables shared between the interrupt and the main loop MUST be volatile
volatile LoraData receivedData;
volatile bool newDataAvailable = false;
volatile unsigned long lastReceiveTime = 0;

// iBUS channel data storage
uint16_t channels[CHANNELS] = {
  1500,1500,1500,1500, // Roll, Pitch, Yaw, Throttle (Default/Failsafe)
  1000,1000,1000,1000, // AUX1–AUX4 (Default)
  1500,1500,1500,1500,1500,1500
};

// Timing variables for Failsafe and Hz calculation
unsigned long startTime = 0;
unsigned long lastHzUpdateTime = 0;
float controlHz = 0.0f;
bool isFailsafeActive = true;

// AUX translation: 0=1500, 1=1300, 2=1700
uint16_t auxValue(uint8_t pos) {
  switch(pos){
    case 1: return 1300;
    case 2: return 1700;
    default: return 1500;
  }
}

// ====== iBUS frame send ======
void sendIBUS(){
  uint8_t packet[32];
  packet[0] = 0x20; packet[1] = 0x40; // Length and command (32 bytes / iBUS)
  for(int i=0;i<CHANNELS;i++){
    packet[2+i*2]   = channels[i] & 0xFF;
    packet[2+i*2+1] = (channels[i] >> 8) & 0xFF;
  }
  uint16_t checksum = 0xFFFF;
  for(int i=0;i<30;i++) checksum -= packet[i];
  packet[30] = checksum & 0xFF;
  packet[31] = checksum >> 8;
  IBUS_SERIAL.write(packet,32);
}

// ====== LoRa Receive Callback (minimal code to prevent crash) ======
void onReceive(int packetSize) {
  // CRITICAL: Keep this function brief. It only reads data and sets a flag.
  if (packetSize == sizeof(LoraData)) {
    // Read raw bytes directly into the volatile struct
    LoRa.readBytes((uint8_t*)&receivedData, sizeof(receivedData));
    
    // Set flag and timestamp for processing in main loop
    newDataAvailable = true;
    lastReceiveTime = millis();
  }
}

// ====== Failsafe Handler ======
void applyFailsafe() {
  if (!isFailsafeActive) {
    // Only print the failsafe message once when it is triggered
    Serial.printf("[%lu ms] FAILSAFE TRIGGERED! No signal for > %dms. Centering channels.\n", 
                  millis() - startTime, FAILSAFE_TIMEOUT_MS);
    
    // Set all primary channels to neutral (1500)
    channels[0] = 1500; // Roll
    channels[1] = 1500; // Pitch
    channels[2] = 1500; // Yaw
    channels[3] = 1500; // Throttle (Crucial for disarming/safe state)
    
    // Set Aux channels to safe positions
    channels[4] = 1000; // AUX1 (Min/Disarm)
    channels[5] = 1500; // AUX2 (Mid)
    
    isFailsafeActive = true;
  }
  sendIBUS(); // Send the failsafe frame continuously
}

void processNewData() {
  // Create a non-volatile copy of the data
  LoraData currentData;
  unsigned long now = millis();
  
  // Protect the access to the volatile shared variable
  noInterrupts();
  // FIX: Use memcpy to safely copy volatile struct data.
  memcpy(&currentData, (const void*)&receivedData, sizeof(LoraData)); 
  newDataAvailable = false;
  interrupts();
  
  // Update Failsafe status
  isFailsafeActive = false;
  
  // Calculate Control Hz
  static unsigned long previousReceiveTime = 0;

  if (previousReceiveTime != 0) {
    unsigned long timeDelta = now - previousReceiveTime;
    if (timeDelta > 0) {
      controlHz = 1000.0f / timeDelta;
    }
  }
  previousReceiveTime = now;
  
  // Map joystick raw values (0-4095) to iBUS range (1000-2000)
  channels[0] = map(currentData.joy1X, 0, 4095, 1000, 2000); // Roll (J1X)
  channels[1] = map(currentData.joy1Y, 0, 4095, 1000, 2000); // Pitch (J1Y)
  channels[2] = map(currentData.joy2X, 0, 4095, 1000, 2000); // Yaw (J2X)
  channels[3] = map(currentData.joy2Y, 0, 4095, 2000, 1000); // Throttle (J2Y)

  channels[4] = auxValue(currentData.aux1); // AUX1 (switch)
  channels[5] = auxValue(currentData.aux2); // AUX2 (switch)
  
  // Send iBUS frame
  sendIBUS();

  // Debug (only print every ~100ms or so to prevent log spam)
  if (now - lastHzUpdateTime > 100) {
    lastHzUpdateTime = now;
    int rssi = LoRa.packetRssi();
    float snr = LoRa.packetSnr();
    
    Serial.printf("[%lu ms] RX OK! Control Hz: %.1f, RSSI: %d, SNR: %.1f\n",
                  now - startTime, controlHz, rssi, snr);
    Serial.printf("  iBUS -> R:%u P:%u Y:%u T:%u | A1:%u A2:%u\n",
                  channels[0], channels[1], channels[2], channels[3], channels[4], channels[5]);
  }
}

void setup(){
  Serial.begin(115200);
  delay(100);
  Serial.println("\n--- LoRa RX iBUS Controller Initializing ---");

  // iBUS Serial init
  IBUS_SERIAL.begin(115200,SERIAL_8N1,-1,IBUS_TX_PIN);

  // LoRa setup
  SPI.begin(SCK, MISO, MOSI, SS);
  LoRa.setPins(SS, RST, DIO0);

  if (!LoRa.begin(LORA_FREQUENCY)) {
    Serial.println("LoRa initialization failed. Check wiring!");
    while (1);
  }
  
  LoRa.setSpreadingFactor(LORA_SF);
  LoRa.setSignalBandwidth(LORA_BANDWIDTH);
  
  // Set up the interrupt callback and start continuous receive mode
  LoRa.onReceive(onReceive);
  LoRa.receive(); 
  
  Serial.printf("LoRa RX Ready. Freq: %.1f MHz, SF: %d, Failsafe Timeout: %dms\n", 
    (float)LORA_FREQUENCY/1E6, LORA_SF, FAILSAFE_TIMEOUT_MS);
    
  startTime = millis();
  lastReceiveTime = startTime; // Start timer now
}

void loop(){
  // 1. Process received data if the flag is set
  if (newDataAvailable) {
    processNewData();
  }
  
  // 2. Failsafe check
  if (millis() - lastReceiveTime > FAILSAFE_TIMEOUT_MS) {
    applyFailsafe();
  }
  
  // Small delay for system stability
  delay(1); 
}
