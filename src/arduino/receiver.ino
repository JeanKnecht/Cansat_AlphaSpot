/*
 * CanSat AlphaSpot - Ground Station Receiver
 * 
 * Arduino Uno program that receives GPS coordinates from the CanSat
 * via RFM69 radio module and outputs them to serial port.
 * 
 * The received data is then processed by the Python live_gps.py script
 * to create a KML file for Google Earth visualization.
 * 
 * Hardware:
 * - Arduino Uno
 * - RFM69 Radio Module
 * - 7-element Yagi Antenna (custom designed)
 */

#include <SPI.h>
#include <RH_RF69.h>

// Radio Configuration
const float RF69_FREQ = 434.50; // MHz (must match transmitter frequency)
const int RFM69_CS = 4;
const int RFM69_INT = 3;
const int RFM69_RST = 2;

// Radio driver instance
RH_RF69 rf69(RFM69_CS, RFM69_INT);

void setup() {
  Serial.begin(115200); // High baud rate for GPS data
    
  // Reset radio module
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);
  
  // Initialize radio
  if (!rf69.init()) {
    Serial.println("ERROR: RFM69 radio init failed");
    while (1) {
      delay(1000); // Hang if radio fails
    }
  }
  
  // Set frequency (must match transmitter)
  rf69.setFrequency(434.50);
  rf69.setTxPower(20, true); // Max power
  
  // Set encryption key (must match transmitter)
  uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  rf69.setEncryptionKey(key);
  
  Serial.print("RFM69 radio @ ");
  Serial.print((int)RF69_FREQ);
  Serial.println(" MHz - Ready to receive");
}

void loop() {
  // Check if radio received a message
  if (rf69.available()) {
    uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    
    if (rf69.recv(buf, &len)) {
      if (len > 0) {
        buf[len] = 0; // Null terminate
        Serial.println((char*)buf); // Output to serial for Python script
      }
    }
  }
}

