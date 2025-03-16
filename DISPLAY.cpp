#include <Wire.h>
#include <TinyGPS++.h>
#include <U8g2lib.h>

// Define LCD
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

// GPS module connected to Serial2 (RX = GPIO16, TX = GPIO17)
static const int RXPin = 16, TXPin = 17;
static const uint32_t GPSBaud = 9600;

TinyGPSPlus gps;
HardwareSerial GPS_Serial(2);

void setup() {
  Serial.begin(115200);
  GPS_Serial.begin(GPSBaud, SERIAL_8N1, RXPin, TXPin);
  u8g2.begin();
}

void loop() {
  while (GPS_Serial.available()) {
    gps.encode(GPS_Serial.read());
  }
  
  if (gps.location.isUpdated()) {
    u8g2.clearBuffer();  // Clear the previous buffer
    u8g2.setFont(u8g2_font_ncenB08_tr);  // Set font for display
    
    // Display Latitude and Longitude
    u8g2.drawStr(0, 10, "Latitude:");
    u8g2.drawStr(0, 20, String(gps.location.lat(), 6).c_str());
    u8g2.drawStr(0, 40, "Longitude:");
    u8g2.drawStr(0, 50, String(gps.location.lng(), 6).c_str());
    
    u8g2.sendBuffer();  // Push buffer to LCD
  }
}
