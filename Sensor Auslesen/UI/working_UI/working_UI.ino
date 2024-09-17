#include <Wire.h>               // Only needed for Arduino 1.6.5 and earlier
#include "SH1106Wire.h"   // legacy: #include "SH1106.h"

SH1106Wire display(0x3c, SDA, SCL);     // ADDRESS, SDA, SCL

void splashscreen(){

  for(int progress = 0; progress < 100; progress++){

  display.clear();
  display.setFont(ArialMT_Plain_16);
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.drawString(64, 12, "ZigBee Monitor");
  display.setFont(ArialMT_Plain_10);
  display.drawString(64, 30, "Vers. 0.0");

  display.drawProgressBar(1, 45, 120, 10, progress);

  display.display();

  delay(16);

  }

}



void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println();

  display.init();
  display.flipScreenVertically();

  splashscreen();

}


void loop() {
  // clear the display
  display.clear();
  display.setFont(ArialMT_Plain_16);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(0, 0, "Sensorwerte");
  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 20, "Temperatur: ");
  display.drawString(0, 30, "Luftfeuchte: ");
  display.display();
  delay(10);
}
