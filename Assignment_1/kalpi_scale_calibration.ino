/*
 * כיול משקל דיגיטלי (תא עומס + HX711)
 * 1. להפעיל כשהמשקל ריק -> מתבצע tare.
 * 2. להניח משקל ידוע (למשל 100 גרם) ולהקליד ב-Serial Monitor את המשקל בגרמים.
 * 3. התוכנית מדפיסה את SCALE_FACTOR - להעתיק אותו ל-kalpi_hardware.ino.
 */
#include <HX711.h>

const uint8_t PIN_HX_DT  = 4;
const uint8_t PIN_HX_SCK = 5;
HX711 scale;

void setup() {
  Serial.begin(9600);
  scale.begin(PIN_HX_DT, PIN_HX_SCK);
  Serial.println(F("Remove all weight... taring"));
  delay(2000);
  scale.tare(20);
  Serial.println(F("Tare done. Place a known weight and type its grams, then Enter"));
}

void loop() {
  if (Serial.available()) {
    float known = Serial.parseFloat();
    if (known > 0) {
      long raw = scale.get_value(20);   // ממוצע 20 דגימות אחרי tare
      float factor = raw / known;
      Serial.print(F("raw = ")); Serial.print(raw);
      Serial.print(F("  SCALE_FACTOR = ")); Serial.println(factor, 2);
      scale.set_scale(factor);
    }
  }
  if (scale.is_ready()) {
    Serial.print(F("reading: "));
    Serial.print(scale.get_units(5), 1);
    Serial.println(F(" g"));
  }
  delay(500);
}
