#include <Wire.h>

#define SDA_PIN 0
#define SCL_PIN 1

#define ADDR 0x36
#define STATUS 0x0B

void setup() {
  Serial.begin(115200);
  delay(2000);

  Wire.setSDA(SDA_PIN);
  Wire.setSCL(SCL_PIN);
  Wire.begin();

  Serial.println("Escaneando I2C...");

  for (byte i = 1; i < 127; i++) {
    Wire.beginTransmission(i);
    if (Wire.endTransmission() == 0) {
      Serial.print("Encontrado: 0x");
      Serial.println(i, HEX);
    }
  }
}

void loop() {
  byte status = readRegister(STATUS);

  int md = (status >> 5) & 1; // Magnet detected
  int ml = (status >> 4) & 1; // Magnet too low
  int mh = (status >> 3) & 1; // Magnet too high

  Serial.print("STATUS: ");
  Serial.print(status);
  Serial.print(" | MD: ");
  Serial.print(md);
  Serial.print(" | ML: ");
  Serial.print(ml);
  Serial.print(" | MH: ");
  Serial.println(mh);

  delay(200);
}

// Función para leer 1 byte desde un registro
byte readRegister(byte reg) {
  Wire.beginTransmission(ADDR);
  Wire.write(reg);
  Wire.endTransmission(false); // restart

  Wire.requestFrom(ADDR, (byte)1);

  if (Wire.available()) {
    return Wire.read();
  } else {
    Serial.println("Error I2C");
    return 0;
  }
}
