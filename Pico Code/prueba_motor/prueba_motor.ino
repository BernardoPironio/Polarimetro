#include <SimpleFOC.h>

BLDCDriver3PWM driver = BLDCDriver3PWM(2, 3, 4, 5);

float angulo = 0;
float velocidad = 0.8;  // 👈 controlás esto

void setup() {
  driver.voltage_power_supply = 12;
  driver.init();

  pinMode(5, OUTPUT);
  digitalWrite(5, LOW);
  
}

void loop() {
  angulo += velocidad;

  float Ua = 6 * sin(angulo);
  float Ub = 6 * sin(angulo - 2.094);
  float Uc = 6 * sin(angulo - 4.188);

  driver.setPwm(Ua, Ub, Uc);

  delay(5);
}