#include <SimpleFOC.h> //libreria para controlar el motor
#include <Wire.h> //maneja la comunicacion I2C

#define SDA_PIN 0 
#define SCL_PIN 1

#define ADDR 0x36
#define STATUS 0x0B //registro del sensor 

#define In1 2
#define In2 3
#define In3 4
#define En 5

BLDCDriver3PWM driver = BLDCDriver3PWM(In1, In2, In3, En); //con esto creo un driver con 3 señales PWM

float angulo = 0; //posicion electrica del motor
float velocidad = 0.05; //cuanto aumento por ciclo

// Estado anterior del imán
bool magnetPrev = false;

void setup() {
  Serial.begin(115200); //aca inciio la comunicacion entre plca y pc

  Wire.setSDA(SDA_PIN);
  Wire.setSCL(SCL_PIN);
  Wire.begin();

  driver.voltage_power_supply = 12; //aviso el voltaje para que se escalen las señales PWM
  driver.init(); //preparo el hardware
  driver.enable(); //activo la salida

  pinMode(5, OUTPUT); //el 5 tiene el enable
  digitalWrite(5, HIGH); //activo el driver
}

void loop() {

  // -------- DETECTOR DE TICS --------
  byte status = readRegister(STATUS); //lee el sensor
  bool magnetNow = (status >> 5) & 1; // MD bit, si true hay iman si es false no hay

  // detectar flanco (tic)
  if (magnetNow && !magnetPrev) {
    Serial.println("TIC");
  }

  magnetPrev = magnetNow; //esto es el tic

  // -------- MOTOR (independiente) --------
  angulo += velocidad; //simulo rotacion

  float Ua = 6 * sin(angulo);
  float Ub = 6 * sin(angulo - 2.094);
  float Uc = 6 * sin(angulo - 4.188); //tres señales sinusoidales defasadas

  driver.setPwm(Ua, Ub, Uc);//las aplico al motir

  delay(5);
}

// -------- I2C --------

byte readRegister(byte reg) {
  Wire.beginTransmission(ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);

  Wire.requestFrom(ADDR, (byte)1);

  if (Wire.available()) {
    return Wire.read(); //me devuelve un dato si hay
  }
  return 0;//y sino me manda cero
}