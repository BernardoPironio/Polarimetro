const int PWM_PIN = 9;
const int DIR_PIN = 8;

void setup() {
  pinMode(PWM_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  digitalWrite(DIR_PIN, HIGH);   // Cambiar a LOW si querés invertir el giro

  // Timer1 Fast PWM 8 bits, ~31.37 kHz
  TCCR1A = _BV(COM1A1) | _BV(WGM10);
  TCCR1B = _BV(WGM12) | _BV(CS10);

  OCR1A = 255;   // 100% duty
}

void loop() {
}