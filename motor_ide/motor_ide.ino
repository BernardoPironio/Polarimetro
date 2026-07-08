/*
 * CNC Shield v3 + DRV8825 – Controle angular X/Y/Z com:
 * - Homing multipasso + centralização do slot (zero muito repetível)
 * - Backlash por eixo (tomada de folga)
 * - Calibração "1 volta" (steps/volta efetivo por eixo)
 * - LUT de 12 setores por eixo (correção periódica com interpolação)
 * - Protocolo serial 115200 bps
 *
 * Pinos (CNC Shield v3):
 * ENABLE D8 (LOW=ativo)
 * X: STEP D2, DIR D5, ENDSTOP D9
 * Y: STEP D3, DIR D6, ENDSTOP D10
 * Z: STEP D4, DIR D7, ENDSTOP D11
 */

#include <math.h>
#include <EEPROM.h>

// -------------------- CONFIG BÁSICA --------------------
const uint8_t PIN_EN       = 8;
const uint8_t X_STEP_PIN   = 2;
const uint8_t Y_STEP_PIN   = 3;
const uint8_t Z_STEP_PIN   = 4;
const uint8_t X_DIR_PIN    = 5;
const uint8_t Y_DIR_PIN    = 6;
const uint8_t Z_DIR_PIN    = 7;
const uint8_t X_ENDSTOP    = 9;  // HIGH fora do slot, LOW no slot
const uint8_t Y_ENDSTOP    = 10;
const uint8_t Z_ENDSTOP    = 11;

const int    N_SECTORS = 12;     // LUT com 12 setores (30° cada)
const float  LUT_SECTOR_DEG = 360.0f / N_SECTORS;

// -------------------- ESTRUTURAS --------------------
struct Axis {
  char     name;
  uint8_t  stepPin, dirPin, endstopPin;

  volatile long stepPos_coroa;   // posição lógica em passos da coroa (módulo steps/volta)
  long     backlash_steps;       // passos da coroa para tomar folga
  bool     invertDir;            // inverte sentido físico do DIR
  float    target_deg;           // cache alvo (não essencial)

  // Calib: steps/volta efetivo por eixo (se >0, tem prioridade sobre razão global)
  long     steps_rev_override;

  // LUT de erro periódico (graus) e flag de uso
  float    lut[N_SECTORS];
  bool     lut_enabled;

  Axis(char n, uint8_t step, uint8_t dir, uint8_t end)
  : name(n), stepPin(step), dirPin(dir), endstopPin(end),
    stepPos_coroa(0), backlash_steps(0), invertDir(false), target_deg(0.0f),
    steps_rev_override(0), lut_enabled(false)
  {
    for (int i=0;i<N_SECTORS;i++) lut[i] = 0.0f;
  }

  Axis() : Axis('?',0,0,0) {}
};

Axis AX[3] = {
  Axis('X', X_STEP_PIN, X_DIR_PIN, X_ENDSTOP),
  Axis('Y', Y_STEP_PIN, Y_DIR_PIN, Y_ENDSTOP),
  Axis('Z', Z_STEP_PIN, Z_DIR_PIN, Z_ENDSTOP),
};

// -------------------- PARÂMETROS GLOBAIS --------------------
int   motor_steps_per_rev   = 200;    // motor (p.ex., 200)
int   microstep             = 32;     // DRV8825: 1/1..1/32
float gear_ratio            = 10.0f;  // motor:coroa (nominal) -> usado só se override=0

float vel_deg_s   = 200.0f;           // alvo de velocidade (deg/s)
float acc_deg_s2  = 2000.0f;         // aceleração (deg/s^2)

float backoff_deg = 5.0f;            // recuo entre passagens de homing (> folga + largura slot)
int   debounceN   = 8;               // leituras iguais para confirmar borda
unsigned long edgeStable_us = 1500;  // janela p/ estabilizar (us)

// -------------------- EEPROM (opcional; simples) --------------------
// Layout simples: cabeçalho + globais + por eixo
struct Persist {
  uint32_t magic;           // 0xA1B2C3D4
  int motor_steps;
  int microstep;
  float gear_ratio;
  float vel_deg_s;
  float acc_deg_s2;
  float backoff_deg;

  struct PerAxis {
    long steps_rev_override;
    long backlash_steps;
    bool invertDir;
    bool lut_enabled;
    float lut[N_SECTORS];
  } pa[3];
} persist;

void loadEEP() {
  EEPROM.get(0, persist);
  if (persist.magic != 0xA1B2C3D4) return; // não carregue se vazio

  motor_steps_per_rev = persist.motor_steps;
  microstep           = persist.microstep;
  gear_ratio          = persist.gear_ratio;
  vel_deg_s           = persist.vel_deg_s;
  acc_deg_s2          = persist.acc_deg_s2;
  backoff_deg         = persist.backoff_deg;

  for (int i=0;i<3;i++) {
    AX[i].steps_rev_override = persist.pa[i].steps_rev_override;
    AX[i].backlash_steps     = persist.pa[i].backlash_steps;
    AX[i].invertDir          = persist.pa[i].invertDir;
    AX[i].lut_enabled        = persist.pa[i].lut_enabled;
    for (int j=0;j<N_SECTORS;j++) AX[i].lut[j] = persist.pa[i].lut[j];
  }
}
void saveEEP() {
  persist.magic       = 0xA1B2C3D4;
  persist.motor_steps = motor_steps_per_rev;
  persist.microstep   = microstep;
  persist.gear_ratio  = gear_ratio;
  persist.vel_deg_s   = vel_deg_s;
  persist.acc_deg_s2  = acc_deg_s2;
  persist.backoff_deg = backoff_deg;

  for (int i=0;i<3;i++) {
    persist.pa[i].steps_rev_override = AX[i].steps_rev_override;
    persist.pa[i].backlash_steps     = AX[i].backlash_steps;
    persist.pa[i].invertDir          = AX[i].invertDir;
    persist.pa[i].lut_enabled        = AX[i].lut_enabled;
    for (int j=0;j<N_SECTORS;j++) persist.pa[i].lut[j] = AX[i].lut[j];
  }
  EEPROM.put(0, persist);
}

// -------------------- CONVERSÕES POR EIXO --------------------
long stepsPerRevCoroa(const Axis &a) {
  if (a.steps_rev_override > 0) return a.steps_rev_override;
  return (long)motor_steps_per_rev * (long)microstep * (long)lroundf(gear_ratio);
}
float stepsPerDeg(const Axis &a) {
  return (float)stepsPerRevCoroa(a) / 360.0f;
}
long degToSteps(const Axis &a, float deg) {
  return lroundf(deg * stepsPerDeg(a));
}
float stepsToDeg(const Axis &a, long steps) {
  return (float)steps / stepsPerDeg(a);
}

// -------------------- HW BÁSICO --------------------
inline void enableDrivers(bool en) { digitalWrite(PIN_EN, en ? LOW : HIGH); }
inline void setDir(const Axis &a, bool dirCW) {
  digitalWrite(a.dirPin, (a.invertDir ? !dirCW : dirCW) ? HIGH : LOW);
}
inline void stepPulse(const Axis &a, unsigned int usPulse=2) {
  digitalWrite(a.stepPin, HIGH);
  delayMicroseconds(usPulse);
  digitalWrite(a.stepPin, LOW);
  delayMicroseconds(usPulse);
}

bool readEndstopRaw(const Axis &a) {
  // HIGH fora do slot; LOW dentro do slot
  return digitalRead(a.endstopPin) == LOW;
}
bool readStableState(const Axis &a, bool wantLOW) {
  int count = 0;
  unsigned long t0 = micros();
  while (micros() - t0 < edgeStable_us) {
    bool isLOW = readEndstopRaw(a);
    if (isLOW == wantLOW) count++;
    else count = 0;
    if (count >= debounceN) return true;
    delayMicroseconds(60);
  }
  return false;
}

// -------------------- BUSCAS DE BORDA/CENTRO --------------------
long seekEdge(Axis &a, bool dirCW, float steps_per_s, bool fallingEdge=true) {
  // Procura HIGH->LOW (falling) ou LOW->HIGH (rising) movendo contínuo
  setDir(a, dirCW);
  bool prevLOW = readEndstopRaw(a);
  unsigned long usPerStep = (unsigned long)(1e6f / steps_per_s);
  if (usPerStep < 5) usPerStep = 5;

  while (true) {
    stepPulse(a);
    delayMicroseconds(usPerStep);
    a.stepPos_coroa += dirCW ? 1 : -1;
    bool nowLOW = readEndstopRaw(a);

    if (fallingEdge) {
      if (!prevLOW && nowLOW) {
        if (readStableState(a, true)) return a.stepPos_coroa;
      }
    } else {
      if (prevLOW && !nowLOW) {
        if (readStableState(a, false)) return a.stepPos_coroa;
      }
    }
    prevLOW = nowLOW;
  }
}

// Encontra o PRÓXIMO centro do slot movendo apenas para frente (CW)
long findNextCenterCW(Axis &a, float steps_per_s) {
  setDir(a, true);
  unsigned long usPerStep = (unsigned long)(1e6f / steps_per_s);
  if (usPerStep < 5) usPerStep = 5;

  // 1) Garanta que está fora do slot
  while (readEndstopRaw(a)) { // LOW -> dentro
    stepPulse(a);
    delayMicroseconds(usPerStep);
    a.stepPos_coroa += 1;
  }

  // 2) Detecta falling HIGH->LOW
  bool prevLOW = false;
  long pFall = 0, pRise = 0;
  while (true) {
    bool nowLOW = readEndstopRaw(a);
    if (!prevLOW && nowLOW) { // HIGH->LOW
      if (readStableState(a, true)) { pFall = a.stepPos_coroa; break; }
    }
    prevLOW = nowLOW;
    stepPulse(a);
    delayMicroseconds(usPerStep);
    a.stepPos_coroa += 1;
  }

  // 3) Continua até sair do slot (rising LOW->HIGH)
  prevLOW = true;
  while (true) {
    bool nowLOW = readEndstopRaw(a);
    if (prevLOW && !nowLOW) {
      if (readStableState(a, false)) { pRise = a.stepPos_coroa; break; }
    }
    prevLOW = nowLOW;
    stepPulse(a);
    delayMicroseconds(usPerStep);
    a.stepPos_coroa += 1;
  }

  // 4) Centro = média (sem inverter sentido)
  long center = (pFall + pRise) / 2;
  //long center = (pFall);
  return center;
}

// -------------------- MOVIMENTO (perfil trapezoidal) --------------------
void moveStepsTrapezoid(Axis &a, long dSteps) {
  if (dSteps == 0) return;
  bool dirCW = dSteps > 0;
  long N = labs(dSteps);
  setDir(a, dirCW);

  float v_max = fabs(vel_deg_s)  * stepsPerDeg(a); // passos/s
  float acc   = fabs(acc_deg_s2) * stepsPerDeg(a); // passos/s^2

  long n_acc = lroundf(v_max*v_max / (2.0f*acc));
  long n_ramp = min(n_acc, N/2);
  float v = 0.0f;

  unsigned long last = micros();
  for (long i=0; i<N; i++) {
    if (i < n_ramp) {
      v = sqrtf(2.0f*acc*(i+1));
      if (v > v_max) v = v_max;
    } else if (i >= N - n_ramp) {
      long j = N - i;
      v = sqrtf(2.0f*acc*j);
      if (v > v_max) v = v_max;
    } else v = v_max;

    if (v < 10.0f) v = 10.0f;

    unsigned long period = (unsigned long)(1e6f / v);
    unsigned long now = micros();
    if (now - last < period) delayMicroseconds(period - (now - last));
    stepPulse(a);
    last = micros();
    a.stepPos_coroa += dirCW ? 1 : -1;
  }
}

// -------------------- LUT (interpolação linear com wrap) --------------------
float lutDeltaDeg(const Axis &a, float theta_deg) {
  if (!a.lut_enabled) return 0.0f;
  // Normaliza 0..360
  while (theta_deg < 0) theta_deg += 360.0f;
  while (theta_deg >= 360.0f) theta_deg -= 360.0f;

  int i = (int)floorf(theta_deg / LUT_SECTOR_DEG);
  int j = (i + 1) % N_SECTORS;
  float t0 = i * LUT_SECTOR_DEG;
  float frac = (theta_deg - t0) / LUT_SECTOR_DEG;

  float d0 = a.lut[i];
  float d1 = a.lut[j];
  return d0*(1.0f - frac) + d1*frac;
}

// -------------------- HOMING --------------------
void homeAxis(Axis &a) {
  const float v1 = 1200.0f;
  const float v2 =  400.0f;
  const float v3 =  120.0f;
  long backoff = degToSteps(a, backoff_deg);

  // 1) Multipass na borda falling (HIGH->LOW), direção CW
  long p1 = seekEdge(a, true,  v1, true);
  moveStepsTrapezoid(a, -backoff);
  long p2 = seekEdge(a, true,  v2, true);
  moveStepsTrapezoid(a, -backoff);
  long p3 = seekEdge(a, true,  v3, true);
  (void)p1; (void)p2; (void)p3; // só para possível debug

   // 2) Centralização do slot SEM inverter: achar pFall e pRise e setar centro
  moveStepsTrapezoid(a, -backoff); // sai do slot
  long center = findNextCenterCW(a, v3);

  // -- MOVA fisicamente até o centro recém-medido (com tomada de folga) --
  float center_deg = stepsToDeg(a, center);
  gotoDeg(a, center_deg);          // usa LUT/backlash do próprio firmware

  // Sincroniza exatamente a posição lógica com o alvo em passos
  a.stepPos_coroa = degToSteps(a, center_deg);
}


void homeAll() {
  enableDrivers(true);
  for (int i=0;i<3;i++) homeAxis(AX[i]);
}

// -------------------- CALIBRAÇÃO "1 VOLTA" (1 fenda; usando CENTRO) --------------------
void calibOneRev(Axis &a) {
  const float v_seek  = 800.0f;  // velocidade para achar centros
  const uint8_t samples = 3;     // nº de intervalos centro->centro para média

  // (Opcional) Calibrar sem LUT para não aplicar correções enquanto mede
  bool lut_prev = a.lut_enabled;
  a.lut_enabled = false;

  // 1) Achar um centro de referência (C0) SEM inverter sentido
  long c0 = findNextCenterCW(a, v_seek);
  float c0_deg = stepsToDeg(a, c0);
  gotoDeg(a, c0_deg);
  a.stepPos_coroa = degToSteps(a, c0_deg); // sincroniza posição lógica

  // 2) Medir 'samples' intervalos centro->centro (sempre CW)
  long sum_delta = 0;
  long c_prev = a.stepPos_coroa;
  for (uint8_t i = 0; i < samples; ++i) {
    long c_next = findNextCenterCW(a, v_seek); // próximo CENTRO
    sum_delta += (c_next - c_prev);             // delta em passos (monotônico, CW)
    c_prev = c_next;
  }

  // 3) Média dos deltas entre centros (arredondada)
  // Use lround se disponível; senão, substitua pela fórmula inteira: (sum_delta + samples/2)/samples
  long mean_delta =
  #ifdef __cplusplus
    (long)lround((double)sum_delta / (double)samples);
  #else
    (long)((sum_delta + (samples/2)) / samples);
  #endif

  // Como existe apenas 1 fenda por volta, steps/rev = intervalo entre CENTROS
  long steps_per_rev = mean_delta;
  if (steps_per_rev <= 0) {
    // fallback de segurança caso algo dê wrap inesperado
    steps_per_rev += stepsPerRevCoroa(a);
  }

  a.steps_rev_override = steps_per_rev;

  // 4) Posiciona fisicamente no último centro medido (referência estável)
  float c_end_deg = stepsToDeg(a, c_prev);
  gotoDeg(a, c_end_deg);
  a.stepPos_coroa = degToSteps(a, c_end_deg);

  // 5) Restaura LUT
  a.lut_enabled = lut_prev;

  // Log
  Serial.print(F("CALIB ONE_REV ")); Serial.print(a.name);
  Serial.print(F(": steps/rev efetivo=")); Serial.println(steps_per_rev);
}

void spinContinuous(Axis &a, bool dirCW) {
  setDir(a, dirCW);

  float v = fabs(vel_deg_s) * stepsPerDeg(a);   // pasos/segundo
  unsigned long usPerStep = (unsigned long)(1000000.0 / v);

  while (true) {
    stepPulse(a);

    a.stepPos_coroa += dirCW ? 1 : -1;

    delayMicroseconds(usPerStep);

    // Permite detener desde el puerto serie
    if (Serial.available()) {
      String s = Serial.readStringUntil('\n');
      s.trim();
      s.toUpperCase();
      if (s == "STOP")
        break;
    }
  }
}



// -------------------- GOTO (com LUT e backlash) --------------------
void gotoDeg(Axis &a, float theta_cmd_deg) {
  // Aplica LUT (se habilitada)
  float theta_corr = theta_cmd_deg + lutDeltaDeg(a, theta_cmd_deg);

  // Converte para passos alvo; normaliza em [0, stepsRev)
  long rev = stepsPerRevCoroa(a);
  long tgt = degToSteps(a, theta_corr);
  tgt = (tgt % rev + rev) % rev;

  long cur = (a.stepPos_coroa % rev + rev) % rev;
  long delta = tgt - cur;

  if (delta >  rev/2)  delta -= rev;
  if (delta < -rev/2)  delta += rev;

  // Tomada de folga: sempre chegar pelo mesmo sentido
  if (delta < 0) {
    moveStepsTrapezoid(a, delta - a.backlash_steps);
    moveStepsTrapezoid(a, a.backlash_steps);
  } else {
    moveStepsTrapezoid(a, delta + a.backlash_steps);
    moveStepsTrapezoid(a, -a.backlash_steps);
  }
}

// -------------------- SERIAL/UI --------------------
String input;

float parseAfter(const String &s, const char* key, float defNAN) {
  int p = s.indexOf(key);
  if (p<0) return defNAN;
  p += strlen(key);
  while (p<(int)s.length() && s[p]==' ') p++;
  int q = p;
  while (q < (int)s.length() && s[q] != ' ' && s[q] != '\n' && s[q] != '\r') q++;
  return s.substring(p,q).toFloat();
}

Axis* axisByChar(char c) {
  c = toupper(c);
  for (int i=0;i<3;i++) if (AX[i].name == c) return &AX[i];
  return NULL;
}

void printStatus() {
  Serial.println(F("=== STATUS ==="));
  Serial.print(F("motor_steps=")); Serial.println(motor_steps_per_rev);
  Serial.print(F("microstep="));    Serial.println(microstep);
  Serial.print(F("ratio(nom)="));   Serial.println(gear_ratio,6);
  Serial.print(F("vel(deg/s)="));   Serial.println(vel_deg_s,1);
  Serial.print(F("acc(deg/s^2)=")); Serial.println(acc_deg_s2,1);
  Serial.print(F("backoff(deg)=")); Serial.println(backoff_deg,2);
  for (int i=0;i<3;i++) {
    long rev = stepsPerRevCoroa(AX[i]);
    Serial.print(AX[i].name); Serial.print(F(": pos="));
    Serial.print(stepsToDeg(AX[i], AX[i].stepPos_coroa),4);
    Serial.print(F(" deg, steps/rev=")); Serial.print(rev);
    Serial.print(F(", backlash=")); Serial.print(AX[i].backlash_steps);
    Serial.print(F(" steps, invert=")); Serial.print(AX[i].invertDir ? 1:0);
    Serial.print(F(", LUT=")); Serial.print(AX[i].lut_enabled ? 1:0);
    Serial.println();
  }
}

void lutShow() {
  for (int k=0;k<3;k++) {
    Serial.print(F("LUT ")); Serial.print(AX[k].name); Serial.print(F(": "));
    for (int i=0;i<N_SECTORS;i++) {
      Serial.print(AX[k].lut[i],4);
      if (i<N_SECTORS-1) Serial.print(F(", "));
    }
    Serial.println();
  }
}

void processLine(String s) {
  s.trim();
  if (s.length()==0) return;

  // Upcase para parsing robusto, mas preserva números
  String S = s; S.toUpperCase();

  if (S.startsWith("SPIN")) {
  Axis *a = nullptr;

  if (S.indexOf('X') >= 0) a = &AX[0];
  else if (S.indexOf('Y') >= 0) a = &AX[1];
  else if (S.indexOf('Z') >= 0) a = &AX[2];

  if (!a) {
    Serial.println("ERR");
    return;
  }

  bool dir = true;
  if (S.indexOf("CCW") >= 0)
    dir = false;

  spinContinuous(*a, dir);

  Serial.println("OK STOP");
  return;
}

  if (S.startsWith("HOME")) {
    if (S.indexOf("HOME X")>=0) homeAxis(AX[0]);
    else if (S.indexOf("HOME Y")>=0) homeAxis(AX[1]);
    else if (S.indexOf("HOME Z")>=0) homeAxis(AX[2]);
    else homeAll();
    Serial.println(F("OK HOME"));
    saveEEP();
    return;
  }

  if (S.startsWith("GOTO")) {
    bool any=false;
    float v;
    v = parseAfter(S, "X=", NAN); if (!isnan(v)) { gotoDeg(AX[0], v); any=true; }
    v = parseAfter(S, "Y=", NAN); if (!isnan(v)) { gotoDeg(AX[1], v); any=true; }
    v = parseAfter(S, "Z=", NAN); if (!isnan(v)) { gotoDeg(AX[2], v); any=true; }
    Serial.println(any ? F("OK GOTO") : F("ERR: use GOTO X=.. Y=.. Z=.."));
    return;
  }

  if (S.startsWith("SPD")) {
    float v = parseAfter(S, "SPD", NAN);
    if (!isnan(v) && v>0) { vel_deg_s = v; Serial.println(F("OK SPD")); saveEEP(); }
    else Serial.println(F("ERR SPD"));
    return;
  }
  if (S.startsWith("ACC")) {
    float v = parseAfter(S, "ACC", NAN);
    if (!isnan(v) && v>0) { acc_deg_s2 = v; Serial.println(F("OK ACC")); saveEEP(); }
    else Serial.println(F("ERR ACC"));
    return;
  }
  if (S.startsWith("SET RATIO")) {
    float r = parseAfter(S, "SET RATIO", NAN);
    if (!isnan(r) && r>0.01f) { gear_ratio = r; Serial.println(F("OK RATIO")); saveEEP(); }
    else Serial.println(F("ERR RATIO"));
    return;
  }
  if (S.startsWith("SET MICROSTEP")) {
    float m = parseAfter(S, "SET MICROSTEP", NAN);
    int mi = (int)m;
    if (mi==1||mi==2||mi==4||mi==8||mi==16||mi==32) { microstep = mi; Serial.println(F("OK MICROSTEP")); saveEEP(); }
    else Serial.println(F("ERR MICROSTEP"));
    return;
  }
  if (S.startsWith("SET STEPS")) {
    float n = parseAfter(S, "SET STEPS", NAN);
    int nn = (int)n;
    if (nn>0 && nn<=400) { motor_steps_per_rev = nn; Serial.println(F("OK STEPS")); saveEEP(); }
    else Serial.println(F("ERR STEPS"));
    return;
  }
  if (S.startsWith("SET BACKLASH")) {
    float vx = parseAfter(S, "X=", NAN);
    float vy = parseAfter(S, "Y=", NAN);
    float vz = parseAfter(S, "Z=", NAN);
    if (!isnan(vx)) AX[0].backlash_steps = (long)vx;
    if (!isnan(vy)) AX[1].backlash_steps = (long)vy;
    if (!isnan(vz)) AX[2].backlash_steps = (long)vz;
    Serial.println(F("OK BACKLASH"));
    saveEEP();
    return;
  }
  if (S.startsWith("ZERO")) {
    float vx = parseAfter(S, "X=", NAN);
    float vy = parseAfter(S, "Y=", NAN);
    float vz = parseAfter(S, "Z=", NAN);
    if (!isnan(vx)) AX[0].stepPos_coroa = degToSteps(AX[0], vx);
    if (!isnan(vy)) AX[1].stepPos_coroa = degToSteps(AX[1], vy);
    if (!isnan(vz)) AX[2].stepPos_coroa = degToSteps(AX[2], vz);
    Serial.println(F("OK ZERO"));
    saveEEP();
    return;
  }

  if (S.startsWith("CALIB ONE_REV")) {
    Axis* a = NULL;
    if (S.indexOf('X')>=0) a=&AX[0];
    else if (S.indexOf('Y')>=0) a=&AX[1];
    else if (S.indexOf('Z')>=0) a=&AX[2];

    if (a) { calibOneRev(*a); saveEEP(); }
    else Serial.println(F("ERR: use CALIB ONE_REV X|Y|Z"));
    return;
  }

  if (S.startsWith("LUT ENABLE")) {
    float ex = parseAfter(S, "X=", NAN);
    float ey = parseAfter(S, "Y=", NAN);
    float ez = parseAfter(S, "Z=", NAN);
    if (!isnan(ex)) AX[0].lut_enabled = ((int)ex)!=0;
    if (!isnan(ey)) AX[1].lut_enabled = ((int)ey)!=0;
    if (!isnan(ez)) AX[2].lut_enabled = ((int)ez)!=0;
    Serial.println(F("OK LUT ENABLE"));
    saveEEP();
    return;
  }

  if (S.startsWith("LUT CLEAR")) {
    Axis* a = NULL;
    if (S.indexOf('X')>=0) a=&AX[0];
    else if (S.indexOf('Y')>=0) a=&AX[1];
    else if (S.indexOf('Z')>=0) a=&AX[2];

    if (a) {
      for (int i=0;i<N_SECTORS;i++) a->lut[i]=0.0f;
      Serial.print(F("OK LUT CLEAR ")); Serial.println(a->name);
      saveEEP();
    } else Serial.println(F("ERR: use LUT CLEAR X|Y|Z"));
    return;
  }

  if (S.startsWith("LUT SET")) {
    // Formato: LUT SET X i val
    // Ex.: "LUT SET X 3 -0.0123"
    Axis* a = NULL;
    if (S.indexOf('X')>=0) a=&AX[0];
    else if (S.indexOf('Y')>=0) a=&AX[1];
    else if (S.indexOf('Z')>=0) a=&AX[2];

    if (!a) { Serial.println(F("ERR: LUT SET X|Y|Z i val")); return; }

    // Extrai índice e valor do fim da string original (não upcase)
    int p1 = s.indexOf(' ');
    p1 = s.indexOf(' ', p1+1); // após "LUT"
    p1 = s.indexOf(' ', p1+1); // após "SET"
    if (p1<0) { Serial.println(F("ERR: LUT SET X i val")); return; }
    int p2 = s.indexOf(' ', p1+1);
    if (p2<0) { Serial.println(F("ERR: LUT SET X i val")); return; }

    int idx = s.substring(p1+1, p2).toInt();
    float val = s.substring(p2+1).toFloat();
    if (idx<0 || idx>=N_SECTORS) { Serial.println(F("ERR: i fora de 0..11")); return; }
    a->lut[idx] = val;
    Serial.print(F("OK LUT SET ")); Serial.print(a->name);
    Serial.print(F("[ ")); Serial.print(idx); Serial.print(F(" ]="));
    Serial.println(val,6);
    saveEEP();
    return;
  }

  if (S=="LUT SHOW") { lutShow(); return; }
  if (S=="STATUS")   { printStatus(); return; }

  Serial.println(F("Comando desconhecido."));
}

// -------------------- SETUP/LOOP --------------------
void setup() {
  Serial.begin(115200);
  pinMode(PIN_EN, OUTPUT); digitalWrite(PIN_EN, HIGH); // desabilita no boot

  for (int i=0;i<3;i++) {
    pinMode(AX[i].stepPin, OUTPUT); digitalWrite(AX[i].stepPin, LOW);
    pinMode(AX[i].dirPin,  OUTPUT); digitalWrite(AX[i].dirPin, LOW);
    pinMode(AX[i].endstopPin, INPUT_PULLUP); // HIGH fora do slot
  }

  // Caso algum eixo esteja invertido fisicamente:
  // AX[0].invertDir = false;
  // AX[1].invertDir = false;
  // AX[2].invertDir = false;

  loadEEP();
  enableDrivers(true);

  Serial.println(F("Pronto. Use: HOME | CALIB ONE_REV X | LUT ... | GOTO X=.. Y=.. Z=.. | STATUS"));
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c=='\n' || c=='\r') {
      if (input.length()>0) { processLine(input); input=""; }
    } else {
      input += c;
      if (input.length()>160) input="";
    }
  }
}
