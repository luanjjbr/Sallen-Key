//
// Sallen-Key-Duino - 6 Controladores com seletor serial
// Base: (c) Luiz Daniel - 2021 | Ganhos calculados no MATLAB
//
// COMANDOS SERIAIS (digite + ENTER):
//   SC  -> Sem Controle (malha aberta)
//   P   -> Proporcional            (Kp=1.53)
//   PI  -> Proporcional+Integral   (Kp=0.185, Ki=1.50)
//   PID -> P+I+D robusto           (Kp=0.25, Ki=2.25, Kd=0.025)
//   AA  -> Avanco                  (eq. diferencas)
//   API -> Avanco+PI robusto       (eq. diferencas)
//   DB  -> Dead-Beat (ts~1s)       (eq. diferencas)
//
// CONVENCAO: integral usa Ki*T*e ; derivada usa Kd/T*(e-e1)
//            (para casar exatamente com o MATLAB)
//

// ---------------- PINOS ----------------
const int pwm_PIN_VDD = 6;
const int pwm_PIN_VNN = 10;
const int pwm_PIN_Vin_SallenKey = 9;
const int pwm_PIN_LED1 = 3;
const int pwm_PIN_LED2 = 11;
const int io_PIN_D12 = 12;
const int io_PIN_A0 = 14;
const int io_PIN_A1 = 15;
const int io_PIN_A2 = 16;
const int analog_PIN_A7_VgK = A7;
const int analog_PIN_A5_Vg  = A5;
const int analog_PIN_A6_Vo  = A6;

const int ci_SampleTime = 10;    // ms (100 Hz)
const float cf_T = 0.010f;       // periodo em segundos (casa com T=10e-3 do MATLAB)

// ---------------- GANHOS / COEFICIENTES (do MATLAB) ----------------
// (1) PROPORCIONAL
const float Kp_P = 1.53f;

// (2) PI ROBUSTO  (zc=0.925, Kc=0.20 -> Kp=0.2, Ki=1.500)
const float Kp_PI = 0.2f, Ki_PI = 1.500f;

// (3) PID ROBUSTO  (Kp=0.25, Ki=2.25, Kd=0.025) MG=40dB MF=97
const float Kp_PID = 0.25f, Ki_PID = 2.250f, Kd_PID = 0.025f;

// (4) AVANCO: 150*(z-0.85)/(z+0.85)
//     u[k] = -a1*u1 + b0*e + b1*e1
const float AV_b0 =  150.0f;
const float AV_b1 = -127.5f;
const float AV_a1 =  0.85f;

// (5) AVANCO+PI robusto (cascata)
//     u[k] = -a1*u1 - a2*u2 + b0*e + b1*e1 + b2*e2
const float API_b0 =  3.800f;
const float API_b1 = -7.030f;
const float API_b2 =  3.249f;
const float API_a1 = -1.300f;
const float API_a2 =  0.300f;

// (6) DEAD-BEAT (Dahlin, ts~1s)
//     u[k] = -a1*u1 - a2*u2 + b0*e + b1*e1 + b2*e2
// DEAD-BEAT VIAVEL (ts=6s, nao satura o PWM)
const float DB_b0 =  1.93387f;
const float DB_b1 = -3.81427f;
const float DB_b2 =  1.90073f;
const float DB_a1 = -0.00575f;
const float DB_a2 = -0.99425f;

#define DEF_INT_UPPER   1.0f
#define DEF_INT_LOWER  -1.0f

// ---------------- ESTADO GLOBAL ----------------
String s_modo = "PID";           // controlador ativo (padrao)

bool b_enableKey_PWM = false, b_printOnce = false;
byte by_WritePWMState = 0;
unsigned long myTime, myOldTime = 0;

float f_vREF = 0.0f, f_vMEAS = 0.0f, f_error = 0.0f, f_out = 0.0f;

// memorias dos controladores
float f_integral = 0.0f;          // integral (PI, PID)
float f_deriv_Memory = 0.0f;      // erro anterior (D do PID)
float e1 = 0.0f, e2 = 0.0f;       // erros anteriores (AA, API, DB)
float u1 = 0.0f, u2 = 0.0f;       // saidas anteriores (AA, API, DB)

// Zera todos os estados (ao trocar de modo)
void resetStates() {
  f_integral = 0.0f;
  f_deriv_Memory = 0.0f;
  e1 = 0.0f; e2 = 0.0f;
  u1 = 0.0f; u2 = 0.0f;
}

// ---------------- SETUP ----------------
void setup() {
  Serial.begin(115200);
  pinMode(A7, INPUT); pinMode(A6, INPUT); pinMode(A5, INPUT);
  pinMode(io_PIN_A0, INPUT_PULLUP); pinMode(io_PIN_A1, INPUT_PULLUP); pinMode(io_PIN_A2, INPUT_PULLUP);
  pinMode(io_PIN_D12, OUTPUT);

  TCCR1B = TCCR1B & B11111000 | B00000001; // PWM 31372.55 Hz (pinos 9,10)

  pinMode(pwm_PIN_VNN, OUTPUT);
  pinMode(pwm_PIN_VDD, OUTPUT);
  analogWrite(pwm_PIN_VNN, 255 >> 1);      // fonte -5Vcc
  pinMode(pwm_PIN_Vin_SallenKey, OUTPUT);

  Serial.println(">Modo:" + s_modo);
  Serial.println("Comandos: SC P PI PID AA API DB");
}

// ---------------- LOOP ----------------
void loop() {
  int i_analog_Vo = 0, i_analog_VgK = 0, i_analog_Vg = 0, PWMval = 0;
  int i_perc_Vo = 0, i_perc_VgK = 0, i_perc_Vg = 0;
  int i_PWMStepVal = 0;

  // ---------- Recebe comando de TEXTO pelo serial ----------
  while (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    cmd.toUpperCase();
    if (cmd == "SC" || cmd == "P" || cmd == "PI" ||
        cmd == "PID" || cmd == "AA" || cmd == "API" || cmd == "DB") {
      s_modo = cmd;
      resetStates();
      Serial.println(">Modo:" + s_modo);
    }
  }

  // ---------- Push-button A2: habilita PWM manual ----------
  if (!digitalRead(io_PIN_A2) && !b_printOnce) {
    b_enableKey_PWM = !b_enableKey_PWM;
    b_printOnce = true;
  }
  if (digitalRead(io_PIN_A2)) b_printOnce = false;
  digitalWrite(LED_BUILTIN, b_enableKey_PWM);

  if (b_enableKey_PWM) {
#define DEF_STEP_PWM 100
    if (!digitalRead(io_PIN_A0)) {
      if (i_PWMStepVal != DEF_STEP_PWM) { i_PWMStepVal = DEF_STEP_PWM; by_WritePWMState = 0; }
    } else {
      if (by_WritePWMState == 5) { i_PWMStepVal = 0; by_WritePWMState = 0; }
    }
    if (by_WritePWMState == 0) { analogWrite(pwm_PIN_Vin_SallenKey, i_PWMStepVal); by_WritePWMState = 5; }
  }

  // ---------- Referencia via botao A1 ----------
  if (!digitalRead(io_PIN_A1)) f_vREF = 0.500f;   // 50%
  else                         f_vREF = 0.000f;

  // ---------- Amostragem temporizada ----------
  myTime = millis();
  if (myTime - myOldTime >= ci_SampleTime) {
    myOldTime = myTime;
    digitalWrite(io_PIN_D12, HIGH);

    // Leituras analogicas (0-1023 -> 0-1000 -> 0.0-1.0)
    i_analog_Vg  = analogRead(analog_PIN_A5_Vg);
    i_analog_Vo  = analogRead(analog_PIN_A6_Vo);
    i_analog_VgK = analogRead(analog_PIN_A7_VgK);
    i_perc_Vo  = map(i_analog_Vo,  0, 1023, 0, 1000);
    i_perc_VgK = map(i_analog_VgK, 0, 1023, 0, 1000);
    i_perc_Vg  = map(i_analog_Vg,  0, 1023, 0, 1000);
    f_vMEAS = (float)(i_perc_Vo / 1000.0f);

    analogWrite(pwm_PIN_LED1, map(i_perc_Vo, 0, 1000, 0, 255));
    analogWrite(pwm_PIN_LED2, map(i_perc_Vg, 0, 1000, 0, 255));

    // ================= CALCULO DO CONTROLADOR =================
    f_error = (f_vREF - f_vMEAS);

    if (s_modo == "P") {
      // ---- Proporcional ----
      f_out = Kp_P * f_error;
    }
    else if (s_modo == "PI") {
      // ---- P + I ----
      f_integral += Ki_PI * cf_T * f_error;
      f_integral  = constrain(f_integral, DEF_INT_LOWER, DEF_INT_UPPER);
      f_out = Kp_PI*f_error + f_integral;
    }
    else if (s_modo == "PID") {
      // ---- P + I + D ----
      f_integral += Ki_PID * cf_T * f_error;
      f_integral  = constrain(f_integral, DEF_INT_LOWER, DEF_INT_UPPER);
      float deriv = (f_error - f_deriv_Memory) * (Kd_PID / cf_T);
      f_out = Kp_PID*f_error + f_integral + deriv;
      f_deriv_Memory = f_error;
    }
    else if (s_modo == "AA") {
      // ---- Avanco (eq. diferencas): u = -a1*u1 + b0*e + b1*e1 ----
      float u = -AV_a1*u1 + AV_b0*f_error + AV_b1*e1;
      e1 = f_error;
      u1 = u;
      f_out = u;
    }
    else if (s_modo == "API") {
      // ---- Avanco+PI (eq. diferencas 2a ordem) ----
      float u = -API_a1*u1 - API_a2*u2 + API_b0*f_error + API_b1*e1 + API_b2*e2;
      e2 = e1;  e1 = f_error;
      u2 = u1;  u1 = u;
      f_out = u;
    }
    else if (s_modo == "DB") {
      // ---- Dead-Beat (eq. diferencas 2a ordem) ----
      float u = -DB_a1*u1 - DB_a2*u2+ DB_b0*f_error + DB_b1*e1 + DB_b2*e2;
      e2 = e1;  e1 = f_error;
      u2 = u1;  u1 = u;
      f_out = u;
    }
    else {  // "SC" - sem controle (referencia direta)
      f_out = f_vREF;
    }

    // Satura saida 0.0-1.0 e converte para PWM 0-255
    f_out = constrain(f_out, 0.0f, 1.0f);
    PWMval = (int)(255.0f * f_out);

    if (!b_enableKey_PWM)
      analogWrite(pwm_PIN_Vin_SallenKey, PWMval);

    // ---------- Plotter Serial ----------
    Serial.println(">Vo:"   + String(i_perc_Vo));
    Serial.println(">Vg:"   + String(i_perc_Vg));
    Serial.println(">VgK:"  + String(i_perc_VgK));
    Serial.println(">Erro:" + String((int)(f_error * 1000.0f)));
    Serial.println(">Ref:"  + String((int)(f_vREF * 1000.0f)));
    Serial.println(">Min:0");
    Serial.println(">Max:1000");

    digitalWrite(io_PIN_D12, LOW);
  }
  digitalWrite(pwm_PIN_VDD, !digitalRead(pwm_PIN_VDD));
}