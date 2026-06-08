//
// Codigo fonte de testes da PCB - Sallen-Key-Duino
// (c) Luiz Daniel - 2021
//
// Ele consiste de rotinas essenciais para execução e testes das respostas
// da planta de segunda ordem, visualização no ambiente Arduino.
// 
// Com base neste código é possível realizar o controlador desejado
// para a planta de interesse. (PD, PID, PI, RST, etc...)
// 

// pins for the PWMs
// The Arduino uses Timer 0 internally for the millis() and delay() functions, 
// so be warned that changing the frequency of this timer will cause those functions to be erroneous. 
// Using the PWM outputs is safe if you don't change the frequency, though.
// Timer0 => pinos 5 && 6
// Timer1 => pinos 9 && 10
// Timer2 => pinos 11 && 3
const int pwm_PIN_VDD = 6;
const int pwm_PIN_VNN = 10;
const int pwm_PIN_Vin_SallenKey = 9;
const int pwm_PIN_LED1 = 3;
const int pwm_PIN_LED2 = 11;
// pins for the IOs
const int io_PIN_D12  = 12;
const int io_PIN_A0 = 14;
const int io_PIN_A1 = 15;
const int io_PIN_A2 = 16;
// pins for the Analog Inputs
const int analog_PIN_A7_VgK = A7;
const int analog_PIN_A5_Vg = A5;
const int analog_PIN_A6_Vo = A6;

const int ci_SampleTime = 10;    // Milisseconds   10ms ~ 100Hz; 100ms ~ 10Hz
// Global Variables
bool b_enableKey_PWM = false, b_printOnce = false;
byte by_WritePWMState = 0;

unsigned long myTime, myOldTime = 0;

float f_integral_Memory = 0.0f, f_deriv_Memory = 0.0f;
float f_PI_output = 0.0f, f_vREF = 0.0f, f_vMEAS = 0.0f;
float f_error = 0.0f;
// Setup Code 
void setup() {
  // put your setup code here, to run once:
  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  pinMode(A7, INPUT); pinMode(A6, INPUT);  pinMode(A5, INPUT); 
  pinMode(io_PIN_A0, INPUT_PULLUP); pinMode(io_PIN_A1, INPUT_PULLUP);  pinMode(io_PIN_A2, INPUT_PULLUP); 
  pinMode(io_PIN_D12, OUTPUT);
/*********************************************************************************
 * Configurações para o Timer 1 ==>> Pinos 9 && 10
TCCR1B = TCCR1B & B11111000 | B00000001; // set timer 1 divisor to 1 for PWM frequency of 31372.55 Hz
TCCR1B = TCCR1B & B11111000 | B00000010; // for PWM frequency of 3921.16 Hz
TCCR1B = TCCR1B & B11111000 | B00000011; // for PWM frequency of 490.20 Hz (The DEFAULT)
TCCR1B = TCCR1B & B11111000 | B00000100; // for PWM frequency of 122.55 Hz
TCCR1B = TCCR1B & B11111000 | B00000101; // for PWM frequency of 30.64 Hz
*********************************************************************************/
  TCCR1B = TCCR1B & B11111000 | B00000001; // for PWM frequency of 31372.55 Hz
/*********************************************************************************
 * Configurações para o Timer 2 ==>> pinos 11 && 3
TCCR2B = TCCR2B & B11111000 | B00000001; // set timer 1 divisor to 1 for PWM frequency of 31372.55 Hz
TCCR2B = TCCR2B & B11111000 | B00000010; // for PWM frequency of 3921.16 Hz
TCCR2B = TCCR2B & B11111000 | B00000011; // for PWM frequency of 490.20 Hz (The DEFAULT)
TCCR2B = TCCR2B & B11111000 | B00000100; // for PWM frequency of 122.55 Hz
TCCR2B = TCCR2B & B11111000 | B00000101; // for PWM frequency of 30.64 Hz
*********************************************************************************/
  //TCCR2B = TCCR1B & B11111000 | B00000001; // for PWM frequency of 31372.55 Hz
  pinMode(pwm_PIN_VNN, OUTPUT); 
  pinMode(pwm_PIN_VDD, OUTPUT);
  analogWrite(pwm_PIN_VNN, 255>>1);   // Essencial para a fonte de -5Vcc
  pinMode(pwm_PIN_Vin_SallenKey, OUTPUT);
  // send an intro:
//  Serial.println("Envie o valor do PWM (0-255) e pressione [ENTER]");
  Serial.println("i_analog_Vo i_analog_Vg i_analog_VgK ERROR LOWER UPPER");  // Para os Titulos do Plotter Serial
  Serial.println();
}
// Loop Code
void loop() {
  int i_analog_Vo = 0, i_analog_VgK = 0, i_analog_Vg = 0, PWMval = 0;
  int i_PWMStepVal = 0;
  int i_perc_Vo = 0, i_perc_VgK = 0, i_perc_Vg = 0;
 

  String s_KEY_EN;
  // Parte que recebe o valor de PWM a partir da entrada Serial
  while (Serial.available() > 0) {
        PWMval = Serial.parseInt();
        if (Serial.read() == '\n') {
        PWMval = constrain(PWMval, 0, 255);
        analogWrite(pwm_PIN_Vin_SallenKey, PWMval);     
    }
  }
  //***********************************************************
  // Push-Button A2 Habilita e desabilita a modificação do PWM 
  if (!digitalRead(io_PIN_A2) && !b_printOnce )
  {
    b_enableKey_PWM = !b_enableKey_PWM;

    b_printOnce = true;
    if (b_enableKey_PWM)
      //Serial.println("K_ENABLED");
      s_KEY_EN = "K_ENABLED";
      else
      s_KEY_EN = "K_DISABLED";
      //Serial.println("KEY PWM DISABLED");
  }
  if (digitalRead(io_PIN_A2))
  {
     b_printOnce = false;
  }
  digitalWrite(LED_BUILTIN, b_enableKey_PWM);
  if (b_enableKey_PWM){
#define DEF_STEP_PWM    100

    if (!digitalRead(io_PIN_A0))
    { if (i_PWMStepVal != DEF_STEP_PWM)
      {  i_PWMStepVal = DEF_STEP_PWM;
         by_WritePWMState = 0;
      }
    } else
    { if (by_WritePWMState == 5)
      { i_PWMStepVal = 0;
        by_WritePWMState = 0;
      }
    }
    if (by_WritePWMState == 0)
    {
      analogWrite(pwm_PIN_Vin_SallenKey, i_PWMStepVal);      
      by_WritePWMState = 5;
    }    
  }
  if (!digitalRead(io_PIN_A1))
  {
    f_vREF = 0.500f;    // 50.0% em valor percentual
  } else
  {
    f_vREF = 0.000f;    // 0.0% em valor percentual
  } 

//***********************************************************
//Temporiza a execução sem travar o código
 myTime = millis();
 if (myTime - myOldTime >= ci_SampleTime)
 {
  myOldTime = myTime;
  
  digitalWrite(io_PIN_D12, HIGH);
  
//***********************************************************
// Leituras analógicas - Escala de 0 ~ 1023
  i_analog_Vg =  analogRead(analog_PIN_A5_Vg);  
  i_analog_Vo = analogRead(analog_PIN_A6_Vo);
  i_analog_VgK = analogRead(analog_PIN_A7_VgK);
//***********************************************************
// Leituras analógicas - Converte para uma escala de 100.0% === 1000
  i_perc_Vo = map(i_analog_Vo, 0, 1023, 0, 1000);
  i_perc_VgK = map(i_analog_VgK, 0, 1023, 0, 1000);
  i_perc_Vg = map(i_analog_Vg, 0, 1023, 0, 1000);

  f_vMEAS = (float)(i_perc_Vo/1000.0f);
  
//***********************************************************
// Envio de PWM para sinalização através dos 2 LEDs na placa  
  analogWrite(pwm_PIN_LED1, map(i_perc_Vo, 0, 1000, 0, 255));
  analogWrite(pwm_PIN_LED2, map(i_perc_Vg, 0, 1000, 0, 255));

//***********************************************************
// Realiza um P bem simples....
// const float cf_Kp = 8.00f, cf_Ki = 0.000f, cf_Kd = 0.0000f;
// Realiza um I bem simples....
// const float cf_Kp = 00.00f, cf_Ki = 0.039f, cf_Kd = 0.0000f;
// Realiza um P+I bem simples....
// const float cf_Kp = 0.502f, cf_Ki = 0.0500f, cf_Kd = 0.0000f;
// Realiza um PID bem simples....
//const float cf_Kp = 0.00f, cf_Ki = 0.0350f, cf_Kd = 0.0f;

// Calculado no MATLAB
// fc = 2.0Hz; Margem de Fase = 76.8 graus.
// const float cf_Kp = 0.6461f, cf_Ki = 0.0631f, cf_Kd = 11.35f;

// Calculado no MATLAB
// fc = 2.0Hz; Margem de Fase = 60 graus.
// const float cf_Kp = 1.0239f, cf_Ki = 0.1000f, cf_Kd = 12.5f;

// Calculado no MATLAB
// fc = 2.0Hz; Margem de Fase = 60 graus.
// const float cf_Kp = 0.6461f, cf_Ki = 0.0631f, cf_Kd = 11.3572f;

 const float cf_Kp = 0.5461f, cf_Ki = 0.0431f, cf_Kd = 11.3572f;
// const float cf_Kp = 0.6461f, cf_Ki = 0.0631f, cf_Kd = 0.0f;

// Calculado no MATLAB
// fc = 3.38Hz; Margem de Fase = 47 graus.
// const float cf_Kp = 3.2379f, cf_Ki = 0.1000f, cf_Kd = 18.5f;


// Os coeficientes acima devem ser ajustados conforme as 
// características do Sallen-Key (e conforme as especificações dinâmicas desejadas)!
//
#define   DEF_INTEGRAL_UPPER    1.0f
#define   DEF_INTEGRAL_LOWER    -1.0f
  f_error = (f_vREF-f_vMEAS);
  //=======================================================================================
  f_integral_Memory += f_error*cf_Ki;
  f_integral_Memory = constrain(f_integral_Memory, DEF_INTEGRAL_LOWER, DEF_INTEGRAL_UPPER);
  //=======================================================================================
  f_PI_output = f_error*(cf_Kp) + f_integral_Memory + (f_error - f_deriv_Memory)*cf_Kd;
  //=======================================================================================
  f_deriv_Memory = f_error;
  f_PI_output = constrain(f_PI_output, 0.0f, 1.0f);
  PWMval = (int)(255.0f*f_PI_output);

 if (!b_enableKey_PWM) 
  analogWrite(pwm_PIN_Vin_SallenKey, PWMval);  

//***********************************************************
// Exibe o valor no "Plotter Serial" do Ambiente Arduino
  //Serial.println(String(i_analog_Vo)+" "+String(i_analog_Vg)+" "+String(i_analog_VgK)+" 0 1023");

  //Serial.println(String(i_perc_Vo)+" "+String(i_perc_Vg)+" "+String(i_perc_VgK)+" "+String((int)(f_error*1000.0f))+" 0 1000");

  Serial.println(">Vo:" + String(i_perc_Vo));
  Serial.println(">Vg:" + String(i_perc_Vg));
  Serial.println(">VgK:" + String(i_perc_VgK));
  Serial.println(">Erro:" + String((int)(f_error * 1000.0f)));
  Serial.println(">Min:0");
  Serial.println(">Max:1000");
  
  // wait a bit for the analog-to-digital converter to stabilize after the last
  // reading:
  digitalWrite(io_PIN_D12, LOW);
 }
  digitalWrite(pwm_PIN_VDD, !digitalRead(pwm_PIN_VDD));
}
