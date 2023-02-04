/*
  Limitador com o arudino nano

  LEMBRAR DE ESCREVER AS VERSOES AQUI

*/

#include <Servo.h>
#define A_sensor A2
#define V_sensor A1
#define Radio 5 //D5
#define p_esc 3 //D3
#define chave 2 //D2
#define n 50
Servo esc;

/*Prototipos das funções*/
float tensao();
float corrente();
float potencia();
long moving_average(int original);
void exibir();
int radio();

/* ==== Variaveis para a função exibir ====*/
float volt;

/* =========================================*/
/* ==== Variaveis para a função da media movel*/

String Status = " ";
int leitura;
int original;              //recebe o valor de AN0
int Val_filtado;          //recebe o valor original filtrado

int       numbers[n];
//=============================================/

// Parâmetros Filtro 1a ordem da corrente
float Tau = 1;
float out_ant = 0;
float Ts = 13e-3; //Tempo de amostragem



// Parâmetros Filtro 1a ordem da tensao
float Tau_2 = 0.5;
float out_ant_2 = 0;
float Ts_2 = 0.1; //Tempo de amostragem

//Variaveis de controle
int sp_sup = 600; //set point superior (Watts)
int sp_inf = 500; //set point inferior (Watts)
int ref_radio_min = 150; //throtlle min para ativar controle (PWM)
int d_sp_acel = 1; // delta de variação do controle aceleração (PWM)
int d_sp_desacel = 3; // delta de variação do controle descida (PWM)

#define chave_ref 150 // valor da chave para desativar o sistema

int limit_sup_for = 132;
int limit_inf_for = 140;
int pwm;

//constante
float pwm_saida;

//const unsigned char PS_16 = (1 << ADPS2);
//const unsigned char PS_32 = (1 << ADPS2) | (1 << ADPS0);
//const unsigned char PS_64 = (1 << ADPS2) | (1 << ADPS1);
//const unsigned char PS_128 = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
// 


void setup() {
  Serial.begin(9600);
  esc.attach(p_esc, 1000, 2000);
  pinMode(Radio, INPUT);
  pinMode(chave,INPUT);
//  ADCSRA &= ~PS_128;
//
//  ADCSRA |= PS_64;
}
//filtro para a corrente
float filter1order(float input) {

  float out = (Tau * out_ant + Ts * input) / (Tau + Ts);
  out_ant = out;
  return out;

}
// filtro para a tensao
float filter1order_2(float input_2) {

  float out_2 = (Tau_2 * out_ant_2 + Ts_2 * input_2) / (Tau_2 + Ts_2);
  out_ant_2 = out_2;
  return out_2;

}

long moving_average(int original)
{

  //desloca os elementos do vetor de média móvel
  for (int i = n - 1; i > 0; i--) numbers[i] = numbers[i - 1];

  numbers[0] = original; //posição inicial do vetor recebe a leitura original

  long acc = 0;          //acumulador para somar os pontos da média móvel

  for (int i = 0; i < n; i++) acc += numbers[i]; //faz a somatória do número de pontos


  return acc / n; //retorna a média móvel


} //end moving_average

float tensao() {
//    return analogRead(V_sensor); // sem calibração
  return (analogRead(V_sensor)*0.03833704 - 0.25925926); // colocar a calibração
//  return (analogRead(V_sensor)*26.516 - 0.2072); // colocar a calibração
  //  lembrando q o arduino vai ate 1023 e 5V
}

float corrente() {
  volt = (2.215 / 1023) * (float) analogRead(A_sensor);
  volt = volt - 2.5 + 0.007;
  //  return (volt/0.04); // sem calibração
  return abs(filter1order( (volt / 0.04) * 6.53594771 + 226.92156862 )); // colocar a calibração
}

float potencia() {

  //aqui pode-se colocar filtro de media eo q vc quiser
  // so cuidado com o tempo q vc irá colocar
  return tensao() * corrente();

}

int radio() {

  if (pulseIn(Radio, HIGH,0) < 1109) return 0;
  return  map(pulseIn(Radio, HIGH), 1109, 1930, 0, 180); //calibrar com o radio;
}
int chave_on() {
  return map(pulseIn(chave, HIGH,0), 940, 1520, 0, 180); //calibrar com o radio;
}

void exibir(int PWM) {

  Serial.print(F("V: "));
  Serial.print(tensao(), 3);
  Serial.print("\t ");
  Serial.print(F("A: "));
  Serial.print(corrente(), 3);
  Serial.print("\t ");
  Serial.print(F("P: "));
  Serial.print(potencia(), 3);
  Serial.print("\t ");
  Serial.print(F("PWM Entrada: "));
  Serial.print((radio()));

  Serial.print(F("\t "));
  Serial.print(F("PWM Saida: "));
  Serial.print((pwm_saida));

  Serial.print("\t ");
  Serial.print(F("Status: "));
  Serial.print((Status));

  Serial.println(" ");
}


float e_ant;
float i_ant;
float vm_ant;
float d_ant;
float PID( float ref, float vm, float Kp, float Ki, float Kd, float Tdf, float Tsamp ) {

  float erro = ref - vm;

  // Proporcional
  float proporcional = Kp * erro;

  // Integral
  float integral = i_ant + Ki * 1/2 * (e_ant + erro) * Tsamp;

  // Derivativo
  float derivativo = (-Kd * (vm - vm_ant) + Tdf * d_ant) / (Tdf + Tsamp);


  float saida = proporcional + integral + derivativo;
  
  if (saida > 100) {
    saida = 100;
    integral = i_ant;
  }
  else if (saida < 0){
    saida = 0;
    integral = i_ant;
  }

  i_ant = integral;
  e_ant = erro;
  vm_ant = vm;
  d_ant = derivativo;
  
  return saida;
}

long lastMillis = 0;
long loops = 0;


void loop() {
 
  float ref = (float) radio();
  ref = ref/180 * 20;

  float pot = potencia();
  
  float vm = PID(ref, pot, 1, 10, 1, 1, 13e-3);


  long currentMillis = millis();
  loops++;
  
  /* By doing complex math, reading sensors, using the "delay" function,
  *  etc you will increase the time required to finish the loop,
  *  which will decrease the number of loops per second.
  */

//  if(currentMillis - lastMillis > 1000){
//    Serial.print("Loops last second:");
//    Serial.println(loops);
//    
//    lastMillis = currentMillis;
//    loops = 0;
//  }
  
  Serial.print(ref);
  Serial.print(',');
  Serial.print(vm/100*180);
  Serial.print(',');
  Serial.println(pot);

  esc.write((int) (vm/100*180));

}


  // put your setup code here, to run once:
