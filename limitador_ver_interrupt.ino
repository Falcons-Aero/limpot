#include<Arduino.h>
#include <Servo.h>

#define A_sensor A2 //sensor de corrente
#define V_sensor A1 //sensor de tensao
#define Radio 5 // D5
#define p_esc 3 // D3
#define chave 2
#define n 50 // numero de amostras p/ o filtro de media movel
Servo esc; // cria o objeto do esc

//
//// Variaveis para a leitura do sinal do radio
//unsigned long counter_1, counter_2, contagem_Atual;
//byte last_Ch1_state, last_Ch2_state;
//int Input_chave_radio, Radio_Input;

// Variaveis para a media movel
int leitura;
int original;
int val_filtrado;
int numbers [50]; // valores para a media movel

// filtro de 1 º ordem

float Tau = 1;
float out_ant = 0;
float Ts = 13e-3; // tempo de amostragem

//Variaveis do controle
int sp_sup = 600; //set point superior (Watts)
int sp_inf = 500; //set point inferior (Watts)
int ref_radio_min = 150; //throtlle min para ativar controle (PWM)

//Variaveis para o PID
float saida;
float erro;
float integral;
float proporcional;
float derivativo;
float i_ant;
float e_ant;
float vm_ant;
float d_ant;

//variaveis para o radio
float pot;
int ref;
int vm;
float volt;

volatile long StartTime = 0;
volatile long CurrentTime = 0;
volatile long Pulses = 0;
int PulseWidth = 0;



void setup(){
Serial.begin(9600); // 250000
esc.attach(p_esc,1000,2000);
// configuração de leitura 

pinMode(Radio, INPUT);
  pinMode(chave,INPUT);    

    attachInterrupt(digitalPinToInterrupt(chave),chavao,CHANGE);

}
float filter1order(float input)
{

    float out = (Tau * out_ant + Ts * input) / (Tau + Ts);
    out_ant = out;
    return out;
}

long moving_average(int original)

{

    // desloca os elementos do vetor de média móvel
    for (int i = n - 1; i > 0; i--)
        numbers[i] = numbers[i - 1];

    numbers[0] = original; // posição inicial do vetor recebe a leitura original

    long acc = 0; // acumulador para somar os pontos da média móvel

    for (int i = 0; i < n; i++)
        acc += numbers[i]; // faz a somatória do número de pontos

    return acc / n; // retorna a média móvel

} // end moving_average

float tensao()
{
    return analogRead(V_sensor) * 0.03703704 - 0.25925926; 
}

float corrente()
{
    volt = (2.215 / 1023) * (float)analogRead(A_sensor);
    volt = volt - 2.5 + 0.007;
    //  return (volt/0.04); // sem calibração
    return filter1order((volt / 0.04) * 6.53594771 + 226.92156862); // colocar a calibração
}

float potencia(){
    return tensao() * corrente();
}


float PID(float ref, float vm, float Kp, float Ki, float Kd, float Tdf, float Tsamp ) {
 
    erro = ref - vm;

  // Proporcional
    proporcional = Kp * erro;

  // Integral
    integral = i_ant + Ki * 1/2 * (e_ant + erro) * Tsamp;

  // Derivativo
    derivativo = (-Kd * (vm - vm_ant) + Tdf * d_ant) / (Tdf + Tsamp);

float saida = proporcional + integral + derivativo;


if (saida > 100){
    saida = 180;
    integral = i_ant;
}
else if(saida<0){
    saida = 0;
    integral = i_ant;
}

i_ant = integral;
e_ant = erro;
vm_ant = vm;
d_ant = derivativo;
  
return saida;
}



int radio() {

  if (pulseIn(Radio, HIGH,0) < 1109) return 0;
  return  map(pulseIn(Radio, HIGH), 1109, 1930, 0, 180); //calibrar com o radio;
}

void chavao(){
 CurrentTime = micros();
  if (CurrentTime > StartTime){
    Pulses = CurrentTime - StartTime;
    StartTime = CurrentTime;
  }
//  Serial.println("BANANA");
}

void loop(){


while (Pulses>1400){ //tirado da bunda :D
  Serial.print("Toma-lhe!!!!");
  esc.write(radio());
  Serial.println(radio());
}

//radio, potencia, KP,KI,KD,tdf,Tempo de amostragem
pot = potencia();
vm = PID(ref,pot,1,10,1,1,13e-3);
//
//Serial.print(ref); // radio azul
//Serial.print(',');
//Serial.print(vm/100*180); // limitador azul
//Serial.print(',');
//Serial.println(pot);
//  
//esc.write((int) (vm/100*180));
//
Serial.println(radio());
Serial.print(",");
Serial.println(Pulses);



//Serial.print("CH8");
//Serial.println(Input_chave_radio);

}