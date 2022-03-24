#include "Arduino.h"
#include "Motores.h"

  //Para não dar erro de sintaxe no Visual Studio
#include "esp32-hal-ledc.h"


  // Efetua as configurações iniciais quando a classes é intanciada
Motores::Motores(){
  /*
    Quando a classe for instanciada serão feitas as configurações:
      *Ponte H
      *Encoder
      *Controlador PID
  */

  config_ponte_H();
  config_encoder();
  configPID();

  millisOld = millis() + 1;
}

  // Efetua as configurações Ponte H
void Motores::config_ponte_H(){
  /*
  IN1 IN2 Estado 
    0   0  desligado
    0   1  sentido 1
    1   0  sentido 2
    1   1  freio
*/

    //Configura as saidas digitais para a Ponte H
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);
  pinMode(ENA,OUTPUT);
  pinMode(ENB,OUTPUT);

    //Configura os canais do PWM no ESP32
  ledcSetup(chanel_pwm_motor_dir, frequencia_pwm, resolucao_pwm);
  ledcAttachPin(ENB, chanel_pwm_motor_dir);
  ledcWrite(ENB, 0);

  ledcSetup(chanel_pwm_motor_esq, frequencia_pwm, resolucao_pwm);
  ledcAttachPin(ENA, chanel_pwm_motor_esq);
  ledcWrite(ENA, 0);
}
  
  // Efetua as configurações do Encoder
void Motores::config_encoder(){
  /*
  Cores dos fios do motor/encoder
  Vermelho - M1 Motor (GND)
  Preto    - GND Encoder
  Amarelo  - Encoder C1
  Verde    - Encoder C2
  Azul     - 3.3V Encoder
  Branco   - M1 motor (+)
*/

    //Configura as entradas para o encoder
  pinMode(Enc_Dir_C1, INPUT);
  pinMode(Enc_Esq_C1, INPUT);  
  pinMode(Enc_Dir_C2, INPUT);
  pinMode(Enc_Esq_C2, INPUT);

    // Devido a problemas de ponteiros em classes essa configuração está sendo realizada no void setup()
  //attachInterrupt(digitalPinToInterrupt(Enc_Dir_C1), *calcula_pulsos_roda_direita(), CHANGE);
  //attachInterrupt(digitalPinToInterrupt(Enc_Esq_C1), *calcula_pulsos_roda_esquerda(), CHANGE);
}

  // Sintonia do Controlador
void Motores::configPID(){
  float kp_dir, ti_dir, td_dir; //Variáveis de sintonia
  float kp_esq, ti_esq, td_esq; //Variáveis de sintonia

    // Sintonia por Ziegler e Nichols
  kp_dir = (1.2 * T) / (theta_dir * K_dir);
  ti_dir = 2 * theta_dir;
  td_dir = 0.5 * theta_dir;

  kp_esq = (1.2 * T) / (theta_esq * K_esc);
  ti_esq = 2 * theta_esq;
  td_esq = 0.5 * theta_esq;

    // Ajuste
  kp_dir *= ajs_ganho_dir[0];
  ti_dir *= ajs_ganho_dir[1];
  td_dir *= ajs_ganho_dir[2];

  kp_esq *= ajs_ganho_esq[0];
  ti_esq *= ajs_ganho_esq[1];
  td_esq *= ajs_ganho_esq[2];

  q0_dir = kp_dir * (1 + (T/(2*ti_dir)) + (td_dir/T));
  q1_dir = -kp_dir * (1 - (T/(2*ti_dir)) + ((2*td_dir) / T));
  q2_dir = kp_dir * td_dir / T;

  q0_esq = kp_esq * (1 + (T/(2*ti_esq)) + (td_esq/T));
  q1_esq = -kp_esq * (1 - (T/(2*ti_esq)) + ((2*td_esq) / T));
  q2_esq = kp_esq * td_esq / T;
}

  // Contagem de pulsos da roda direita - uso da interrupção externa
void Motores::calcula_pulsos_roda_direita(){

  c1_dir = digitalRead(Enc_Dir_C1);           //Lê C1 (pino de interrupção)   
          
  if ((c1_dir_old == LOW) && c1_dir == HIGH){ // se C1 for para ligado
    boolean c2_dir = digitalRead(Enc_Dir_C2); 

      //Reverse
    if (c2_dir == LOW && sentido_dir)                 
      sentido_dir = false; 
      
      //Forward
    else if (c2_dir == HIGH && !sentido_dir)           
      sentido_dir = true;  
  }
  
  c1_dir_old = c1_dir;                      

  sentido_dir ? pulsos_motor_dir++ : pulsos_motor_dir--; 
}

  // Contagem de pulsos da roda esquerda - uso da interrupção externa
void Motores::calcula_pulsos_roda_esquerda(){

  c1_esq = digitalRead(Enc_Esq_C1); //Lê C1 (pino de interrupção)  
          
  if ((c1_esq_old == LOW) && c1_esq == HIGH){ // se C1 for para ligado
    boolean c2_esq = digitalRead(Enc_Esq_C2); 

      //Reverse
    if (c2_esq == LOW && sentido_esq)                 
      sentido_esq = false; 
      
      //Forward
    else if (c2_esq == HIGH && !sentido_esq)           
      sentido_esq = true;  
  }
  
  c1_esq_old = c1_esq;                      

  sentido_esq ? pulsos_motor_esq-- : pulsos_motor_esq++; 
}

  // Calcula as velocidades
void Motores::calcula_velocidade(){
  dt = (millis() - millisOld) / (float)1000;
  millisOld = millis();

    // Calcula a velocidade de cada roda
  vel_motor_dir = const_dir * (float)pulsos_motor_dir / dt;     // velocidade (m/s)
  vel_motor_esq = const_esq * (float)pulsos_motor_esq / dt;     // velocidade (m/s)

  pulsos_motor_dir = 0;
  pulsos_motor_esq = 0;

    // Calcula velocidade - Linear, Angular e Ângulo de ginada (Yaw)
  vel_linear = (vel_motor_dir + vel_motor_esq) / 2;
  vel_angular = (vel_motor_dir - vel_motor_esq) / dist_entre_rodas;
  yaw += vel_angular * dt * const_conv_rad_grau;

    // Mantem o angulo do robô entre os parâmetros de (0 - 360) 
  //if(yaw < 0) yaw = 360 - yaw;
  //if(yaw > 360) yaw = yaw - 360;
}

  // Efetua o controle da velocidade do motor com base no setpoint calculado
void Motores::PID(){
  /*
    São usados dois controladores um para cada motor
  */

    //Atualiza o vetor de erros do controlador
  err_dir[2] = err_dir[1];
  err_esq[2] = err_esq[1];
  err_dir[1] = err_dir[0];
  err_esq[1] = err_esq[0];
  err_dir[0] = vel_motor_dir_setpoint - vel_motor_dir;
  err_esq[0] = vel_motor_esq_setpoint - vel_motor_esq;

    //Calcula novo valor para a saída do controlador
  pwm_motor_dir = pwm_motor_dir + (q0_dir * err_dir[0]) + (q1_dir * err_dir[1]) + (q2_dir * err_dir[2]);
  pwm_motor_esq = pwm_motor_esq + (q0_esq * err_esq[0]) + (q1_esq * err_esq[1]) + (q2_esq * err_esq[2]);

    //Mantem os valores do PWM na faixa permitida
  if(estado_motor_dir == STOP) {pwm_motor_dir = 0;}
  else if(estado_motor_dir == POSITIVO){
    pwm_motor_dir = (pwm_motor_dir > 255) ? 255 : pwm_motor_dir;
    pwm_motor_dir = (pwm_motor_dir < 0) ? 0 : pwm_motor_dir;

  } else {
    pwm_motor_dir = (pwm_motor_dir > 0) ? 0 : pwm_motor_dir;
    pwm_motor_dir = (pwm_motor_dir < -255) ? -255 : pwm_motor_dir;
  }

  if(estado_motor_esq == STOP) {pwm_motor_esq = 0;}
  else if(estado_motor_esq == POSITIVO){
    pwm_motor_esq = (pwm_motor_esq > 255) ? 255 : pwm_motor_esq;
    pwm_motor_esq = (pwm_motor_esq < 0) ? 0 : pwm_motor_esq;
  } else {

    pwm_motor_esq = (pwm_motor_esq > 0) ? 0 : pwm_motor_esq;
    pwm_motor_esq = (pwm_motor_esq < -255) ? -255 : pwm_motor_esq;
  }

    //Seta novo duty cicle do PWM
  ledcWrite(chanel_pwm_motor_dir, pwm_motor_dir);
  ledcWrite(chanel_pwm_motor_esq, pwm_motor_esq);
}

  // Recebe os comandos na forma de String ( float | float '\n' )
void Motores::recebeComando(String comando){
  Serial.println(comando);

    // 0 - v_lin || 1 - v_ang
  float vel[2];
  byte count_comando = 0;
  bool decodificando = true;
  String msg = "";

  byte i = -1;
  while(decodificando){
    i++;

    if(comando[i] == '\0'){
      vel[count_comando] = msg.toFloat();
      decodificando = false;

    } else if(comando[i] == '|'){
      vel[count_comando] = msg.toFloat();
      count_comando++;
      msg = "";
      
    } else {
      msg += comando[i];
    }
  }

  setVelocidade(vel[0], vel[1]);
}

  //Calcula a velocidade de cada motor com base nas velocidades (v_lin, v_ang) [linear e angular] passadas
float Motores::setVelocidade(float v_lin, float v_ang){
  vel_motor_dir_setpoint = v_lin + (v_ang * dist_entre_rodas) / 2;
  vel_motor_esq_setpoint = v_lin - (v_ang * dist_entre_rodas) / 2;

    // Controla direção dos motores
  if(vel_motor_dir_setpoint > 0){
    digitalWrite(IN1,LOW);
    digitalWrite(IN2,HIGH);
      estado_motor_dir = POSITIVO;

  } else if(vel_motor_dir_setpoint < 0){
    digitalWrite(IN1,HIGH);
    digitalWrite(IN2, LOW);
    estado_motor_dir = NEGATIVO;

  } else {
    pwm_motor_dir = 0;
    digitalWrite(IN1,HIGH);
    digitalWrite(IN2, HIGH);
    estado_motor_dir = STOP;
  }

  if(vel_motor_esq_setpoint > 0){
    digitalWrite(IN3,LOW);
    digitalWrite(IN4,HIGH);
    estado_motor_esq = POSITIVO;

  } else if(vel_motor_esq_setpoint < 0) {
    digitalWrite(IN3,HIGH);
    digitalWrite(IN4, LOW);
    estado_motor_esq = NEGATIVO;

  } else {
    pwm_motor_esq = 0;
    digitalWrite(IN3,HIGH);
    digitalWrite(IN4, HIGH);
    estado_motor_esq = STOP;
  }
}

float Motores::getVelDireita(){
  return vel_motor_dir;
}

float Motores::getVelEsquerda(){
  return vel_motor_esq;
}

int Motores::getPWMEsquerda(){
  return pwm_motor_esq;
}

int Motores::getPWMDireita(){
  return pwm_motor_dir;
}

void Motores::getDados(float *v_lin, float *v_ang, float *yaw_ang){
  *v_lin = vel_linear;
  *v_ang = vel_angular;
  *yaw_ang = yaw;
}