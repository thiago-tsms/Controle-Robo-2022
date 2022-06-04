#ifndef DEADRECKONING_H
#define DEADRECKONING_H

#include <Arduino.h>



// ====================================================
//  --- --- --- CONFIGURAÇÃO DE COMPILAÇÃO --- --- ---
// ====================================================
#define Print_Dados_Pulsos false
#define Print_Dados_Velocidade false
#define Print_Dados_Velocidade_Angular_Linear false

 

  /* Definição dos pinos da Ponte H */
#define IN1   19
#define IN2   23
#define ENA   18    //Motor Esquerda (PWM)
#define IN3   5
#define IN4   13
#define ENB   12    //Motor Direita (PWM)

  /* Definição dos pinos dos Encoders */
#define Enc_Dir_C1 36  //Pino com Interrupção (ISR)
#define Enc_Dir_C2 39
#define Enc_Esq_C1 35  //Pino com Interrupção (ISR)
#define Enc_Esq_C2 34

class DeadReckoning {
  
  private:

// ===================================================
//  --- --- --- VARIÁVEIS DE CONFIGURAÇÃO --- --- ---
// ===================================================

  const float pi = 3.14159265359;
  const float resolucao_enc_dir = 473;      // resolução experimental(PPR)
  const float resolucao_enc_esc = 473;      // resolução experimental(PPR)

  const float raio_roda_dir = 0.067 / 2;    // diâmetro da roda (m)
  const float raio_roda_esq = 0.067 / 2;    // diâmetro da roda (m)
  const float dist_entre_rodas = 0.1796;    // distância entre as rodas (m) -- 19 O padrão

  const float T = 0.1;                  //Período de amostragem (seg)
  const float K_dir = 0.00401372549;    //Ganho motor direito
  const float K_esc = 0.00431372549;    //Ganho motor esquerdo
  const float thao_dir = 0.738;         //Constante de tempo (seg)
  const float thao_esq = 0.738;         //Constante de tempo (seg)
  const float theta_dir = 0.1;          //Atrazo (seg)
  const float theta_esq = 0.1;          //Atrazo (seg)
  const float ajs_ganho_dir[3] = {0.85, 0.9, 0.85};   //Ajuste de ganho kp, ti, td (será multiplicado, ajuste em %)
  const float ajs_ganho_esq[3] = {0.85, 0.9, 0.85};   //Ajuste de ganho kp, ti, td (será multiplicado, ajuste em %)

  const int chanel_pwm_motor_dir = 0;   //Canal usado para o PWM do motor direito
  const int chanel_pwm_motor_esq = 1;   //Canal usado para o PWM do motor esquerdo
  const int resolucao_pwm = 8;          //Resolução em bits do PWM - 8 bits (0-255)
  const int frequencia_pwm = 490;       //Fequência do PWM (Hhz)



// ===================================
//  --- --- --- VARIÁVEIS --- --- ---
// ===================================

    /* Setando a velocidade (linear, angular) */
  float vel_motor_dir_setpoint = 0;     //Velocidades de referência motor Direito
  float vel_motor_esq_setpoint = 0;     //Velocidades de referência motor Esquerdo
  enum EM {POSITIVO = 1, STOP = 0, NEGATIVO = -1};  // Possíveis estados do motor
  uint8_t estado_motor_dir = STOP;         //Estado do motor direito
  uint8_t estado_motor_esq = STOP;         //Estado do motor esquerdo

    /* Contagem de pulsos do encoder */
  volatile int pulsos_motor_dir = 0;  //Contagem de pulsos encoder Direito
  volatile int pulsos_motor_esq = 0;  //Contagem de pulsos encoder Esquerdo
  volatile boolean c1_dir;            //Para leitura do C1 Direito (pino de interrupção)
  volatile boolean c1_esq;            //Para leitura do C1 Esquerdo (pino de interrupção)
  volatile boolean c1_dir_old;        //Estado anterior de C1 Direito
  volatile boolean c1_esq_old;        //Estado anterior de C1 Esquerdo
  volatile boolean sentido_dir;       //Sentido de rotação motor Direita
  volatile boolean sentido_esq;       //Sentido de rotação motor Esquerdo

    /* Cálculo da velocidade */
  const float const_dir = 2 * pi * raio_roda_dir / resolucao_enc_dir;   // comprimento da roda(m)
  const float const_esq = 2 * pi * raio_roda_esq / resolucao_enc_esc;   // comprimento da roda(m)
  const float const_conv_rad_grau = 180 / pi;
  float vel_motor_dir = 0;        //Velocidade motor Direito
  float vel_motor_esq = 0;        //Velocidade motor Esquerdo

    /* PID */
  float err_dir[3];               //Vetor de erros para ajuste da velocidade
  float err_esq[3];               //Vetor de erros para ajuste da velocidade
  float q0_dir, q1_dir, q2_dir;   //Ganhos de sintonia
  float q0_esq, q1_esq, q2_esq;   //Ganhos de sintonia
  int pwm_motor_dir = 0;          //Potencia do motor Direito
  int pwm_motor_esq = 0;          //Potencia do motor Esquerdo

    /* --- CONSTANTE DE TEMPO --- */
  unsigned long millisOld = 0;
  float dt = 0;

    /* Valores de saída */
  float vel_linear = 0, vel_angular = 0, yaw = 0; // Yaw - Ginada
    


// =================================
//	--- --- --- FUNÇÕES --- --- ---
// =================================

    // Construtor
  public: DeadReckoning(){
  }

    // Inicia as configuraçãoes
  public: void iniciar(){
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
  private: void config_ponte_H(){
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
  private: void config_encoder(){
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
  private: void configPID(){
    float kp_dir, ti_dir, td_dir;   //Variáveis de sintonia
    float kp_esq, ti_esq, td_esq;   //Variáveis de sintonia

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

    q0_dir = kp_dir * (1 + (T / (2 * ti_dir)) + (td_dir / T));
    q1_dir = -kp_dir * (1 - (T / (2 * ti_dir)) + ((2 * td_dir) / T));
    q2_dir = kp_dir * td_dir / T;

    q0_esq = kp_esq * (1 + (T / (2 * ti_esq)) + (td_esq / T));
    q1_esq = -kp_esq * (1 - (T / (2 * ti_esq)) + ((2 * td_esq) / T));
    q2_esq = kp_esq * td_esq / T;
  }

    // Contagem de pulsos da roda direita - uso da interrupção externa
  public: void calcula_pulsos_roda_direita(){

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
  public: void calcula_pulsos_roda_esquerda(){

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
  public: void calcula_velocidade(){
    dt = (millis() - millisOld) / (float)1000;
    millisOld = millis();

      // Calcula a velocidade de cada roda
    vel_motor_dir = const_dir * (float)pulsos_motor_dir / dt;     // velocidade (m/s)
    vel_motor_esq = const_esq * (float)pulsos_motor_esq / dt;     // velocidade (m/s)

    #if Print_Dados_Pulsos
      print_dados_velocidade(&pulsos_motor_dir, &pulsos_motor_esq);
    #endif

    pulsos_motor_dir = 0;
    pulsos_motor_esq = 0;

      // Calcula velocidade - Linear, Angular e Ângulo de ginada (Yaw)
    vel_linear = (vel_motor_dir + vel_motor_esq) / 2;
    vel_angular = (vel_motor_dir - vel_motor_esq) / dist_entre_rodas;
    yaw += vel_angular * dt * const_conv_rad_grau;

      // Mantem o angulo do robô entre os parâmetros de (0 - 360) 
    //if(yaw < 0) yaw = 360 - yaw;
    //if(yaw > 360) yaw = yaw - 360;
    

    #if Print_Dados_Velocidade
      print_dados_velocidade(&vel_motor_dir, &vel_motor_esq);
    #endif

    #if Print_Dados_Velocidade_Angular_Linear
      print_dados_velocidade(&vel_linear, &vel_angular);
    #endif
  }

    // Efetua o controle da velocidade do motor com base no setpoint calculado
  public: void controlador_PID(){
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

    //Calcula a velocidade de cada motor com base nas velocidades (v_lin, v_ang) [linear e angular] passadas
  public: void setVelocidade(float v_lin, float v_ang){
    vel_motor_dir_setpoint = v_lin + (v_ang * dist_entre_rodas) / 2;
    vel_motor_esq_setpoint = v_lin - (v_ang * dist_entre_rodas) / 2;

      // Controla direção dos motores
    if(vel_motor_dir_setpoint > 0){
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      estado_motor_dir = POSITIVO;

    } else if(vel_motor_dir_setpoint < 0){
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      estado_motor_dir = NEGATIVO;

    } else {
      pwm_motor_dir = 0;
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, HIGH);
      estado_motor_dir = STOP;
    }

    if(vel_motor_esq_setpoint > 0){
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
      estado_motor_esq = POSITIVO;

    } else if(vel_motor_esq_setpoint < 0) {
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
      estado_motor_esq = NEGATIVO;

    } else {
      pwm_motor_esq = 0;
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, HIGH);
      estado_motor_esq = STOP;
    }
  }

    // Obtem dados
  public: void getDados(float *v_lin, float *v_ang, float *yaw_ang){
    *v_lin = vel_linear;
    *v_ang = vel_angular;
    *yaw_ang = yaw;
  }

    // Printa dados
  private: void print_dados_velocidade(volatile int *p1, volatile int *p2){
    Serial.print(*p1);
		Serial.print(", ");
		Serial.println(*p2);
  }

    // Printa dados
  private: void print_dados_velocidade(float *v1, float *v2){
    Serial.print(*v1);
		Serial.print(", ");
		Serial.println(*v2);
  }

};

#endif
