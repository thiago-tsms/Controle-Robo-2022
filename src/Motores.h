#ifndef Motores_H
#define Motores_H

#include <Arduino.h>

/* Definição dos pinos da Ponte H */
#define IN1   19
#define IN2   18
#define ENA   5   //Motor Esquerda (PWM)
#define IN3   17
#define IN4   16
#define ENB   4  //Motor Direita (PWM)

/* Definição dos pinos dos Encoders */
#define Enc_Dir_C1 36  //Pino com Interrupção
#define Enc_Dir_C2 39
#define Enc_Esq_C1 34  //Pino com Interrupção
#define Enc_Esq_C2 35

class Motores{
  
  private:
      /* --- Escopo de funções --- */
    void config_ponte_H();
    void config_encoder();
    void configPID();

  /* --- --- VARIÁVEIS AJUSTÁVEIS --- --- */
    const float pi = 3.14159265359;
    const float resolucao_enc_dir = 473;      // resolução experimental(PPR)
    const float resolucao_enc_esc = 473;      // resolução experimental(PPR)

    const float raio_roda_dir = 0.067 / 2;    // diâmetro da roda (m)
    const float raio_roda_esq = 0.067 / 2;    // diâmetro da roda (m)
    const float dist_entre_rodas = 0.1796;      // distância entre as rodas (m) -- 19 O padrão

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

  /* - - - - - - - - - - - - - - - - - - - */


      /* Setando a velocidade (linear, angular) */
    float vel_motor_dir_setpoint = 0;     //Velocidades de referência motor Direito
    float vel_motor_esq_setpoint = 0;     //Velocidades de referência motor Esquerdo
    enum EM {POSITIVO = 1, STOP = 0, NEGATIVO = -1};  // Possíveis estados do motor
    byte estado_motor_dir = STOP;         //Estado do motor direito
    byte estado_motor_esq = STOP;         //Estado do motor esquerdo


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
    

  public:
      /* --- Escopo de funções --- */
    Motores();
    void calcula_pulsos_roda_direita();
    void calcula_pulsos_roda_esquerda();
    void calcula_velocidade();
    void PID();
    void recebeComando(String comando);
    float getVelDireita();
    float getVelEsquerda();
    int getPWMDireita();
    int getPWMEsquerda();
    float setVelocidade(float v_lin, float v_ang);
    void getDados(float *v_lin, float *v_ang, float *yaw);
};

#endif
