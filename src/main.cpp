#include "Arduino.h"

  /* --- BIBLIOTECAS --- */
#include "Comunicacao_Wifi.h"
#include "Sensores.h"
#include "DeadReckoning.h"



// ===================================================
//	--- --- --- CONFIGURAÇÃO DE COMPILAÇÃO --- --- ---
// ===================================================

  // Análise do consumo de dados das tarefas
#define Analise_Stack_Task_Controle false
#define Analise_Stack_Task_Comunicacao false
#define Analise_Stack_Task_Sinalizacao false
#define Modo_Wifi_Ativo false
#define Sensores_Ativos true
#define DeadReckoning_Ativo true



// =======================================
//  --- --- --- CONFIGURAÇÕES --- --- ---
// =======================================

  // Botão e led de sinalização
#define BTN_Sinalizacao 12
#define LED_Sinalizacao 14


  // Configuração de Tasks
#define usStackDepth_Task_Controle 6000
#define uxPriority_Task_Controle 3
#define vTaskDelay_Task_Controle 150

#define usStackDepth_Task_Comunicacao 4000
#define uxPriority_Task_Comunicacao 2
#define vTaskDelay_Task_Comunicacao 100

#define usStackDepth_Task_Sinalizacao 2000
#define uxPriority_Task_Sinalizacao 1



// ===================================
//  --- --- --- VARIÁVEIS --- --- ---
// ===================================

  // Classes instanciadas
Sensores sensores;
DeadReckoning deadReckoning;


// ==========================================
//  --- --- --- FUNÇÃO PRINCIPAL --- --- ---
// ==========================================

  // Escopo de Funções
void IRAM_ATTR configurar_rede();
void task_controle(void *parametros);
void task_comunicacao(void *parametros);
void task_sinalizacao(void *parametros);


  // Função principal
void setup(){

    // Configura a comunicação serial
  Serial.begin(115200);
  Serial.println("\nSistema Iniciando ...");

    //Inicializa pinos para sinalizações
  pinMode(BTN_Sinalizacao, INPUT);
  pinMode(LED_Sinalizacao, OUTPUT);

  // Configura as Interrupções Externas
  attachInterrupt(digitalPinToInterrupt(BTN_Sinalizacao), configurar_rede, CHANGE);

  #if DeadReckoning_Ativo
    attachInterrupt(digitalPinToInterrupt(Enc_Dir_C1), calcula_pulsos_roda_direita, CHANGE);
    attachInterrupt(digitalPinToInterrupt(Enc_Esq_C1), calcula_pulsos_roda_esquerda, CHANGE);
  #endif

  #if Sensores_Ativos
    sensores.iniciar();
  #endif

  #if DeadReckoning_Ativo
    deadReckoning.iniciar();
  #endif

    // Cria as tarefas - FreeRTOS
  xTaskCreate(task_controle, "task-controle", usStackDepth_Task_Controle, NULL, uxPriority_Task_Controle, NULL);

  #if Modo_Wifi_Ativo
    xTaskCreate(task_comunicacao, "task-comunicacao", usStackDepth_Task_Comunicacao, NULL, uxPriority_Task_Comunicacao, NULL);
    xTaskCreate(task_sinalizacao, "task-sinalizacao", usStackDepth_Task_Sinalizacao, NULL, uxPriority_Task_Sinalizacao, NULL);
  #endif

  Serial.println("... Sistema Iniciado");
}

void loop(){}



// =================================
//	--- --- --- TAREFAS --- --- ---
// =================================

  // Tarefa de controle
void task_controle(void *parametros){
  TickType_t xLastWakeTime = xTaskGetTickCount();
  #if Analise_Stack_Task_Controle
    UBaseType_t uxHighWaterMark;
  #endif

  while(1){

    #if Sensores_Ativos
      sensores.leitura_sensores();
      sensores.processa_dados();
      sensores.interpreta_dados();
    #endif

    #if Analise_Stack_Task_Controle
      uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
      String txt = "task-controle - stack:: alocado: " + String(usStackDepth_Task_Controle) + " - usado: " + String(usStackDepth_Task_Controle - uxHighWaterMark) + " - restante: " + String(uxHighWaterMark);
      Serial.println(txt);
    #endif
    vTaskDelayUntil(&xLastWakeTime, vTaskDelay_Task_Controle);
  }
}


  // Comunicação Wifi
void task_comunicacao(void *parametros){
  TickType_t xLastWakeTime = xTaskGetTickCount();
  #if Analise_Stack_Task_Comunicacao
    UBaseType_t uxHighWaterMark;
  #endif

  while(1){
    if(get_status_sinalizacao() == Status_Sinalizacao_t::Wifi_Manager) wifi_manager();
    else {
      set_status_sinalizacao(Status_Sinalizacao_t::Wifi_Iniciado);
      start_comunicacao(xLastWakeTime);
    }

    #if Analise_Stack_Task_Comunicacao
      uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
      String txt = "task-comunicacao - stack:: alocado: " + String(usStackDepth_Task_Comunicacao) + " - usado: " + String(usStackDepth_Task_Comunicacao - uxHighWaterMark) + " - restante: " + String(uxHighWaterMark);
      Serial.println(txt);
    #endif

    vTaskDelayUntil(&xLastWakeTime, vTaskDelay_Task_Comunicacao);
  }
}


  // Sinalização WIFI (Conectado, Desconectado, Configurando Wifi)
void task_sinalizacao(void *parametros){
  #if Analise_Stack_Task_Sinalizacao
    UBaseType_t uxHighWaterMark;
  #endif

  while(1){
    switch((int)get_status_sinalizacao()){
      case Status_Sinalizacao_t::Wifi_Off:
        digitalWrite(LED_Sinalizacao, LOW);
        break;
      
      case Status_Sinalizacao_t::Wifi_Manager:
        digitalWrite(LED_Sinalizacao, LOW);
        break;

      case Status_Sinalizacao_t::Wifi_Iniciado:
        digitalWrite(LED_Sinalizacao, !digitalRead(LED_Sinalizacao));
        break;

      case Status_Sinalizacao_t::Wifi_Conectado:
        digitalWrite(LED_Sinalizacao, HIGH);
        vTaskDelay(pdMS_TO_TICKS(400));
        digitalWrite(LED_Sinalizacao, LOW);
        vTaskDelay(pdMS_TO_TICKS(200));
        digitalWrite(LED_Sinalizacao, HIGH);
        vTaskDelay(pdMS_TO_TICKS(400));
        digitalWrite(LED_Sinalizacao, LOW);
        break;

      case Status_Sinalizacao_t::Cliente_Conectado:
        digitalWrite(LED_Sinalizacao, HIGH);
        break;
    }

    #if Analise_Stack_Task_Sinalizacao
      uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
      String txt = "task-sinalizacao - stack:: alocado: " + String(usStackDepth_Task_Sinalizacao) + " - usado: " + String(usStackDepth_Task_Sinalizacao - uxHighWaterMark) + " - restante: " + String(uxHighWaterMark);
      Serial.println(txt);
    #endif
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}



// ================================================
//	--- --- --- FUNÇÕES DE INTERRUPÇÃO --- --- ---
// ================================================

  // Altera o modo de operação - Função de interrupção
void IRAM_ATTR configurar_rede(){
  static long long time;

  if(digitalRead(BTN_Sinalizacao) == HIGH){
    time = millis();

  } else {
    long dt = long(millis() - time);
    if(dt > 3000) {
      if(get_status_sinalizacao() == Status_Sinalizacao_t::Wifi_Manager) set_status_sinalizacao(Status_Sinalizacao_t::Wifi_Off);
      else if(get_status_sinalizacao() != Status_Sinalizacao_t::Wifi_Manager) set_status_sinalizacao(Status_Sinalizacao_t::Wifi_Manager);
    }
    time = 0;
  }
}


  /* --- INICIALIZAÇÃO DAS INTERRUPÇÕES --- */
#if DeadReckoning_Ativo
    // IRAM_ATTR -> indicar que esse trecho de código ficará na seção do barramento de instruções da RAM (maior velocidade)
  void IRAM_ATTR calcula_pulsos_roda_direita(){deadReckoning.calcula_pulsos_roda_direita();}
  void IRAM_ATTR calcula_pulsos_roda_esquerda(){deadReckoning.calcula_pulsos_roda_esquerda();}
#endif


// ========================================
//	--- --- --- FUNÇÕES GERAIS --- --- ---
// ========================================
