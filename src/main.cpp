#include "Arduino.h"

#include "Comunicaco_Wifi.cpp"


// ===================================================
//	--- --- --- CONFIGURAÇÃO DE COMPILAÇÃO --- --- ---
// ===================================================

  // Análise do consumo de dados das tarefas
#define Analise_Sack_Task_Controle false
#define Analise_Sack_Task_Comunicacao false
#define Analise_Sack_Task_Sinalizacao false



// =========================================
//	  --- --- --- CONFIGURAÇÕES --- --- ---
// =========================================

  // Botão e led de sinalização
#define BTN_Sinalizacao 12
#define LED_Sinalizacao 14


  // Configuração de Tasks
#define usStackDepth_Task_Controle 3000
#define uxPriority_Task_Controle 3
#define vTaskDelay_Task_Controle 50

#define usStackDepth_Task_Comunicacao 6000
#define uxPriority_Task_Comunicacao 2
#define vTaskDelay_Task_Comunicacao 100

#define usStackDepth_Task_Sinalizacao 1000
#define uxPriority_Task_Sinalizacao 1



// ============================================
//	  --- --- --- FUNÇÃO PRINCIPAL --- --- ---
// ============================================

  // Escopo de Funções
void task_controle(void *parametros);
void task_comunicacao(void *parametros);
void task_sinalizacao(void *parametros);
void inicializar();


  // Função principal
void setup(){
  inicializar();

    // Cria as tarefas - FreeRTOS
  xTaskCreate(task_controle, "task-controle", usStackDepth_Task_Controle, NULL, uxPriority_Task_Controle, NULL);
  xTaskCreate(task_comunicacao, "task-comunicacao", usStackDepth_Task_Comunicacao, NULL, uxPriority_Task_Comunicacao, NULL);
  xTaskCreate(task_sinalizacao, "task-sinalizacao", usStackDepth_Task_Sinalizacao, NULL, uxPriority_Task_Sinalizacao, NULL);
}

void loop(){}



// ===================================
//	  --- --- --- TAREFAS --- --- ---
// ===================================

  // Tarefa de controle
void task_controle(void *parametros){
  TickType_t xLastWakeTime = xTaskGetTickCount();
  #if Analise_Sack_Task_Controle
    UBaseType_t uxHighWaterMark;
  #endif

  while(1){


    #if Analise_Sack_Task_Controle
      uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
      Serial.println(uxHighWaterMark);
    #endif
    vTaskDelayUntil(&xLastWakeTime, vTaskDelay_Task_Controle);
  }
}


  // Comunicação Wifi
void task_comunicacao(void *parametros){
  TickType_t xLastWakeTime = xTaskGetTickCount();
  #if Analise_Sack_Task_Comunicacao
    UBaseType_t uxHighWaterMark;
  #endif

  while(1){


    #if Analise_Sack_Task_Comunicacao
      uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
      Serial.println(uxHighWaterMark);
    #endif
    vTaskDelayUntil(&xLastWakeTime, vTaskDelay_Task_Comunicacao);
  }
}


  // Sinalização WIFI (Conectado, Desconectado, Configurando Wifi)
void task_sinalizacao(void *parametros){
  #if Analise_Sack_Task_Sinalizacao
    UBaseType_t uxHighWaterMark;
  #endif

  while(1){
    switch(get_status_sinalizacao){
      case STATUS_SINALIZACAO::Wifi_Off:
        digitalWrite(led_mod_operacao, LOW);
        break;

      case STATUS_SINALIZACAO::Wifi_Iniciado:
        digitalWrite(led_mod_operacao, !digitalRead(led_mod_operacao));
        break;

      case STATUS_SINALIZACAO::Wifi_Conectado:
        digitalWrite(led_mod_operacao, HIGH);
        vTaskDelay(pdMS_TO_TICKS(400));
        digitalWrite(led_mod_operacao, LOW);
        vTaskDelay(pdMS_TO_TICKS(200));
        digitalWrite(led_mod_operacao, HIGH);
        vTaskDelay(pdMS_TO_TICKS(200));
        digitalWrite(led_mod_operacao, LOW);
        break;

      case STATUS_SINALIZACAO::Cliente_Conectado:
        digitalWrite(led_mod_operacao, HIGH);
        break;
    }

    #if Analise_Sack_Task_Sinalizacao
      uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
      Serial.println(uxHighWaterMark);
    #endif
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}



// ==========================================
//	  --- --- --- FUNÇÕES GERAIS --- --- ---
// ==========================================

  // Efetua as configurações iniciais
void inicializar(){

    //Inicializa sinalizações
  pinMode(BTN_Sinalizacao, INPUT);
  pinMode(LED_Sinalizacao, OUTPUT);

    // Configura a comunicação serial
  Serial.begin(115200);
  Serial.println("\n\n\nIniciando ...");

    // Configura as Interrupções Externas
  /*attachInterrupt(digitalPinToInterrupt(btn_mod_operacao), modo_operacao, CHANGE);
  attachInterrupt(digitalPinToInterrupt(Enc_Dir_C1), calcula_pulsos_roda_direita, CHANGE);
  attachInterrupt(digitalPinToInterrupt(Enc_Esq_C1), calcula_pulsos_roda_esquerda, CHANGE);*/

    //Configura os sensores
  /*sensores.inicio();*/

    // Configura WiFi
  /*init_wifi();*/

  Serial.println("Sistema iniciado\n");
}