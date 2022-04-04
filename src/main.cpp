#include "Arduino.h"


  // Configuração de Tasks
#define usStackDepth_Task_Controle 3000
#define uxPriority_Task_Controle 3
#define vTaskDelay_Task_Controle 50

#define usStackDepth_Task_Comunicacao 6000
#define uxPriority_Task_Comunicacao 2
#define vTaskDelay_Task_Comunicacao 100

#define usStackDepth_Task_Sinalizacao 1000
#define uxPriority_Task_Sinalizacao 1
#define vTaskDelay_Task_Sinalizacao 1000


	/* --- ESCOPO DE FUNLÇÕES --- */
void task_controle(void *parametros);
void task_comunicacao(void *parametros);
void task_sinalizacao(void *parametros);


void setup(){
  Serial.begin(115200);

    // Cria as tarefas - FreeRTOS
  xTaskCreate(task_controle, "task-controle", usStackDepth_Task_Controle, NULL, 2, NULL);
  xTaskCreate(task_comunicacao, "task-comunicacao", usStackDepth_Task_Comunicacao, NULL, 2, NULL);
  xTaskCreate(task_sinalizacao, "task-sinalizacao", usStackDepth_Task_Sinalizacao, NULL, 1, NULL);
}

void loop(){}

  // Tarefa de controle
void task_controle(void *parametros){
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while(1){
    vTaskDelayUntil(&xLastWakeTime, vTaskDelay_Task_Controle);
  }
}


  // Comunicação Wifi
void task_comunicacao(void *parametros){
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while(1){
    vTaskDelayUntil(&xLastWakeTime, vTaskDelay_Task_Comunicacao);
  }
}


  // Sinalização WIFI (Conectado, Desconectado, Configurando Wifi)
void task_sinalizacao(void *parametros){
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while(1){
    vTaskDelayUntil(&xLastWakeTime, vTaskDelay_Task_Sinalizacao);
  }
}