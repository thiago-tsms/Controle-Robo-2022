#ifndef DATA_EXCHANGE_JSON_H
#define DATA_EXCHANGE_JSON_H

  /* --- BIBLIOTECAS --- */
#include <ArduinoJson.h>



// ===================================
//  --- --- --- VARIÁVEIS --- --- ---
// ===================================

#define JSON_VLIN "VLIN"
#define JSON_VANG "VANG"
#define JSON_ROLL "ROLL"
#define JSON_PITCH "PITCH"
#define JSON_YAW "YAW"

typedef enum {
  VLIN = 0b0000000000000001,
  VANG = 0b0000000000000010,
} mask_json_t;

typedef struct {
  float vel_linear;
  float vel_angular;
  float roll;
  float pitch;
  float yaw;
} Json_transaction_send_t;

typedef struct {
  uint16_t mask;
  float vel_linear;
  float vel_angular;
} Json_transaction_recv_t;

#define QUEUE_LENGHT_WIFI_SEND 6
#define QUEUE_LENGHT_WIFI_RECV 10
#define QUEUE_SIZE_WIFI_SEND sizeof(Json_transaction_send_t)
#define QUEUE_SIZE_WIFI_RECV sizeof(Json_transaction_send_t)

QueueHandle_t queue_wifi_send;
QueueHandle_t queue_wifi_recv;

Json_transaction_send_t queue_data_send;
Json_transaction_recv_t queue_data_recv;



// =================================
//	--- --- --- FUNÇÕES --- --- ---
// =================================

void iniciar_data_exchange_json(){
    // Cria Queue
  queue_wifi_send = xQueueCreate(QUEUE_LENGHT_WIFI_RECV, QUEUE_SIZE_WIFI_RECV);
  vQueueAddToRegistry(queue_wifi_send, "queue-send-wifi");
  queue_wifi_recv = xQueueCreate(QUEUE_LENGHT_WIFI_SEND, QUEUE_SIZE_WIFI_SEND);
  vQueueAddToRegistry(queue_wifi_recv, "queue-recv-wifi");
}

void serialization_json(String *json_string, Json_transaction_send_t *data){
  DynamicJsonDocument json(1024);
  *json_string = "";

  json[JSON_VLIN] = data->vel_linear;
  json[JSON_VANG] = data->vel_angular;
  json[JSON_ROLL] = data->roll;
  json[JSON_PITCH] = data->pitch;
  json[JSON_YAW] = data->yaw;

  serializeJson(json, *json_string);
}

void deserialize_json(String *dado, Json_transaction_recv_t *data){
  DynamicJsonDocument json(512);
  deserializeJson(json, *dado);
  data->mask = 0;

  if(json.containsKey(JSON_VLIN)){
    data->mask += mask_json_t::VLIN;
    data->vel_linear = json[JSON_VLIN];
  }

  if(json.containsKey(JSON_VANG)){
    data->mask += mask_json_t::VANG;
    data->vel_angular = json[JSON_VANG];
  }
}

#endif


/*#define QUEUE_LENGHT_INTERRUPT_TIMER 6
#define QUEUE_SIZE_INTERRUPT_TIMER sizeof(action_interrupt_timer_t)
#define QUEUESET_LENGHT_RECV (QUEUE_LENGHT_INTERRUPT_TIMER + QUEUE_LENGHT_WIFI_RECV + 0)
*/

   /*// Cria semáforo
  ESP_LOGI(TAG_MAIN, "Criando Semáforo");
  semaphore_lighting_states = xSemaphoreCreateBinary();
  xSemaphoreGive(semaphore_lighting_states);
  semaphore_led_states = xSemaphoreCreateBinary();
  xSemaphoreGive(semaphore_led_states);

//QueueSetHandle_t queueSet_control_recv;
  
    // Cria Set
  ESP_LOGI(TAG_MAIN, "Criando Set");
  queueSet_control_recv = xQueueCreateSet(QUEUESET_LENGHT_RECV);
  xQueueAddToSet(queue_interrupt_timer, queueSet_control_recv);
  xQueueAddToSet(queue_wifi_recv, queueSet_control_recv);
*/