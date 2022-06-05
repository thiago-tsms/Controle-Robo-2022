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



// =================================
//	--- --- --- FUNÇÕES --- --- ---
// =================================

  // Escopo de Funções
void iniciar_data_exchange_json();
void serialization_json(String *json_string, Json_transaction_send_t *data);
void deserialize_json(String *json_string, Json_transaction_recv_t *data);


  // Inicializa as queues
void iniciar_data_exchange_json(){
    // Cria Queue
  queue_wifi_send = xQueueCreate(QUEUE_LENGHT_WIFI_RECV, QUEUE_SIZE_WIFI_RECV);
  vQueueAddToRegistry(queue_wifi_send, "queue-send-wifi");
  queue_wifi_recv = xQueueCreate(QUEUE_LENGHT_WIFI_SEND, QUEUE_SIZE_WIFI_SEND);
  vQueueAddToRegistry(queue_wifi_recv, "queue-recv-wifi");
}

  // Converte os dados para um JSON String
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

  // Obtem os dados de um JSON String
void deserialize_json(String *json_string, Json_transaction_recv_t *data){
  DynamicJsonDocument json(512);
  deserializeJson(json, *json_string);
  data->mask = 0x0000;

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