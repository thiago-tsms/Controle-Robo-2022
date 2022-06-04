#ifndef DATA_EXCHANGE_JSON_H
#define DATA_EXCHANGE_JSON_H

//#include <ArduinoJson.h>

enum json_action {
    DATA = 0,
    CONFIGURATION
};

#endif

/*void deserialize_json(String dado){
  DynamicJsonDocument json(1024);
  deserializeJson(json, dado);

  states.luz_1 = json["L1"];
  states.luz_2 = json["L2"];
  states.led = json["LED"];
  states.modo = json["MODO"];
  states.led_r = json["R"];
  states.led_g = json["G"];
  states.led_b = json["B"];
}

void serialization_json(String *dado){
  DynamicJsonDocument json(1024);
  *dado = "";

  json["L1"] = states.luz_1;
  json["L2"] = states.luz_2;
  json["LED"] = states.led;
  json["MODO"] = states.modo;
  json["R"] = states.led_r;
  json["G"] = states.led_g;
  json["B"] = states.led_b;

  serializeJson(json, *dado);
}*/