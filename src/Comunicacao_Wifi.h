#ifndef Comunicacao_H
#define Comunicacao_H

  /* --- BIBLIOTECAS --- */
#include "Arduino.h"
#include <WiFi.h>
#include <WebServer.h>
#include "FS.h"
#include "SPIFFS.h"
#include "DataExchangeJSON.h"



// =========================================
//	  --- --- --- CONFIGURAÇÕES --- --- ---
// =========================================

  // Parâmetros de AP
const char* ssid_ap = "Robo IFF";
const char* password_ap = "";
IPAddress ip_ap(192,168,1,1);
IPAddress gateway_ap(192,168,1,1);
IPAddress mask_ap(255,255,255,0);
const char* host_name = "Robo IFF";



// =========================================
//	  --- --- --- VARIÁVEIS --- --- ---
// =========================================

static WebServer server(80);


  // Indica os estatus da comunicacao (Usado para a sinalização via LED)
enum Status_Sinalizacao_t {
  Wifi_Off = 0x0,
  Wifi_Manager,     // Modo de configuração de SSID e Password
  Wifi_Iniciado,    // Buscando WIFI
  Wifi_Conectado,   // Buscando Cliente
  Cliente_Conectado
};


  // Parâmetros para STA
String ssid_st;
String password_st;
String ip_st;
uint16_t port_st;


  // Parâmetros para Sinalização
Status_Sinalizacao_t status_comunicacao = Status_Sinalizacao_t::Wifi_Off;


	/* --- ESCOPO DE FUNLÇÕES --- */
void read_dados_flash();
void save_dados_flash(String ssid, String password, String ip, String port);
void config_wifi_params();
void start_comunicacao(TickType_t xLastWakeTime);
void send_recv_msg(WiFiClient *client);
void wifi_manager();
void config_param_Wifi();
void envia_html_pag_congiguracao();
String html_pag_configuracao();
String html_msg_confimacao();
void set_status_sinalizacao(Status_Sinalizacao_t status);
Status_Sinalizacao_t get_status_sinalizacao();



// ========================================= 
//	  --- --- --- MEMORIA FLASH --- --- ---
// ========================================= 

  // Lê dados salvos na flash
void read_dados_flash(){
  File file;
  String dados;
  String str_aux;

    //Lê parâmetros salvos na flash
  while(!SPIFFS.begin(true));
  
  file = SPIFFS.open("/dados_server.txt", FILE_READ);

  ssid_st = file.readStringUntil('|');
  password_st = file.readStringUntil('|');
  ip_st = file.readStringUntil('|');
  port_st = uint16_t(file.readStringUntil('|').toInt());
  
  file.close();
  SPIFFS.end();
}

  // Salva dados na flash
void save_dados_flash(String ssid, String password, String ip, String port){
  File file;

  String dados = ssid + "|" + password + "|" + ip + "|" + String(port) + "|";
  
  while(!SPIFFS.begin(true));
  file = SPIFFS.open("/dados_server.txt", FILE_WRITE);
  file.print(dados);
  file.close();
  SPIFFS.end();
}



// ============================================ 
//	  --- --- --- COMUNICAÇÃO WIFI --- --- ---
// ============================================

  // Entra no modo de comunicação

void start_comunicacao(TickType_t xLastWakeTime){

  WiFiClient client;
  
    // Lê os parâmetros de conexção salvos na flash
  read_dados_flash();

    // Configura o rádio
  WiFi.mode(WIFI_STA);
  WiFi.enableSTA(true);
  WiFi.setHostname(host_name);

  while(status_comunicacao == Status_Sinalizacao_t::Wifi_Iniciado){

      //Tenta efetuar conexão no Wifi
    WiFi.begin(ssid_st.c_str(), password_st.c_str());
    for(int8_t i = 0; (WiFi.status() != WL_CONNECTED) && (i < 15); i++){
      vTaskDelayUntil(&xLastWakeTime, 300);
    }
    if(WiFi.status() == WL_CONNECTED){
      if(status_comunicacao == Status_Sinalizacao_t::Wifi_Iniciado) status_comunicacao = Status_Sinalizacao_t::Wifi_Conectado;
      Serial.println("Wifi conectado:\nHostname: " + String(WiFi.getHostname()) + "\nIP: " + WiFi.localIP().toString());
    } else {
      if(status_comunicacao == Status_Sinalizacao_t::Wifi_Conectado) status_comunicacao = Status_Sinalizacao_t::Wifi_Iniciado;
    }

      // Enquanto conectado com wifi
    while(WiFi.status() == WL_CONNECTED && status_comunicacao == Status_Sinalizacao_t::Wifi_Conectado){

        // Testa efetuar conexão com o cliente
      client.connect(ip_st.c_str(), port_st);
      for(int8_t i = 0; !client.connected() && (i < 15); i++){
        vTaskDelayUntil(&xLastWakeTime, 300);
      }
      if(client.connected()){
        client.setTimeout(0);
        status_comunicacao = Status_Sinalizacao_t::Cliente_Conectado;
        Serial.println("Cliete conectado:\nIP: " + client.remoteIP().toString() + "\nPorta: " + String(client.remotePort()));
      }

        // Enquanto conectado com o cliente
      while (client.connected() && status_comunicacao == Status_Sinalizacao_t::Cliente_Conectado){
        send_recv_msg(&client);
      }
      client.stop();
      status_comunicacao = Status_Sinalizacao_t::Wifi_Conectado;
      xQueueReset(queue_wifi_send);
      xQueueReset(queue_wifi_recv);
    }
  }

  client.stop();
  WiFi.enableSTA(false);
  WiFi.disconnect();
}

  // Envia e recebe dados do cliente
void send_recv_msg(WiFiClient *client){

   

    // Recebe dados da task de controle e os envia ao cliente
  if(uxQueueMessagesWaiting(queue_wifi_send) > 0){
    /*
      Verifica se há mensagens no buffer de comunicação, se houver ela é enviada do cliente. Apenas uma é enviada por vez.
      ** uxQueueMessagesWaiting - verifica quantas mensagens há no buffer
      ** xQueueReceive - lê uma mensagem contida no buffer
      ** (*client).print - envia mensagem String via WiFi ao cliente conectado
    */

    Json_transaction_send_t queue_data_send;
    String json_string[512];
    xQueueReceive(queue_wifi_send, &queue_data_send, 0);
    serialization_json(json_string, &queue_data_send);
    (*client).print(json_string->c_str());
  }

    // Recebe dados do clientes e os envia a task de controle
  if((*client).available()){
    /*
      Verifica se há mensagens a serem recebidas do cliente, se houcer ela é rebebida e colocada no buffer.
      ** (*client).available - verifica se há dados enviados pelo cliente
      ** (*client).readStringUntil - lê dados enviados pelo cliente atá um caracter especificado
      ** xQueueSend - salva uma mensagem no buffer
    */

    Json_transaction_recv_t queue_data_recv;
    String json_string = (*client).readString();
    deserialize_json(&json_string, &queue_data_recv);
    (*client).flush();
    xQueueSend(queue_wifi_recv, &queue_data_recv, 0);
  }
}



// ======================================== 
//	  --- --- --- WIFI MANAGER --- --- ---
// ======================================== 

void wifi_manager(){
  TickType_t xLastWakeTime = xTaskGetTickCount();
  Serial.println("Configurando AP - Wifi Manager");

  server.stop();
  WiFi.disconnect();
  WiFi.mode(WIFI_AP);
  WiFi.enableAP(true);
  WiFi.softAP(ssid_ap, password_ap);
  WiFi.softAPConfig(ip_ap, gateway_ap, mask_ap);
  WiFi.begin();
  server.begin();
  Serial.println("AP configurado");

  server.on("/", HTTP_GET, envia_html_pag_congiguracao);
  server.on("/config", HTTP_POST, config_param_Wifi);

      // Aguarda no modo de configuração até que ela seja encerrada
  while(status_comunicacao == Status_Sinalizacao_t::Wifi_Manager){
    server.handleClient();
    vTaskDelayUntil(&xLastWakeTime, 50);
  }

  server.close();
  WiFi.enableAP(false);
  WiFi.disconnect();

  Serial.println("Wifi Manager encerrado");
}

  // Envia página de configuração
void envia_html_pag_congiguracao(){
  server.send(200,"text/html", html_pag_configuracao());
}

  // Configura os parametros de configuração para conexão Wifi
void config_param_Wifi(){
  String ssid = server.arg("ssid");
  String password = server.arg("password");
  String ip = server.arg("ip");
  String port = server.arg("port");

  Serial.println("Dados coletados: \nSSID: " + ssid + "\nPassword: " + password +"\nIP: " + ip + "\nPort: " + String(port));
  Serial.println("Salvando dados na flash");
  save_dados_flash(ssid, password, ip, port);
  Serial.println("Dados salvos na flash");

  server.send(200, "text/html", html_msg_confimacao());
}

  // Página de configuração de ssid, password, ip, port (html)
String html_pag_configuracao(){
  return String("") +
    "<!DOCTYPE HTML>"+
    "<html lang='pt-br'>"+
      "<head>"+
        "<meta charset='utf-8'/>"+
        "<title>Configuração de Conexão</title>"+

        "<style>"+
          "body {margin:0%; padding: 2%; background-color: rgb(101 17 144); color: white;}"+
          "h2, h3, h4 {text-align: center; margin-block-start: 0.4em; margin-block-end: 0.4em;}"+
          "form {display: grid; grid-template-columns: 100%;}"+
          ".atributo {margin-top: 15px; width: 60%; justify-self: center; height: fit-content;}"+
          "h4 {font-weight: 100;}"+
          "label {font-size: 1.2em;}"+
          "input {border-style: none; padding: 1.4%; width: 97.2%; border-radius: 2px; font-size: 1.2em; text-align: center;}"+
          "button {width: 100%; margin-top: 5px; padding: 15px; border-radius: 5px; background-color: #146d0e; color: #ffffff; font-weight: bolder; border: none; font-size: 1.5em;}"+
        "</style>"+
      "</head>"+

    "<body>"+
      "<h2>Conexão Wifi do robô</h1>"+
      "<h3>Insira os parâmetros de configuração para a conexão do robô com a rede Wifi.</h3>"+
      "<h4>A cominicação se dá via socket</h4>"+

      "<form action='/config' method='POST'>"+
          "<div class='atributo'>"+
            "<label for='ssid'>Nome da rede</label><br/>"+
            "<input id='ssid' name='ssid' placeholder='Rede 1'>"+
          "</div>"+
        
          "<div class='atributo'>"+
            "<label for='password'>Senha</label><br/>"+
            "<input id='password' name='password' placeholder='********'>"+
          "</div>"+

          "<div class='atributo'>"+
            "<label for='ip'>IP</label><br/>"+
            "<input id='ip' name='ip' placeholder='192.168.0.1'>"+
          "</div>"+

          "<div class='atributo'>"+
            "<label for='port'>Porta</label><br/>"+
            "<input id='port' name='port' placeholder='8080'>"+
          "</div>"+

          "<div class='atributo'>"+
            "<button type='submit'>Salvar</button>"+
          "</div>"+
      "</form>"+
    "</body>"+
    "</html>";
}

  // Mensagem de confirmação de configuração (html)
String html_msg_confimacao(){
  return String("") + "<html lang='pt-br'>"+
  "<head>"+
    "<meta charset='utf-8'/>"+
      "<title>Configuração de Conexão</title>"+
      "<script>alert('Atributos Salvos!');</script>"+
    "</head>"+
  "</html>";
}



// ================================================== 
//	  --- --- --- FUNÇÕES DE SINALIZAÇÃO --- --- ---
// ================================================== 

void set_status_sinalizacao(Status_Sinalizacao_t status){
  status_comunicacao = status;
}

Status_Sinalizacao_t get_status_sinalizacao(){
  return status_comunicacao;
}

#endif