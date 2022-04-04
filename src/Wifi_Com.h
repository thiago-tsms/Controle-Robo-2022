#include "Arduino.h"

/* --- BIBLIOTECAS --- */
#include <WiFi.h>
#include <WebServer.h>
#include "FS.h"
#include "SPIFFS.h"


  // Dados a serem enviados via Wifi
typedef struct{
  float v_lin;
  float v_ang;
  float roll;
  float pitch;
  float yaw;
} MSG_SEND;

typedef struct{
  float v_lin;
  float v_ang;
} MSG_RESV;

  // Usado para a truca de menssagem entre task
QueueHandle_t queue_send;
QueueHandle_t queue_recv;
int queue_size = 10;


  // Modos de operação da comunicação Wifi - Web Page para comunicação || Tenta estabelecer comunicação com os dados salvos
enum STATUS_WIFI {Config_Com = 1, Start_Com = 2};
volatile STATUS_WIFI status_comunicacao = Start_Com;

  // Indica os estatus da comunicacao (Usado para a sinalização via LED)
enum STATUS_SINALIZACAO {Wifi_Config = 0, Wifi_Desc = 1,  Wifi_Conect = 2, Client_Conect = 3};
STATUS_SINALIZACAO status_operacao = Wifi_Desc;
static WebServer server(80);

  // Parâmetros de Asses Point
const char* ssid_ap = "Robo IFF";  // Enter SSID here
const char* password_ap = "";
IPAddress ip_ap(192,168,1,1);
IPAddress gateway_ap(192,168,1,1);
IPAddress mask_ap(255,255,255,0);
const char* host_name = "Robo IFF";

  // Parâmetros de Station
String ssid_st;
String password_st;
String ip_st;
uint16_t port_st;


/* Escopo de funções*/
void init_wifi();
void read_dados_flash();
void save_dados_flash(String ssid, String password, String ip, String port);
void start_comunicacao();
void enviar_receber_msg(WiFiClient *client);
void config_comunicacao();
void envia_pag_congiguracao();
void config_param_Wifi();
String html_pag_config();
String html_msg_confimacao();

void init_wifi(){
    // Cria um buffer para a troca de mensagens entre task
  queue_send = xQueueCreate(queue_size, sizeof(MSG_SEND));
  queue_recv = xQueueCreate(queue_size, sizeof(MSG_RESV));
}

  // Lê dados da flash
void read_dados_flash(){
  File file;
  String dados;
  String str_aux;
  int ind_aux;

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

  // Abre página de configuração para conexão socket
void config_comunicacao(){
  TickType_t xLastWakeTime = xTaskGetTickCount();
  Serial.println("Configurando AP");
  status_operacao = Wifi_Config;

  server.stop();
  WiFi.disconnect();
  WiFi.mode(WIFI_AP);
  WiFi.enableAP(true);
  WiFi.softAP(ssid_ap, password_ap);
  WiFi.softAPConfig(ip_ap, gateway_ap, mask_ap);
  WiFi.begin();
  server.begin();
  Serial.println("AP configurado");

  server.on("/", HTTP_GET, envia_pag_congiguracao);
  server.on("/config", HTTP_POST, config_param_Wifi);

      // Aguarda no modo de configuração até que ela seja encerrada
  while(status_comunicacao == Config_Com){
      server.handleClient();
      vTaskDelayUntil(&xLastWakeTime, 10);
  }

  server.close();
  WiFi.enableAP(false);
  WiFi.disconnect();
}

  // Entra no modo de comunicação
void start_comunicacao(TickType_t xLastWakeTime){

  WiFiClient client;
  
    // Lê os parâmetros de conexção salvos na flash
  read_dados_flash();

    // Configura o rádio
  WiFi.mode(WIFI_STA);
  WiFi.enableSTA(true);
  WiFi.setHostname(host_name);

  while(status_comunicacao == Start_Com){

      //Tenta efetuar conexão no Wifi
    WiFi.begin(ssid_st.c_str(), password_st.c_str());
    for(int8_t i = 0; (WiFi.status() != WL_CONNECTED) && (i < 15); i++){
      vTaskDelayUntil(&xLastWakeTime, 300);
    }
    if(WiFi.status() == WL_CONNECTED){
      status_operacao = Wifi_Conect;
      Serial.println("Wifi conectado:\nHostname: " + String(WiFi.getHostname()) + "\nIP: " + WiFi.localIP().toString());
    } else {
      status_operacao = Wifi_Desc;
    }

      // Enquanto conectado com wifi
    while(WiFi.status() == WL_CONNECTED && status_comunicacao == Start_Com){

        // Testa efetuar conexão com o cliente
      client.connect(ip_st.c_str(), port_st);
      for(int8_t i = 0; !client.connected() && (i < 15); i++){
        vTaskDelayUntil(&xLastWakeTime, 300);
      }
      if(client.connected()){
        client.setTimeout(0);
        status_operacao = Client_Conect;
        Serial.println("Cliete conectado:\nIP: " + client.remoteIP().toString() + "\nPorta: " + String(client.remotePort()));
      } else {
        status_operacao = Wifi_Conect;
      }

        // Enquanto conectado com o cliente
      while (client.connected() && status_comunicacao == Start_Com){
        enviar_receber_msg(&client);
      }
      client.stop();
    }
  }

  client.stop();
  WiFi.enableSTA(false);
  WiFi.disconnect();
}

  // Envia e recebe dados do cliente
void enviar_receber_msg(WiFiClient *client){

    // Recebe dados da task de controle e os envia ao cliente
  if(uxQueueMessagesWaiting(queue_send) > 0){
    /*
      Verifica se há mensagens no buffer de comunicação, se houver ela é enviada do cliente. Apenas uma é enviada por vez.
      ** uxQueueMessagesWaiting - verifica quantas mensagens há no buffer
      ** xQueueReceive - lê uma mensagem contida no buffer
      ** (*client).print - envia mensagem String via WiFi ao cliente conectado
    */

    MSG_SEND msg_send_format;
    xQueueReceive(queue_send, &msg_send_format, 0);
    String msg_send = String(msg_send_format.v_lin, 4) + "|" + String(msg_send_format.v_ang, 4) + "|" + String(msg_send_format.roll, 4) + "|" + String(msg_send_format.pitch, 4) + "|" + String(msg_send_format.yaw, 4) + "\n";
    //String msg_send = String(msg_send_format.v_lin, 4) + "|" + String(msg_send_format.v_ang, 4) + "\n";
   (*client).print(msg_send);
   //Serial.println(msg_send);
  }

    // Recebe dados do clientes e os envia a task de controle
  if((*client).available()){
    /*
      Verifica se há mensagens a serem recebidas do cliente, se houcer ela é rebebida e colocada no buffer.
      ** (*client).available - verifica se há dados enviados pelo cliente
      ** (*client).readStringUntil - lê dados enviados pelo cliente atá um caracter especificado
      ** xQueueSend - salva uma mensagem no buffer
    */

    //String msg_recv = (*client).readString();
    MSG_RESV msg_recv_format;
    msg_recv_format.v_lin = (*client).readStringUntil('|').toFloat();
    msg_recv_format.v_ang = (*client).readStringUntil('\n').toFloat();
    (*client).flush();
    xQueueSend(queue_recv, &msg_recv_format, 0);
    //Serial.println(String(msg_recv_format.v_lin, 2));
  }
}

  // Envia página de configuração
void envia_pag_congiguracao(){
  server.send(200,"text/html", html_pag_config());
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

  server.send(200,"text/html", html_msg_confimacao());
  status_comunicacao = Start_Com;
}

  // Página de configuração de ssid, password, ip, port
String html_pag_config(){
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

  // Mensagem de confirmação de configuração
String html_msg_confimacao(){
  return String("") + "<html lang='pt-br'>"+
  "<head>"+
    "<meta charset='utf-8'/>"+
      "<title>Configuração de Conexão</title>"+
      "<script>alert('Atributos Salvos!');</script>"+
    "</head>"+
  "</html>";
}
