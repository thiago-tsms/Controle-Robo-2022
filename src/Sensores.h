#ifndef Sensores_H
#define Sensores_H

#include <Arduino.h>

	/* --- BIBLIOTECAS --- */
#include <BMI160Gen.h>
#include <Wire.h>
#include <math.h>
#include "Kalman.h"



// ====================================================
//  --- --- --- CONFIGURAÇÃO DE COMPILAÇÃO --- --- ---
// ====================================================
#define Print_Dados_Lidos_BMI160_1 false
#define Print_Dados_Lidos_BMI160_2 false
#define Print_Dados_Convertidos_BMI160_1 false
#define Print_Dados_Convertidos_BMI160_2 false
#define Print_Passa_Baixa_BMI160_1 false
#define Print_Passa_Baixa_BMI160_2 false
#define Print_Dados_Fusao false


class Sensores {

	private:

// ===================================================
//  --- --- --- VARIÁVEIS DE CONFIGURAÇÃO --- --- ---
// ===================================================

  const int i2c_addr_imu_1 = 0x68;	// Endereço I2C do CI 1
  const int i2c_addr_imu_2 = 0x69;	// Endereço I2C do CI 2

  const int i2c_sda_pin = 21;	// Pino do SDA
  const int i2c_scl_pin = 22;	// Pino do SCL

  const float gr = 9.80665;       // Valor de gravidade
  const float pi = 3.141592654;   // Pi

  const float const_aRaw = 4.0 / 32768.0;			    // Valor para calculo da aceleração linear
  const float const_gRaw = 250.0 / 32768.0;	  	  // Valor para calculo da aceleração angular
  const float const_calc_ang_acel = 180.0 / pi;		// Valor para o cálculo do angulo a partir do acelerômetro		

  float peso_filtro_passa_baixa_acelerometro = 0.8;
  float peso_filtro_passa_baixa_giroscopio = 0.8;

  float peso_fusao_sensorial_aceleracao = 0.5;
  float peso_fusao_sensorial_giroscopio = 0.5;



// ===================================
//  --- --- --- VARIÁVEIS --- --- ---
// ===================================

    /* Classes instanciadas */
  BMI160GenClass IMU_1;
  BMI160GenClass IMU_2;
  TwoWire I2C = TwoWire(0);

    /* Constantes de tempo */
  unsigned long millisOld = 0;
  float dt = 0;

    /* Filtro de Kalman */
  Kalman kalmanRoll;
  Kalman kalmanPitch;

    /* Estrutura dos dados lidos */
  typedef struct {
      // Dados do acelerômetro
    int axRaw, ayRaw, azRaw;						          // raw values (Valores brutos)			
    float ax, ay, az;               				      // valores convertidos
    float axPB, ayPB, azPB;							          // valores após passa baixa
    float axPBOld = 0, ayPBOld = 0, azPBOld = 0;	// valores passa baixa da iteração anterior

      // Dados do giroscópio
    int gxRaw, gyRaw, gzRaw;        			      	// raw values (Valores brutos)
    float gx, gy, gz;               			      	// valores convertidos
    float gxPB, gyPB, gzPB;							          // valores após passa baixa
    float gxPBOld = 0, gyPBOld = 0, gzPBOld = 0;	// valores passa baixa da iteração anterior
  } Dados_Sensor_t;

    /* Estrutura de dados dos sensores */
  Dados_Sensor_t dados_IMU_1;
  Dados_Sensor_t dados_IMU_2;

    /* Fusão Sensorial */
  float axFusao = 0, ayFusao = 0, azFusao = 0;
  float gxFusao = 0, gyFusao = 0, gzFusao = 0;

    /* Ângulos calculados */
  float roll_acel = 0, pitch_acel = 0, yaw_acel = 0; 	//Roll - Rolagem , Pitch - Arfagem, Yaw - Ginada
  float roll_giro = 0, pitch_giro = 0, yaw_giro = 0; 	//Roll - Rolagem , Pitch - Arfagem, Yaw - Ginada
  float roll = 0, pitch = 0, yaw = 0; 				        //Roll - Rolagem , Pitch - Arfagem, Yaw - Ginada

    /* Velocidade */
  float aceleracao = 0, aceleracaoOld = 0;
  float velocidade = 0, velocidadeOld = 0;
  float distacia = 0, distaciaOld = 0, distaciaTotal = 0;



// =================================
//	--- --- --- FUNÇÕES --- --- ---
// =================================

		// Construtor
	public: Sensores(){

	}

		// Inicia os sensores
	public: void iniciar(){

			// Inicia o I2C
		//I2C.begin(i2c_sda_pin, i2c_scl_pin, 400000);
    I2C.begin();
    I2C.setPins(i2c_sda_pin, i2c_scl_pin);
    delay(10);

		configura_BMI160(&IMU_1, i2c_addr_imu_1, BMI160GenClass::Mode::I2C_MODE);
		configura_BMI160(&IMU_2, i2c_addr_imu_2, BMI160GenClass::Mode::I2C_MODE);
    
	}

    // Lê todos os sensores
	public: void leitura_sensores(){
		leitura_BMI160(&IMU_1, &dados_IMU_1);
		leitura_BMI160(&IMU_2, &dados_IMU_2);

			// Tempo decorrido entre as leituras
    dt = (millis() - millisOld) / 1000.0;
    millisOld = millis();

    #if Print_Dados_Lidos_BMI160_1
      print_dados_lidos(&(dados_IMU_1.axRaw), &(dados_IMU_1.ayRaw), &(dados_IMU_1.azRaw), &(dados_IMU_1.gxRaw), &(dados_IMU_1.gyRaw), &(dados_IMU_1.gzRaw));
    #endif

    #if Print_Dados_Lidos_BMI160_2
      print_dados_lidos(&(dados_IMU_2.axRaw), &(dados_IMU_2.ayRaw), &(dados_IMU_2.azRaw), &(dados_IMU_2.gxRaw), &(dados_IMU_2.gyRaw), &(dados_IMU_2.gzRaw));
    #endif
	}

    // Pré processa os dados
	public: void processa_dados(){
		passa_baixa_IMU(&dados_IMU_1);
		passa_baixa_IMU(&dados_IMU_2);

    #if Print_Dados_Convertidos_BMI160_1
      print_dados(&(dados_IMU_1.ax), &(dados_IMU_1.ay), &(dados_IMU_1.az), &(dados_IMU_1.gx), &(dados_IMU_1.gy), &(dados_IMU_1.gz));
    #endif

    #if Print_Dados_Convertidos_BMI160_2
      print_dados(&(dados_IMU_2.ax), &(dados_IMU_2.ay), &(dados_IMU_2.az), &(dados_IMU_2.gx), &(dados_IMU_2.gy), &(dados_IMU_2.gz));
    #endif

    #if Print_Passa_Baixa_BMI160_1
      print_dados(&(dados_IMU_1.axPB), &(dados_IMU_1.ayPB), &(dados_IMU_1.azPB), &(dados_IMU_1.gxPB), &(dados_IMU_1.gyPB), &(dados_IMU_1.gzPB));
    #endif

    #if Print_Passa_Baixa_BMI160_2
      print_dados(&(dados_IMU_2.axPB), &(dados_IMU_2.ayPB), &(dados_IMU_2.azPB), &(dados_IMU_2.gxPB), &(dados_IMU_2.gyPB), &(dados_IMU_2.gzPB));
    #endif

			// Aplica a fusão sensorial a partir dos dois acelerômetros
    fusao_sensorial(&(dados_IMU_1.axPB), &(dados_IMU_2.axPB), &axFusao, peso_fusao_sensorial_aceleracao);
    fusao_sensorial(&(dados_IMU_1.ayPB), &(dados_IMU_2.ayPB), &ayFusao, peso_fusao_sensorial_aceleracao);
    fusao_sensorial(&(dados_IMU_1.azPB), &(dados_IMU_2.azPB), &azFusao, peso_fusao_sensorial_aceleracao);

			// Aplica a fusão sensorial a partir dos dois giroscópios
    fusao_sensorial(&(dados_IMU_1.gxPB), &(dados_IMU_2.gxPB), &gxFusao, peso_fusao_sensorial_giroscopio);
    fusao_sensorial(&(dados_IMU_1.gyPB), &(dados_IMU_2.gyPB), &gyFusao, peso_fusao_sensorial_giroscopio);
    fusao_sensorial(&(dados_IMU_1.gzPB), &(dados_IMU_2.gzPB), &gzFusao, peso_fusao_sensorial_giroscopio);

    #if Print_Dados_Fusao
      print_dados(&axFusao, &ayFusao, &azFusao, &gxFusao, &gyFusao, &gzFusao);
    #endif
	}

    // Efetua os cálculas de angulos e velocidades
  public: void calcula_angulos(){

    	// Efetua o cálculo dos ângulos
    angulo_acelerometro();
    angulo_giroscopio();

			// Aplica o filtro de kalman
    roll = kalmanRoll.filtro(roll_acel, gxFusao, dt);
    pitch = kalmanPitch.filtro(pitch_acel, gyFusao, dt);
  }

	  // Aplica Filtro Passa baixa
	private: void passa_baixa_IMU(Dados_Sensor_t *dados){
		
			// Converte os valores obtidos
    dados->ax = dados->axRaw * const_aRaw;
    dados->ay = dados->ayRaw * const_aRaw;
    dados->az = dados->azRaw * const_aRaw;
    dados->gx = dados->gxRaw * const_gRaw;
    dados->gy = dados->gyRaw * const_gRaw;
    dados->gz = dados->gzRaw * const_gRaw;

    	// Aplica filtro passa baixa nos valores obtidos pelo acelerômetro
    filtro_passa_baixa(&(dados->ax), &(dados->axPB), &(dados->axPBOld), peso_filtro_passa_baixa_acelerometro);
    filtro_passa_baixa(&(dados->ay), &(dados->ayPB), &(dados->ayPBOld), peso_filtro_passa_baixa_acelerometro);
    filtro_passa_baixa(&(dados->az), &(dados->azPB), &(dados->azPBOld), peso_filtro_passa_baixa_acelerometro);

			// Aplica filto passa baixa nos valores obtidos pelo giroscópio
    filtro_passa_baixa(&(dados->gx), &(dados->gxPB), &(dados->gxPBOld), peso_filtro_passa_baixa_giroscopio);
    filtro_passa_baixa(&(dados->gy), &(dados->gyPB), &(dados->gyPBOld), peso_filtro_passa_baixa_giroscopio);
    filtro_passa_baixa(&(dados->gz), &(dados->gzPB), &(dados->gzPBOld), peso_filtro_passa_baixa_giroscopio);
	}

		// Aplica o filtro passa baixa
	private: void filtro_passa_baixa(float *varNew, float *varFiltNew, float *varFiltOld, float peso_filtro){
		(*varFiltNew) = peso_filtro * (*varFiltOld) + (1 - peso_filtro) * (*varNew);
		(*varFiltOld) = (*varFiltNew);
	}

	  // Aplica a fusão sensorial
	private: void fusao_sensorial(float *varNew_1, float *varNew_2, float *varFilt, float peso_filtro){
		(*varFilt) = peso_filtro * (*varNew_1) + (1 - peso_filtro) * (*varNew_2);
	}

		// Calcula o ângulo a partir dos valores do acelerômetro
	private: void angulo_acelerometro(){
    roll_acel = -atan2(ayFusao, sqrt(pow(axFusao, 2) + pow(azFusao, 2))) * const_calc_ang_acel;
    pitch_acel = -atan2(axFusao, sqrt(pow(ayFusao, 2) + pow(azFusao, 2))) * const_calc_ang_acel;
    //yaw_acel - Não é possível calcular                                                        
  }

    //Calcula o ângulo a partir dos valores do giroscópio
  private: void angulo_giroscopio(){
    //roll_giro -= gxFusao * dt;      //Theta
    //pitch_giro += gyFusao * dt;     //Phi
    yaw_giro += gzFusao * dt;       //Psi

    //if(yaw_giro < 0) yaw_giro = 360 - yaw_giro;
    //if(yaw_giro > 360) yaw_giro = yaw_giro - 360;

    /*
    roll_giro = roll - gxFusao * dt;        //Theta
    pitch_giro = pitch + gyFusao * dt;      //Phi
    yaw_giro = yaw + gzFusao * dt;          //Psi
    */
  }

		// Calcula aceleração e velocidade
	private: void calcula_velocidade(){
		/*float x, y, z;

		x = axFusao * axFusao * gr;
		y = ayFusao * ayFusao * gr;
		z = azFusao * azFusao * gr;

		aceleracao = dados_IMU_1.ax * gr;
		//aceleracao = sqrt(x + y + z) - 1;
		//aceleracao = aceleracao * gr;
		//velocidade = aceleracao * gr * dt;

		velocidade = (aceleracao + aceleracaoOld) * dt / 2;
		distacia = (velocidade + velocidadeOld) * dt / 2;

		aceleracaoOld = aceleracao;
		velocidadeOld = velocidade;
		distaciaOld = distacia;

		distaciaTotal += distacia * 100;

		Serial.println(String(distaciaTotal, 4));*/
	}

    // Obtem os angulos
  public: void get_angulo(float *r, float *p, float *y){
    *r = roll;
    *p= pitch;
    *y = yaw;
  }

// =================================
//	--- --- --- BMI160 --- --- ---
// =================================

		// Configura o sensor BMI160
	private: void configura_BMI160(BMI160GenClass *IMU, int i2c_addr, BMI160GenClass::Mode modo_operacao){

			// Inicia a comunicação por I2C (modo de operação, Classe do I2C ,endereço I2C, pino de interrupção)
		IMU->begin(modo_operacao, I2C, i2c_addr, -1);
		delay(100);

			// Set the accelerometer range to 250 degrees/second
		IMU->setGyroRange(250);
		IMU->setAccelerometerRange(4);
		delay(10);

			/*   ---   CALIBRAÇÃO   ---   */  
		IMU->autoCalibrateXAccelOffset(0);
		IMU->autoCalibrateYAccelOffset(0);
		IMU->autoCalibrateZAccelOffset(1);
		delay(10);
		
		IMU->setGyroOffsetEnabled(1);
		IMU->autoCalibrateGyroOffset();
		IMU->setAccelOffsetEnabled(1);
		delay(10);
	}

	  // Lê dados do sensor
	private: void leitura_BMI160(BMI160GenClass *IMU, Dados_Sensor_t *dados){

			// Lê os valores crus - (read raw gyro measurements from device)
		IMU->readGyro(dados->gxRaw, dados->gyRaw, dados->gzRaw);

			// Lê os valores crus - (read raw accelerometer measurements from device)
		IMU->readAccelerometer(dados->axRaw, dados->ayRaw, dados->azRaw);
	}



// ==================================
//	--- --- --- IMU9250 --- --- ---
// ==================================



// ===============================
//	--- --- --- PRINT --- --- ---
// ===============================

    // Printa dados após leitura, sem nenhuma converção
  private: void print_dados_lidos(int *axRaw, int *ayRaw, int *azRaw, int *gxRaw, int *gyRaw, int *gzRaw){
    Serial.print(*axRaw);
		Serial.print(", ");
		Serial.print(*ayRaw);
		Serial.print(", ");
		Serial.print(*azRaw);
    Serial.print(", ");
    Serial.print(*gxRaw);
		Serial.print(", ");
		Serial.print(*gyRaw);
		Serial.print(", ");
		Serial.println(*gzRaw);
  }

    // Printa dados
  private: void print_dados(float *ax, float *ay, float *az, float *gx, float *gy, float *gz){
    Serial.print(*ax);
		Serial.print(", ");
		Serial.print(*ay);
		Serial.print(", ");
		Serial.print(*az);
    Serial.print(", ");
    Serial.print(*gx);
		Serial.print(", ");
		Serial.print(*gy);
		Serial.print(", ");
		Serial.println(*gz);
  }




  // ==========================================================================
  //	--- --- --- SET E GET DE DADOS DE CONFIGURAÇÃO DE CALIBRAGEM --- --- ---
  // ==========================================================================

  // Peso do filtro passa baixa acelerômetro
  public: void set_peso_filtro_passa_baixa_acelerometro(float data){
    peso_filtro_passa_baixa_acelerometro = data;
  }
  public: float get_peso_filtro_passa_baixa_acelerometro(){
    return peso_filtro_passa_baixa_acelerometro;
  }
  //-------------------------------------------------------------


  // Peso do filtro passa baixa giroscópio
  public: void set_peso_filtro_passa_baixa_giroscopio(float data){
    peso_filtro_passa_baixa_giroscopio = data;
  }

  public: float get_peso_filtro_passa_baixa_giroscopio(){
    return peso_filtro_passa_baixa_giroscopio;
  }
  //-------------------------------------------------------------


  // Peso filtro fusão sensorial acelerômetro
  public: void set_peso_fusao_sensorial_aceleracao(float data){
    peso_fusao_sensorial_aceleracao = data;
  }

  public: float get_peso_fusao_sensorial_aceleracao(){
    return peso_fusao_sensorial_aceleracao;
  }
  //-------------------------------------------------------------

  // Peso filtro fusão sensorial girosópio
  public: void set_peso_fusao_sensorial_giroscopio(float data){
    peso_fusao_sensorial_giroscopio = data;
  }

  public: float get_peso_fusao_sensorial_giroscopio(){
    return peso_fusao_sensorial_giroscopio;
  }
  //-------------------------------------------------------------

};

#endif