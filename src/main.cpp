//#include <BMI160Gen.h>
//#include <Wire.h>

#include "Sensores.h"

/*const int i2c_addr = 0x68;

TwoWire I2C = TwoWire(1);

void setup() {
  Serial.begin(115200); // initialize Serial communication
  while (!Serial);    // wait for the serial port to open

  // initialize device
  //I2C.setPins(21, 22);
  I2C.begin(21,22,400000);
  //I2C.beginTransmission(0x68);
  BMI160.begin(BMI160GenClass::I2C_MODE, I2C, 0x68, -1);
}

void loop() {
  int gx, gy, gz;         // raw gyro values

  // read raw gyro measurements from device
  BMI160.readGyro(gx, gy, gz);

  // display tab-separated gyro x/y/z values
  Serial.print("g:\t");
  Serial.print(gx);
  Serial.print("\t");
  Serial.print(gy);
  Serial.print("\t");
  Serial.print(gz);
  Serial.println();

  delay(500);
}*/

Sensores sensores;

void setup(){
  Serial.begin(115200);
  sensores.inicia();
}

void loop(){
  sensores.leitura_sensores();
  sensores.calcula_dados();
  //sensores.print_dados();
  delay(50);
}