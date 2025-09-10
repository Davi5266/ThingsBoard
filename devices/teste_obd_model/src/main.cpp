#include <Arduino.h>
#include <Wire.h>
// #include "FreematicsPlus.h"   // ou FreematicsOne.h, dependendo do firmware
// #include <FreematicsPlus.h>
#include <FreematicsONE.h>

CFreematics telematics;

void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println("=== Detectando modelo Freematics ===");

  // Inicializa I2C (usado por IMU e outros sensores)
  Wire.begin();

  // Testa IMU
  bool hasIMU = telematics.beginIMU();
  if (hasIMU) {
    Serial.println("IMU detectado ✅");
  } else {
    Serial.println("IMU não encontrado ❌");
  }

  // Testa GPS
  bool hasGPS = telematics.beginGPS();
  if (hasGPS) {
    Serial.println("GPS detectado ✅");
  } else {
    Serial.println("GPS não encontrado ❌");
  }

  // Agora inferimos o modelo
  if (hasGPS && hasIMU) {
    Serial.println("Provavelmente modelo H");
  } else if (hasGPS && !hasIMU) {
    Serial.println("Provavelmente modelo B");
  } else if (!hasGPS && !hasIMU) {
    Serial.println("Provavelmente modelo A");
  } else {
    Serial.println("Modelo não identificado com certeza.");
  }
}

void loop() {
  // nada
}
