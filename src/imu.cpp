#include "imu.h"
#include "config/config.h"
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>

// ── Variables globales exportadas ─────────────────────────────────
float angulo        = 0.0f;
bool  imuDisponible = false;

// ── Estado interno ────────────────────────────────────────────────
static Adafruit_MPU6050 mpu;
static unsigned long    tUltimaLectura = 0;

// ── initIMU() ─────────────────────────────────────────────────────
bool initIMU() {
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);

  if (!mpu.begin()) {
    Serial.println("[IMU] ERROR: MPU6050 no detectado.");
    Serial.println("[IMU] Verifica cableado: SDA->GPIO25, SCL->GPIO26");
    imuDisponible = false;
    return false;
  }

  // Rango ±250°/s: mas que suficiente para este robot
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);

  // Filtro paso-bajo 21 Hz: reduce ruido sin agregar mucho retardo
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  tUltimaLectura = millis();
  imuDisponible  = true;
  Serial.println("[IMU] MPU6050 OK — SDA:25, SCL:26, rango:250°/s");
  return true;
}

// ── actualizarAngulo() ────────────────────────────────────────────
//
//  Lee velocidad angular Z (rad/s) y la integra para obtener grados.
//  Cuanto mas frecuente se llame, mas precisa es la integracion.
//
//  Formula:
//    angulo += velocidad_angular_Z (°/s) * dt (s)
//
//  Convenio de signos:
//    angulo positivo = robot giró a la IZQUIERDA
//    angulo negativo = robot giró a la DERECHA
// ─────────────────────────────────────────────────────────────────
void actualizarAngulo() {
  if (!imuDisponible) return;

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  unsigned long ahora = millis();
  float dt = (ahora - tUltimaLectura) / 1000.0f;
  tUltimaLectura = ahora;

  // Descarta dt irreal (primer ciclo o pausa larga)
  if (dt <= 0.0f || dt > 0.5f) return;

  // g.gyro.z esta en rad/s → convertir a grados/s
  angulo += (g.gyro.z * 180.0f / PI) * dt;
}

// ── resetearAngulo() ──────────────────────────────────────────────
void resetearAngulo() {
  angulo = 0.0f;
  tUltimaLectura = millis();
}
