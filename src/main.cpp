/*
 * =====================================================
 *  WRO 2026 - Mosaic Masters
 *  ESP32-WROOM-32D + TB6612FNG + MPU6050
 *
 *  main.cpp — Patron cuadrado perfecto
 *
 *  El robot ejecuta 4 lados de un cuadrado:
 *    avanzar → girar 90° izq → avanzar → girar 90° izq → ...
 *
 *  Todos los parametros (velocidad, tiempo, PID) estan en:
 *    src/config/config.h
 *
 *  La logica de movimiento esta en:
 *    src/movements.h / movements.cpp
 * =====================================================
 */

#include <Arduino.h>
#include "config/config.h"
#include "motors.h"
#include "imu.h"
#include "movements.h"

// ── Numero de lados del patron ────────────────────────────────────
// 4 = cuadrado, 3 = triangulo, 6 = hexagono, etc.
#define LADOS_PATRON   4

// ── Angulo de giro en cada esquina (grados) ───────────────────────
// 90.0f = cuadrado perfecto (izquierda)
// Cambia a -90.0f para girar a la derecha
#define ANGULO_GIRO   90.0f

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(500);

  // Inicializa motores
  motorsInit();

  // Inicializa IMU (MPU6050)
  bool imuOK = initIMU();

  Serial.println("\n=============================");
  Serial.println(" WRO 2026 - Patron Cuadrado");
  Serial.println("=============================");
  Serial.printf("  Lados:      %d\n",    LADOS_PATRON);
  Serial.printf("  Angulo:     %.0f°\n", ANGULO_GIRO);
  Serial.printf("  Tiempo lado:%d ms\n", TIEMPO_LADO_MS);
  Serial.printf("  Velocidad:  %d/255\n",VEL_AVANCE);
  Serial.printf("  PID dir:    Kp=%.1f Ki=%.2f Kd=%.1f\n",
                PID_DIR_KP, PID_DIR_KI, PID_DIR_KD);
  Serial.printf("  PID giro:   Kp=%.1f Ki=%.2f Kd=%.1f\n",
                PID_GIRO_KP, PID_GIRO_KI, PID_GIRO_KD);
  Serial.printf("  IMU:        %s\n", imuOK ? "OK" : "NO DETECTADO");
  Serial.println("=============================\n");

  // Espera 3 segundos antes de empezar (tiempo para soltar el robot)
  Serial.println("Iniciando en 3 segundos...");
  for (int i = 3; i > 0; i--) {
    Serial.printf("  %d...\n", i);
    delay(1000);
  }
  Serial.println("INICIO\n");
}

void loop() {
  // ── Patron cuadrado: 4 lados de igual longitud ─────────────────
  //
  //  Esquema del recorrido (vista desde arriba):
  //
  //    INICIO
  //      ↑ lado 1
  //      ┌──────
  //      │      ← lado 4
  //      │
  //  lado 2 ↓    (cada giro es 90° a la izquierda)
  //      │
  //      └──────
  //            lado 3 →
  //
  //  Al completar los 4 lados el robot deberia estar
  //  exactamente en el punto de inicio.

  for (int lado = 1; lado <= LADOS_PATRON; lado++) {
    Serial.printf("── LADO %d/%d ──────────────────\n", lado, LADOS_PATRON);

    // ── 1. Avanzar recto con PID de direccion ──────────────────
    Serial.printf("   Avanzando %d ms...\n", TIEMPO_LADO_MS);
    avanzar(TIEMPO_LADO_MS);

    // ── 2. Pausa breve entre movimientos ───────────────────────
    delay(TIEMPO_PAUSA_MS);

    // ── 3. Girar con PID (excepto despues del ultimo lado) ─────
    if (lado < LADOS_PATRON) {
      Serial.printf("   Girando %.0f grados...\n", ANGULO_GIRO);
      girar(ANGULO_GIRO);
      delay(TIEMPO_PAUSA_MS);
    }
  }

  // ── Patron completo ────────────────────────────────────────────
  Serial.println("\n✓ PATRON CUADRADO COMPLETO\n");
  motorsStop();

  // Se detiene para siempre — reinicia el ESP32 para repetir
  while (true) { delay(1000); }
}
