#include "movements.h"
#include "config/config.h"
#include "pid.h"
#include "motors.h"
#include "imu.h"
#include <Arduino.h>

// ================================================================
//  avanzar(ms)
//
//  Mueve el robot hacia adelante durante 'ms' milisegundos.
//
//  PID de DIRECCION:
//    Setpoint  = angulo de referencia al inicio del avance (0° relativo)
//    Medicion  = angulo actual del IMU
//    Salida    = correccion diferencial entre motor izq y der
//
//  Como se aplica la correccion:
//
//    vel_izq = VEL_AVANCE - correccion
//    vel_der = VEL_AVANCE + correccion
//
//  Si el robot gira a la IZQUIERDA (angulo sube):
//    error = 0 - angulo_positivo = negativo
//    correccion es negativa
//    vel_izq = VEL_AVANCE + |corr|  → motor izq va MAS rapido
//    vel_der = VEL_AVANCE - |corr|  → motor der va MAS lento
//    → el robot corrige girando de vuelta a la derecha ✓
//
//  Si el robot gira a la DERECHA (angulo baja):
//    lo contrario → corrige girando a la izquierda ✓
// ================================================================
void avanzar(int ms) {
  // Crea el PID con los valores de config.h
  ControladorPID pidDir(PID_DIR_KP, PID_DIR_KI, PID_DIR_KD,
                        -PID_DIR_MAX, PID_DIR_MAX);
  pidDir.resetear();

  // El angulo de referencia es el angulo ACTUAL al inicio del movimiento
  // (no siempre es 0, puede haber un giro previo)
  float anguloReferencia = angulo;

  unsigned long inicio = millis();

  while (millis() - inicio < (unsigned long)ms) {
    actualizarAngulo();

    float correccion = pidDir.calcular(anguloReferencia, angulo);

    int velL = constrain((int)(VEL_AVANCE - correccion), VEL_MINIMA, 255);
    int velR = constrain((int)(VEL_AVANCE + correccion), VEL_MINIMA, 255);
    motorAmbos(velL, velR, true);

    // Debug cada 250ms
    static unsigned long tDebug = 0;
    if (millis() - tDebug > 250) {
      tDebug = millis();
      float error = anguloReferencia - angulo;
      Serial.printf("[AVANZAR] t=%lums  ang=%.1f°  err=%.1f°  corr=%.1f  L=%d  R=%d\n",
                    millis() - inicio, angulo, error, correccion, velL, velR);
    }

    delay(5);  // ~200 Hz de actualizacion del PID
  }

  motorsStop();
}

// ================================================================
//  retroceder(ms)
//
//  Retrocede sin PID de direccion (movimiento corto).
//  Si necesitas retroceder recto, se puede agregar el PID igual
//  que en avanzar(), invirtiendo el signo de la correccion.
// ================================================================
void retroceder(int ms) {
  motorAmbos(VEL_AVANCE, VEL_AVANCE, false);
  delay(ms);
  motorsStop();
}

// ================================================================
//  girar(grados)
//
//  Gira el robot exactamente 'grados' usando PID.
//
//    grados > 0 = giro a la IZQUIERDA
//    grados < 0 = giro a la DERECHA
//
//  PID de GIRO:
//    Setpoint  = angulo objetivo = angulo_actual + grados
//    Medicion  = angulo actual del IMU
//    Salida    = velocidad de giro (+ = izq, - = der)
//
//  Como se aplica la velocidad de giro:
//    Si salida > 0 (girar izquierda):
//      motor izq: ATRAS  a |salida|
//      motor der: ADELANTE a |salida|
//    Si salida < 0 (girar derecha):
//      motor izq: ADELANTE a |salida|
//      motor der: ATRAS   a |salida|
//
//  El loop termina cuando:
//    |error| <= GIRO_TOLERANCIA  (llegamos al angulo objetivo)
//    o se supera GIRO_TIMEOUT_MS (seguridad ante fallos del IMU)
// ================================================================
void girar(float grados) {
  if (!imuDisponible) {
    // Sin IMU: giro temporizado (menos preciso)
    Serial.println("[GIRAR] IMU no disponible. Usando tiempo fijo.");
    // Ajusta este valor empiricamente segun tu robot
    int msGiro90 = 600;
    int ms = (int)(abs(grados) / 90.0f * msGiro90);
    bool izquierda = (grados > 0);
    motorLeft (VEL_GIRO, !izquierda);  // izq: atras si giro izq
    motorRight(VEL_GIRO,  izquierda);  // der: adelante si giro izq
    delay(ms);
    motorsStop();
    return;
  }

  // ── Con IMU: PID de giro ─────────────────────────────────────
  ControladorPID pidGiro(PID_GIRO_KP, PID_GIRO_KI, PID_GIRO_KD,
                         -PID_GIRO_MAX, PID_GIRO_MAX);
  pidGiro.resetear();

  float anguloObjetivo = angulo + grados;

  Serial.printf("[GIRAR] %.1f° → objetivo: %.1f°\n", grados, anguloObjetivo);

  unsigned long inicio = millis();

  while (millis() - inicio < GIRO_TIMEOUT_MS) {
    actualizarAngulo();

    float error = anguloObjetivo - angulo;

    // Condicion de llegada: dentro de la tolerancia
    if (abs(error) <= GIRO_TOLERANCIA) {
      Serial.printf("[GIRAR] OK  angulo=%.1f°  error=%.1f°  t=%lums\n",
                    angulo, error, millis() - inicio);
      break;
    }

    // Calcula la velocidad de giro con el PID
    float velGiro = pidGiro.calcular(anguloObjetivo, angulo);
    int   vel     = constrain((int)abs(velGiro), VEL_MINIMA, VEL_GIRO);

    // Aplica direccion segun el signo de la salida del PID
    if (velGiro > 0) {
      // Girar a la IZQUIERDA: izq atras, der adelante
      motorLeft (vel, false);
      motorRight(vel, true);
    } else {
      // Girar a la DERECHA: izq adelante, der atras
      motorLeft (vel, true);
      motorRight(vel, false);
    }

    delay(5);  // ~200 Hz
  }

  motorsStop();
}
