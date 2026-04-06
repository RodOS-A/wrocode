#include "motors.h"
#include "config/config.h"
#include <Arduino.h>

// ── motorsInit() ──────────────────────────────────────────────────
void motorsInit() {
  // Pines de direccion
  pinMode(PIN_MOTOR_L_IN1, OUTPUT);
  pinMode(PIN_MOTOR_L_IN2, OUTPUT);
  pinMode(PIN_MOTOR_R_IN1, OUTPUT);
  pinMode(PIN_MOTOR_R_IN2, OUTPUT);
  pinMode(PIN_MOTOR_STBY,  OUTPUT);

  // Configura los canales PWM del ESP32 (LEDC)
  ledcSetup(PWM_CANAL_L, PWM_FRECUENCIA, PWM_RESOLUCION);
  ledcSetup(PWM_CANAL_R, PWM_FRECUENCIA, PWM_RESOLUCION);
  ledcAttachPin(PIN_MOTOR_L_PWM, PWM_CANAL_L);
  ledcAttachPin(PIN_MOTOR_R_PWM, PWM_CANAL_R);

  // Activa el driver TB6612FNG (STBY HIGH = activo)
  digitalWrite(PIN_MOTOR_STBY, HIGH);

  motorsStop();
}

// ── motorsStop() ──────────────────────────────────────────────────
void motorsStop() {
  digitalWrite(PIN_MOTOR_L_IN1, LOW);
  digitalWrite(PIN_MOTOR_L_IN2, LOW);
  ledcWrite(PWM_CANAL_L, 0);

  digitalWrite(PIN_MOTOR_R_IN1, LOW);
  digitalWrite(PIN_MOTOR_R_IN2, LOW);
  ledcWrite(PWM_CANAL_R, 0);
}

// ── motorLeft() ───────────────────────────────────────────────────
void motorLeft(int vel, bool adelante) {
  vel = constrain(vel, 0, 255);
  digitalWrite(PIN_MOTOR_L_IN1, adelante ? HIGH : LOW);
  digitalWrite(PIN_MOTOR_L_IN2, adelante ? LOW  : HIGH);
  ledcWrite(PWM_CANAL_L, vel);
}

// ── motorRight() ──────────────────────────────────────────────────
void motorRight(int vel, bool adelante) {
  vel = constrain(vel, 0, 255);
  digitalWrite(PIN_MOTOR_R_IN1, adelante ? HIGH : LOW);
  digitalWrite(PIN_MOTOR_R_IN2, adelante ? LOW  : HIGH);
  ledcWrite(PWM_CANAL_R, vel);
}

// ── motorAmbos() ──────────────────────────────────────────────────
void motorAmbos(int velL, int velR, bool adelante) {
  motorLeft(velL,  adelante);
  motorRight(velR, adelante);
}
