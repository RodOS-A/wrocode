#pragma once

// ================================================================
//  motors.h — Control de bajo nivel del TB6612FNG
//
//  Este modulo NO conoce el PID ni los movimientos. Solo sabe
//  como hablarle directamente al driver de motores.
//
//  Funciones disponibles:
//    motorsInit()              — configura pines y PWM
//    motorsStop()              — frena ambos motores
//    motorLeft(vel, adelante)  — controla motor izquierdo
//    motorRight(vel, adelante) — controla motor derecho
//    motorAmbos(velL, velR, adelante) — controla ambos a la vez
// ================================================================

// Inicializa pines y LEDC. Llama una vez en setup().
void motorsInit();

// Para ambos motores inmediatamente (corta la corriente).
void motorsStop();

// Controla el motor izquierdo individualmente.
//   vel:      0-255
//   adelante: true = hacia adelante, false = hacia atras
void motorLeft(int vel, bool adelante);

// Controla el motor derecho individualmente.
void motorRight(int vel, bool adelante);

// Controla ambos motores con velocidades independientes.
// Util para aplicar la correccion del PID directamente.
void motorAmbos(int velL, int velR, bool adelante);
