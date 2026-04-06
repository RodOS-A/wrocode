#include "encoder.h"
#include "config/config.h"
#include <Arduino.h>

// ================================================================
//  encoder.cpp — Implementacion con interrupciones de hardware
//
//  Por que interrupciones?
//  Los encoders Hall generan pulsos muy rapido (cientos por segundo).
//  Si leemos los pines en el loop() principal, podemos perdernos pulsos
//  porque el loop hace otras cosas (PID, serial, etc.).
//  Las interrupciones de hardware detienen el programa al instante
//  cuando llega un pulso, cuentan, y regresan — sin perder nada.
//
//  Por que "volatile"?
//  Las variables modificadas dentro de una interrupcion deben ser
//  "volatile". Esto le dice al compilador: "no optimices esta variable,
//  su valor puede cambiar en cualquier momento desde afuera del flujo
//  normal del programa". Sin volatile, el compilador puede cachear el
//  valor y el programa nunca veria las actualizaciones de la ISR.
// ================================================================

// ── Contadores de pulsos (modificados por las ISRs) ───────────────
volatile long contadorIzq = 0;
volatile long contadorDer = 0;

// ── ISR (Interrupt Service Routine) — Motor Izquierdo ─────────────
//
//  Se ejecuta en cada flanco de subida del canal A (C1 verde).
//  Lee el canal B (C2 amarillo) para saber la direccion:
//    C2 = HIGH al pulsar C1 → motor girando hacia adelante → sumar
//    C2 = LOW  al pulsar C1 → motor girando hacia atras   → restar
//
//  IRAM_ATTR: coloca la funcion en RAM interna del ESP32.
//  Las ISRs deben estar en RAM (no en flash) porque el acceso
//  a flash puede estar en cache y causaria retrasos o crashes.
// ─────────────────────────────────────────────────────────────────
void IRAM_ATTR isrEncoderIzq() {
  if (digitalRead(PIN_ENC_L_B) == HIGH) {
    contadorIzq++;
  } else {
    contadorIzq--;
  }
}

// ── ISR — Motor Derecho ───────────────────────────────────────────
void IRAM_ATTR isrEncoderDer() {
  if (digitalRead(PIN_ENC_R_B) == HIGH) {
    contadorDer++;
  } else {
    contadorDer--;
  }
}

// ── encoderInit() ─────────────────────────────────────────────────
void encoderInit() {
  // Configura C1 y C2 como entradas con pull-up interno.
  // INPUT_PULLUP evita lecturas flotantes cuando el encoder
  // no esta activo (el resistor interno mantiene el pin en HIGH).
  pinMode(PIN_ENC_L_A, INPUT_PULLUP);
  pinMode(PIN_ENC_L_B, INPUT_PULLUP);
  pinMode(PIN_ENC_R_A, INPUT_PULLUP);
  pinMode(PIN_ENC_R_B, INPUT_PULLUP);

  // Adjunta las interrupciones al flanco de SUBIDA del canal A.
  // RISING = dispara cuando el pin pasa de LOW a HIGH.
  // Cada pulso del encoder genera un flanco de subida.
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_L_A), isrEncoderIzq, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_R_A), isrEncoderDer, RISING);

  Serial.println("[ENC] Encoders inicializados — interrupciones activas");
  Serial.printf("[ENC] Pines: IZQ A=%d B=%d | DER A=%d B=%d\n",
                PIN_ENC_L_A, PIN_ENC_L_B, PIN_ENC_R_A, PIN_ENC_R_B);
}

// ── resetEncoders() ───────────────────────────────────────────────
void resetEncoders() {
  // Deshabilita interrupciones brevemente para leer/escribir de
  // forma atomica (evita que una ISR modifique el contador a mitad
  // de la escritura, lo que daria un valor corrupto).
  noInterrupts();
  contadorIzq = 0;
  contadorDer = 0;
  interrupts();
}

// ── getEncLeft() / getEncRight() ──────────────────────────────────
long getEncLeft() {
  noInterrupts();
  long val = contadorIzq;
  interrupts();
  return val;
}

long getEncRight() {
  noInterrupts();
  long val = contadorDer;
  interrupts();
  return val;
}

// ── getEncAvg() ───────────────────────────────────────────────────
long getEncAvg() {
  noInterrupts();
  long avg = (contadorIzq + contadorDer) / 2;
  interrupts();
  return avg;
}

// ── encoderPrint() ────────────────────────────────────────────────
void encoderPrint() {
  noInterrupts();
  long izq = contadorIzq;
  long der = contadorDer;
  interrupts();
  Serial.printf("[ENC] Izq=%ld  Der=%ld  Avg=%ld\n", izq, der, (izq + der) / 2);
}
