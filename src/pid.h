#pragma once
#include <Arduino.h>

// ================================================================
//  pid.h — Clase ControladorPID reutilizable
//
//  Puedes crear tantas instancias como necesites.
//  Cada instancia tiene su propio estado (integral, error previo).
//  Ejemplo de uso:
//
//    ControladorPID mi_pid(2.5f, 0.05f, 1.2f, -150.0f, 150.0f);
//    mi_pid.resetear();
//
//    // En cada iteracion del loop:
//    float correccion = mi_pid.calcular(setpoint, medicion);
// ================================================================

class ControladorPID {
public:

  // ── Ganancias (los 3 parametros que ajustas en campo) ──────────
  float kp;   // Proporcional: fuerza de reaccion al error actual
  float ki;   // Integral:     corrige errores pequeños persistentes
  float kd;   // Derivativo:   amortigua cambios bruscos del error

  // ── Limites de la salida ───────────────────────────────────────
  float salidaMin;
  float salidaMax;

  // ── Estado interno (no modificar manualmente) ──────────────────
  float          errorAnterior = 0.0f;
  float          integral      = 0.0f;
  unsigned long  tAnterior     = 0;

  // ── Constructor ────────────────────────────────────────────────
  ControladorPID(float p, float i, float d,
                 float minOut = -255.0f, float maxOut = 255.0f)
      : kp(p), ki(i), kd(d), salidaMin(minOut), salidaMax(maxOut) {}

  // ── calcular(setpoint, medicion) ───────────────────────────────
  //
  //  Llama en cada iteracion del loop para obtener la salida.
  //
  //  setpoint  = valor que quieres alcanzar  (ej: 0° = ir recto)
  //  medicion  = valor que mide el sensor    (ej: angulo actual)
  //  retorna   = correccion a aplicar        (ej: diferencial de velocidad)
  //
  //  Internamente calcula:
  //    error     = setpoint - medicion
  //    termino P = kp * error
  //    termino I = ki * integral(error * dt)
  //    termino D = kd * (error - errorAnterior) / dt
  //    salida    = P + I + D  (limitada entre salidaMin y salidaMax)
  // ──────────────────────────────────────────────────────────────
  float calcular(float setpoint, float medicion) {
    unsigned long ahora = millis();
    float dt = (ahora - tAnterior) / 1000.0f;  // milisegundos → segundos
    tAnterior = ahora;

    // Ignora el primer ciclo o pausas largas (dt irreal)
    if (dt <= 0.0f || dt > 0.5f) return 0.0f;

    float error = setpoint - medicion;

    // Termino P — reacciona al error actual
    float termP = kp * error;

    // Termino I — acumula area bajo la curva del error
    integral += error * dt;
    // Anti-windup: evita que la integral crezca sin limite
    // cuando los motores ya estan saturados
    if (ki != 0.0f) {
      integral = constrain(integral, salidaMin / ki, salidaMax / ki);
    }
    float termI = ki * integral;

    // Termino D — velocidad de cambio del error (derivada numerica)
    float termD = kd * (error - errorAnterior) / dt;
    errorAnterior = error;

    // Suma y limita
    float salida = termP + termI + termD;
    return constrain(salida, salidaMin, salidaMax);
  }

  // ── resetear() ─────────────────────────────────────────────────
  //  Limpia el estado interno. Llama esto antes de iniciar un
  //  nuevo movimiento para que la integral y el error anterior
  //  no arrastren valores de la maniobra pasada.
  // ──────────────────────────────────────────────────────────────
  void resetear() {
    errorAnterior = 0.0f;
    integral      = 0.0f;
    tAnterior     = millis();
  }

  void imprimirGanancias() const {
    Serial.printf("[PID] Kp=%.2f  Ki=%.3f  Kd=%.2f  lim=[%.0f, %.0f]\n",
                  kp, ki, kd, salidaMin, salidaMax);
  }
};
