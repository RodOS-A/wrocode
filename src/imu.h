#pragma once

// ================================================================
//  imu.h — Interfaz del modulo IMU (MPU6050)
//
//  El MPU6050 mide velocidad angular con el giroscopio (rad/s).
//  Integrando el eje Z obtenemos el angulo de rotacion acumulado
//  (cuantos grados giro el robot desde que se reseteo).
//
//  Variables exportadas:
//    angulo         — angulo actual en grados (+ = izq, - = der)
//    imuDisponible  — true si el MPU6050 fue detectado en I2C
// ================================================================

extern float angulo;
extern bool  imuDisponible;

// Inicializa el MPU6050. Llama en setup().
// Retorna true si se detecto correctamente.
bool initIMU();

// Actualiza el angulo acumulado leyendo el giroscopio.
// Llama lo mas seguido posible (en cada iteracion del loop).
void actualizarAngulo();

// Resetea el angulo a 0. Usa antes de cada movimiento de referencia.
void resetearAngulo();
