#pragma once

// ================================================================
//  movements.h — Movimientos de alto nivel con PID integrado
//
//  Cada funcion encapsula su propio controlador PID.
//  Los parametros del PID se leen desde config/config.h.
//
//  Funciones:
//
//    avanzar(ms)        — avanza recto por 'ms' milisegundos
//                         PID mantiene la direccion usando el IMU
//
//    retroceder(ms)     — retrocede por 'ms' milisegundos
//
//    girar(grados)      — gira exactamente 'grados' usando PID
//                         + = izquierda
//                         - = derecha
//                         El robot se detiene cuando llega al objetivo
//
//  Ejemplo de uso:
//    avanzar(2000);       // avanza 2 segundos
//    girar(90.0f);        // gira 90° a la izquierda
//    girar(-45.0f);       // gira 45° a la derecha
// ================================================================

// Avanza por tiempo (ms). Usa si no tienes encoders calibrados.
void avanzar(int ms);

// Avanza exactamente 'pulsos' pulsos de encoder con PID de direccion.
// Mas preciso que avanzar(ms). Calibra ENCODER_PPR en config.h primero.
void avanzarPulsos(int pulsos);

void retroceder(int ms);
void girar(float grados);
