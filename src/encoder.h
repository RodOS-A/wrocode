#pragma once

// ================================================================
//  encoder.h — Lectura de encoders Hall por interrupciones
//
//  Cada motor tiene 2 señales: C1 (canal A) y C2 (canal B).
//  Usamos interrupciones de hardware en C1 para contar pulsos.
//  El nivel de C2 al momento del pulso determina la DIRECCION.
//
//  Convenio de signos:
//    Pulsos positivos = motor avanzando hacia adelante
//    Pulsos negativos = motor retrocediendo
//
//  Uso tipico:
//    encoderInit();
//    resetEncoders();
//    // ... mover motores ...
//    long pulsosIzq = getEncLeft();
//    long pulsosDer = getEncRight();
// ================================================================

// Inicializa pines e interrupciones. Llama en setup().
void encoderInit();

// Reinicia ambos contadores a 0.
// Llama antes de cada movimiento que uses distancia por pulsos.
void resetEncoders();

// Devuelve pulsos acumulados del motor izquierdo.
// Positivo = adelante, Negativo = atras.
long getEncLeft();

// Devuelve pulsos acumulados del motor derecho.
long getEncRight();

// Devuelve el promedio de pulsos de ambos motores (distancia media).
long getEncAvg();

// Imprime estado de encoders por Serial (debug).
void encoderPrint();
