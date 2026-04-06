#pragma once

// ================================================================
//  config.h — Configuracion central del robot WRO 2026
//
//  ESTE ES EL UNICO ARCHIVO QUE NECESITAS EDITAR para ajustar
//  el comportamiento del robot. Todos los demas modulos leen
//  sus valores desde aqui.
// ================================================================

// ────────────────────────────────────────────────────────────────
//  PINES — Motores (TB6612FNG)
//
//  Cambia estos valores si reconnectas cables.
// ────────────────────────────────────────────────────────────────
#define PIN_MOTOR_L_IN1   16   // Direccion motor izquierdo (AIN1)
#define PIN_MOTOR_L_IN2   17   // Direccion motor izquierdo (AIN2)
#define PIN_MOTOR_L_PWM   18   // Velocidad motor izquierdo (PWMA)
#define PIN_MOTOR_R_IN1   19   // Direccion motor derecho  (BIN1)
#define PIN_MOTOR_R_IN2   21   // Direccion motor derecho  (BIN2)
#define PIN_MOTOR_R_PWM   22   // Velocidad motor derecho  (PWMB)
#define PIN_MOTOR_STBY     4   // Standby driver (HIGH = activo)

// ────────────────────────────────────────────────────────────────
//  PINES — IMU (MPU6050 via I2C)
//
//  GPIO 21/22 ya estan ocupados por los motores,
//  por eso el I2C usa pines alternativos.
// ────────────────────────────────────────────────────────────────
#define PIN_I2C_SDA   25
#define PIN_I2C_SCL   26

// ────────────────────────────────────────────────────────────────
//  CONFIGURACION PWM (generacion de señal para TB6612FNG)
// ────────────────────────────────────────────────────────────────
#define PWM_FRECUENCIA   5000   // Hz — frecuencia de la señal PWM
#define PWM_RESOLUCION      8   // bits — rango de velocidad: 0-255
#define PWM_CANAL_L         0   // Canal LEDC para motor izquierdo
#define PWM_CANAL_R         1   // Canal LEDC para motor derecho

// ────────────────────────────────────────────────────────────────
//  VELOCIDADES
//
//  Rango: 0 (parado) a 255 (maxima potencia).
//  Ajusta segun tu bateria y el peso del robot.
// ────────────────────────────────────────────────────────────────
#define VEL_AVANCE    160   // Velocidad al avanzar recto
#define VEL_GIRO      130   // Velocidad al girar en el lugar
#define VEL_MINIMA     50   // Por debajo de esto el motor no arranca

// ────────────────────────────────────────────────────────────────
//  TIEMPOS — Patron cuadrado
//
//  Ajusta TIEMPO_LADO_MS para que cada lado mida lo que quieres.
//  Ej: si a VEL_AVANCE=160 el robot recorre ~30cm en 1500ms,
//  entonces TIEMPO_LADO_MS=1500 hara lados de ~30cm.
// ────────────────────────────────────────────────────────────────
#define TIEMPO_LADO_MS    2000   // ms que dura cada lado del cuadrado
#define TIEMPO_PAUSA_MS    400   // ms de pausa entre movimientos

// ────────────────────────────────────────────────────────────────
//  PID — Direccion (ir recto)
//
//  Este PID corrige la desviacion del robot mientras avanza.
//  Entrada:  angulo actual (grados) vs angulo de referencia (0°)
//  Salida:   correccion diferencial entre motor izq y der
//
//  Tuning:
//    KP alto  → corrige rapido pero puede oscilar
//    KP bajo  → corrige lento, el robot se desvía
//    KI       → elimina el desvio residual constante (offset)
//    KD       → amortigua las oscilaciones del KP
// ────────────────────────────────────────────────────────────────
#define PID_DIR_KP    2.5f
#define PID_DIR_KI    0.05f
#define PID_DIR_KD    1.2f
#define PID_DIR_MAX   150.0f   // Correccion maxima aplicable (±vel)

// ────────────────────────────────────────────────────────────────
//  PID — Giro (llegar exactamente a un angulo objetivo)
//
//  Este PID gira el robot hasta alcanzar el angulo pedido.
//  Entrada:  angulo objetivo (ej: 90°) vs angulo actual
//  Salida:   velocidad de giro (+ = izquierda, - = derecha)
//
//  Tuning:
//    KP alto  → llega rapido pero puede pasarse del objetivo
//    KP bajo  → llega lento
//    KD       → frena antes de llegar para no pasarse
// ────────────────────────────────────────────────────────────────
#define PID_GIRO_KP    4.0f
#define PID_GIRO_KI    0.01f
#define PID_GIRO_KD    1.5f
#define PID_GIRO_MAX   float(VEL_GIRO)   // Velocidad maxima de giro

// ────────────────────────────────────────────────────────────────
//  PARAMETROS DE GIRO
// ────────────────────────────────────────────────────────────────
#define GIRO_TOLERANCIA   2.0f    // Se considera "llegado" si error < 2°
#define GIRO_TIMEOUT_MS   4000    // Tiempo maximo para completar un giro

// ────────────────────────────────────────────────────────────────
//  PINES — Encoders Hall (motores de traccion)
//
//  Pinout del conector JST de 6 cables del motor:
//    M1  = Blanco → AO1/BO1 del TB6612FNG
//    M2  = Rojo   → AO2/BO2 del TB6612FNG
//    VCC = Negro  → 3.3V ESP32
//    GND = Azul   → GND comun
//    C1  = Verde  → Canal A del encoder (estas GPIOs abajo)
//    C2  = Amarillo → Canal B del encoder (estas GPIOs abajo)
// ────────────────────────────────────────────────────────────────
#define PIN_ENC_L_A   32   // Encoder izq — C1 (verde)   → GPIO 32
#define PIN_ENC_L_B   33   // Encoder izq — C2 (amarillo)→ GPIO 33
#define PIN_ENC_R_A   27   // Encoder der — C1 (verde)   → GPIO 27
#define PIN_ENC_R_B   14   // Encoder der — C2 (amarillo)→ GPIO 14

// Pulsos por revolucion del encoder Hall
// El motor N20 con encoder Hall tipicamente tiene 7 pulsos/rev en el eje del motor.
// Despues de la caja reductora, los pulsos reales dependen de la relacion de reduccion.
// CALIBRAR: haz girar la rueda 1 vuelta completa a mano, cuenta los pulsos en serial.
#define ENCODER_PPR   7

// ────────────────────────────────────────────────────────────────
//  SERIAL
// ────────────────────────────────────────────────────────────────
#define SERIAL_BAUD   115200
