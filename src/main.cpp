/*
 * =====================================================
 *  WRO 2026 - Mosaic Masters - Motor Test + PID v2.0
 *  Hardware: ESP32-WROOM-32D + TB6612FNG + MPU6050
 * =====================================================
 *
 * COMANDOS POR SERIAL (115200 baud):
 *   w = adelante (con PID si esta activado)
 *   s = atras
 *   a = giro izq    d = giro derecha
 *   q = parar
 *   p = activar/desactivar PID de direccion
 *   c = calibrar (resetear angulo a 0)
 *   t = test automatico
 *   1-9 = velocidad (50 a 250)
 *   h = ayuda
 *
 * CABLEADO MOTORES (TB6612FNG):
 *   GPIO 16 -> AIN1  (direccion motor izquierdo)
 *   GPIO 17 -> AIN2  (direccion motor izquierdo)
 *   GPIO 18 -> PWMA  (velocidad motor izquierdo)
 *   GPIO 19 -> BIN1  (direccion motor derecho)
 *   GPIO 21 -> BIN2  (direccion motor derecho)
 *   GPIO 22 -> PWMB  (velocidad motor derecho)
 *   GPIO  4 -> STBY  (HIGH = driver activo)
 *
 * CABLEADO IMU (MPU6050):
 *   GPIO 25 -> SDA   (datos I2C)
 *   GPIO 26 -> SCL   (reloj I2C)
 *   3.3V    -> VCC
 *   GND     -> GND
 *
 * NOTA: GPIO 21/22 ya estan ocupados por los motores.
 *   Por eso el I2C usa GPIO 25/26 en lugar de los pines
 *   por defecto del ESP32.
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// =====================================================
//  PINES — motores
// =====================================================

#define MOTOR_L_IN1  16
#define MOTOR_L_IN2  17
#define MOTOR_L_PWM  18
#define MOTOR_R_IN1  19
#define MOTOR_R_IN2  21
#define MOTOR_R_PWM  22
#define MOTOR_STBY    4

// Pines I2C alternativos (21/22 estan en uso)
#define I2C_SDA      25
#define I2C_SCL      26

// =====================================================
//  CONFIGURACION PWM
// =====================================================

#define PWM_FREQ   5000
#define PWM_RES       8   // 8 bits → 0 a 255
#define PWM_CH_L      0
#define PWM_CH_R      1

// =====================================================
//  VARIABLES GLOBALES
// =====================================================

int  velocidadActual = 150;
bool pidActivado     = false;  // El PID empieza apagado

// =====================================================
//
//  ╔══════════════════════════════════════════════╗
//  ║           SISTEMA PID — EXPLICACION          ║
//  ╚══════════════════════════════════════════════╝
//
//  PID = Proporcional + Integral + Derivativo.
//  Es un algoritmo de control que corrige el error
//  entre lo que QUIERES y lo que MIDES.
//
//  En este robot: queremos ir RECTO.
//  El error = angulo actual - angulo deseado (0 grados).
//  La SALIDA del PID es la correccion que aplicamos
//  a los motores para compensar la desviacion.
//
//  Ejemplo visual:
//
//       Setpoint (quiero)
//           |
//           |  error = setpoint - medicion
//           ↓
//  [ PID ] ─→ salida ─→ [ Motores ] ─→ robot se mueve
//                                           |
//                              ←────────────┘
//                          retroalimentacion (feedback)
//                          medicion del MPU6050
//
//  ─────────────────────────────────────────────────
//  TERMINO P (Proporcional):
//    salida_P = Kp × error
//
//    Reacciona directamente al error actual.
//    Si el robot esta desviado 10°, aplica una
//    correccion proporcional a esos 10°.
//    → Kp grande: reaccion rapida pero puede oscilar.
//    → Kp pequeno: reaccion lenta, el error tarda en
//      corregirse.
//
//  TERMINO I (Integral):
//    salida_I = Ki × Σ(error × dt)
//
//    Acumula el error a lo largo del tiempo.
//    Sirve para corregir errores pequeños persistentes
//    que el termino P no puede eliminar por si solo
//    (offset estatico). Si el robot siempre va 2° a la
//    derecha sin corregirse, la integral lo compensa.
//    → Ki muy grande puede causar "windup" (el robot
//      sobrecompensa y oscila).
//
//  TERMINO D (Derivativo):
//    salida_D = Kd × (error - errorAnterior) / dt
//
//    Reacciona a la VELOCIDAD del cambio del error.
//    Si el error esta disminuyendo rapido, el termino D
//    "frena" la correccion para evitar que el robot se
//    pase del setpoint.
//    → Actua como amortiguador: reduce las oscilaciones.
//    → Kd muy grande amplifica el ruido del sensor.
//
//  SALIDA FINAL:
//    salida = (Kp × error) + (Ki × integral) + (Kd × derivada)
//
//  Esta salida se usa como DIFERENCIAL entre los dos
//  motores: un motor acelera y el otro desacelera para
//  corregir la direccion del robot.
//
// =====================================================

class ControladorPID {
public:
  // ── Ganancias (los tres parametros que ajustas) ──
  float kp;   // Proporcional
  float ki;   // Integral
  float kd;   // Derivativo

  // ── Estado interno del controlador ──
  float errorAnterior = 0;      // Error del ciclo anterior (para calcular D)
  float integral      = 0;      // Suma acumulada del error (termino I)
  unsigned long tAnterior = 0;  // Tiempo del ultimo calculo (para delta t)

  // ── Limites de salida (evitan saturacion) ──
  float salidaMin;
  float salidaMax;

  // Constructor: inicializa ganancias y limites
  ControladorPID(float p, float i, float d,
                 float minOut = -200.0f, float maxOut = 200.0f)
      : kp(p), ki(i), kd(d), salidaMin(minOut), salidaMax(maxOut) {}

  /*
   * calcular(setpoint, medicion)
   *
   * Llama a esta funcion en cada iteracion del loop.
   * setpoint  = el valor que quieres alcanzar (p.ej. 0 grados)
   * medicion  = el valor que mide el sensor    (angulo actual)
   * retorna   = la correccion a aplicar a los motores
   */
  float calcular(float setpoint, float medicion) {
    unsigned long ahora = millis();
    float dt = (ahora - tAnterior) / 1000.0f;  // convierte ms a segundos
    tAnterior = ahora;

    // Descarta el primer ciclo o pausas largas (dt irreal)
    if (dt <= 0.0f || dt > 0.5f) return 0.0f;

    // ── Termino P ──
    float error = setpoint - medicion;
    float termP = kp * error;

    // ── Termino I ──
    // Acumula el area bajo la curva del error
    integral += error * dt;

    // Anti-windup: limita la integral para que no crezca sin control
    // si el motor ya esta saturado (no puede corregir mas)
    integral = constrain(integral, salidaMin / ki, salidaMax / ki);
    float termI = ki * integral;

    // ── Termino D ──
    // Velocidad de cambio del error (derivada numerica)
    float derivada = (error - errorAnterior) / dt;
    float termD    = kd * derivada;
    errorAnterior  = error;  // Guarda para el proximo ciclo

    // ── Suma y limita la salida ──
    float salida = termP + termI + termD;
    salida = constrain(salida, salidaMin, salidaMax);

    return salida;
  }

  /*
   * resetear() — limpia el estado interno del PID.
   * Llama esto cuando el robot se detiene o cambia de modo,
   * para que la integral no arrastre valores viejos.
   */
  void resetear() {
    errorAnterior = 0;
    integral      = 0;
    tAnterior     = millis();
  }

  // Muestra las ganancias actuales por Serial
  void imprimirGanancias() {
    Serial.printf("[PID] Kp=%.2f  Ki=%.2f  Kd=%.2f\n", kp, ki, kd);
  }
};

// =====================================================
//  MPU6050 — Giroscopio / Acelerometro
//
//  El MPU6050 mide:
//    - Aceleracion lineal en 3 ejes (m/s²)
//    - Velocidad angular en 3 ejes (rad/s)
//
//  Para saber si el robot gira, usamos el eje Z
//  del giroscopio (giro.z = yaw rate).
//  Integrando esa velocidad angular obtenemos el
//  angulo acumulado (cuantos grados giro el robot).
// =====================================================

Adafruit_MPU6050 mpu;
bool     imuDisponible  = false;
float    angulo         = 0.0f;  // Angulo acumulado en grados
unsigned long tIMU      = 0;

// PID para mantener la direccion recta
// Ajusta Kp, Ki, Kd segun el comportamiento real del robot:
//   - Si oscila mucho: baja Kp o sube Kd
//   - Si corrige muy lento: sube Kp
//   - Si siempre queda desviado: sube Ki
ControladorPID pidDireccion(2.5f, 0.05f, 1.2f, -150.0f, 150.0f);

bool initIMU() {
  Wire.begin(I2C_SDA, I2C_SCL);
  if (!mpu.begin()) {
    Serial.println("[IMU] MPU6050 no encontrado. Revisa cableado SDA/SCL.");
    Serial.println("[IMU] Continuando SIN PID de direccion.");
    return false;
  }

  // Rango del giroscopio: ±250 grados/segundo es suficiente para este robot
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);

  // Filtro paso-bajo: reduce el ruido de alta frecuencia del sensor
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  tIMU = millis();
  Serial.println("[IMU] MPU6050 OK — PID de direccion disponible.");
  return true;
}

/*
 * actualizarAngulo()
 *
 * Lee el giroscopio y acumula el angulo de rotacion (yaw).
 * Debe llamarse lo mas frecuente posible dentro del loop.
 *
 * Formula:
 *   angulo += velocidad_angular_z (°/s) × dt (s)
 */
void actualizarAngulo() {
  if (!imuDisponible) return;

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  unsigned long ahora = millis();
  float dt = (ahora - tIMU) / 1000.0f;
  tIMU = ahora;

  if (dt <= 0.0f || dt > 0.5f) return;

  // g.gyro.z esta en rad/s → convertimos a grados/s multiplicando por 180/PI
  angulo += (g.gyro.z * 180.0f / PI) * dt;
}

// =====================================================
//  FUNCIONES DE CONTROL DE MOTORES
// =====================================================

void motorsStop() {
  digitalWrite(MOTOR_L_IN1, LOW);
  digitalWrite(MOTOR_L_IN2, LOW);
  ledcWrite(PWM_CH_L, 0);

  digitalWrite(MOTOR_R_IN1, LOW);
  digitalWrite(MOTOR_R_IN2, LOW);
  ledcWrite(PWM_CH_R, 0);

  pidDireccion.resetear();  // Limpia el PID al frenar
  Serial.println(">> PARADO");
}

void motorLeft(int speed, bool forward) {
  speed = constrain(speed, 0, 255);
  digitalWrite(MOTOR_L_IN1, forward ? HIGH : LOW);
  digitalWrite(MOTOR_L_IN2, forward ? LOW  : HIGH);
  ledcWrite(PWM_CH_L, speed);
}

void motorRight(int speed, bool forward) {
  speed = constrain(speed, 0, 255);
  digitalWrite(MOTOR_R_IN1, forward ? HIGH : LOW);
  digitalWrite(MOTOR_R_IN2, forward ? LOW  : HIGH);
  ledcWrite(PWM_CH_R, speed);
}

// ─────────────────────────────────────────────────
//  moveForward CON PID:
//
//  La correccion del PID se aplica como diferencial:
//
//    vel_izq = velocidad - correccion
//    vel_der = velocidad + correccion
//
//  Si el robot gira a la derecha (angulo > 0):
//    error = 0 - angulo = negativo
//    correccion = negativa
//    vel_izq = velocidad + |correccion| → izquierdo va MAS rapido
//    vel_der = velocidad - |correccion| → derecho va MAS lento
//    Resultado: el robot corrige girando de vuelta a la izquierda
//
//  Si el robot gira a la izquierda (angulo < 0): lo contrario.
// ─────────────────────────────────────────────────
void moveForward(int speed) {
  if (pidActivado && imuDisponible) {
    actualizarAngulo();
    float correccion = pidDireccion.calcular(0.0f, angulo);

    int velIzq = constrain((int)(speed - correccion), 0, 255);
    int velDer = constrain((int)(speed + correccion), 0, 255);

    motorLeft(velIzq, true);
    motorRight(velDer, true);

    // Muestra datos de debug cada ~200ms para no saturar el serial
    static unsigned long tDebug = 0;
    if (millis() - tDebug > 200) {
      tDebug = millis();
      Serial.printf("[PID] angulo=%.1f°  corr=%.1f  izq=%d  der=%d\n",
                    angulo, correccion, velIzq, velDer);
    }
  } else {
    // Sin PID: ambos motores a la misma velocidad
    motorLeft(speed, true);
    motorRight(speed, true);
    Serial.printf(">> ADELANTE (vel: %d/255)\n", speed);
  }
}

void moveBackward(int speed) {
  motorLeft(speed, false);
  motorRight(speed, false);
  Serial.printf(">> ATRAS (vel: %d/255)\n", speed);
}

void turnLeft(int speed) {
  motorLeft(speed, false);
  motorRight(speed, true);
  Serial.printf(">> GIRO IZQ (vel: %d/255)\n", speed);
}

void turnRight(int speed) {
  motorLeft(speed, true);
  motorRight(speed, false);
  Serial.printf(">> GIRO DER (vel: %d/255)\n", speed);
}

// =====================================================
//  TEST AUTOMATICO
// =====================================================

void runAutoTest() {
  Serial.println("=== INICIANDO TEST AUTOMATICO ===");

  Serial.println("Adelante 3s (con PID si activo)...");
  unsigned long inicio = millis();
  while (millis() - inicio < 3000) {
    moveForward(velocidadActual);
    delay(10);   // Frecuencia del loop de PID: ~100 Hz
  }
  motorsStop();
  delay(500);

  Serial.println("Atras 2s...");
  moveBackward(velocidadActual);
  delay(2000);
  motorsStop();
  delay(500);

  Serial.println("Giro izquierda 1s...");
  turnLeft(velocidadActual);
  delay(1000);
  motorsStop();
  delay(500);

  Serial.println("Giro derecha 1s...");
  turnRight(velocidadActual);
  delay(1000);
  motorsStop();

  Serial.println("=== TEST COMPLETO ===");
}

// =====================================================
//  MENU DE AYUDA
// =====================================================

void printHelp() {
  Serial.println("------------------------------");
  Serial.println("  WRO 2026 — Controles:");
  Serial.println("  w = adelante (PID si activo)");
  Serial.println("  s = atras");
  Serial.println("  a = giro izquierda");
  Serial.println("  d = giro derecha");
  Serial.println("  q = parar");
  Serial.println("  p = activar/desactivar PID");
  Serial.println("  c = calibrar (resetear angulo)");
  Serial.println("  t = test automatico");
  Serial.println("  1-9 = cambiar velocidad");
  Serial.println("  h = esta ayuda");
  Serial.printf ("  Velocidad: %d/255\n", velocidadActual);
  Serial.printf ("  PID: %s\n", pidActivado ? "ACTIVADO" : "desactivado");
  Serial.printf ("  IMU: %s\n", imuDisponible ? "conectado" : "no detectado");
  Serial.println("------------------------------");
}

// =====================================================
//  SETUP — se ejecuta UNA SOLA VEZ al encender
// =====================================================

void setup() {
  Serial.begin(115200);
  delay(500);

  // Pines de direccion
  pinMode(MOTOR_L_IN1, OUTPUT);
  pinMode(MOTOR_L_IN2, OUTPUT);
  pinMode(MOTOR_R_IN1, OUTPUT);
  pinMode(MOTOR_R_IN2, OUTPUT);
  pinMode(MOTOR_STBY,  OUTPUT);

  // Configuracion PWM (LEDC del ESP32)
  ledcSetup(PWM_CH_L, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_CH_R, PWM_FREQ, PWM_RES);
  ledcAttachPin(MOTOR_L_PWM, PWM_CH_L);
  ledcAttachPin(MOTOR_R_PWM, PWM_CH_R);

  // Activa el driver TB6612FNG
  digitalWrite(MOTOR_STBY, HIGH);
  motorsStop();

  // Intenta conectar el MPU6050
  imuDisponible = initIMU();

  Serial.println("\n=============================");
  Serial.println(" WRO 2026 - Motor Test + PID");
  Serial.println("=============================");
  printHelp();
}

// =====================================================
//  LOOP — se ejecuta continuamente
//
//  IMPORTANTE: Si moveForward se llama con PID activo,
//  este loop es el que lo hace funcionar. Cuanto mas
//  rapido corre el loop, mas preciso es el control.
//  Por eso NO hay delays largos aqui.
// =====================================================

bool moviendoAdelante = false;  // Rastrea si 'w' esta activo

void loop() {
  // Actualiza el angulo en cada iteracion
  // (aunque no se este moviendo, para que no haya saltos)
  actualizarAngulo();

  // Procesa comandos Serial si hay datos disponibles
  if (Serial.available()) {
    char cmd = Serial.read();
    moviendoAdelante = false;  // Cualquier tecla cancela el modo adelante

    switch (cmd) {
      case 'w':
        moviendoAdelante = true;
        // No llama moveForward aqui — se llama abajo en cada iteracion
        break;

      case 's':
        moveBackward(velocidadActual);
        delay(1);
        break;

      case 'a':
        turnLeft(velocidadActual);
        delay(1);
        break;

      case 'd':
        turnRight(velocidadActual);
        delay(1);
        break;

      case 'q':
        motorsStop();
        break;

      case 'p':
        // Activa o desactiva el PID
        if (!imuDisponible) {
          Serial.println("[PID] No disponible: MPU6050 no conectado.");
        } else {
          pidActivado = !pidActivado;
          pidDireccion.resetear();
          angulo = 0.0f;  // Resetea el angulo al cambiar modo
          Serial.printf("[PID] %s\n", pidActivado ? "ACTIVADO" : "desactivado");
          pidDireccion.imprimirGanancias();
        }
        break;

      case 'c':
        // Calibra: resetea el angulo de referencia a 0
        angulo = 0.0f;
        pidDireccion.resetear();
        Serial.println("[CAL] Angulo reseteado a 0.");
        break;

      case 't':
        runAutoTest();
        break;

      case 'h':
        printHelp();
        break;

      default:
        if (cmd >= '1' && cmd <= '9') {
          velocidadActual = (cmd - '0') * 25 + 25;
          Serial.printf(">> Velocidad: %d/255\n", velocidadActual);
        }
        break;
    }
  }

  // Si 'w' esta activo, ejecuta moveForward en cada iteracion del loop
  // Esto permite que el PID se actualice continuamente (~100+ Hz)
  if (moviendoAdelante) {
    moveForward(velocidadActual);
  }
}
