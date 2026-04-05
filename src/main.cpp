/*
 * WRO 2026 - Mosaic Masters - Motor Test
 * ESP32-WROOM-32D + TB6612FNG
 *
 * Controla 2 motores DC (ruedas) via Serial Monitor.
 *
 * Comandos:
 *   w = adelante    s = atras
 *   a = giro izq    d = giro der
 *   q = parar
 *   1-9 = velocidad (25 a 225)
 *   t = test automatico (adelante, atras, giros, parar)
 *
 * Cableado ESP32 -> TB6612FNG:
 *   GPIO 16 -> AIN1    (dir motor izq)
 *   GPIO 17 -> AIN2    (dir motor izq)
 *   GPIO 18 -> PWMA    (vel motor izq)
 *   GPIO 19 -> BIN1    (dir motor der)
 *   GPIO 21 -> BIN2    (dir motor der)
 *   GPIO 22 -> PWMB    (vel motor der)
 *   GPIO  4 -> STBY    (HIGH = activo)
 *
 *   TB6612FNG VCC  -> ESP32 3.3V
 *   TB6612FNG VM   -> Bateria (6-12V)
 *   TB6612FNG GND  -> GND comun
 *   Motor A (AO1/AO2) -> Motor izquierdo
 *   Motor B (BO1/BO2) -> Motor derecho
 */

#include <Arduino.h>

// --- Pin definitions ---
#define MOTOR_L_IN1  16   // AIN1
#define MOTOR_L_IN2  17   // AIN2
#define MOTOR_L_PWM  18   // PWMA
#define MOTOR_R_IN1  19   // BIN1
#define MOTOR_R_IN2  21   // BIN2
#define MOTOR_R_PWM  22   // PWMB
#define MOTOR_STBY    4   // Standby

// --- PWM config ---
#define PWM_FREQ     5000
#define PWM_RES      8     // 8-bit: 0-255
#define PWM_CH_L     0     // LEDC channel for left motor
#define PWM_CH_R     1     // LEDC channel for right motor

// --- State ---
int currentSpeed = 150;    // Default speed (0-255)

// --- Motor control functions ---

void motorsStop() {
  digitalWrite(MOTOR_L_IN1, LOW);
  digitalWrite(MOTOR_L_IN2, LOW);
  digitalWrite(MOTOR_R_IN1, LOW);
  digitalWrite(MOTOR_R_IN2, LOW);
  ledcWrite(PWM_CH_L, 0);
  ledcWrite(PWM_CH_R, 0);
  Serial.println(">> STOP");
}

void motorLeft(int speed, bool forward) {
  digitalWrite(MOTOR_L_IN1, forward ? HIGH : LOW);
  digitalWrite(MOTOR_L_IN2, forward ? LOW : HIGH);
  ledcWrite(PWM_CH_L, speed);
}

void motorRight(int speed, bool forward) {
  digitalWrite(MOTOR_R_IN1, forward ? HIGH : LOW);
  digitalWrite(MOTOR_R_IN2, forward ? LOW : HIGH);
  ledcWrite(PWM_CH_R, speed);
}

void moveForward(int speed) {
  motorLeft(speed, true);
  motorRight(speed, true);
  Serial.printf(">> ADELANTE (vel: %d)\n", speed);
}

void moveBackward(int speed) {
  motorLeft(speed, false);
  motorRight(speed, false);
  Serial.printf(">> ATRAS (vel: %d)\n", speed);
}

void turnLeft(int speed) {
  motorLeft(speed, false);
  motorRight(speed, true);
  Serial.printf(">> GIRO IZQ (vel: %d)\n", speed);
}

void turnRight(int speed) {
  motorLeft(speed, true);
  motorRight(speed, false);
  Serial.printf(">> GIRO DER (vel: %d)\n", speed);
}

void runAutoTest() {
  Serial.println("=== TEST AUTOMATICO ===");

  Serial.println("Adelante 2s...");
  moveForward(currentSpeed);
  delay(2000);

  motorsStop();
  delay(500);

  Serial.println("Atras 2s...");
  moveBackward(currentSpeed);
  delay(2000);

  motorsStop();
  delay(500);

  Serial.println("Giro izquierda 1s...");
  turnLeft(currentSpeed);
  delay(1000);

  motorsStop();
  delay(500);

  Serial.println("Giro derecha 1s...");
  turnRight(currentSpeed);
  delay(1000);

  motorsStop();
  Serial.println("=== TEST COMPLETO ===");
}

void printHelp() {
  Serial.println("--- WRO Motor Test ---");
  Serial.println("  w = adelante");
  Serial.println("  s = atras");
  Serial.println("  a = giro izquierda");
  Serial.println("  d = giro derecha");
  Serial.println("  q = parar");
  Serial.println("  1-9 = velocidad");
  Serial.println("  t = test automatico");
  Serial.println("  h = ayuda");
  Serial.printf("  Velocidad actual: %d/255\n", currentSpeed);
  Serial.println("----------------------");
}

void setup() {
  Serial.begin(115200);
  delay(500);

  // Direction pins
  pinMode(MOTOR_L_IN1, OUTPUT);
  pinMode(MOTOR_L_IN2, OUTPUT);
  pinMode(MOTOR_R_IN1, OUTPUT);
  pinMode(MOTOR_R_IN2, OUTPUT);
  pinMode(MOTOR_STBY, OUTPUT);

  // PWM setup
  ledcSetup(PWM_CH_L, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_CH_R, PWM_FREQ, PWM_RES);
  ledcAttachPin(MOTOR_L_PWM, PWM_CH_L);
  ledcAttachPin(MOTOR_R_PWM, PWM_CH_R);

  // Enable motor driver
  digitalWrite(MOTOR_STBY, HIGH);
  motorsStop();

  Serial.println("\n=============================");
  Serial.println(" WRO 2026 - Motor Test v1.0");
  Serial.println(" ESP32 + TB6612FNG");
  Serial.println("=============================");
  printHelp();
}

void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();

    switch (cmd) {
      case 'w': moveForward(currentSpeed);  break;
      case 's': moveBackward(currentSpeed); break;
      case 'a': turnLeft(currentSpeed);     break;
      case 'd': turnRight(currentSpeed);    break;
      case 'q': motorsStop();               break;
      case 't': runAutoTest();              break;
      case 'h': printHelp();                break;
      default:
        if (cmd >= '1' && cmd <= '9') {
          currentSpeed = (cmd - '0') * 25 + 25;
          Serial.printf(">> Velocidad: %d/255\n", currentSpeed);
        }
        break;
    }
  }
}
