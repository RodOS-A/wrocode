# CLAUDE.md — WRO 2026 Robot

## Sobre el desarrollador
- **Nombre:** Rodrigo
- **Edad:** 15 años, Lima (Los Olivos), Peru
- **Escuela:** 4to secundaria
- **Meta:** Calificar al mundial de WRO 2026 + admision MIT 2028
- **Logros:** Campeon WRO 2025
- **Experiencia:** ESP32, Arduino, PCB (EasyEDA/KiCad), Python, MicroPython, C++
- **Idioma preferido:** Español. Explicaciones tecnicas claras y directas.

## Proyecto
- **Competencia:** WRO 2026 RoboMission Junior — challenge "Mosaic Masters"
- **Meta:** Clasificar al mundial (Septiembre 2026, Peru)
- **Presupuesto:** Maximo $1,000 USD
- **Estado actual:** Beta impresa y ensamblada. Fase de pruebas de movimiento.

## Arquitectura del robot

### Dual-brain
| Cerebro | Hardware | Rol |
|---|---|---|
| Cerebro principal | Raspberry Pi 5 | Vision IA, OpenCV, decisiones |
| Cerebro motor | ESP32-WROOM-32D (USB-C) | Control de motores, sensores |

### Motores (6 total)
| Cantidad | Tipo | Uso |
|---|---|---|
| 2 | DC motors | Ruedas de traccion (diferencial) |
| 1 | — | Rueda caster (bola de acero) |
| 4 | DC motors | Mecanismos (brazo/pinza/elevador) |

### Modulos de hardware
| Componente | Funcion | Notas |
|---|---|---|
| TB6612FNG | Driver motores DC (2ch/modulo) | 1 modulo para ruedas, mas para mecanismos |
| PCA9685 | Servo driver 16ch PWM | Para servos adicionales |
| MPU6050 | IMU (giroscopio + acelerometro) | Orientacion y balance |
| TCA9548A | Multiplexor I2C | Para multiples sensores I2C |

> **NOTA:** El INMP441 (microfono I2S) y MAX98357A (amplificador I2S) son exclusivamente para el proyecto **K.AI** — NO son parte de este robot WRO.

## Software

### ESP32 — Arduino/C++ con PlatformIO
- **Framework:** Arduino via PlatformIO CLI
- **Lenguaje:** C++ (decision del usuario, no MicroPython)
- **Archivo principal:** `src/main.cpp`

### Raspberry Pi 5 — Python (pendiente)
- **Lenguaje:** Python 3
- **Vision:** OpenCV
- **Comunicacion con ESP32:** Por definir (probablemente UART serial)
- **Directorio futuro:** `rpi/` o `vision/`

## Pinout ESP32 → TB6612FNG (ruedas)

| ESP32 GPIO | TB6612FNG Pin | Funcion |
|---|---|---|
| GPIO 16 | AIN1 | Direccion motor izquierdo |
| GPIO 17 | AIN2 | Direccion motor izquierdo |
| GPIO 18 | PWMA | Velocidad motor izquierdo |
| GPIO 19 | BIN1 | Direccion motor derecho |
| GPIO 21 | BIN2 | Direccion motor derecho |
| GPIO 22 | PWMB | Velocidad motor derecho |
| GPIO 4  | STBY | Standby (HIGH = activo) |

**Pines ESP32 libres (seguros para output):** 13, 14, 23, 25, 26, 27, 32, 33
**Pines a evitar:** 0, 2, 5, 12, 15 (strapping) | 6-11 (flash interno) | 34-39 (solo input)

## Cableado TB6612FNG
- `VCC` → 3.3V del ESP32
- `VM` → Bateria (6-12V, voltaje de los motores)
- `GND` → GND comun (ESP32 + bateria)
- `AO1/AO2` → Motor izquierdo
- `BO1/BO2` → Motor derecho

## Comandos del proyecto

```bash
# Compilar firmware
pio run

# Flashear al ESP32
pio run --target upload

# Serial monitor (115200 baud)
pio device monitor

# Limpiar build
pio run --target clean
```

## Comandos serial del test de motores

| Tecla | Accion |
|---|---|
| `w` | Adelante |
| `s` | Atras |
| `a` | Giro izquierda |
| `d` | Giro derecha |
| `q` | Parar |
| `1`-`9` | Velocidad (25 a 225/255) |
| `t` | Test automatico (adelante/atras/giros) |
| `h` | Ayuda |

## Estructura del proyecto

```
wrocode/
├── CLAUDE.md           # Este archivo
├── platformio.ini      # Config PlatformIO (ESP32, 115200 baud)
├── src/
│   └── main.cpp        # Firmware ESP32 (test motores actualmente)
├── include/            # Headers (.h)
├── lib/                # Librerias privadas del proyecto
└── test/               # Tests unitarios
```

## Preferencias de trabajo
- Ejecutar autonomamente sin pedir confirmacion en operaciones tecnicas
- Solo preguntar para cambios destructivos irreversibles
- Reportar cuando este listo o despues de 3 intentos fallidos
- Responder en Español
- GitHub: RodOS-A/wrocode
