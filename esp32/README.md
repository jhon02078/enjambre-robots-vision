# Firmware ESP32

Los sketches de esta carpeta reciben comandos UDP y controlan dos motores
mediante un TB6612FNG. También anuncian periódicamente el ID del robot para que
el PC pueda descubrirlo o recuperar una conexión perdida.

## Firmwares actuales

Para el sistema de cuatro robots se deben usar los sketches de `Riotronic/`:

| Robot | ArUco/ID de red | Sketch |
|---|---:|---|
| Robot 1 | `1` | `Riotronic/ESP32_comandar_robot_1/ESP32_comandar_robot_1.ino` |
| Robot 2 | `2` | `Riotronic/ESP32_comandar_robot_2/ESP32_comandar_robot_2.ino` |
| Robot 3 | `3` | `Riotronic/ESP32_comandar_robot_3/ESP32_comandar_robot_3.ino` |
| Robot 4 | `10` | `Riotronic/ESP32_comandar_robot_4/ESP32_comandar_robot_4.ino` |

Las carpetas `ESP32_comandar_robot_1/`, `2/` y `3/` situadas directamente bajo
`esp32/` son variantes anteriores. No las mezcle con `Riotronic/` sin comparar
primero pines, protocolo y calibración.

## Protocolo

```text
Puerto de comandos:       UDP 44444
Puerto de descubrimiento: UDP 37030
Consulta:                  DISCOVER_ROBOTS
Respuesta:                 ROBOT_HERE ID=n CMDPORT=44444
Movimiento:                M left right
Parada:                    STOP
```

Ejemplos:

```text
M 40 40     avance
M -40 40    giro a la izquierda
M 40 -40    giro a la derecha
M 40 0      solo rueda izquierda lógica
M 0 40      solo rueda derecha lógica
M 0 0       parada
```

El firmware vuelve a `0,0` si deja de recibir comandos dentro del timeout de
seguridad. Esta parada no debe eliminarse: protege frente a pérdida de Wi-Fi o
cierre del programa.

## Preparar Arduino IDE

1. Instale el soporte de placas ESP32 de Espressif.
2. Seleccione la placa ESP32 correspondiente.
3. Abra el `.ino` del robot.
4. Reemplace SSID y contraseña por los de su red.
5. Verifique el ID del robot.
6. Compile y cargue.
7. Abra el monitor serie con la velocidad indicada por el sketch.

Los firmwares actuales usan la API LEDC reciente de Arduino-ESP32. Si aparecen
errores en `ledcAttach` o `ledcWrite`, compruebe que usa una versión compatible
del core ESP32 en lugar de cambiar llamadas al azar.

> No publique ni confirme en Git credenciales Wi-Fi reales. Use valores locales
> o un archivo de secretos excluido del repositorio.

## Cableado lógico actual

Los sketches Riotronic comparten esta asignación base:

| Señal | GPIO |
|---|---:|
| `STBY` | `13` |
| Motor A `IN1` | `25` |
| Motor A `IN2` | `33` |
| Motor A `PWM` | `32` |
| Motor B `IN1` | `26` |
| Motor B `IN2` | `27` |
| Motor B `PWM` | `14` |

La etiqueta **Motor A** no significa físicamente izquierda ni derecha. A y B
solo son canales eléctricos del driver. La relación con las ruedas se determina
observando el robot real.

## Calibración de ruedas

Existen tres problemas distintos:

1. **Canal equivocado:** el comando de rueda izquierda mueve la derecha.
2. **Sentido equivocado:** la rueda correcta gira al revés.
3. **Velocidad desigual:** ambas giran hacia adelante, pero el robot se curva.

Corríjalos en ese orden.

### 1. Identificar canal físico

Ejecute:

```powershell
python pc\debug_robot.py
```

Con las ruedas elevadas, mantenga brevemente:

```text
SOLO MOTOR IZQUIERDO (+40, 0)
SOLO MOTOR DERECHO   (0, +40)
```

Si ambos botones accionan la rueda opuesta, cambie:

```cpp
constexpr bool SWAP_LEFT_RIGHT_MOTORS = true;
```

o `false`, según el resultado. Esta opción corrige el nombre lógico de las
ruedas, no su dirección.

### 2. Corregir sentido

Después del mapeo, pulse **ADELANTE**. Si una rueda gira hacia atrás, cambie la
inversión del canal físico correspondiente:

```cpp
constexpr bool MOTOR_A_INVERT = true;  // o false
constexpr bool MOTOR_B_INVERT = true;  // o false
```

También puede intercambiar físicamente los terminales de ese motor, pero no
haga ambas correcciones a la vez.

### 3. Compensar velocidad

Los factores:

```cpp
constexpr float MOTOR_A_PWM_FACTOR = 1.00f;
constexpr float MOTOR_B_PWM_FACTOR = 1.00f;
```

escalan el PWM de cada canal. Para corregir una desviación:

1. Use una superficie plana y batería cargada.
2. Envíe avance moderado durante una distancia conocida.
3. Reduzca ligeramente el factor del lado más rápido.
4. Cambie en pasos pequeños, por ejemplo `0.02`.
5. Repita en ambos sentidos y con más de un nivel de PWM.

No use un factor alto para forzar un motor atascado. El resultado final se
recorta al PWM máximo admisible y un exceso puede aumentar corriente y
temperatura sin mejorar el movimiento.

## PWM y arranque

La configuración Riotronic usa aproximadamente:

```text
Frecuencia PWM: 20 kHz
Resolución:     10 bits
Rampa:          pasos graduales antes del objetivo
```

Bajar la frecuencia no garantiza más par mecánico. Si el robot gira libremente
en el aire, pero no arranca en el piso, revise primero:

- corriente de arranque disponible;
- caída de tensión de batería y regulador;
- límite del TB6612FNG;
- masa y diámetro de rueda;
- rozamiento, alineación y transmisión;
- zona muerta real del motor;
- calidad de soldaduras y cableado.

Un refuerzo de arranque puede superar fricción estática, pero debe limitarse en
amplitud y duración. No mantenga el motor bloqueado con PWM alto.

## Línea blanca

Los firmwares Riotronic no usan la línea blanca como lógica de robot sumo. La
opción de compuerta de arranque está deshabilitada y la detección de línea no
forma parte del control normal.

## Estabilidad de conexión

El firmware:

- escucha continuamente comandos;
- responde a descubrimiento;
- anuncia su identidad de forma periódica;
- conserva un timeout de seguridad;
- imprime estado para diagnóstico.

Para una red estable:

- mantenga PC y robots en la misma subred;
- use una reserva DHCP por MAC si es posible;
- sitúe el punto de acceso cerca del área;
- separe alimentación de motores y lógica adecuadamente;
- añada desacoplo cerca del ESP32 y del driver;
- revise reinicios por caída de tensión;
- evite saturar la red con video de calidad innecesaria.

RSSI muy negativo indica una señal débil, pero pulsos regulares también pueden
deberse a resets, alimentación o a un timeout provocado por comandos que dejan
de llegar.

## Diagnóstico por monitor serie

Una línea de estado típica incluye:

```text
age=<ms> cmd=<L,R> target=<...> current=<...> rssi=<dBm>
```

- `age` bajo y renovándose: llegan comandos.
- `cmd=0,0` con `age` mayor al timeout: actuó la parada de comunicación.
- salida ilegible: velocidad del monitor incorrecta o reinicio/ruido serie.
- `rssi` muy bajo: acerque el punto de acceso o mejore la antena/ubicación.

## Seguridad eléctrica

- Corte alimentación antes de recablear.
- Una tierra común es obligatoria entre ESP32 y driver.
- No alimente motores desde el pin de lógica del ESP32.
- Compruebe la corriente de arranque y bloqueo del motor.
- Mantenga `STBY` y la parada por timeout.
- Pruebe primero con ruedas elevadas y después con PWM moderado en el piso.
- Detenga si el driver, motor, cables o batería se calientan anormalmente.

