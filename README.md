# Enjambre de robots con visión cenital

Plataforma experimental para localizar y controlar varios robots móviles desde
una cámara cenital. Una Raspberry Pi publica video MJPEG, el computador detecta
marcadores ArUco, transforma las posiciones a coordenadas métricas y envía
comandos UDP a cuatro robots con ESP32.

El repositorio también incluye:

- navegación hacia objetivos seleccionados en un mapa 2D;
- control independiente para los robots ArUco `1`, `2`, `3` y `10`;
- paredes editables, campos de seguridad y planificación de trayectorias;
- filtrado de posición y orientación;
- descubrimiento y reconexión automática de robots;
- una coreografía multirrobot desde la interfaz principal;
- identificación experimental de modelos lineales y angulares;
- ajuste de funciones de transferencia y recomendación de controladores PID;
- firmware, modelos CAD, archivos de PCB y material multimedia.

> **Aviso legal:** este repositorio es de consulta académica. No se concede
> permiso para copiar, modificar, redistribuir o comercializar el código, los
> diseños o el material incluido sin autorización expresa del autor.

## Arquitectura

![Arquitectura del sistema](docs/diagramas/Arquitectura%20del%20Sistema.png)

El flujo principal es:

1. La cámara de la Raspberry Pi publica `http://<IP_RASPBERRY>:5000/video`.
2. El PC detecta los ArUco del área y calcula una homografía píxel-metro.
3. Los ArUco de los robots producen una pose filtrada `{x, y, yaw}`.
4. La interfaz calcula objetivos, trayectorias y acciones de control.
5. El PC envía `M <izquierda> <derecha>` por UDP a cada ESP32.
6. Los ESP32 anuncian periódicamente su identidad para mantener actualizada la
   tabla de conexiones.

### Identificadores ArUco

El proyecto usa el diccionario OpenCV `DICT_4X4_50`.

| ID | Uso |
|---:|---|
| `1` | Robot 1 |
| `2` | Robot 2 |
| `3` | Robot 3 |
| `10` | Robot 4 |
| `4` | Esquina inferior izquierda del área |
| `5` | Esquina inferior derecha del área |
| `6` | Esquina superior derecha del área |
| `7` | Esquina superior izquierda del área |

La posición física de los marcadores `4-7` importa: cambiar su orden invierte o
deforma el sistema de coordenadas.

### Red y protocolo

Todos los equipos deben estar en la misma red local.

| Función | Transporte | Puerto o ruta |
|---|---|---|
| Video de la Raspberry | HTTP/MJPEG | TCP `5000`, ruta `/video` |
| Descubrimiento de robots | UDP broadcast | `37030` |
| Comandos de motores | UDP | `44444` |

Mensajes principales:

```text
PC -> broadcast: DISCOVER_ROBOTS
ESP32 -> PC:      ROBOT_HERE ID=<id> CMDPORT=44444
PC -> robot:      M <left_pct> <right_pct>
PC -> robot:      STOP
```

Los porcentajes de rueda se limitan al intervalo `[-100, 100]`. El firmware
detiene los motores si deja de recibir comandos válidos durante el tiempo de
seguridad configurado.

## Estructura del repositorio

| Ruta | Contenido |
|---|---|
| [`pc/`](pc/) | Servidor de visión, control, diagnóstico e identificación |
| [`pc/pc_servidor_vision.py`](pc/pc_servidor_vision.py) | Aplicación principal Tkinter |
| [`pc/debug_robot.py`](pc/debug_robot.py) | Diagnóstico manual de motores por UDP |
| [`pc/identificacion_modelos/`](pc/identificacion_modelos/) | Barridos, ajuste de modelos, PID y visor de resultados |
| [`raspberry/`](raspberry/) | Servidor MJPEG con Picamera2 |
| [`esp32/`](esp32/) | Firmware de control para los cuatro robots |
| [`hardware/`](hardware/) | Modelos CAD y diseño de PCB |
| [`docs/`](docs/) | Diagramas, informe, imágenes, GIF y videos |

Cada componente tiene una guía específica:

- [Guía del software de PC](pc/README.md)
- [Guía de identificación de modelos](pc/identificacion_modelos/README.md)
- [Guía de la Raspberry y cámara](raspberry/README.md)
- [Guía del firmware ESP32](esp32/README.md)
- [Guía de hardware](hardware/README.md)
- [Índice de documentación y multimedia](docs/README.md)

## Requisitos

### Computador

- Python 3.10 o superior;
- Windows o Linux con soporte para Tkinter;
- OpenCV con módulo `aruco`;
- NumPy, Pillow, SciPy y Matplotlib;
- acceso de red a la Raspberry y a los ESP32.

### Raspberry Pi

- Raspberry Pi OS con el stack `libcamera`;
- cámara compatible con Picamera2;
- Python 3, Flask, Picamera2 y `python3-libcamera`;
- conexión estable por Wi-Fi o Ethernet.

### Robots

- ESP32;
- driver TB6612FNG o cableado equivalente al firmware;
- dos motores DC y una fuente capaz de entregar la corriente de arranque;
- un marcador ArUco visible desde la cámara;
- firmware con un ID único y credenciales de red propias.

## Puesta en marcha

### 1. Preparar Python en el PC

Desde la raíz del repositorio:

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
python -m pip install -r pc\requirements.txt
```

En Linux:

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r pc/requirements.txt
```

Si Tkinter no está disponible en Debian o Ubuntu:

```bash
sudo apt install python3-tk
```

### 2. Preparar y arrancar la Raspberry

Siga la [guía de la cámara](raspberry/README.md), compruebe primero la cámara
con `rpicam-hello` y después ejecute:

```bash
python3 raspberry_camara.py
```

Abra `http://<IP_RASPBERRY>:5000/video` en un navegador. No continúe hasta que
el video sea estable.

### 3. Cargar el firmware

Use los sketches actuales de [`esp32/Riotronic/`](esp32/Riotronic/) y asigne:

| Robot físico | Firmware | ArUco |
|---|---|---:|
| Robot 1 | `ESP32_comandar_robot_1.ino` | `1` |
| Robot 2 | `ESP32_comandar_robot_2.ino` | `2` |
| Robot 3 | `ESP32_comandar_robot_3.ino` | `3` |
| Robot 4 | `ESP32_comandar_robot_4.ino` | `10` |

Antes de flashear, cambie las credenciales Wi-Fi y verifique el mapeo de ruedas.
La [guía ESP32](esp32/README.md) contiene el procedimiento de calibración.

### 4. Montar el área

1. Fije la cámara y evite cambiar su inclinación durante una sesión.
2. Coloque los ArUco `4`, `5`, `6` y `7` en las esquinas indicadas.
3. Mida el ancho y alto útiles del área en metros.
4. Fije el marcador de cada robot centrado y con una orientación conocida.
5. Evite reflejos, desenfoque, recortes y oclusiones de los marcadores.

### 5. Abrir el servidor de visión

```powershell
python pc\pc_servidor_vision.py
```

En la barra superior:

1. Escriba la URL MJPEG.
2. Configure `W(m)` y `H(m)` con las medidas físicas.
3. Pulse **Conectar**.
4. Espere a que el mapa muestre la homografía y los robots.
5. Active **CONTROL ON** únicamente cuando la pose sea estable.

Use **PARAR** ante cualquier comportamiento inesperado.

## Uso de la interfaz principal

### Objetivos y navegación

- **Clic derecho en el mapa:** asigna un objetivo al robot activo.
- **Clic central:** elimina el objetivo del robot activo.
- **Robot activo:** selecciona qué robot recibe el siguiente objetivo.
- **Evitar choques:** activa la separación entre robots y obstáculos.

El controlador trabaja con estados de orientación, avance y evasión. Si el
objetivo está detrás del robot, primero mantiene un giro en el sitio y después
habilita el avance cuando el error angular entra en el rango configurado.

### Paredes

1. Active **Editar paredes**.
2. Dibuje segmentos con el botón izquierdo sobre el mapa.
3. Use **Deshacer pared** para retirar el último segmento.
4. Pulse **Guardar paredes** para persistirlas.
5. Active **Campo paredes** para mostrar el límite discontinuo del campo.

Las paredes se guardan en [`pc/paredes.json`](pc/paredes.json). Su campo de
seguridad se usa tanto en la visualización como en la planificación y evasión.

### Coreografía

El botón **Coreografía** inicia una secuencia para los robots visibles. La
aplicación asigna formaciones y transiciones dentro del área útil. La secuencia
se cancela con el mismo botón o con **PARAR**.

Antes de usarla:

- verifique que todos los robots estén localizados;
- retire obstáculos no representados en el mapa;
- deje margen en los bordes;
- pruebe primero con velocidades moderadas;
- mantenga acceso inmediato al botón de parada.

### Persistencia

La interfaz guarda automáticamente sus ajustes en `pc/ui_config.json`. Las
direcciones descubiertas se conservan en `pc/robots_cache.json`, y las paredes
en `pc/paredes.json`.

Los dos primeros archivos son estado local de ejecución y normalmente no deben
versionarse. No edite estos JSON mientras la aplicación está abierta.

## Identificación y PID

La herramienta de identificación usa la misma cámara y el mismo protocolo que
la aplicación principal, pero mueve un solo robot durante barridos controlados.

```powershell
python pc\identificacion_modelos\app_identificacion.py
```

Puede estimar:

- modelo lineal: comando común de ruedas a velocidad longitudinal;
- modelo angular: comando diferencial a velocidad de giro;
- respuesta en frecuencia medida;
- función de transferencia con varios polos y ceros;
- métricas de ajuste y calidad;
- PID lineal y angular recomendado.

Los resultados se guardan por robot en
`pc/identificacion_modelos/resultados/robot_<id>/`. Consulte la
[guía experimental completa](pc/identificacion_modelos/README.md) antes de
iniciar un barrido.

Para explorar resultados existentes:

```powershell
python pc\identificacion_modelos\ver_resultados.py
```

## Diagnóstico

| Síntoma | Revisión recomendada |
|---|---|
| No abre el video | Probar `/video` en el navegador, revisar IP, puerto y firewall |
| La homografía aparece y desaparece | Mejorar luz, enfoque y tamaño de ArUco; fijar cámara y esquinas |
| Un robot no aparece en el mapa | Confirmar su ID, visibilidad, homografía y dimensiones `W/H` |
| El robot aparece fuera del mapa | Revisar orden de ArUco `4-7` y orientación del marcador del robot |
| No se descubre un ESP32 | Confirmar misma subred y permitir UDP `37030/44444` |
| Adelante funciona, pero los giros están invertidos | Calibrar intercambio e inversión de motores en el firmware |
| El robot oscila en reposo | Mejorar detección, filtrado, iluminación y fijación del ArUco |
| No arranca en el piso | Revisar batería, corriente del driver, fricción y PWM mínimo antes de subir ganancias |
| La identificación se pausa | Reubicar el robot, recuperar homografía o aumentar el margen útil de forma segura |
| El modelo ajustado no representa los datos | Repetir barridos, evitar saturación y revisar métricas de coherencia/calidad |

Para probar motores sin activar navegación:

```powershell
python pc\debug_robot.py
```

Realice esa prueba inicialmente con las ruedas elevadas.

## Seguridad

- Disponga de un interruptor físico de alimentación.
- No deje una prueba o coreografía sin supervisión.
- Eleve las ruedas en la primera prueba de cada firmware.
- No aumente el PWM para compensar una batería descargada o un atasco mecánico.
- Verifique la corriente admisible del driver y de los motores.
- Use `PARAR` antes de modificar parámetros críticos.
- No publique credenciales Wi-Fi incluidas en sketches locales.

## Verificación del software

Comprobación sintáctica básica:

```powershell
python -m py_compile pc\pc_servidor_vision.py
python -m py_compile pc\debug_robot.py
python -m py_compile pc\identificacion_modelos\app_identificacion.py
python -m py_compile pc\identificacion_modelos\ver_resultados.py
```

Prueba sintética del ajuste:

```powershell
python pc\identificacion_modelos\test_synthetic_identificacion.py
```

Estas pruebas no reemplazan la validación física del sentido de motores, la
parada de seguridad y la calidad de la localización.

## Autores

Proyecto desarrollado por **Jhon Meneses**, **Jean Carlos Meneses** y **Holger Sanmartin**.

© 2026. Todos los derechos reservados.
