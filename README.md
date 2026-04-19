# Enjambre de Robots con Visión por Computadora (Hybrid Edge-Server Visual Servoing)

> **Copyright (c) 2026 Jhon Meneses. Todos los derechos reservados.**
> 
> Este repositorio contiene código propietario asociado a un proyecto de investigación académica. **No se otorga ninguna licencia** para el uso, copia, modificación, distribución, compilación o ingeniería inversa de este software, ni en su totalidad ni en parte. Queda estrictamente prohibido su uso para fines académicos, comerciales o personales por terceros sin autorización explícita y por escrito del autor. Este repositorio sirve como registro de autoría (Prior Art).

## Descripción del Proyecto

Sistema distribuido para el control, monitoreo y coordinación de múltiples robots móviles. La arquitectura del sistema divide el procesamiento de visión y la actuación para optimizar los recursos, operando de la siguiente manera:

- **PC (Servidor):** Actúa como servidor central de visión y control del enjambre.
- **Raspberry Pi (Edge):** Nodo de cámara encargado de la captura y transmisión del entorno (Stream MJPEG).
- **ESP32 (Actuación):** Firmware embebido en cada robot móvil para la recepción de comandos y actuación.
- **Localización:** Detección de marcadores ArUco mediante visión por computadora para la estimación de pose en el espacio de trabajo.

## Estructura del Repositorio

- `pc/vision_server/`: Aplicación principal de visión y algoritmos de control.
- `raspberry/camera_stream/`: Servidor de transmisión de video de la cámara.
- `esp32/`: Firmware de cada robot (control de motores y comunicación).
- `docs/`: Diagramas de arquitectura, esquemas electrónicos, fotos y documentación.

## Estado actual

🚧 **En desarrollo activo.** Proyecto de investigación en curso. 

## Autores

**Jhon Meneses y Jean Carlos Meneses**

---

## Demo del sistema


### Evidencia visual
- Detección de marcadores ArUco en tiempo real
- Estimación de pose planar `(x, y, yaw)`
- Navegación multi-robot en workspace compartido
- Evitación de colisiones mediante campos potenciales
- Comunicación distribuida entre PC, Raspberry Pi y robots ESP32

### Recursos sugeridos para mostrar
- `docs/media/capturas/frame_aruco.png`
- `docs/media/capturas/mapa_2d.png`
- `docs/media/fotos/robot_fisico.jpg`
- `docs/media/videos/demo_cruce.mp4`


```md
![Frame con detección ArUco](docs/media/fotos/captura_interfaz_1.png)
![Funcionamiento del enjambre con campos potenciales](docs/media/gifs/demo_funcionamiento.gif)
````

---

## Objetivo

Desarrollar una plataforma experimental de robótica móvil cooperativa basada en visión por computadora, capaz de:

* localizar múltiples robots en un entorno compartido,
* estimar su pose en coordenadas métricas,
* asignar y seguir objetivos de navegación,
* evitar colisiones entre robots y con las paredes del workspace,
* y coordinar todo el sistema mediante una arquitectura distribuida de bajo costo.

---

## Arquitectura del sistema

La arquitectura del sistema se organiza en tres bloques principales:

### 1. Raspberry Pi (captura y streaming)

La Raspberry Pi actúa como nodo de adquisición de video. Captura el entorno desde una vista cenital y publica el flujo en la red local mediante **streaming MJPEG**.

### 2. PC (visión, control y supervisión)

La PC recibe el stream de video, detecta marcadores ArUco, estima la pose de cada robot, calcula la homografía del workspace, corrige errores geométricos y ejecuta el algoritmo de navegación y evitación de colisiones. Además, incluye una interfaz gráfica para monitoreo y ajuste.

### 3. Robots móviles con ESP32

Cada robot móvil incorpora un ESP32 encargado de recibir comandos por UDP, controlar el driver de motores y ejecutar mecanismos de seguridad como timeout y rampa PWM.

---

## Características principales

* Control centralizado de múltiples robots móviles
* Detección de marcadores ArUco usando OpenCV
* Homografía para convertir coordenadas de imagen a coordenadas métricas
* Estabilización del workspace mediante histéresis y filtrado temporal
* Corrección geométrica por paralaje
* Navegación multi-robot con asignación de objetivos
* Evitación de colisiones mediante campos potenciales
* Comunicación UDP para descubrimiento y control de robots
* Interfaz gráfica para supervisión en tiempo real
* Firmware embebido basado en ESP32 para actuación diferencial
* Integración de hardware propio, PCB personalizada y piezas impresas en 3D

---

## Flujo general de operación

1. La Raspberry Pi captura el video desde la cámara cenital.
2. El stream MJPEG se transmite por red local a la PC.
3. La PC detecta marcadores ArUco de referencia y de robots.
4. Se calcula la homografía del workspace para convertir puntos de píxeles a metros.
5. Se estima la pose planar de cada robot: posición `(x, y)` y orientación `yaw`.
6. Se calculan los vectores de atracción y repulsión para navegación y evitación.
7. La PC envía comandos diferenciales por UDP a cada robot.
8. Cada ESP32 aplica PWM a los motores mediante el driver TB6612FNG.
9. La realimentación visual cierra el lazo de control.

---

## Identificadores del sistema

### Robots

* **ID 1:** Robot 1
* **ID 2:** Robot 2
* **ID 3:** Robot 3

### Workspace

* **ID 4:** esquina inferior izquierda
* **ID 5:** esquina inferior derecha
* **ID 6:** esquina superior derecha
* **ID 7:** esquina superior izquierda

Estos marcadores permiten definir el sistema de referencia del entorno y estimar la pose de cada robot dentro del área de trabajo.

---

## Comunicación

### Streaming de video

* **Protocolo:** HTTP / MJPEG
* **Endpoint:** `/video`
* **Puerto:** `5000`

### Descubrimiento de robots

* **Protocolo:** UDP Broadcast
* **Puerto:** `37030`
* **Consulta enviada por la PC:** `DISCOVER_ROBOTS`
* **Respuesta esperada del robot:** `ROBOT_HERE ID=<id> CMDPORT=44444`

### Comandos de movimiento

* **Protocolo:** UDP
* **Puerto:** `44444`
* **Formato de comando:** `M L R`

Donde:

* `L` = velocidad de la rueda izquierda en porcentaje `[-100, 100]`
* `R` = velocidad de la rueda derecha en porcentaje `[-100, 100]`

---

## Algoritmo de control

La navegación implementada combina:

### Seguimiento de objetivo

Cada robot recibe un objetivo en el workspace y orienta su movimiento hacia él.

### Máquina de estados

El sistema contempla estados de operación por robot, tales como:

* reposo,
* orientación inicial,
* avance normal,
* evasión.

### Evitación de colisiones

Se utiliza una estrategia reactiva basada en **campos potenciales**:

* un término atractivo hacia la meta,
* términos repulsivos por proximidad de otros robots,
* y repulsión respecto a las paredes del workspace.

### Control diferencial

Las acciones calculadas se convierten en comandos izquierda/derecha que son enviados al robot como velocidades diferenciales.

---

## Interfaz gráfica

La aplicación de PC incluye una GUI para supervisión y control del sistema.

### Funcionalidades principales

* Visualización del frame procesado
* Dibujo del polígono del workspace
* Mapa 2D con posición de robots
* Selección de robot activo
* Asignación de objetivos por interacción
* Visualización de vectores:

  * atracción
  * repulsión
  * resultante
* Ajuste de parámetros en tiempo real:

  * dimensiones del workspace
  * radio de evitación
  * ganancias de control
  * altura del marcador para corrección de paralaje

---

## Hardware utilizado

### Unidad de captura

* Raspberry Pi
* Cámara cenital compatible con Picamera2

### Estación base

* PC o laptop para visión, control y supervisión

### Robots móviles

* ESP32 DevKitC
* Driver de motores TB6612FNG
* Motores DC
* Batería
* PCB personalizada
* Estructura mecánica impresa en 3D
* Marcadores ArUco en cada robot

### Elementos de referencia

* Marcadores ArUco en las esquinas del workspace

---

## Software utilizado

### En la PC

* Python 3
* OpenCV
* NumPy
* Tkinter
* Pillow
* sockets UDP

### En la Raspberry Pi

* Python 3
* Flask
* Picamera2
* libcamera

### En los robots

* Arduino framework para ESP32
* WiFi / WiFiUDP
* PWM mediante LEDC

---

## Estructura ampliada del repositorio

```text
enjambre-robots-vision/
├── README.md
├── .gitignore
├── pc/
│   └── vision_server/
├── raspberry/
│   └── camera_stream/
├── esp32/
│   ├── robot_1/
│   ├── robot_2/
│   └── robot_3/
├── hardware/
│   ├── cad/
│   ├── pcb/
│   ├── wiring/
│   └── bom/
├── docs/
│   ├── media/
│   ├── report/
│   └── diagrams/
└── results/
```

### Descripción por carpetas

* `pc/vision_server/`: servidor central de visión, GUI y control.
* `raspberry/camera_stream/`: streaming MJPEG desde la Raspberry Pi.
* `esp32/`: firmware de los robots móviles.
* `hardware/cad/`: modelos STL y piezas mecánicas.
* `hardware/pcb/`: esquemáticos, archivos fuente e imágenes de la PCB.
* `hardware/wiring/`: diagramas de conexión y pinout.
* `hardware/bom/`: lista de materiales.
* `docs/media/`: fotos, videos, capturas y demostraciones.
* `docs/report/`: informe técnico, paper o memoria del proyecto.
* `results/`: resultados experimentales, logs y trayectorias.

---

## Requisitos

### PC

* Python 3.10 o superior
* OpenCV
* NumPy
* Pillow
* Tkinter

### Raspberry Pi

* Python 3
* Flask
* Picamera2
* libcamera

### Robots

* ESP32 programado con el firmware correspondiente
* Driver TB6612FNG
* Red Wi-Fi local disponible

---

## Instalación

### 1. Clonar el repositorio

```bash
git clone https://github.com/jhon02078/enjambre-robots-vision.git
cd enjambre-robots-vision
```

### 2. Instalar dependencias en la PC

```bash
pip install opencv-python numpy pillow
```

### 3. Instalar dependencias en la Raspberry Pi

```bash
pip install flask
```

> `picamera2` normalmente se instala desde los paquetes recomendados del sistema en Raspberry Pi OS.

---

## Ejecución rápida

### 1. Raspberry Pi

Ejecutar el servidor de cámara:

```bash
python raspberry/camera_stream/raspberry_camara.py
```

Esto levantará el stream MJPEG en:

```text
http://<IP_RPI>:5000/video
```

### 2. PC

Ejecutar la aplicación principal de visión y control:

```bash
python pc/vision_server/pc_servidor_vision.py
```

Luego:

* configurar la URL del stream,
* verificar que se detecten los ArUco,
* activar el control,
* y asignar objetivos a los robots.

### 3. ESP32

Cargar el firmware en cada robot ajustando el identificador correspondiente:

```cpp
static const int ROBOT_ID = 1;   // cambiar para cada robot
```

---

## Hardware del robot

El repositorio incluye material relacionado con la implementación física del robot:

### `hardware/cad/`

* STL del chasis y piezas mecánicas impresas en 3D

### `hardware/pcb/`

* archivo fuente del esquemático
* imágenes del circuito
* fotografías de la PCB fabricada y montada

### `hardware/wiring/`

* diagrama de conexiones
* pinout ESP32 ↔ TB6612FNG
* notas de alimentación

### `hardware/bom/`

* lista de materiales del robot y del sistema

---

## Resultados

El sistema fue validado en diferentes escenarios experimentales, incluyendo:

* navegación de un robot hacia una meta,
* coordinación simultánea de múltiples robots,
* cruce de trayectorias,
* perturbaciones por vibración y oclusión parcial.

### Resultados observados

* localización estable en tiempo real,
* navegación efectiva hacia objetivos,
* activación reactiva de la evasión,
* reducción de colisiones,
* buena separación entre adquisición, procesamiento y actuación.

---

## Validación visual sugerida

Se recomienda incluir en `docs/media/`:

* captura del frame de cámara con marcadores detectados
* captura del mapa 2D con vectores de control
* foto del sistema físico montado
* video corto del escenario de cruce entre robots
* imagen de la PCB y del robot ensamblado

---

## Limitaciones actuales

* Dependencia de la visibilidad de los marcadores ArUco
* Sensibilidad a iluminación, vibraciones y oclusiones
* Posibles mínimos locales en campos potenciales
* Precisión dependiente de la calibración intrínseca de la cámara
* Robustez condicionada por la calidad de la red Wi-Fi

---

## Trabajo futuro

* Calibración completa de cámara
* Corrección de distorsión óptica
* Filtrado dinámico de pose
* Mejora de robustez ante oclusiones
* Estrategias anti-mínimos locales
* Integración de planificación global
* Escalado a un mayor número de robots

---

## Informe técnico / Paper

Este repositorio puede complementarse con:

* informe técnico del proyecto,
* paper académico,
* capturas de resultados,
* documentación de hardware,
* diagramas de arquitectura y flujo.

Ejemplo sugerido:

```text
docs/report/informe_proyecto.pdf
```

---

## Colaboradores

* **Jhon Meneses**
* **JeanC3029**

---

## Cita y uso académico

Este repositorio **no es de libre uso**.
Si necesitas referenciar este trabajo en un contexto académico, solicita autorización expresa al autor.

---

## Nota final

Este repositorio documenta tanto la parte de software como la implementación física de un sistema multi-robot experimental basado en visión artificial, con fines de investigación, validación académica y registro de autoría.

```
