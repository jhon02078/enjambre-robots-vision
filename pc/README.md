# Software del computador

Esta carpeta contiene la aplicación principal de visión y control, una
herramienta de diagnóstico UDP y el módulo de identificación dinámica.

## Contenido

| Archivo o carpeta | Función |
|---|---|
| `pc_servidor_vision.py` | Interfaz principal, visión, navegación y control |
| `debug_robot.py` | Prueba manual de ruedas por UDP |
| `identificacion_modelos/` | Barridos, modelos, PID y visor de resultados |
| `requirements.txt` | Dependencias Python |
| `paredes.json` | Segmentos de pared persistentes |
| `ui_config.json` | Preferencias locales guardadas por la interfaz |
| `robots_cache.json` | Últimas IP y puertos descubiertos |

## Instalación

Desde la raíz del repositorio:

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
python -m pip install -r pc\requirements.txt
```

Para comprobar que OpenCV incluye ArUco:

```powershell
python -c "import cv2; print(cv2.__version__, hasattr(cv2, 'aruco'))"
```

Si Matplotlib o SciPy no están disponibles, reinstale los requisitos dentro del
mismo entorno virtual con el que ejecutará la aplicación.

## Aplicación principal

```powershell
python pc\pc_servidor_vision.py
```

### Topología

- Cámara Raspberry por HTTP/MJPEG.
- ArUco de área `4`, `5`, `6` y `7`.
- Robots `1`, `2`, `3` y `10`.
- Descubrimiento por UDP `37030`.
- Comandos por UDP `44444`.

La aplicación mantiene una homografía filtrada, corrige el paralaje aproximado
por altura del marcador y suaviza la pose de cada robot. Los objetivos y las
paredes se expresan en metros, no en píxeles.

### Barra superior

| Control | Uso |
|---|---|
| `IP cam URL` | URL completa del stream `/video` |
| `Conectar` | Abre o vuelve a abrir el stream |
| `W(m)`, `H(m)` | Dimensiones físicas del área |
| `Alt.Rob(m)` | Altura del marcador para corrección de paralaje |
| `CONTROL ON` | Habilita el envío automático de movimiento |
| `PARAR` | Desactiva control, limpia objetivos y envía parada |
| `Coreografía` | Inicia o detiene la secuencia multirrobot |
| `Robot activo` | Robot que recibirá el siguiente objetivo |
| `Evitar choques` | Habilita la repulsión entre robots |
| `R(m)` | Radio de separación entre robots |
| `Editar paredes` | Permite dibujar segmentos |
| `Campo paredes` | Muestra el campo como contorno discontinuo |
| `Guardar paredes` | Escribe `paredes.json` |
| `Deshacer pared` | Elimina el último segmento |
| `Clear(m)` | Separación usada por el planificador respecto a paredes |
| `Campo(m)` | Alcance configurable del campo de pared |
| `Klin(%/m)` | Ganancia lineal del controlador clásico |
| `Kang(%/rad)` | Ganancia angular del controlador clásico |
| `Rebuscar robots` | Fuerza un nuevo descubrimiento UDP |

Los ajustes editables se guardan después de los cambios y se restauran en la
siguiente ejecución.

### Controles del mapa

| Acción | Resultado |
|---|---|
| Clic derecho | Asigna un objetivo al robot activo |
| Clic central | Elimina su objetivo |
| Clic izquierdo con edición activa | Define extremos de una pared |

Para dibujar una pared se realizan dos clics izquierdos: inicio y final. Al
desactivar la edición, el clic izquierdo no debe crear objetivos.

### Navegación

La navegación combina:

- error de distancia;
- error angular normalizado;
- giro en el sitio cuando el objetivo está fuera del cono frontal;
- histéresis para no alternar rápidamente entre orientar y avanzar;
- acción mínima y refuerzo de arranque;
- campos de repulsión entre robots;
- paredes y margen de seguridad;
- planificación A* sobre una cuadrícula métrica.

La trayectoria calculada se divide en puntos intermedios. El robot sigue el
próximo punto y conserva aparte el objetivo final.

Los modos internos más relevantes son:

| Modo | Comportamiento |
|---|---|
| `IDLE` | Sin objetivo o dentro de la tolerancia |
| `ORIENT` | Giro en el sitio hasta alinear el robot |
| `RUN` | Avance y corrección angular simultáneos |
| `AVOID` | Prioriza separación frente a una colisión cercana |

### PID identificado

`pc_servidor_vision.py` contiene ganancias independientes para cada robot. Su
uso se controla con:

```python
USE_IDENTIFIED_ROBOT_PID = False
```

Con `False`, la interfaz usa principalmente `Klin` y `Kang` junto con la lógica
de movimiento. Con `True`, aplica los PID por robot definidos en
`IDENTIFIED_PID_GAINS`.

No active esta opción únicamente porque el ajuste experimental tenga bajo
error. Antes verifique:

- unidades y signos;
- intervalo de muestreo real;
- saturación y PWM mínimo;
- anti-windup;
- retardo de cámara/red;
- respuesta física con límites conservadores.

## Coreografía

La coreografía usa solo los robots con pose visible al comenzar cada ciclo.
Genera sucesivamente formaciones de cuadrado, diamante, intercambio, fila y
órbita, recortadas al área útil.

El botón habilita el control automáticamente. Para una ejecución segura:

1. Localice todos los robots.
2. Represente las paredes reales o retire obstáculos.
3. Active la evasión.
4. Confirme que la escala `W/H` es correcta.
5. Pulse **Coreografía**.
6. Use **Detener coreo** o **PARAR** para terminar.

La coreografía no sustituye una barrera física ni un interruptor de emergencia.

## Paredes y archivo JSON

`paredes.json` contiene segmentos en coordenadas métricas. Debe mantenerse como
JSON válido. La interfaz lo carga al iniciar y lo sobrescribe únicamente al
pulsar **Guardar paredes**.

El alcance visual y repulsivo se controla con `Campo(m)`. El margen del
planificador se controla de forma independiente con `Clear(m)`.

Conviene guardar una copia del archivo cuando la disposición física sea
definitiva.

## Persistencia local

| Archivo | Versionar | Motivo |
|---|---|---|
| `paredes.json` | Sí, si representa el entorno compartido | Es parte del mapa |
| `ui_config.json` | No | Preferencias de una estación |
| `robots_cache.json` | No | IP temporales asignadas por DHCP |
| `identificacion_modelos/resultados/` | Normalmente no | Datos generados y pesados |

No almacene contraseñas ni información sensible en estos archivos.

## Diagnóstico manual

```powershell
python pc\debug_robot.py
```

La herramienta descubre un robot y permite mantener presionadas acciones de
prueba. Envía comandos repetidos, no un único pulso:

```text
Adelante:             M 40 40
Giro izquierda:       M -40 40
Giro derecha:         M 40 -40
Solo motor izquierdo: M 40 0
Solo motor derecho:   M 0 40
```

Empiece con las ruedas elevadas. Si los botones de motor individual accionan la
rueda opuesta, corrija `SWAP_LEFT_RIGHT_MOTORS` en el firmware. Si una rueda
gira en sentido incorrecto, corrija la inversión de su canal.

## Identificación

La guía completa está en
[`identificacion_modelos/README.md`](identificacion_modelos/README.md).

Comandos principales:

```powershell
python pc\identificacion_modelos\app_identificacion.py
python pc\identificacion_modelos\ver_resultados.py
python pc\identificacion_modelos\test_synthetic_identificacion.py
```

## Solución de problemas

### La cámara conecta, pero hay mucho retraso

- compruebe que la Raspberry usa una cola corta;
- use red de 5 GHz o Ethernet si está disponible;
- reduzca resolución/FPS antes de reducir la seguridad;
- cierre otros consumidores del stream;
- confirme que OpenCV no acumula un búfer grande.

### La homografía se pierde

- mantenga visibles los cuatro ArUco de área;
- aumente su tamaño en píxeles;
- reduzca reflejos y desenfoque;
- evite mover la cámara;
- compruebe el diccionario `DICT_4X4_50`.

### La pose oscila con todo inmóvil

- fije rígidamente cámara y marcador;
- use más luz difusa y una exposición más corta;
- retire superficies brillantes junto al ArUco;
- compruebe que el marcador no está curvado;
- ajuste el filtrado solo después de mejorar la imagen.

### El robot deja de recibir comandos

- pulse **Rebuscar robots**;
- permita UDP de entrada y salida en el firewall;
- revise RSSI, alimentación del ESP32 y reinicios;
- use reservas DHCP o IP estables;
- confirme que ningún segundo servidor está enviando comandos.

### El robot se detiene al girar

- determine si la interfaz sigue enviando comandos;
- revise PWM mínimo y zona muerta;
- compruebe que ambas ruedas puedan arrancar en sentidos opuestos;
- valide batería y corriente del puente H;
- no compense un problema mecánico elevando indiscriminadamente el PID.

## Verificación rápida

```powershell
python -m py_compile pc\pc_servidor_vision.py
python -m py_compile pc\debug_robot.py
python -m py_compile pc\identificacion_modelos\app_identificacion.py
python -m py_compile pc\identificacion_modelos\ver_resultados.py
```
