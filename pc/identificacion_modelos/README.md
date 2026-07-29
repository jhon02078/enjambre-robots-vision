# Identificación de modelos por visión

Aplicación experimental para medir la dinámica de los robots usando el lazo de
visión cenital del proyecto. La herramienta genera excitaciones senoidales,
registra posición y orientación, estima respuestas en frecuencia, ajusta
funciones de transferencia y propone controladores PID.

Soporta los robots ArUco `1`, `2`, `3` y `10`.

> Esta herramienta mueve físicamente el robot. Toda prueba debe realizarse
> supervisada, con un área despejada y acceso inmediato a **STOP** y al corte de
> alimentación.

## Qué se identifica

Un robot diferencial tiene una entrada por cada rueda, pero es más útil
describirlo en coordenadas de movimiento:

```text
u_v = (left + right) / 2      entrada lineal
u_w = (right - left) / 2     entrada angular
```

La cámara entrega `x`, `y` y `yaw`. A partir de esas señales se estiman:

```text
v = velocidad longitudinal en m/s
w = velocidad angular en rad/s
```

El modelo completo se representa inicialmente como dos canales desacoplados:

```text
v(s) = Gv(s) * u_v(s)
w(s) = Gw(s) * u_w(s)
```

Por eso existen dos experimentos:

- **Barrido lineal:** aplica el mismo comando en ambas ruedas y mide `v`.
- **Barrido angular:** aplica comandos opuestos y mide `w`.

No son dos robots ni dos plantas sin relación. Son los dos ejes principales del
mismo robot. El archivo `modelo_diferencial.json` los reúne en una matriz 2x2
diagonal. Este supuesto simplifica el control; si el robot presenta acoplamiento
fuerte, desbalance mecánico o deslizamiento, las métricas y las trayectorias
medidas lo harán visible.

## Método

Para cada frecuencia:

1. Se envían comandos periódicos a una tasa aproximada de 20 Hz.
2. Se descartan los ciclos de asentamiento.
3. Se registran los ciclos de medición.
4. Se remuestrea la señal a tiempo uniforme.
5. Se suavizan y derivan `x`, `y` y `yaw` para obtener `v` y `w`.
6. Se extraen amplitud, fase y una métrica de calidad para cada repetición.
7. Se combinan las repeticiones aceptadas usando su calidad como peso.
8. Se ajustan y comparan varias estructuras de función de transferencia.

La familia de modelos es:

```text
                    Π (1 + Tz_i s)
G(s) = K -------------------------------- exp(-L s)
                    Π (1 + Tp_i s)
```

El ajuste automático puede usar hasta tres polos y dos ceros. También conserva
un modelo de primer orden con retardo como referencia para evitar elegir un
modelo complejo que solo memorice ruido.

## Requisitos

Use el mismo entorno virtual del software de PC:

```powershell
.\.venv\Scripts\Activate.ps1
python -m pip install -r pc\requirements.txt
```

Dependencias principales:

- `numpy`;
- `opencv-contrib-python`;
- `Pillow`;
- `scipy`;
- `matplotlib`;
- Tkinter.

La topología física debe estar operativa:

- video MJPEG accesible;
- ArUco de área `4`, `5`, `6` y `7`;
- ArUco del robot seleccionado;
- ESP32 y PC en la misma subred;
- UDP `37030` y `44444` permitido por el firewall.

## Ejecutar

Desde la raíz:

```powershell
python pc\identificacion_modelos\app_identificacion.py
```

La URL inicial es `http://192.168.1.10:5000/video`. Modifíquela si la Raspberry
usa otra dirección.

## Preparación del experimento

1. Cargue el firmware definitivo que usará durante el control.
2. Use una batería cargada y no cambie ruedas, peso ni alimentación entre
   identificación y validación.
3. Coloque el robot cerca del centro del área.
4. Oriente el robot de forma que tenga espacio libre para el barrido lineal.
5. Retire otros robots y obstáculos.
6. Conecte la cámara y espere `Homografía: OK`.
7. Confirme que la interfaz muestra la IP y la edad de pose del robot elegido.
8. Empiece con amplitudes bajas y un rango de frecuencia corto.

No sostenga el robot en el aire para identificar el movimiento que después
realizará en el piso: la carga, fricción y adherencia forman parte de la planta.
La prueba elevada solo sirve para comprobar cableado y sentido de giro.

## Parámetros de la interfaz

### Cámara y geometría

| Campo | Significado |
|---|---|
| `Cam URL` | Dirección del stream MJPEG |
| `W` | Ancho físico entre referencias, en metros |
| `H` | Alto físico entre referencias, en metros |
| `Robot` | ID ArUco y ESP32 que se moverá |

`W` y `H` deben coincidir con el área usada por
`pc_servidor_vision.py`. Un error en estas medidas escala directamente las
velocidades y la ganancia identificada.

### Excitación lineal y angular

| Campo | Significado |
|---|---|
| `Lineal Amp(%)` | Amplitud del comando común de ruedas |
| `Lineal f min/max` | Banda lineal evaluada, en Hz |
| `Lineal N` | Número de frecuencias lineales |
| `Angular Amp(%)` | Amplitud del comando diferencial |
| `Angular f min/max` | Banda angular evaluada, en Hz |
| `Angular N` | Número de frecuencias angulares |
| `reps` | Repeticiones independientes por frecuencia |

Los puntos se distribuyen en la banda seleccionada. Más puntos y repeticiones
mejoran la resolución, pero aumentan mucho la duración y el recorrido.

Valores iniciales conservadores:

```text
Lineal:  amplitud 30-35%, 0.20-1.20 Hz, 10-12 puntos
Angular: amplitud 25-30%, 0.20-2.00 Hz, 12-14 puntos
Repeticiones: 2
```

La excitación debe superar la zona muerta del motor sin saturar habitualmente
en `100%`. Si el robot no se mueve, no se identifica la planta; si satura, la
respuesta deja de ser aproximadamente lineal.

### Tiempo, análisis y seguridad

| Campo | Significado | Valor inicial |
|---|---|---:|
| `settle cycles` | Ciclos descartados al comenzar cada frecuencia | `1.0` |
| `measure cycles` | Ciclos usados para calcular el Bode | `2.0` |
| `margen borde(m)` | Distancia mínima al límite del área | `0.10` |
| `excursion max(m)` | Desplazamiento lineal máximo desde el origen del segmento | `0.18` |
| `analisis Hz` | Frecuencia de remuestreo para el análisis | `40` |
| `vent.deriv(s)` | Ventana de suavizado usada al derivar la pose | `0.18` |
| `calidad min` | Umbral mínimo para aceptar una repetición | `0.05` |

Reglas prácticas:

- Aumente `measure cycles` a `3-5` si la respuesta es ruidosa.
- Use `reps=3` para un modelo final; `2` es útil durante las pruebas.
- Aumente moderadamente `vent.deriv` si `v` o `w` tienen mucho ruido.
- Reduzca la ventana si borra cambios rápidos o introduce demasiado desfase.
- No eleve `calidad min` hasta dejar muy pocos puntos aceptados.
- Aumentar `excursion max` requiere más espacio físico, no solo cambiar el
  número en la interfaz.

## Ejecución recomendada

### Prueba angular

1. Centre el robot.
2. Seleccione **Barrido angular**.
3. Observe que gira alrededor de su centro sin una traslación excesiva.
4. Detenga si se desplaza, pierde el ArUco o se acerca a un objeto.
5. Revise la respuesta angular antes de continuar.

El barrido angular suele ser el primero porque necesita menos área.

### Prueba lineal

1. Oriente el robot hacia la zona con mayor distancia libre.
2. Use `f min >= 0.20 Hz` y amplitud de `30-35%` inicialmente.
3. Pulse **Barrido lineal**.
4. Reubique manualmente el robot si una corrida se pausa.
5. Inicie una corrida nueva después de reubicarlo.

La aplicación puede omitir frecuencias lineales demasiado bajas cuando estima
que superarían la excursión configurada. Esto evita que toda la prueba se
consuma llegando al borde.

### Prueba combinada

**Lineal + angular** ejecuta ambos canales y produce el conjunto más completo.
Úsela después de validar por separado amplitudes, espacio y detección.

## Paradas de seguridad

La aplicación envía parada cuando:

- se pulsa **STOP**;
- se pierde el ArUco del robot;
- la homografía deja de ser válida;
- el robot entra en el margen del borde;
- se supera la excursión lineal;
- termina un segmento o una frecuencia;
- finaliza o se cancela el experimento.

Si el mensaje indica `Experimento pausado`, la corrida no se reanuda
automáticamente. Corrija la causa, centre el robot y comience una nueva prueba.
Esto evita que un movimiento inesperado continúe después de manipularlo.

## Resultados

Cada corrida usa:

```text
pc/identificacion_modelos/resultados/robot_<id>/YYYYMMDD_HHMMSS/
```

Para el cuarto robot, `<id>` es `10`.

| Archivo | Contenido |
|---|---|
| `samples.csv` | Tiempo, modo, frecuencia, repetición, comandos, pose y velocidades |
| `frequency_response.json` | Ganancia, fase y calidad medidas por frecuencia |
| `quality_metrics.json` | Aceptación y calidad de repeticiones |
| `model.json` | Modelos elegidos, ecuaciones, polos, ceros, retardo y candidatos |
| `modelo_diferencial.json` | Modelo conjunto lineal/angular del robot |
| `pid_params.json` | PID lineal y angular recomendado |
| `experiment_summary.json` | Parámetros y resumen de la corrida |
| `bode_lineal.png` | Bode medido y ajustado para `Gv` |
| `bode_angular.png` | Bode medido y ajustado para `Gw` |
| `timeseries.png` | Comandos y señales temporales |

## Cómo evaluar un resultado

No elija un modelo solo porque tiene más polos. Revise:

1. **Cobertura:** debe haber puntos aceptados en buena parte de la banda.
2. **Repetibilidad:** las repeticiones de una frecuencia deben ser similares.
3. **Magnitud:** el modelo debe seguir la caída o resonancias observadas.
4. **Fase:** los retardos y polos deben reproducir la tendencia medida.
5. **Series temporales:** no deben dominar pérdidas de pose, saturación o
   periodos sin movimiento.
6. **Complejidad:** un modelo más complejo debe reducir el error de forma
   significativa y mantener parámetros físicamente razonables.
7. **Validación:** pruebe el modelo con una corrida distinta a la utilizada para
   ajustarlo.

Una buena curva ajustada sobre pocos puntos de baja calidad no es evidencia
suficiente. Tampoco conviene identificar con una amplitud y controlar después
en una región de PWM completamente distinta.

## PID calculado

`pid_params.json` contiene controladores recomendados para:

- error de distancia a comando lineal en porcentaje;
- error de orientación a comando angular en porcentaje.

Las unidades habituales son:

```text
Lineal:
  Kp  %/m
  Ki  %/(m*s)
  Kd  %*s/m

Angular:
  Kp  %/rad
  Ki  %/(rad*s)
  Kd  %*s/rad
```

Son valores iniciales, no una garantía de estabilidad. La cámara, el tiempo de
red, las saturaciones, la zona muerta, los campos de obstáculos y la lógica de
estados añaden dinámica que el modelo SISO no representa completamente.

Valide primero con límites bajos de PWM, anti-windup y parada accesible. La
aplicación de identificación no modifica automáticamente
`pc_servidor_vision.py`.

## Visor de resultados

Abrir la interfaz y seleccionar una carpeta:

```powershell
python pc\identificacion_modelos\ver_resultados.py
```

Última corrida de un robot:

```powershell
python pc\identificacion_modelos\ver_resultados.py --robot 1
python pc\identificacion_modelos\ver_resultados.py --robot 10
```

Corrida específica:

```powershell
python pc\identificacion_modelos\ver_resultados.py --run pc\identificacion_modelos\resultados\robot_1\YYYYMMDD_HHMMSS
```

Otros modos:

```powershell
python pc\identificacion_modelos\ver_resultados.py --robot 1 --no-show
python pc\identificacion_modelos\ver_resultados.py --robot 1 --open-png
python pc\identificacion_modelos\ver_resultados.py --robot 1 --regen
```

La interfaz permite revisar ecuaciones, parámetros, PID, métricas, Bode y series
temporales sin abrir manualmente cada JSON.

## Prueba sintética

Ejecute:

```powershell
python pc\identificacion_modelos\test_synthetic_identificacion.py
```

La prueba verifica con datos conocidos la estimación de respuesta en frecuencia,
el ajuste de modelos y los cálculos asociados. No prueba cámara, red ni motores.

## Problemas frecuentes

| Problema | Acción |
|---|---|
| `Homografía no disponible` | Asegurar visibilidad de `4-7`, mejorar iluminación y esperar estabilización |
| La pose salta | Fijar cámara/ArUco, eliminar reflejos y aumentar moderadamente la ventana de derivación |
| El robot llega al borde | Centrarlo, subir `f min`, reducir amplitud o excursión y orientar hacia espacio libre |
| Se rechazan casi todos los puntos | Revisar amplitud, calidad de pose y umbral `calidad min` |
| No hay movimiento a baja amplitud | Medir zona muerta y aumentar la excitación gradualmente |
| El Bode presenta puntos aislados | Repetir, revisar pérdidas UDP y usar más ciclos de medición |
| El modelo complejo es inestable o absurdo | Preferir un candidato más simple y repetir con datos de mejor calidad |
| El PID funciona peor en navegación | Retunar considerando saturación, latencia y lógica de movimiento del servidor |

## Recomendación final

Conserve al menos una corrida de identificación y otra de validación por robot.
Registre batería, masa, ruedas, superficie y versión del firmware: cambiar
cualquiera de esos elementos puede cambiar la planta.
