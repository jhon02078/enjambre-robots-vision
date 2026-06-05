# Identificacion de modelos por vision

Herramienta Tkinter para identificar modelos dinamicos de los robots usando la camara, ArUco, homografia y comandos UDP `M L R`.

Robots soportados por defecto: ArUco/ID `1`, `2`, `3` y `10`.

## Ejecutar

Desde la raiz del proyecto:

```powershell
py -3.13 pc\identificacion_modelos\app_identificacion.py
```

El entorno recomendado debe tener:

- `numpy`
- `opencv-contrib-python`
- `pillow`
- `scipy`
- `matplotlib`

## Flujo recomendado

1. Conecta la camara MJPEG.
2. Verifica que se dibuje la homografia con ArUco `4,5,6,7`.
3. Verifica que el robot elegido tenga ArUco visible y discovery UDP. Para el cuarto robot usa ID `10`.
4. Usa amplitud conservadora, por ejemplo `40-50%`.
5. Corre primero `Barrido angular`, luego `Barrido lineal`, o usa `Lineal + angular`.

La app hace auto-stop si pierde homografia, pierde el ArUco del robot o el robot se acerca demasiado al borde.

## Parametros nuevos de identificacion

- `Lineal Amp(%)`, `f min/max`, `N`: amplitud y frecuencias solo para avance.
- `Angular Amp(%)`, `f min/max`, `N`: amplitud y frecuencias solo para giro.
- `reps`: repeticiones por frecuencia. Usa `2` o `3` si quieres un Bode mas confiable.
- `analisis Hz`: tasa usada para remuestrear cada segmento antes del calculo en frecuencia.
- `vent.deriv(s)`: ventana de suavizado para derivar `x,y,yaw` y obtener `v,w`.
- `calidad min`: rechazo de repeticiones con poca componente senoidal util.

Valores recomendados iniciales:

```text
Lineal:  Amp 30-40%, f 0.20-1.20 Hz, N 10-14
Angular: Amp 25-40%, f 0.20-2.00 Hz, N 12-18
reps: 2
analisis Hz: 40
vent.deriv(s): 0.15-0.25
calidad min: 0.05-0.15
```

## Como se interpreta el modelo

El robot diferencial se guarda como un solo modelo en coordenadas desacopladas:

```text
u_v = (left + right) / 2
u_w = (right - left) / 2
```

```text
v = Gv(s) * u_v
w = Gw(s) * u_w
```

Por eso se miden dos canales: avance y giro. En los resultados aparece como `modelo_diferencial.json`, una matriz 2x2 diagonal para el robot completo.

Cada canal se ajusta ahora con una familia de modelos de transferencia de la forma:

```text
G(s) = K * prod(1 + Tz_i s) / prod(1 + Tp_i s) * exp(-L s)
```

La herramienta remuestrea cada repeticion a tiempo uniforme, suaviza/deriva la pose para calcular `v,w`, estima la respuesta en frecuencia por repeticion y promedia cada frecuencia con pesos de calidad. Luego prueba automaticamente modelos con varios polos y ceros, por defecto hasta 3 polos y 2 ceros, conserva el primer orden con retardo como referencia y guarda el candidato que mejor ajusta la respuesta en frecuencia medida. En `model.json` se incluyen la ecuacion, polos, ceros, constantes de tiempo, retardo, error del ajuste y una tabla de candidatos evaluados.

## Evitar que llegue al borde

El barrido lineal necesita espacio porque el robot se traslada. Usa:

- `Amp(%)`: 30 a 40 para empezar.
- `f min`: 0.20 Hz o mayor.
- `excursion max(m)`: 0.15 a 0.20 m.

Si una frecuencia lineal desplaza demasiado al robot, la app la marca como incompleta, manda `STOP` y continua solo si el robot sigue dentro de la zona segura.

## Resultados

Cada corrida se guarda en:

```text
pc/identificacion_modelos/resultados/robot_N/YYYYMMDD_HHMMSS/
```

Para el cuarto robot las corridas quedan en `resultados/robot_10/`.

Archivos generados:

- `samples.csv`
- `frequency_response.json`
- `quality_metrics.json`
- `model.json`
- `modelo_diferencial.json`
- `experiment_summary.json`
- `pid_params.json`
- `bode_lineal.png`
- `bode_angular.png`
- `timeseries.png`

## Prueba sintetica

```powershell
py -3.13 pc\identificacion_modelos\test_synthetic_identificacion.py
```

## Visualizar resultados

Abrir la interfaz con la ultima corrida del robot 1:

```powershell
py -3.13 pc\identificacion_modelos\ver_resultados.py --robot 1
```

Para ver el cuarto robot:

```powershell
py -3.13 pc\identificacion_modelos\ver_resultados.py --robot 10
```

Corrida especifica:

```powershell
py -3.13 pc\identificacion_modelos\ver_resultados.py --run pc\identificacion_modelos\resultados\robot_1\YYYYMMDD_HHMMSS
```

Solo imprimir resumen en consola:

```powershell
py -3.13 pc\identificacion_modelos\ver_resultados.py --robot 1 --no-show
```

Abrir PNG con el visor del sistema:

```powershell
py -3.13 pc\identificacion_modelos\ver_resultados.py --robot 1 --open-png
```
