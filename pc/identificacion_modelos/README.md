# Identificacion de modelos por vision

Herramienta Tkinter para identificar modelos dinamicos de los robots usando la camara, ArUco, homografia y comandos UDP `M L R`.

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
3. Verifica que el robot elegido tenga ArUco visible y discovery UDP.
4. Usa amplitud conservadora, por ejemplo `40-50%`.
5. Corre primero `Barrido angular`, luego `Barrido lineal`, o usa `Lineal + angular`.

La app hace auto-stop si pierde homografia, pierde el ArUco del robot o el robot se acerca demasiado al borde.

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

Archivos generados:

- `samples.csv`
- `frequency_response.json`
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
