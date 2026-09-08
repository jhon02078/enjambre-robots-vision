# Resumen estadistico de la campana final

## Navegacion y evitacion

| Escenario | Controlador | Exitos | Tasa (IC95% Wilson) |
|---|---|---:|---:|
| cruce_2 | APF puro | 4/10 | 40.0% (16.8-68.7%) |
| cruce_2 | APF+FSM | 9/10 | 90.0% (59.6-98.2%) |
| cruce_2_perpendicular | APF puro | 9/10 | 90.0% (59.6-98.2%) |
| cruce_2_perpendicular | APF+FSM | 10/10 | 100.0% (72.2-100.0%) |
| ambos cruces | APF puro (combinado) | 13/20 | 65.0% (43.3-81.9%) |
| ambos cruces | APF+FSM (combinado) | 19/20 | 95.0% (76.4-99.1%) |

En los dos cruces combinados, APF+FSM alcanzo 95% frente a 65% de APF puro (Fisher bilateral p=0.0436, diferencia absoluta +30 puntos porcentuales).

Las comparaciones inferenciales estan en `statistical_tests.csv`. La duracion se compara solo entre corridas exitosas para evitar interpretar una parada temprana como mejor tiempo.

## Latencia inyectada

| Solicitada (ms) | Real media (ms) | P95 medio (ms) | Sobrecoste (ms) | Exitos |
|---:|---:|---:|---:|---:|
| 0 | 0.00 | 0.00 | 0.00 | 8/8 |
| 50 | 57.86 | 64.95 | 7.86 | 8/8 |
| 100 | 107.61 | 115.03 | 7.61 | 8/8 |
| 150 | 155.52 | 163.76 | 5.52 | 8/8 |
| 200 | 207.87 | 214.83 | 7.87 | 8/8 |

En el intervalo ensayado de 0 a 200 ms no se observaron perdidas de estabilidad ni fallos de llegada (40/40). Esto demuestra robustez experimental dentro del intervalo, pero no constituye una prueba formal de estabilidad ni permite extrapolar por encima de 200 ms.

Los niveles de retardo se ejecutaron en orden ascendente. Por tanto, cualquier tendencia debe interpretarse junto con posibles efectos de orden, calentamiento y bateria; la evidencia principal es la tasa de exito y la ausencia de divergencia, no una mejora de rendimiento causada por el retardo.

## Calidad de vision

La homografia fue valida en promedio 100.00% de los frames de las 80 corridas. La deteccion media de los robots activos fue 98.62% y el procesamiento visual medio fue 84.67 ms.

## Exactitud de localizacion

En cinco puntos independientes, el RMSE fue 6.52 cm con homografia, 1.97 cm tras paralaje y 0.60 cm tras alineacion XY. El error maximo final fue 0.96 cm.

## Calibracion intrinseca

La calibracion uso 25 vistas y obtuvo RMS de reproyeccion de 0.163 px. En los cinco puntos estaticos no redujo el error metrico central: el RMSE crudo cambio de 6.56 a 6.96 cm y, tras paralaje, de 1.87 a 1.91 cm. La deteccion media aumento 6.22 puntos porcentuales.

Este resultado debe reportarse como una evaluacion, no como evidencia de mejora geometrica: en el area central la homografia ya absorbe buena parte de la deformacion y la muestra pareada es pequena.

