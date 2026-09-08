# Herramientas experimentales del paper

Este paquete añade instrumentación cuantitativa a `pc_servidor_vision.py` sin
activarla durante el uso normal. Los CSV se escriben en un hilo independiente
para reducir la perturbación del lazo de control.

## Componentes

| Archivo | Función |
|---|---|
| `runtime.py` | Registrador CSV asíncrono y cola de retardo controlado |
| `camera_stream.py` | Lectura MJPEG robusta con reconexión y buffer acotado |
| `calibrar_camara.py` | Captura de checkerboard y calibración intrínseca |
| `camera.py` | Carga, escalado y aplicación de la calibración |
| `analizar_experimento.py` | Métricas y figuras de una corrida |
| `comparar_corridas.py` | Estadística agrupada y comparaciones entre condiciones |
| `reproducir_resultados.py` | Regeneración completa del conjunto publicado |
| `verificar_publicacion.py` | Integridad, estructura y saneamiento del paquete público |
| `test_pipeline.py` | Prueba sintética del registro y análisis |

La campaña publicada y sus comandos de reproducción están documentados en
[`paper/README.md`](../../paper/README.md).

## Archivos por corrida

```text
paper/resultados/raw/<escenario>/<fecha>_<condicion>_rNN/
├── manifest.json
├── frames.csv
├── control.csv
├── network.csv
└── events.csv
```

- `frames.csv`: detección, homografía, pose sin paralaje, pose corregida y pose
  filtrada para cada robot y frame.
- `control.csv`: errores, estado, vectores APF, acción lineal/angular y comandos
  de ruedas.
- `network.csv`: encolado, envío, ACK, RTT y retardo realmente aplicado.
- `events.csv`: objetivos, transiciones, pérdidas de pose, referencias físicas y
  paradas.
- `manifest.json`: commit, configuración, geometría, ganancias y condición.

## Procesar una corrida

```powershell
python pc\experimentos\analizar_experimento.py `
  paper\resultados\raw\cruce_2\YYYYMMDD_HHMMSS_cruce_2_propuesto_r01
```

Genera `summary.json`, métricas por robot, trayectorias, series temporales,
distancias entre robots y resultados de localización cuando existen marcas GT.

## Comparar todas las corridas

```powershell
python pc\experimentos\comparar_corridas.py --reprocess
```

Las estadísticas incluyen media, desviación estándar e intervalo de confianza
del 95 %. No interprete un grupo con `n=1` como evidencia estadística.

## Compatibilidad UDP

El PC envía `M L R S seq`. Un firmware anterior seguirá leyendo `M L R` e
ignorará el resto. Los sketches experimentales responden:

```text
ACK ID=n SEQ=s RXMS=t
```

Sin el firmware experimental se registran comandos, pero no se obtiene RTT.
