# Datos y resultados

## Datos crudos

Cada directorio incluido por `seleccion_final_80.json` contiene:

| Archivo | Contenido |
|---|---|
| `manifest.json` | Escenario, condición, parámetros, versiones y resultado |
| `frames.csv` | Detección, homografía, pose corregida y pose filtrada |
| `control.csv` | Errores, estado APF/FSM, acciones y comandos de rueda |
| `network.csv` | Envíos, ACK, RTT y retardo aplicado |
| `events.csv` | Objetivos, transiciones, paradas y eventos de seguridad |

Los datos crudos suman 80 corridas válidas para análisis. Una corrida válida
puede terminar en éxito o en fallo de tarea; solo se excluyeron abortos de
infraestructura según los criterios definidos para la campaña.

## Resultados derivados

`final_80/` contiene una fila por corrida, estadística agrupada, intervalos de
confianza, pruebas inferenciales y fidelidad del retardo inyectado. Las figuras
PNG son salidas derivadas y pueden variar visualmente entre versiones de
Matplotlib aunque las tablas numéricas coincidan.

Las validaciones estáticas están en `procesados/localizacion_grid/` y
`procesados/intrinseca_off_on/`. Sus entradas públicas están en
`../configuracion_publica/`.

No edite los CSV publicados. Para repetir el análisis use
`python pc/experimentos/reproducir_resultados.py` desde la raíz.
