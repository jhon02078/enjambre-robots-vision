# Paquete reproducible del artículo

Este directorio acompaña el artículo **Centralized Multi-Robot Control Based on
Overhead Vision and Artificial Potential Fields** de Jhon Meneses, Jean Carlos
Meneses, Holger Sanmartín y Tito Calva.

## Contenido publicado

- `resultados/seleccion_final_80.json`: selección explícita de las 80 corridas.
- `resultados/raw/`: cinco archivos crudos por corrida (`manifest.json` y cuatro
  CSV), sin pilotos ni repeticiones descartadas.
- `resultados/final_80/`: tablas, contrastes y figuras derivados.
- `configuracion_publica/`: matriz intrínseca y mediciones estáticas empleadas
  para validar calibración, paralaje y alineación XY.
- `resultados/raw/calibracion/images_tilted/`: 25 vistas aceptadas del tablero
  de 9 por 6 esquinas internas, con cuadros de 20 mm.
- `../pc/experimentos/`: instrumentación, análisis, pruebas y regeneración.
- `reproducibilidad/`: procedencia, instantánea del software experimental y
  hashes SHA-256.

La selección contiene 20 cruces frontales, 20 cruces perpendiculares y 40
pruebas de latencia entre 0 y 200 ms. En los cruces se comparan APF puro y
APF+FSM con 10 corridas por controlador y escenario. Las pruebas de latencia
contienen ocho corridas por nivel.

## Preparación

Desde la raíz del repositorio:

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
python -m pip install -r pc\requirements.txt
```

En Linux, active el entorno con `source .venv/bin/activate`.

## Verificar integridad

```powershell
python pc\experimentos\verificar_publicacion.py
```

La verificación exige 80 rutas únicas, los nueve grupos previstos, los cinco
archivos de cada corrida, ausencia de rutas privadas y coincidencia de los
hashes publicados.

## Regenerar resultados

```powershell
python pc\experimentos\reproducir_resultados.py
```

El comando vuelve a calcular las métricas por corrida, la comparación de
controladores, el barrido de latencia y las validaciones de localización e
intrínseca. Escribe en `paper/resultados/regenerados/`, compara las tablas y
resúmenes con `final_80/` y no modifica los resultados publicados. Los
subdirectorios `processed/` creados junto a cada corrida están ignorados por
Git.

## Alcance de la reproducibilidad

Los CSV publicados permiten regenerar las cifras, tablas y gráficas del
artículo. Los manifiestos conservan `git.dirty=true` porque esa fue la condición
real durante la campaña; no se reescribió la procedencia para aparentar un
estado limpio. `reproducibilidad/software/` conserva una instantánea de fin de
campaña, pero no demuestra que cada corrida física usara un árbol idéntico byte
por byte. Esta limitación no afecta el recálculo de los resultados desde los
registros, pero sí impide prometer una reconstrucción histórica exacta de cada
ejecución.

Se retiraron únicamente rutas absolutas del equipo y credenciales Wi-Fi. Los
valores experimentales, marcas de tiempo, parámetros, resultados favorables y
fallos de tarea se conservaron.
