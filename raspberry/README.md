# Cámara Raspberry Pi

Esta carpeta contiene el servidor HTTP/MJPEG usado por las aplicaciones de
visión del PC.

## Archivo

`raspberry_camara.py` configura Picamera2, codifica directamente en MJPEG y
publica:

| Ruta | Función |
|---|---|
| `/` | Página mínima con enlace al stream |
| `/video` | Flujo `multipart/x-mixed-replace` consumido por OpenCV |

Configuración actual:

```text
Resolución:   1280 x 720
FPS pedidos:  30
Calidad:      Quality.LOW
Buffers:      3
Queue:        False
Host:         0.0.0.0
Puerto:       5000
```

`queue=False` y una cola corta priorizan latencia baja. La tasa real depende de
la cámara, exposición, red y capacidad de la Raspberry.

## Requisitos

- Raspberry Pi OS reciente;
- cámara detectada por `libcamera`;
- Picamera2;
- Flask;
- acceso TCP al puerto `5000`.

Instalación típica:

```bash
sudo apt update
sudo apt install -y python3-picamera2 python3-libcamera python3-flask
```

En Raspberry Pi OS se recomienda usar los paquetes del sistema para Picamera2 y
libcamera, pues están integrados con el stack de la cámara.

## Comprobar la cámara

Según la versión de Raspberry Pi OS:

```bash
rpicam-hello
```

o:

```bash
libcamera-hello
```

La vista previa debe funcionar antes de iniciar Flask.

## Verificación necesaria del script

En la revisión actual del archivo hay dos errores tipográficos en el punto de
entrada. Antes de copiarlo o ejecutarlo en la Raspberry, verifique que la
primera línea y el bloque final sean exactamente:

```python
#!/usr/bin/env python3
```

```python
if __name__ == "__main__":
    iniciar_camara()
    app.run(
        host=HOST,
        port=PORT,
        threaded=True,
        debug=False,
        use_reloader=False,
    )
```

Sin el `#` del shebang o sin los guiones bajos de `__name__`, Python no iniciará
correctamente el servidor.

## Ejecutar

En la Raspberry, dentro de esta carpeta:

```bash
python3 raspberry_camara.py
```

Consulte su IP:

```bash
hostname -I
```

Desde el PC abra:

```text
http://<IP_RASPBERRY>:5000/
http://<IP_RASPBERRY>:5000/video
```

Use la segunda URL en `pc_servidor_vision.py` y
`app_identificacion.py`.

## Ajustes de imagen

Edite al inicio del script:

```python
WIDTH = 1280
HEIGHT = 720
FPS = 30
ENC_QUALITY = Quality.LOW
BUFFER_COUNT = 3
QUEUE = False
```

Prioridades recomendadas:

1. ArUco nítido y suficientemente grande.
2. Latencia baja y estable.
3. FPS suficiente para el control.
4. Resolución adicional solo si mejora realmente la detección.

Subir resolución, FPS y calidad al mismo tiempo aumenta CPU, memoria y ancho de
banda. Para control es preferible una señal estable a una cifra nominal alta.

Una exposición muy larga genera desenfoque de movimiento aunque el stream tenga
30 o 60 FPS. Mejore la iluminación antes de subir artificialmente la tasa.

## Rotación

La configuración usa:

```python
transform=Transform(rotation=0)
```

Cambie la rotación solo si la cámara está montada girada. Después verifique el
orden visual de los marcadores `4-7`; una rotación no corregida puede hacer muy
confusa la comparación entre imagen y mapa.

## Arranque automático opcional

Puede crear un servicio `systemd` apuntando a la ruta real del script:

```ini
[Unit]
Description=Stream MJPEG de la cámara
After=network-online.target

[Service]
Type=simple
User=pi
WorkingDirectory=/ruta/al/proyecto/raspberry
ExecStart=/usr/bin/python3 /ruta/al/proyecto/raspberry/raspberry_camara.py
Restart=on-failure
RestartSec=2

[Install]
WantedBy=multi-user.target
```

Cambie usuario y rutas antes de instalar el servicio.

## Solución de problemas

| Síntoma | Acción |
|---|---|
| `Picamera2` no importa | Instalar el paquete del sistema y usar el Python correcto |
| Cámara ocupada | Cerrar otras vistas previas o procesos Picamera2 |
| `/video` no abre desde el PC | Revisar IP, puerto, firewall y aislamiento del router |
| Imagen congelada | Reiniciar el proceso y revisar cable/cámara/alimentación |
| Latencia creciente | Reducir consumidores, calidad/FPS y revisar la red |
| ArUco borroso | Más luz, menor exposición y enfoque/montaje correctos |
| Flask arranca dos veces | Mantener `use_reloader=False` |

## Seguridad de red

Flask escucha en todas las interfaces y no incorpora autenticación. Úselo solo
en una red local de confianza; no exponga el puerto `5000` directamente a
Internet.

