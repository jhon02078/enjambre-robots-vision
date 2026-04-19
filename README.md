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

## Autor

**Jhon Meneses**
