# Hardware del robot

Esta carpeta agrupa los modelos mecánicos imprimibles y la documentación
disponible de la placa electrónica.

## Estructura

```text
hardware/
├── cad/
│   ├── BaseYTorre.STL
│   ├── Engrane.STL
│   ├── Llanta.STL
│   └── TapaMotor.STL
└── pcb/
    ├── source/
    ├── imagenes/
    └── README.md
```

## Modelos CAD

| Archivo | Uso previsto |
|---|---|
| `BaseYTorre.STL` | Chasis/base y soporte principal |
| `Engrane.STL` | Elemento de transmisión |
| `Llanta.STL` | Rueda |
| `TapaMotor.STL` | Cubierta o retención del motor |

Los STL son mallas de fabricación, no archivos paramétricos. Antes de imprimir:

1. Revise unidades y escala en el laminador.
2. Compruebe diámetros de ejes y tolerancias.
3. Oriente piezas para reducir soportes y debilidad entre capas.
4. Valide que tornillos, motor, batería y PCB no interfieran.
5. Imprima una pieza de prueba cuando cambie de impresora o material.

La precisión dimensional final depende de material, contracción, altura de capa
y calibración de la impresora.

## Electrónica

La subcarpeta [`pcb/`](pcb/) contiene el esquemático disponible y fotografías de
la fabricación y montaje.

Componentes centrales identificados:

- ESP32 DevKitC;
- puente H TB6612FNG;
- regulador LM1584EN;
- motores DC;
- alimentación de motores y lógica.

Consulte [`pcb/README.md`](pcb/README.md) antes de fabricar o energizar.

## Integración mecánica y visión

El ArUco del robot debe:

- quedar horizontal y visible desde toda el área;
- estar rígidamente fijado;
- conservar la misma orientación respecto al frente del robot;
- evitar cables, tornillos o reflejos sobre su borde;
- usar el ID que coincide con el firmware.

La altura del marcador se configura en `Alt.Rob(m)` para compensar el paralaje.
Mídala desde el plano del suelo hasta la superficie impresa.

## Lista de comprobación

- Ruedas paralelas y sin rozamiento lateral.
- Centro de masa dentro del apoyo.
- Batería fijada y con capacidad de corriente suficiente.
- PCB aislada de piezas metálicas.
- Cables lejos de engranes y ruedas.
- Capacitores de desacoplo instalados.
- Polaridad y tierra común verificadas.
- Botón o conector de desconexión accesible.
- ArUco centrado, plano y asociado al ID correcto.

## Seguridad

No fabrique la PCB únicamente a partir de una imagen. Revise el archivo fuente,
las reglas eléctricas, huellas, anchos de pista, polaridad y disipación. Los
motores pueden producir corrientes de arranque muy superiores a su consumo en
vacío.

