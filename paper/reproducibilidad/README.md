# Procedencia del paquete

Las corridas se registraron en la rama local `codex/paper-experimentos`. Todos
los manifiestos señalan el commit base
`fa313c5a48dfcaf4d082ceb2a64612aa9e175c83` y `dirty=true`.

La carpeta `software/` es una instantánea de fin de campaña que incluye el
servidor de visión, el servidor de cámara y el firmware de los robots 2 y 3.
Las herramientas de adquisición y análisis se publican en
`../../pc/experimentos/`. Las credenciales Wi-Fi se sustituyeron por un include
local y una plantilla ficticia.

`SHA256SUMS.txt` protege los datos, configuraciones, scripts e instantánea
publicados. Se valida con:

```powershell
python pc\experimentos\verificar_publicacion.py
```

La instantánea mejora la trazabilidad, pero no convierte retrospectivamente un
árbol sucio en un commit exacto por corrida. Esta salvedad debe mantenerse en
cualquier declaración de disponibilidad o reproducibilidad del artículo.
