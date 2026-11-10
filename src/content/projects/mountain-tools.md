---
title: "mountain-tools: manual de uso"
category: "hobby-projects"
description: "El manual completo de mountain-tools, mi caja de herramientas en Python para datos de montaña. Por ahora trae una: photo topo, que dibuja tu ruta GPX sobre una foto del cerro calzando la cámara contra el terreno."
pubDate: 2026-11-10
heroImage: /assets/img/headers/mountain-tools.jpg
author: Gonzalo Olguin
tags: [montaña, gpx, python, herramientas, fotos]
---

Esto es el manual de [mountain-tools](https://github.com/gonzal0lguin/mountain-tools), un repositorio en Python donde voy juntando las herramientas que me hago para la montaña. Es un repo guagua: hoy tiene una sola herramienta, **photo topo**, pero está armado para que vayan llegando más, todas detrás de un mismo comando (`mtn`) y compartiendo las mismas piezas: el modelo de elevación, el lector de GPX, los metadatos de las fotos y los nombres de los cerros.

Si te interesa más el *por qué* que el *cómo*, la historia está en [Mirar un cerro como lo miraría un robot](/projects/mirar-como-un-robot/): es la misma matemática con la que un robot proyecta su trayectoria a la imagen de su cámara. Acá voy al grano.

## Las herramientas

| herramienta | comando | qué hace | dónde probarla |
| --- | --- | --- | --- |
| photo topo | `mtn topo` | calza la cámara de una foto contra el terreno y dibuja una ruta GPX encima | [en este sitio](/ascents/tools/photo-topo/), con mis fotos |

Cuando haya más, van a aparecer en esta tabla y cada una va a tener su sección acá abajo.

## Instalación

Necesitas Python 3.10 o más nuevo. Las dependencias son sólo NumPy, SciPy y Pillow: nada de GDAL, nada de rasterio, nada que compilar.

```bash
git clone https://github.com/gonzal0lguin/mountain-tools && cd mountain-tools
python3 -m venv .venv && .venv/bin/pip install -e .
.venv/bin/mtn --help
```

Recomiendo de verdad usar el entorno virtual (`.venv`). En mi computador el Python del sistema tiene una mezcla de NumPy 1 y 2 que se cae al importar, y me costó un rato darme cuenta.

Los modelos de elevación y las respuestas de OpenStreetMap se guardan en `~/.cache/mountain-tools`. Si quieres que vayan a otro lado, cambia la variable `MOUNTAIN_TOOLS_CACHE`.

## Photo topo

Una línea de ruta sobre una foto sólo queda bien si se sabe dónde estaba la cámara: dónde estaba parada, hacia dónde apuntaba, cuánto estaba inclinada y girada, y qué focal tenía. Photo topo calcula todo eso haciendo que la foto calce con el terreno. Lee el horizonte de la foto, calcula el horizonte que tendría el [Copernicus GLO-30](https://dataspace.copernicus.eu/explore-data/data-collections/copernicus-contributing-missions/collections-description/COP-DEM) desde una cámara candidata, y va moviendo la cámara hasta que los dos coinciden. Con esa cámara se proyecta cada punto del track, y para cada uno se revisa si hay línea de vista o si queda tapado por un filo.

Son tres pasos:

```bash
mtn topo fit   foto.jpg ruta.gpx       # calcula la cámara → foto.scene.json
mtn topo edit  foto.scene.json         # ajustas y le das estilo en el navegador
mtn topo render foto.scene.json        # → foto.topo.jpg
```

### Lo que necesitas

- **Una foto donde se vea el filo contra el cielo.** Es lo que se compara. Una cara del cerro que llena toda la foto, sin cielo, también se puede pero cuesta más (ver [puntos de control](#cuando-no-calza)).
- **El GPX de la salida.** Un track del reloj o una ruta, da lo mismo.
- **La foto original, con su EXIF.** Ahí viene el GPS, la brújula y el lente, que le dicen a la búsqueda por dónde empezar. WhatsApp y casi todas las apps de mensajería borran esos datos, así que usa la foto que sacaste del teléfono, no la que te mandaron.

Si la foto no tiene GPS, dale la posición a mano con `--lat` y `--lon`. Si la brújula está mala, `--heading`; y si no sabe el lente, `--focal35` con la focal equivalente en 35 mm (la cámara principal de casi todos los teléfonos anda por los 24–26 mm).

Hay una cosa que ayuda mucho y no tienes que hacer nada: si sacaste la foto mientras grababas el track, la hora de la foto dice dónde estabas parado en ese segundo, y la búsqueda parte de ahí.

### Paso 1: calcular la cámara

```bash
.venv/bin/mtn topo fit examples/union.jpg examples/union.gpx
```

La primera vez en una zona se demora, porque tiene que bajar los modelos de elevación: unos 40 MB por cada cuadrado de 1°, y la primera búsqueda mira los 360° así que puede pedir cuatro a seis. Lo va diciendo por la terminal, así que si parece pegado, probablemente está descargando. Después de eso un ajuste toma unos segundos y termina así:

```
camera -33.74815, -70.05055 at 3255 m, 24 m from where it started
heading 210.6°, tilt 10.5°, roll 2.3°, lens 0.763 of the width
skylines 7.4 px apart on 96% of the ridge
wrote examples/union.scene.json
```

Las dos cifras de la última línea son las que importan:

- **px apart**: a cuántos pixeles queda, en promedio, el horizonte de la foto del horizonte calculado.
- **of the ridge**: en qué parte del horizonte los dos coinciden.

Hay que mirarlas juntas. Un error bajo sobre un 40% del filo puede ser que calzó perfecto un pedazo equivocado. Si la coincidencia baja del 60% o el error pasa del 0,8% del ancho de la foto, el comando te dice que el ajuste es débil y que lo revises en el editor.

Todas las opciones:

| opción | para qué |
| --- | --- |
| `-o`, `--output` | dónde guardar la escena (por defecto `foto.scene.json`, al lado de la foto) |
| `--lat`, `--lon` | posición de la cámara, si la foto no tiene GPS o lo tiene malo |
| `--heading` | hacia dónde apuntaba, en grados desde el norte |
| `--focal35` | focal equivalente en 35 mm |
| `--reach` | hasta cuántos kilómetros buscar horizonte (por defecto 60) |
| `--no-peaks` | no preguntar a OpenStreetMap por los nombres de los cerros |

Un detalle: la escena guarda la foto y el GPX por nombre, así que los tres archivos tienen que estar en la misma carpeta. Si no, el comando se niega.

#### Qué pasa por dentro

Para quien le interese, son cuatro piezas, y cada una tiene su explicación larga al principio de su archivo en el repo:

1. **Leer el horizonte de la foto** (`skyline.py`). No se clasifican pixeles en cielo o cerro, porque la nieve con cielo nublado es más blanca que la nube. Se busca un borde: una fila por columna donde hay un cambio fuerte, lo de arriba es suave y no salta mucho de una columna a otra. El camino más barato de izquierda a derecha se encuentra con programación dinámica. Eso solo se confunde con las nubes (el borde más alto es el de la nube), así que después aprende el color del cielo de esa foto y cobra por cada pixel con color de cielo que quede *debajo* del borde. Las columnas en que no está seguro las deja vacías en vez de adivinar.
2. **Calcular el horizonte del modelo** (`terrain.py`). Es el mismo LiDAR simulado del [post del robot](/projects/mirar-como-un-robot/): un abanico de rayos desde la cámara, recorriendo el terreno y quedándose con el ángulo más alto.
3. **Calzarlos** (`fit.py`). Es una calibración de extrínsecos contra un cerro. Girar, inclinar o hacer zoom no requiere volver a mirar el terreno, sólo reproyectar, así que es barato; mover la cámara sí, y por eso la posición se mueve poco y con cuidado. El error se satura, para que la columna donde el lector siguió un árbol pierda la votación en vez de arrastrar todo.
4. **Proyectar la ruta** (`scene.py`). Cada punto del track se pasa a rumbo, ángulo y distancia desde la cámara, y se revisa si se ve o queda tapado.

### Paso 2: el editor

```bash
.venv/bin/mtn topo edit examples/union.scene.json
```

Abre una página en tu navegador, servida desde tu propio computador (`127.0.0.1`, puerto 8765; cámbialo con `--port`, y con `--no-browser` no abre nada). Tiene tres pestañas.

#### Alinear

Sobre la foto aparece en blanco el contorno del terreno, y en rosado el horizonte que se leyó de la foto.

- **Arrastra** la foto para mover el contorno. **Rueda** para cambiar el zoom del lente, **Shift + rueda** para girarla.
- **Ajustar al horizonte** termina el trabajo después de un arrastre a ojo.
- Si lo que está mal es la posición de la cámara, muévela unos metros y **Recalcular desde aquí**: el terreno se vuelve a calcular desde ese punto, partiendo de tu alineación.
- Si la línea rosada sigue una nube o una fila de árboles, **Dibuja** el filo real encima o **Borra** el tramo malo, y vuelve a ajustar. **Calcular de nuevo desde cero** olvida todo y parte otra vez desde lo que decía la foto.
- **Puntos de control**: si reconoces una cumbre en la foto, elígela, aprieta *Marcarla en la foto* y haz clic sobre ella. El contorno salta ahí y todos los ajustes siguientes la respetan.

#### Estilo

Acá se pasa la mayor parte del tiempo, la verdad.

- **Ruta**: color, ancho, continua, segmentada o punteada, opacidad y suavizado. Qué hacer con la parte que queda detrás del cerro (igual, segmentada u oculta). Desde y hasta qué kilómetro dibujar, o sólo la subida.
- **Tramos**: *Elegir un tramo* y dos clics sobre la ruta, y esa parte tiene su propio color, línea y ancho. Lo uso para la variante, el glaciar o el pedazo de trepe.
- **Símbolos**: campamento, refugio, bandera, agua y otros, con nombre si quieres. Si lo sueltas sobre la línea se queda pegado a la ruta; si no, queda pegado a ese punto del cerro aunque re-apuntes la cámara.
- **Cumbres**: los cerros con nombre que se ven, sacados de OpenStreetMap. Marcas los que quieres, arrastras el nombre a donde quede bien y la línea indicadora lo sigue. La altura y las mayúsculas se prenden y apagan.
- **Corregir la ruta**: con celdas de 30 metros, cerca de la cámara la ruta a veces queda flotando o enterrada. Activa *Ajustar ruta* y arrastra la línea a donde va de verdad. Cada arrastre deja un punto de control (doble clic para quitarlo) y la ruta se dobla alrededor.
- **Texto** que se arrastra a cualquier parte, y **Encuadre** (`4:5`, `1:1`, `9:16`, `16:9`) para publicar.

#### Exportar

JPG o PNG a la resolución completa de la foto y con el encuadre elegido, y el archivo de escena para seguir otro día. En el editor local, *guardar* sobrescribe el `.scene.json` en tu disco.

### Paso 3: dibujarla desde la terminal

```bash
.venv/bin/mtn topo render examples/union.scene.json --width 1600
```

Lee el mismo estilo que dejaste en el editor y escribe `union.topo.jpg` (o lo que digas con `-o`). Sirve para generar varias de una vez o meterlo en un script, y además es la prueba de que la escena significa lo mismo fuera del navegador: las reglas de dibujo están escritas dos veces a propósito, una en Python y otra en JavaScript.

### Sin instalar nada

El editor también está [en este sitio](/ascents/tools/photo-topo/), con tres fotos mías cargadas (Unión, San Gabriel y la [travesía del Algarrobo](/ascents/algarrobo-traverse/)). Funciona sin servidor porque la escena guarda todo como ángulos desde la cámara: re-apuntar es aritmética que hace el navegador. Lo que no puede hacer ahí es *mover* la cámara, porque para eso se necesita el modelo de elevación, que vive en el lado de Python. También puedes arrastrar a esa página una escena y una foto tuyas.

:::gallery{cols=2 ratio=auto}
![El track de la travesía del Algarrobo sobre una foto, con seis cumbres con nombre](mountain-tools/algarrobo-topo.jpg "Travesía del Algarrobo: la ruta cruza toda la foto y OpenStreetMap pone los nombres")
![El track de la subida al San Gabriel sobre una foto al amanecer](mountain-tools/san-gabriel-topo.jpg "San Gabriel al amanecer: nieve del mismo color que el cielo")
:::

### Cuando no calza

El ajuste es automático pero no infalible. Lo que me ha pasado, y cómo lo arreglo:

- **Calzó otro pedazo de filo.** Un tramo parecido de la cordillera puede calzar casi igual de bien. Arrastra el contorno a ojo a donde va y *Ajustar al horizonte*. Si sigue mal, *Recalcular desde aquí*.
- **Nubes sobre el filo.** Casi siempre las maneja solo. Esta foto de Punta Canaleta fue la que le enseñó: antes seguía la parte de arriba de los cúmulos y terminaba 60° corrido. Si igual se confunde, borra el tramo de la línea rosada que va por la nube.
- **Árboles, cactus o una persona delante del filo.** El lector no los distingue. Borra o dibuja.
- **Una cara que llena la foto, sin horizonte.** Ahí sirven los puntos de control: con dos o tres cumbres marcadas a mano la cámara queda amarrada. Un clic se toma en serio pero no a ciegas: si está muy en contra del resto, queda una línea celeste que no se cierra, avisando que algo no cuadra.
- **La ruta cerca de la cámara queda rara.** Si la foto es de la misma salida, el tramo cercano se dibuja con las posiciones y alturas del reloj en vez del modelo de elevación, partiendo en línea recta desde tus pies. Más allá de unos cientos de metros vuelve al modelo, y en ese cambio la ruta a veces se quiebra. Para eso está *Ajustar ruta*.
- **El GPS de la foto está malo.** Pasa más de lo que uno cree. `--lat` y `--lon`, o mueve la cámara en el editor local.

:::gallery{cols=2 ratio=auto}
![Foto de Punta Canaleta con cúmulos sobre el filo](mountain-tools/canaleta-foto.jpg "La foto original: cúmulos justo sobre el filo")
![La misma foto con la ruta dibujada](mountain-tools/canaleta-topo.jpg "El resultado, con la parte tapada en segmentado")
:::

### El archivo de escena

`fit` escribe un `*.scene.json` al lado de la foto. Es todo el resultado y la única conexión entre las partes: el ajuste lo escribe, el editor y `render` lo leen, y tú también puedes.

La regla de diseño más importante del repo es que **ahí no hay pixeles del terreno**. Cada punto del horizonte, de los filos y del track es un **rumbo, un ángulo de elevación y una distancia** desde la cámara. Por eso girar, inclinar o hacer zoom es reproyectar números que ya están, y por eso el editor funciona como página estática. Un pedacito, de la escena del Unión:

```json
{
  "photo": { "file": "union.jpg", "width": 2016, "height": 1512 },
  "camera": { "lat": -33.74815, "lon": -70.050551, "ground": 3254.8, "eye": 1.6,
              "yaw": 210.6268, "pitch": 10.4911, "roll": 2.2778, "lens": 0.7634 },
  "prior": { "position": "photo GPS", "heading": "photo compass", "lens_from": "photo" },
  "fit": { "error": 7.4, "agreement": 0.96, "moved": 23.7 },
  "track": { "points": [[198.489, 18.023, 697, 1, 1.068, 3495, 0], "…"] },
  "peaks": [{ "name": "Cerro Unión", "ele": 3652, "az": 203.06, "alt": 22.605, "dist": 971, "show": true }]
}
```

| clave | qué tiene |
| --- | --- |
| `photo` | nombre, ancho y alto de la foto a la que se refieren los pixeles |
| `camera` | `lat`, `lon`, `ground` (m), `eye` (m sobre el suelo), `yaw`, `pitch`, `roll` (grados), `lens` (focal dividida por el ancho, así sobrevive a achicar la foto) |
| `prior`, `fit` | de dónde partió la búsqueda y cómo terminó |
| `skyline`, `ridges` | el horizonte del modelo y los filos dentro de él |
| `track.points` | por punto: `[rumbo, ángulo, distancia, visible, km, altura, del_track]` |
| `track.pins` | las correcciones hechas a mano en *Ajustar ruta* |
| `peaks`, `labels`, `marks` | cumbres con nombre, textos anclados a una dirección, símbolos |
| `controls` | los puntos de control: un pixel (como fracción de la foto) y la cumbre que es |
| `photo_skyline` | el horizonte leído de la foto, en pixeles, con huecos donde no se confió |
| `style` | todo lo visual; los largos son fracciones del ancho de la foto |

El README del repo tiene la tabla completa.

### Como librería

Todo lo que hace el comando se puede llamar desde Python. Por ejemplo, ajustar y dibujar con otro color sin pasar por el editor:

```python
from mountain_tools.phototopo.scene import build_scene, save_scene
from mountain_tools.phototopo.render import render

scene = build_scene("foto.jpg", "ruta.gpx")
scene["style"]["track"]["color"] = "#00c2ff"
scene["style"]["hidden"]["mode"] = "dashed"
save_scene(scene, "foto.scene.json")
render(scene, "foto.jpg").save("foto.topo.jpg", quality=92)
```

Y las piezas compartidas sirven solas:

```python
from mountain_tools.dem import Dem
from mountain_tools.gpx import read_gpx
from mountain_tools.phototopo.terrain import scan

dem = Dem()
dem.elevation(-33.7463, -70.0479)          # altura del modelo en ese punto, en metros
track = read_gpx("ruta.gpx")               # puntos, alturas, horas y waypoints
# el horizonte desde ahí, como un LiDAR simulado: rumbo 210° ± 30°, un rayo cada 0,1°, hasta 60 km
view = scan(dem, -33.7482, -70.0506, 1.6, center=210, half=30, step=0.1, reach=60_000)
view.skyline, view.skyline_distance        # ángulo y distancia del horizonte en cada rumbo
```

Todos los ángulos son en grados, los rumbos en sentido horario desde el norte y las distancias en metros.

### El editor en tu propio sitio

Así está puesto en esta página. El editor son dos archivos, `editor.js` y `editor.css`, sin compilar nada y sin dependencias. Los copias a cualquier hosting estático, pones al lado una escena y su foto, y lo montas:

```html
<link rel="stylesheet" href="/phototopo/editor.css" />
<div id="topo"></div>
<script type="module">
  import { mountEditor } from "/phototopo/editor.js";
  mountEditor(document.getElementById("topo"), {
    demos: [{ name: "Cerro Unión", scene: "/phototopo/union.scene.json", photo: "/phototopo/union.jpg" }],
    strings: { Align: "Alinear", Style: "Estilo", Export: "Exportar" },
  });
</script>
```

- `demos` son las escenas que aparecen en el menú; la primera se carga al tiro.
- `strings` traduce la interfaz, usando el texto en inglés como llave. Lo que falte queda en inglés.
- `api` apunta a un `mtn topo edit` corriendo, si quieres poder mover la cámara y guardar.

Los colores salen de variables CSS (`--pt-bg`, `--pt-panel`, `--pt-fg`, `--pt-muted`, `--pt-line`, `--pt-accent`, `--pt-accent-fg`), así que se adapta al tema de cualquier sitio. La foto puede ser una copia más chica que la original: la escena guarda las dimensiones con que se ajustó y el editor escala.

### Límites

- **El modelo tiene celdas de 30 metros.** El terreno a menos de unos 100 metros no tapa nada (si no, el propio montículo donde estás parado taparía toda la vista), y una ruta a unos cientos de metros puede quedar decenas de pixeles corrida. Las caras lejanas quedan mejor.
- **Sólo entre los 50° norte y sur.** Más allá los cuadros del Copernicus cambian de forma y el lector no los entiende.
- **Sin distorsión del lente.** Los teléfonos ya la corrigen; un ojo de pez sin corregir no va a calzar.
- **Una foto a la vez.**
- El renderer de la terminal busca una fuente del sistema (DejaVu, Noto, Liberation, Arial). Si no encuentra ninguna, "Cerro Unión" sale con un cuadrado en vez de la ó; apunta `MOUNTAIN_TOOLS_FONT` a un `.ttf` que tengas.

## Para meterle mano

```bash
.venv/bin/pip install -e '.[dev]'
.venv/bin/python -m pytest -m "not slow"     # segundos
.venv/bin/python -m pytest                   # además vuelve a ajustar las fotos de ejemplo
```

Si tienes ROS instalado (como yo), sus plugins de pytest se cuelan y lo botan al partir: antepón `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`.

Las fotos de `examples/` son la prueba de regresión, y cada una rompió algo alguna vez: el Unión (un cono solitario, donde el horizonte casi no dice dónde estabas parado), el San Gabriel (nieve del color del cielo), el Algarrobo, la de los cúmulos de Punta Canaleta y dos sacadas sobre una arista de nieve, donde la ruta cercana no puede salir del modelo. Las tolerancias de los tests salen de ajustes que revisé a ojo, así que si un cambio saca una cámara de ahí, lo que corresponde es mirar la foto antes de agrandar la tolerancia.

![La ruta sobre una foto tomada en una arista de nieve](mountain-tools/arista-topo.jpg "Sobre la arista de Punta Canaleta: el tramo junto a la cámara sale del track del reloj, no del modelo")

### Agregar una herramienta

Una herramienta nueva es un subpaquete al lado de `phototopo` y un subcomando en `cli.py` (`mtn <herramienta> …`). Lo que necesiten dos herramientas sube un nivel, al lado de `dem.py` y `gpx.py`:

```
src/mountain_tools/
  geo.py dem.py gpx.py exif.py osm.py   compartido: geodesia, elevación, tracks, fotos, nombres
  cli.py                                `mtn`, un subcomando por herramienta
  phototopo/                            la primera
```

Cada módulo parte con un comentario que explica para qué es y por qué funciona así, no qué hace línea por línea. Y se prefiere un lector escrito a mano antes que una dependencia cuando el formato es chico: el GPX son unas cuantas expresiones regulares, y el GeoTIFF del Copernicus se lee sin GDAL.

## Lo que viene

Para photo topo, más o menos en orden: puntos de control en lugares que no son cumbres con nombre (un portezuelo, un punto de la ruta), un cambio más suave entre el tramo dibujado con el track y el dibujado con el modelo, usar la prominencia para elegir qué cumbres nombrar, colorear el track por altura y varias fotos de una misma ruta.

Y fuera de photo topo, las otras herramientas que hoy viven como scripts sueltos en este sitio, como los panoramas de cumbre y el análisis de rutas, tienen pinta de terminar acá también. Pero eso cuando les toque.
