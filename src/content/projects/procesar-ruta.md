---
title: "Procesar ruta: un analizador de tracks GPX para montaña"
category: "hobby-projects"
description: "Una herramienta gratuita para ver, unir, marcar y analizar tus archivos GPX: mapa, perfil, vista 3D, subidas, parciales y un planificador de horarios. Todo en tu navegador."
pubDate: 2026-10-19 19:00:00 -03
author: Gonzalo Olguin
tags: [montaña, gpx, mapas, herramientas]
heroImage: /assets/img/headers/route-tool.jpg
---

Después de cada salida a la montaña me quedaba con el mismo problema: el track estaba en el reloj o en Wikiloc, partido en dos o tres archivos porque se me cortó la grabación, y las apps que uso muestran lo básico (distancia, desnivel y una línea en el mapa). Quería algo que respondiera las preguntas que uno se hace de verdad en la montaña: *¿cuánto me demoré en la subida final?, ¿a qué ritmo subo con mochila?, ¿a qué hora tengo que salir para volver con luz?*

Así nació **[Procesar ruta](/ascents/route-tool)**, la misma herramienta que genera los mapas de mi bitácora de montañismo, ahora abierta para que cualquiera la use con sus propios archivos. Es gratis, no necesita cuenta y **tus archivos nunca salen de tu navegador**: todo se procesa en tu computador o celular.

## Cómo se usa

### 1. Sube uno o varios archivos GPX

Arrastra tus archivos `.gpx` a la zona de carga (o haz clic para elegirlos). Funciona con lo que exportan Garmin, Coros, Suunto, Strava, Wikiloc, Komoot y casi cualquier app.

Si grabaste la ruta en varias partes, súbelas todas juntas: la herramienta las **ordena por la hora en que fueron grabadas**, sin importar sus nombres, y las une en una sola ruta. En el ejemplo, los archivos de Punta Canaleta tienen nombres que no calzan con el orden real, y aun así quedan bien: subida del primer día, ataque a la cumbre desde el campamento y bajada.

![Zona de carga con tres archivos GPX ordenados cronológicamente](route-tool/upload.jpg "Tres archivos de la misma salida, unidos en orden cronológico")

Puedes agregar o quitar archivos cuando quieras y la ruta se recalcula al tiro.

### 2. Lee la ruta

Arriba aparecen las cifras principales: distancia, desnivel positivo, altitud máxima y tiempo en movimiento. Debajo, el mapa y el perfil de elevación están conectados: al pasar el cursor por uno se mueve un punto en el otro, con la distancia, altitud, pendiente, hora y pulso en ese punto.

![Tarjeta de la ruta con estadísticas, mapa coloreado por pendiente y perfil de elevación](route-tool/card.jpg "El mapa coloreado por pendiente, con el campamento y la cumbre detectados automáticamente")

Algunas cosas que conviene saber:

- **Colorear por**: en la leyenda del mapa puedes cambiar entre pendiente, altitud, velocidad, velocidad vertical (subiendo en rojo, bajando en azul) y pulso, si tu reloj lo registró.
- **Días y campamentos**: cualquier pausa de más de 3 horas se toma como una noche. La herramienta separa la ruta en días y marca dónde acampaste.
- **El botón ▶︎** recorre la ruta de principio a fin.
- **Capas**: topográfica, satélite o calles, en el botón de capas.
- Haz clic en cualquier punto de la ruta para ver sus coordenadas y abrirlo en **Google Earth 3D** o **Google Maps 3D**, con un pin en ese punto.

### 3. Analiza un tramo

Arrastra el mouse (o el dedo) sobre el perfil para seleccionar un tramo. Ves su distancia, desnivel, pendiente media, tiempo, velocidad vertical y velocidad, y el tramo queda resaltado en el mapa. Es la forma más rápida de saber cuánto te demoraste en ese acarreo eterno.

![Tramo seleccionado en el perfil de elevación con sus estadísticas](route-tool/selection.jpg "El ataque a la cumbre: 502 m de desnivel a 805 m/h")

### 4. Marca puntos

Este es el uso que más me interesaba. Presiona **Marcar** y haz clic en el mapa o en el perfil para dejar un punto. Si haces clic sobre la ruta, el punto se pega a ella; si haces clic fuera (por ejemplo, un mirador al otro lado del valle), queda exactamente donde hiciste clic.

Cada punto tiene un tipo (campamento, cumbre, mirador, agua, refugio, paso clave, peligro, estacionamiento o nota), un nombre y una nota opcional. Se pueden arrastrar, editar y borrar, y también puedes escribir o pegar sus coordenadas para dejarlo en un lugar exacto.

Sirve, por ejemplo, para la cumbre a la que no llegaste: márcala donde está y el punto más alto de tu track aparece como **Punto más alto** en vez de cumbre, y la cumbre muestra a qué distancia quedó de tu ruta.

![Editor de puntos sobre el mapa con los tipos de punto](route-tool/marks.jpg "Marcando un paso clave con su nota")

Lo importante: **los puntos se guardan en el GPX que descargas** como waypoints estándar, así que aparecen con su ícono en Garmin y en la mayoría de las apps. Es una buena forma de preparar un track para alguien que va a hacer la ruta después: dónde hay agua, dónde está el paso delicado, dónde conviene acampar. Además, el navegador recuerda tus puntos: si vuelves a subir los mismos archivos, siguen ahí.

También puedes marcar desde la pestaña **Paradas**: muestra todas las detenciones de más de 7 minutos, y con un clic conviertes cualquiera en un punto.

### 5. Mira el terreno en 3D

El botón **3D** muestra la ruta sobre el relieve real, con imagen satelital o mapa topográfico. Puedes inclinar y girar la vista, y con ▶︎ la cámara vuela siguiendo la ruta. Es la mejor forma que encontré de entender por dónde va realmente una ruta en un valle cerrado.

Y si prefieres Google Earth, el botón **Abrir en Google Earth** descarga la ruta completa como KML (con la línea de cada día y todos los puntos) y abre Google Earth en otra pestaña. Ahí se importa desde *Proyectos → Nuevo proyecto → Importar archivo KML desde el computador*; en Google Earth Pro o en la app del celular basta con abrir el archivo.

![Vista 3D de la ruta sobre imagen satelital y relieve](route-tool/3d.jpg "La misma ruta en 3D, con los puntos marcados")

### 6. Análisis detallado

Bajo el mapa está el análisis completo, separado en pestañas:

- **Resumen**: km-esfuerzo, velocidad de ascenso y de descenso, tu ritmo comparado con la norma DIN 33466, tiempo detenido, qué parte de la ruta es subida, llano o bajada, el oxígeno disponible en el punto más alto y las horas de luz de ese día.
- **Subidas y bajadas**: detecta cada subida y bajada importante, con su desnivel, pendiente media y máxima, tiempo y velocidad vertical. Al hacer clic en una, el mapa se acerca a ese tramo.
- **Parciales**: tiempo, ritmo, desnivel y pendiente por cada kilómetro (o cada 2, 5 o 10 km en rutas largas).
- **Pendiente** y **Altitud**: cuánta distancia hiciste en cada rango de pendiente y cuánto tiempo estuviste en cada franja de altura.
- **Pulso**: tiempo en cada zona de frecuencia cardíaca, si tu reloj la registró.

:::carousel{ratio=auto caption="Algunas de las pestañas del análisis detallado"}
![Pestaña Resumen](route-tool/tab-summary.jpg "Resumen: esfuerzo, ritmo y condiciones")
![Pestaña Subidas y bajadas](route-tool/tab-climbs.jpg "Subidas y bajadas detectadas automáticamente")
![Pestaña Pendiente](route-tool/tab-grade.jpg "Distancia por rango de pendiente, con los mismos colores del mapa")
:::

Dos cifras que vale la pena explicar:

- **Km-esfuerzo**: la distancia más 1 km por cada 100 m de subida y por cada 150 m de bajada. Es el *Leistungskilometer* que usa el Club Alpino Suizo, y sirve para comparar salidas muy distintas entre sí: 11 km con 1.700 m de desnivel "cuestan" como 40 km en plano.
- **Ritmo vs. DIN 33466**: la norma alemana para estimar tiempos de marcha (4 km/h en plano, 300 m/h de subida, 500 m/h de bajada). Si tu ritmo es 0,5×, te demoras la mitad de lo que dice la norma. Este número es la clave de la siguiente pestaña.

### 7. Planifica tu próxima salida

La pestaña **Planificar** estima los horarios de la ruta: eliges la hora de salida, tu ritmo (que parte en el que mediste en tu propio track) y cuántos minutos descansas por hora. Te muestra la hora estimada de llegada a cada punto marcado, a la cumbre y al final, y te avisa si terminarías de noche: a qué hora deberías salir para volver con luz y la hora límite en la cumbre para dar media vuelta.

![Planificador con hora de salida, ritmo y horarios estimados](route-tool/tab-plan.jpg "Horarios estimados para cada punto, con el atardecer en cuenta")

Funciona también con rutas que no grabaste: sube el GPX de una ruta que bajaste de internet o que dibujaste en un planificador, ajusta el ritmo al tuyo, y tienes una estimación realista para tu próxima salida.

## Cómo está hecha

Para los curiosos:

- El sitio está hecho con [Astro](https://astro.build). El análisis del GPX es el mismo código para mis bitácoras (donde se calcula al construir el sitio) y para la herramienta (donde corre en tu navegador).
- El mapa usa [Leaflet](https://leafletjs.com) con mapas de [OpenTopoMap](https://opentopomap.org), OpenStreetMap e imágenes satelitales de Esri.
- La vista 3D usa [MapLibre GL](https://maplibre.org) con el relieve de los *Terrain Tiles* de Mapzen alojados en AWS. Todo gratuito y sin claves de API, y solo se descarga cuando presionas 3D.
- Las horas de amanecer y atardecer se calculan con las ecuaciones solares de la NOAA, y el oxígeno disponible con la atmósfera estándar.

Más adelante la quiero mover a una sección de **herramientas** propia, junto a otras que tengo en mente. Si la usas y se te ocurre algo que le falte, ¡cuéntame!
