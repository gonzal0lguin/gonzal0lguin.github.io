---
title: "Sem-NaVAE: navegación sin mapa guiada por semántica"
category: "academic-projects"
description: "El paper que salió de mi tesis, aceptado en IEEE RA-L: generar trayectorias con un CVAE y seleccionarlas con un VLM, en tiempo real y sin mapas."
pubDate: 2026-06-03
author: Gonz
tags: [robotics, navigation, CVAE, VLM, RA-L, paper]
heroImage: /assets/img/posts/sem-navae/hero.png
---

> **Sem-NaVAE: Semantically-Guided Outdoor Mapless Navigation via Generative Trajectory Priors**
> Gonzalo Olguín, Javier Ruiz-del-Solar.
> *IEEE Robotics and Automation Letters*, vol. 11, no. 8, pp. 9335–9342, agosto 2026.
>
> [arXiv](https://arxiv.org/abs/2602.01429) · IEEE Xplore *(agregar DOI)* · [video de un run completo](https://youtu.be/i3R5ey5O2yk)
>
> Financiado parcialmente por FONDECYT 1251823 y ANID CIA250010.

Este es el paper que salió de mi [tesis de magíster](/projects/tesis-navegacion-multinivel). Si te interesa el detalle largo — la derivación completa de las pérdidas, el dataset, todas las figuras — anda al post de la tesis. Acá cuento la versión del paper, que es bastante más compacta y además tiene cosas que la tesis no tiene: más *baselines*, un *ablation study* y una prueba de largo alcance en el parque O'Higgins.

## El problema en una frase

Un robot en exterior tiene que decidir por dónde ir. Si entrenas una red determinista a predecir la trayectoria "correcta", y en los datos aparece tanto rodear el árbol por la izquierda como por la derecha, la red aprende el promedio — que pasa justo por el árbol.

Los modelos generativos arreglan eso aprendiendo la **distribución** de trayectorias posibles. Pero el abanico que producen es geométricamente válido y semánticamente ciego: una ruta por el pasto y otra por la vereda son ambas físicamente posibles, y cuál prefieres depende de contexto que no se escribe bien en una función de costo.

**Sem-NaVAE** separa estrictamente las dos cosas:

- un **CVAE** genera >200 hipótesis de movimiento por paso, geométricamente factibles;
- un **VLM liviano (CLIPSeg)** las puntúa con segmentación de vocabulario abierto, usando *prompts* en lenguaje natural;
- un **planificador local del estado del arte** ejecuta los comandos de velocidad.

![Overview de Sem-NaVAE](sem-navae/overview.png "El generador propone hipótesis; se filtran por colisión, se proyectan a un mapa semántico FPV construido con el VLM, y se selecciona la óptima. La actualización no es periódica: la nueva mejor candidata se compara con la que se está ejecutando")

## Las tres contribuciones

**1. Planificación generativa restringida por percepción.** Una arquitectura CVAE con un objetivo de entrenamiento *Log-Mean-Exp* de tipo "múltiple opción", y decodificación restringida por PointNet. El resultado es un *manifold* de más de 200 trayectorias factibles que cubre la mayoría de las direcciones de movimiento disponibles.

El objetivo LME es la pieza que hace posible la multimodalidad real. Como cada muestra del dataset tiene **varias** trayectorias de referencia, mover la sumatoria dentro del logaritmo:

$$
\log \frac{1}{M}\sum_{m=1}^{M} p_\psi(\mathbf{y}^{(m)}\mid z, \mathbf{x})
$$

hace que el gradiente lo domine la referencia con mayor probabilidad bajo la distribución actual. El modelo se puede **especializar en un modo** sin que las otras alternativas válidas lo castiguen. Es matemáticamente análogo a los *Importance Weighted Autoencoders* y opera como un *Winner-Takes-All* suave.

**2. Selección semántica *zero-shot*.** Al desacoplar estrictamente el generador geométrico del modelo de segmentación de vocabulario abierto, el sistema se adapta a preferencias y terrenos nuevos sin reentrenar nada. Cambiar de comportamiento es editar una lista de strings y sus costos.

**3. Estrategia de actualización asíncrona.** Planificación global de baja frecuencia desacoplada de la ejecución local de alta frecuencia, con una política de cambio basada en histéresis que evita la oscilación de trayectorias y resuelve oclusiones temporales reevaluando continuamente el costo de la trayectoria en ejecución contra las nuevas candidatas.

Este tercer punto es el que más me costó llegar a formular bien, y el *ablation* lo confirma como no-trivial. Mientras el robot ejecuta una trayectoria, los costos de los *waypoints* ya recorridos se congelan y los futuros se reproyectan sobre cada nueva segmentación. El cambio solo se gatilla si:

$$
J(\mathbf{y}_\text{new}) < J(\mathbf{y}_\text{curr}) - \varepsilon
$$

Las frecuencias son $f_{gen} = 0{,}5$ Hz para la generación geométrica y $f_{clip} = 2{,}5$ Hz para la actualización semántica; el planificador local maneja obstáculos dinámicos a 50 Hz.

| Módulo | Tiempo de inferencia (200 predicciones) |
|---|---|
| Procesamiento de observaciones | 10 ± 0,74 ms |
| NaVAE | 103 ± 8 ms |
| CLIPSeg | 398 ± 13 ms |
| Filtro de colisiones | 0,28 ± 0,89 ms |
| Mapa de costos y costos | 3 ± 0,12 ms |

Que CLIPSeg corra **localmente** en ~400 ms es justamente lo que separa este enfoque de los que usan un VLM completo vía API: esos tienen latencias del orden de segundos y no pueden replanificar de forma continua.

## Setup y evaluación

Los datos se recolectaron teleoperando un Husarion Panther (LiDAR Ouster de 64 canales, cámara RGB genérica, IMU, GPS-RTK) en el campus de ingeniería de la Universidad de Chile. El dataset final tiene **1.706 muestras** (~8.500 trayectorias candidatas) derivadas de 15 recorridos completos cortados cada 0,5 s.

El punto clave: los mapas semánticos, AMCL y NavFN se usan **exclusivamente** para supervisión de entrenamiento y evaluación offline. El robot no tiene acceso a ninguno durante la inferencia.

Para evaluar generalización fuera de distribución (OOD), las 5 configuraciones de prueba — rutas de 120 a 240 m — se corrieron en un sector del campus estrictamente no visto. Ese sector difiere cuantitativamente de la distribución de entrenamiento: **41% menos caminos peatonales, 30% más densidad de obstáculos y 60% más vegetación**. Además la distribución semántica cambia: los caminos peatonales pasan de baldosas homogéneas en entrenamiento a una mezcla de baldosas, pavimento y gravilla en evaluación. Cada prueba se repitió 6 veces.

### Baselines

Esta es la parte que la tesis no tiene, y que mejora harto el argumento:

- **Nav2** — método basado en mapas, que establece una cota superior geométrica.
- **NoMaD** — política de difusión moderna, con mapa topológico.
- **MTG** — el *baseline* CVAE principal.
- **MTG'** — MTG reentrenado con nuestro módulo PointNet.
- **VL-TGS** — implementación propia, limitando la salida a las 6 trayectorias (sin colisión) más diversas en FDE.

### Resultados

| Modelo | SR ↑ | FD ↓ | EPT% ↑ | NTR% ↓ | T<sub>ratio</sub> ↓ | #RB ↓ |
|---|---|---|---|---|---|---|
| Nav2 *(cota superior, con mapa)* | 0,97 ± 0,17 | 5,85 ± 0,82 | 98,6 ± 0,93 | — | 1,28 ± 0,29 | — |
| **Sem-NaVAE** | **0,90 ± 0,30** | 6,02 ± 0,51 | **95,0 ± 3,12** | 23,4 ± 6,88 | **1,33 ± 0,03** | 2 ± 1 |
| NoMaD | 0,80 ± 0,40 | **5,19 ± 0,98** | 94,8 ± 0,61 | 22,9 ± 1,98 | 3,26 ± 0,70 | 2 ± 1 |
| MTG' | 0,63 ± 0,48 | 6,83 ± 0,87 | 90,7 ± 6,81 | **22,1 ± 2,32** | 2,27 ± 0,36 | 3 ± 1 |
| VL-TGS | 0,40 ± 0,49 | 7,34 ± 1,86 | 89,2 ± 15,1 | 25,8 ± 10,0 | 4,12 ± 0,88 | 4 ± 2 |
| MTG | 0,17 ± 0,37 | 12,8 ± 1,21 | 77,5 ± 13,4 | 32,7 ± 7,67 | 2,62 ± 0,56 | 4 ± 1 |

(SR: tasa de éxito. FD: distancia de Fréchet contra una teleoperación humana en la misma ruta. EPT: % de la trayectoria ejecutada que se mantiene en zonas transitables preferidas. NTR: % de *waypoints generados* que caen en regiones no transitables. T<sub>ratio</sub>: tiempo autónomo / tiempo teleoperado. #RB: *recovery behaviors* gatillados por run.)

Contra los *baselines* sin mapa métrico, Sem-NaVAE mejora **10% en SR, 0,2% en EPT y 44% en T<sub>ratio</sub>**, y queda competitivo contra Nav2 — que tiene mapa, o sea, que juega con ventaja.

![Comparación de abanicos generados por los distintos modelos](sem-navae/baselines.png "Izquierda: mapa de elevación con las trayectorias propuestas (azul = colisión, verde = espacio libre). Derecha: las trayectorias generadas proyectadas a la imagen FPV")

Un par de lecturas sobre esta tabla:

**El NTR comparable con MTG' es un trade-off deliberado, no una debilidad.** El CVAE está entrenado para generar un volumen grande y diverso de trayectorias, algunas de las cuales exploran regiones no transitables. El módulo de selección las filtra, y por eso el EPT termina **mejor** que el de MTG' y NoMaD. Lo que importa no es la calidad promedio del abanico sino que el abanico contenga buenas opciones.

**Por qué fallan los demás.** MTG tiene baja variabilidad de salida por la sensibilidad de su encoder LiDAR a cambios de dominio. MTG' mejora eso con el PointNet pre-entrenado — de hecho consigue el mejor NTR — pero le falta la curvatura geométrica necesaria para pasar por espacios estrechos. NoMaD consigue mejor FD porque sigue de cerca su mapa topológico previo, pero su dependencia puramente visual lo degrada con variaciones mínimas de iluminación. Y VL-TGS sufre por latencia: como replanifica solo cuando completa una trayectoria entera, la navegación es inherentemente lenta y no apta para tiempo real.

### Ablation study

Dos rutas distintas, tres repeticiones cada una:

| Modelo | SR ↑ | FD ↓ | EPT% ↑ | NTR% ↓ | T<sub>ratio</sub> ↓ | #RB ↓ |
|---|---|---|---|---|---|---|
| **Sem-NaVAE** | **6/6** | **5,48 ± 0,22** | 94,5 ± 1,36 | **23,8 ± 0,67** | **1,34 ± 0,08** | **1 ± 1** |
| Sem-NaVAE-DWA | **6/6** | 5,53 ± 0,34 | **95,2 ± 1,90** | 23,2 ± 2,06 | 1,40 ± 0,18 | 2 ± 1 |
| NaVAE-FixF | 4/6 | 5,95 ± 0,45 | 90,7 ± 3,5 | 23,5 ± 1,29 | 1,37 ± 0,05 | 3 ± 2 |
| Sem-NaVAE-NoCol | 2/6 | 7,27 ± 0,36 | 82,2 ± 0,74 | 43,5 ± 4,94 | 1,41 ± 0,05 | 4 ± 1 |
| Sem-NaVAE-BaseL | 2/6 | 8,27 ± 0,98 | 84,5 ± 2,12 | 54,0 ± 0,70 | 1,56 ± 0,21 | 5 ± 2 |

Tres conclusiones:

- **La función de pérdida importa mucho.** Con la pérdida estándar de MUSE-VAE (`BaseL`), el desempeño se desploma: menos diversidad de predicción y muchas más colisiones (NTR sube de 23,8% a 54%). Lo mismo al quitar la pérdida de colisión (`NoCol`).
- **La actualización asíncrona también.** Con frecuencia fija (`FixF`), el sistema puede volver a una trayectoria que no es mejor que la anterior. Sin información global, eso es problemático en mínimos locales donde el robot queda atrapado — se ve en los #RB, que suben de 1 a 3.
- **El planificador local da lo mismo.** DWA y el basado en RL dan resultados prácticamente iguales, que es exactamente lo esperado: el sistema es agnóstico al nivel inferior, siempre que éste tenga buena evasión de obstáculos.

## Resultados cualitativos

![Trayectorias ejecutadas variando clases y costos de selección](sem-navae/costs.png "Abajo: variar el costo de la clase 'grass'. Arriba: agregar la clase 'sand', que sin estar presente se trata como pavimento normal")

Lo que más me gusta mostrar: bajar el costo de `grass` hace que el robot corte derecho por el pasto; subirlo lo hace dar la vuelta por el pavimento. Y se puede **agregar una clase nueva** (`sand`, para distinguir gravilla de pavimento) que el modelo nunca vio como categoría separada. Sin reentrenar nada. Eso es lo que da el vocabulario abierto.

![Navegación de largo alcance en un parque urbano](sem-navae/park-run.png "Seis instantes del recorrido con sus trayectorias seleccionadas y ejecutadas sobre imagen satelital")

La segunda prueba cualitativa es la que más me costó montar y la que más satisfacción me dio: una navegación de **largo alcance (>1 km)** en un parque urbano, usando solo odometría GPS. El robot llega con intervención mínima y sin ningún mapa.

Lo honesto de esa figura es que se nota que las trayectorias seleccionadas **no están en un intervalo constante** y que muchas no son enteramente factibles — pero el selector cambia en el momento adecuado para evitar colisiones o terreno no deseado. Esa es, más o menos, toda la tesis en una imagen.

## Limitaciones

Dos que dejamos explícitas en el paper:

1. **Diversidad geográfica.** La evaluación cubre rutas extensas, pero está limitada a entornos de campus y parque. Falta validar generalización con clima variado, condiciones de iluminación adversas y tráfico peatonal más denso.
2. **Pocas repeticiones por ruta.** La varianza en algunas métricas lo refleja. Hace falta testeo más extensivo para desglosar bien los modos de falla en casos borde.

## Conclusión

El aporte que creo que vale no es ninguna de las dos mitades por separado — los CVAE generativos existen, CLIPSeg existe — sino el **desacoplamiento estricto entre el generador geométrico y el selector semántico**. Esa es la propiedad arquitectónica que permite operación en tiempo real y adaptabilidad *zero-shot* al mismo tiempo, mientras el mecanismo de actualización asíncrona la hace resiliente a oclusiones temporales.

---

El [post de la tesis](/projects/tesis-navegacion-multinivel) tiene la versión larga: el dataset, las derivaciones completas, las figuras de arquitectura detalladas y varias cosas que no cabían en 8 páginas.
