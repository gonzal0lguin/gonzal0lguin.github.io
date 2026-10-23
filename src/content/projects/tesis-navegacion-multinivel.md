---
title: "Planificación multinivel para navegación robótica en ambientes de exterior"
category: "academic-projects"
description: "Mi tesis de magíster: un planificador global sin mapa que propone trayectorias con un CVAE y las elige con un modelo de visión-lenguaje."
pubDate: 2026-03-25
author: Gonz
tags: [robotics, navigation, CVAE, VLM, ROS, thesis]
heroImage: /assets/img/posts/sem-navae/park-run.png
---

> Tesis para optar al grado de Magíster en Ciencias de la Ingeniería, mención Eléctrica, y memoria para optar al título de Ingeniero Civil Eléctrico. Universidad de Chile, FCFM, Departamento de Ingeniería Eléctrica. Profesor guía: Javier Ruiz del Solar. Profesor co-guía: Francisco Leiva. Comisión: Martin Adams y Rodrigo Verschae. Financiado parcialmente por el proyecto FONDECYT 1251823.
>
> El documento completo está [acá](/assets/pdf/tesis_olguin_gonzalo_v3.3.pdf) (88 páginas, con todo el detalle que este post deliberadamente no tiene). El paper que salió de esta tesis lo cuento por separado en [Sem-NaVAE](/projects/sem-navae).

## De qué se trata

El problema es viejo y suena simple: que un robot llegue desde un punto A a un punto B, afuera, sin que nadie le pase un mapa.

La respuesta clásica es el pipeline *Map–Plan–Control*: construyes un mapa métrico, planificas una ruta óptima sobre él con A\* o RRT, y un controlador la ejecuta. Funciona muy bien en una bodega. Afuera no tanto: el mapa hay que construirlo, mantenerlo y corregirlo, y el mundo se mueve más rápido de lo que uno lo actualiza.

La alternativa que se ha impuesto es aprender el comportamiento desde datos. Pero ahí aparece un problema bastante elegante, que es el que me enganchó: **la unimodalidad**. Si entrenas una red determinista a predecir "la" trayectoria correcta y en los datos aparecen tanto rodear el árbol por la izquierda como por la derecha, la red aprende el promedio de ambas. El promedio de dos rutas que esquivan un árbol es una ruta que pasa justo por el árbol.

Los modelos generativos resuelven esto: en vez de aprender una función $f(\mathbf{x})$, aprenden una distribución $p(\mathbf{y}\mid\mathbf{x})$ y uno puede muestrear de ella. En vez de una ruta, un abanico de rutas.

Pero eso abre otro problema. El abanico es **geométricamente** válido, no **semánticamente**. Un robot en un parque puede generar una ruta sobre el pasto y otra por el sendero pavimentado; ambas son físicamente posibles, y cuál prefieres depende de normas sociales y de contexto que no se escriben bien en una función de costo tradicional.

La tesis propone juntar las dos cosas en un esquema de **"Generar y Seleccionar"**:

- un **modelo generativo (CVAE con *prior* aprendido)** que propone muchas hipótesis de movimiento viables,
- un **modelo de visión-lenguaje (VLM)** que las jerarquiza según preferencias escritas en lenguaje natural ("prefiere el pavimento", "evita el pasto", "no pases por la arena").

Todo esto como el nivel **superior** de una arquitectura de dos niveles: el inferior es un planificador local del estado del arte, intercambiable, que se encarga de los comandos de velocidad.

### Hipótesis

**H1.** Un modelo generativo condicional con *prior* aprendido captura la multimodalidad de la navegación en entornos no estructurados, superando la tendencia al promedio de los planificadores clásicos mediante la **proposición** de un abanico de trayectorias geométricamente viables.

**H2.** Integrar un VLM dota al sistema de discriminación semántica de vocabulario abierto, permitiendo **seleccionar**, entre esas propuestas, las que satisfacen preferencias simples en lenguaje natural — criterios de decisión inaccesibles para un enfoque puramente geométrico.

## La arquitectura

![Diagrama general de la solución al problema de navegación global](sem-navae/global-systemn.png "El planificador global completo: generación (CVAE) arriba, selección (VLM) abajo, y el planificador local como consumidor final")

El planificador global se divide en dos etapas secuenciales:

1. **Generación.** El submódulo *NaVAE* toma las observaciones del robot y el objetivo global, e infiere una distribución de trayectorias futuras. De ahí se muestrea un conjunto $\mathcal{Y}$ de $K$ trayectorias.
2. **Selección.** Se filtran las inviables por colisión, se construye un mapa de costos semántico con el VLM, se proyectan las trayectorias a ese mapa, y se elige la de menor costo. Un módulo final decide *cuándo* cambiar de trayectoria (no es trivial, lo explico más abajo).

### Generación: NaVAE

El contexto de entrada son tres modalidades:

$$
\mathbf{x} = \{\mathbf{x}_\text{lidar}, \mathbf{x}_\text{traj}, \mathbf{x}_\text{goal}\}
$$

donde $\mathbf{x}_\text{lidar}$ son las últimas $N_l$ nubes de puntos voxelizadas (con $(x,y,z)$ e intensidad), $\mathbf{x}_\text{traj}$ el historial de los últimos $N_v$ estados del robot (posición y velocidades lineal y angular) y $\mathbf{x}_\text{goal}$ el objetivo global en coordenadas polares $(\rho, \theta)$ relativas al robot.

La salida es una secuencia de $N_w$ posiciones futuras en el marco local del robot.

![Arquitectura del modelo generativo basado en CVAE con prior aprendido](sem-navae/arquitechture.png "Arriba el flujo de entrenamiento (con el encoder posterior); abajo el de inferencia, donde solo se usa el prior")

La elección clave es usar un **CVAE con *prior* aprendido** en vez de un CVAE *vanilla*. En el *vanilla* el *prior* es una gaussiana estándar fija; acá el *prior* $p_\theta(z\mid\mathbf{x})$ es una red que codifica el contexto, de modo que la distribución latente desde la que se muestrea en inferencia **ya depende del entorno**. El *posterior* $q_\phi(z\mid\mathbf{x},\mathbf{y})$, que sí ve la trayectoria real, solo se usa en entrenamiento para arrastrar al *prior* hacia la distribución correcta.

Algunos detalles de diseño que me costó bastante llegar a ellos:

**PointNet pre-entrenado.** La nube de puntos pasa por un PointNet que hace segmentación binaria (*transitable* / *no transitable*) punto a punto. Las etiquetas se generan proyectando cada punto sobre el mapa semántico global del dataset y filtrando los que superan 1.5 veces la altura del robot (para descartar techos y copas de árboles). Después esa segmentación se rasteriza a una grilla 2D de 0,2 m/celda, se suavizan las grillas positiva y negativa con un kernel gaussiano, y se obtiene un mapa de calor $\mathbf{I}_{seg}$:

$$
\mathbf{I}_{seg} = \mathcal{G}_\sigma * (M^+ - M^- + \mathbf{1})
$$

Ese mapa de calor condiciona al decoder. Es importante: sin él el modelo tiende a ignorar la geometría del entorno (el clásico *posterior collapse*).

![Proceso de generación de ground truth para la nube de puntos](sem-navae/ground-truth-pointcloud.png "Izquierda: nube voxelizada. Centro: mapa semántico local según la pose. Derecha: las etiquetas proyectadas sobre la nube 3D")

**El decoder predice velocidades, no posiciones.** Una GRU autorregresiva de $N_w$ celdas que predice $\hat{\mathbf{y}} = \{\mathbf{v}_1,\dots,\mathbf{v}_{N_w}\}$, y la trayectoria se recupera integrando:

$$
\hat{\mathbf{p}}_{t+k} = \hat{\mathbf{p}}_{t+k-1} + \mathbf{v}_k\,\Delta t
$$

Predecir en el espacio de velocidades da trayectorias cinemáticamente suaves y evita saltos espaciales físicamente irrealizables. En inferencia, además, en cada paso se muestrean $R = 1000$ velocidades y se elige la que cae dentro de zonas positivas de $\mathbf{I}_{seg}$ y cuya aceleración no supera 0,5 m/s². Es decir, la percepción restringe la generación paso a paso.

**La función de pérdida y el truco del *log-mean-exp*.** Acá está la parte que más me entretuvo. El dataset tiene **múltiples** trayectorias *ground truth* por muestra (una por cada dirección factible). Si uno usa la NLL estándar, el modelo aprende el promedio de todas — exactamente el problema que queríamos evitar. La solución es mover la sumatoria dentro del logaritmo:

$$
\log \frac{1}{M}\sum_{m=1}^{M} p_\psi(\mathbf{y}^{(m)}\mid z, \mathbf{x})
$$

Matemáticamente es análogo a los *Importance Weighted Autoencoders*, y operacionalmente actúa como un *Multiple Choice Learning* o *Winner-Takes-All* suave: el gradiente lo domina la muestra $\mathbf{y}^{(m)}$ que tiene mayor probabilidad bajo la distribución actual. Así el modelo se puede **especializar en un modo** sin ser castigado por las otras alternativas válidas.

A eso se suma una pérdida de colisión, que proyecta las predicciones al mapa semántico local y penaliza las que caen en zonas no transitables:

$$
\mathcal{L}_\text{NaVAE} = \beta D_\text{KL} + \mathcal{L}_\text{recon} + \lambda \mathcal{L}_\text{col}
$$

### Selección: el VLM

![Diagrama general del sistema de selección de trayectorias](sem-navae/overview.png "Filtrado de colisiones, mapa de costos con CLIPSeg, costo por trayectoria y actualización asíncrona")

Cuatro pasos:

**1. Filtrado de colisiones.** Se construye un mapa de elevación local (18×18 m, 0,1 m/celda) a partir del LiDAR y se descartan las trayectorias cuyo cambio de inclinación entre *waypoints* consecutivos supere $\theta_c = 14°$, evaluado sobre un cuadrado del tamaño del *footprint* del robot. Esto también ayuda en terreno con pendientes suaves, donde el mapa de elevación no refleja bien la transitabilidad.

**2. Mapa de costos con CLIPSeg.** Se le pasa la imagen FPV y un conjunto de *prompts* en lenguaje natural (`pavement`, `grass`, `tree`, `wall`, `stairs`, `person`, `sky`...). CLIPSeg devuelve un mapa de probabilidad por clase; se toma el argmax por píxel y se asigna un costo a cada clase. Las clases se agrupan en tres niveles:

- **No transitable estricto** — obstáculos infranqueables (paredes, árboles, escaleras). Costo alto.
- **Transitable suave** — físicamente navegable pero no preferido (césped, tierra suelta). Costo $c > 0$, menor que el estricto.
- **Transitable preferido** — veredas, pavimento. Costo $c = 0$.

Lo bonito es que esto es **vocabulario abierto**: cambiar la preferencia de navegación es editar una lista de strings y sus costos. No hay reentrenamiento.

**3. Costo por trayectoria.** Las trayectorias viven en el marco local del robot; se transforman al marco de la cámara y se proyectan al plano de la imagen con el modelo *pinhole*:

$$
\mathbf{y}' = K[I_3|0]\,\mathbf{y}_c
$$

![Proyección de una trayectoria al plano de la imagen](sem-nave/camera-projection.png "Del sistema del robot al plano imagen, donde vive el mapa de costos")

Y el costo es la suma descontada de los costos en cada *waypoint* proyectado:

$$
J(\mathbf{y}') = \sum_{j=1}^{N}\gamma^j\left[\mathbf{I}_c(p_x^j, p_y^j)\mathbb{1}_{u_j=0} + C_u\mathbb{1}_{u_j=1}\right]
$$

El factor $\gamma$ descuenta los *waypoints* lejanos (son costos futuros, con más incertidumbre). El término con $C_u$ maneja un problema real: al pasar de 3D a 2D hay pérdida de profundidad, y una trayectoria que rodea un árbol puede aparecer *detrás* del árbol en la imagen. A esos *waypoints* ocluidos se les asigna un costo intermedio fijo, porque honestamente no se sabe cuál es su costo real. Finalmente se suma un término $J_{goal}$ que premia acercarse al objetivo en posición y orientación.

**4. Actualización asíncrona.** Este punto me parece el más interesante de todo el módulo de selección, y es de los que más tuve que iterar.

Un planificador clásico recalcula periódicamente y ejecuta solo el primer *waypoint*. Acá eso es subóptimo: si las trayectorias tienen secciones iniciales rectas, uno termina siempre eligiendo "lo mismo" y nunca comprometiéndose con una ruta. Por otro lado, los costos calculados con oclusiones **dejan de ser válidos** a medida que uno avanza.

La solución fue desacoplar dos frecuencias:

- $f_{gen}$ (0,5 Hz): generación geométrica de nuevas trayectorias.
- $f_{clip}$ (2,5 Hz): actualización semántica.

Mientras el robot ejecuta una trayectoria, los costos de los *waypoints* ya recorridos se **congelan** y los futuros se **reproyectan** sobre cada nueva segmentación. Así el sistema refina su estimación a medida que se acerca, resolviendo oclusiones y corrigiendo errores de interpretación (una sombra que parecía un obstáculo) sin tener que regenerar geometría.

Y el cambio de trayectoria solo ocurre si la nueva candidata mejora el costo por un factor de histéresis $\varepsilon > 0$:

$$
J(\mathbf{y}_\text{new}) < J(\mathbf{y}_\text{curr}) - \varepsilon
$$

que es lo que evita que el robot oscile entre dos rutas casi equivalentes.

**Recovery behavior.** Como el sistema no tiene información global, puede quedar atrapado en mínimos locales donde ninguna trayectoria generada es factible. Para eso hay una heurística simple: buscar con *raycasting* (con búsqueda binaria, por eficiencia) en $N$ direcciones sobre el mapa de elevación, y rotar al robot hacia la dirección libre angularmente más cercana al objetivo.

![Diagrama de búsqueda de direcciones libres de colisiones](sem-navae/recovery.png "Izquierda: objetivo más lejano que el radio de búsqueda. Derecha: el caso contrario")

Es una heurística, y en el documento soy explícito en que no es suficiente en todos los casos. Pero según la Tabla de resultados se usa en casi todas las pruebas al menos una vez, así que resultó ser bastante importante para la robustez del sistema completo.

## Datos

![Mapas del dataset: imagen satelital, mapas de ocupación con SLAM y mapas semánticos resultantes](sem-navae/dataset.png "Los dos sectores de la FCFM: A (entrenamiento) y B (evaluación)")

El dataset se construyó teleoperando el Panther en la Facultad de Ciencias Físicas y Matemáticas de la Universidad de Chile: un entorno urbano no estructurado, con caminos peatonales, áreas verdes y hartos cambios de altura.

El detalle importante es **cómo se generan los múltiples *ground truth* por muestra**. Usando la funcionalidad de Rosbag se recrean las trayectorias del robot sobre el mapa semántico; con un radio de búsqueda $R_{max} = 15$ m se encuentran todos los puntos alcanzables considerando el *footprint*; se llama a NavFN para obtener los *waypoints* hacia cada uno; y se descartan las trayectorias con FDE menor a 0,5 m entre sí (para que sean genuinamente diversas).

Resultado: 15 trayectorias completas del sector A, cortadas en ventanas de 0,5 s → **1.706 muestras**, cada una con imagen FPV, nube de puntos, IMU, pose, mapa semántico local, 5 s de trayectoria pasada, 20 s de trayectoria futura y 6 opciones factibles de 20 s.

Y el punto que más me importa subrayar: **los mapas semánticos, AMCL y NavFN solo existen para el entrenamiento y la evaluación offline**. Durante la navegación el sistema no tiene acceso a nada de eso.

## Resultados

### Pre-entrenamiento y entrenamiento

PointNet converge bien: **0,811 de meanIoU**, 0,931 de accuracy por punto y 0,890 de accuracy promedio por clase, en unos 10 minutos de entrenamiento. Los errores están casi todos en los bordes entre clases, lo que es esperable dada la naturaleza ruidosa de las nubes.

NaVAE se entrena 1.000 épocas (~7 horas en una RTX 4070), con KL *annealing* para que el modelo primero aprenda a reconstruir y después regularice el espacio latente. En evaluación, el 72% de las trayectorias generadas cae en espacios transitables, y cualitativamente se ve lo que se buscaba: las predicciones **no replican una sola ruta**, sino que abren un abanico de alternativas que se curva para evitar obstáculos.

### Navegación real

![Setup experimental del Husarion Panther](sem-navae/panther-setup.png "Husarion Panther con LiDAR Ouster OS-0 de 64 canales, cámara USB, GNSS UBlox ZedF9P y Jetson Orin AGX")

![Vista satelital de los sectores de prueba](sem-navae/sectors.png "En rojo el sector de entrenamiento (A) y en azul el de evaluación (B), con las posiciones objetivo numeradas")

Se compara NaVAE contra MTG (el *baseline* CVAE principal) y MTG' (MTG reentrenado con nuestro módulo PointNet). Los tres usan **exactamente el mismo sistema de selección**, así que la comparación aísla la calidad del generador. 3 pruebas en el sector A (entrenamiento) y 5 en el B (evaluación, fuera de distribución), cada una repetida 6 veces.

| Modelo | Sector | SR ↑ | SPL ↑ | EPT% ↑ | NTR% ↓ | T<sub>ratio</sub> ↓ |
|---|---|---|---|---|---|---|
| **NaVAE** | A | **0,83** | **0,51** | **90,0** | 38,3 | **1,69** |
| **NaVAE** | B | **0,90** | **0,76** | **95,0** | 23,4 | **1,33** |
| MTG' | A | 0,50 | 0,45 | 84,2 | **35,4** | 2,21 |
| MTG' | B | 0,63 | 0,63 | 90,7 | **22,1** | 2,41 |
| MTG | A | 0,28 | 0,22 | 79,4 | 43,8 | 2,24 |
| MTG | B | 0,17 | 0,16 | 77,5 | 32,7 | 2,62 |

(SR: tasa de éxito. SPL: éxito ponderado por largo de trayectoria. EPT: % de trayectoria ejecutada en zona transitable preferida. NTR: % de trayectorias *generadas* en zonas no transitables. T<sub>ratio</sub>: tiempo de navegación / tiempo de teleoperación.)

La mejora está principalmente en tasa de éxito, SPL y tiempo: **+33%, +10% y +23%** en el sector A, y **+20%, +8% y +44%** en el sector B, comparado con el mejor *baseline*.

Hay un resultado que parece malo y no lo es: **NaVAE tiene peor NTR que MTG'**. Es decir, genera más trayectorias que caen en zonas no transitables. Pero eso es una consecuencia directa de generar *muchas* más trayectorias: el promedio baja. Lo que importa es que el EPT (lo que el robot efectivamente recorre) es el mejor de todos — el módulo de selección hace su trabajo y filtra. Mientras haya suficientes trayectorias factibles en el abanico, generar algunas inviables no cuesta nada.

![Comparación de los abanicos generados por los tres modelos](sem-navae/alldata-comparisons.png "Mapa de elevación (verde: sin colisión, azul: waypoints en colisión) y todas las trayectorias proyectadas a la imagen FPV")

Esta figura es la que mejor resume la diferencia entre los tres modelos. MTG base produce **siempre el mismo abanico**, independiente de la observación — un caso bastante claro de *posterior collapse*, agravado por su dependencia al formato específico de nube de puntos del trabajo original. MTG' tiene más diversidad y muestra multimodalidad en algunos casos, pero sus trayectorias redundantes en su mayoría no son factibles. NaVAE muestra un abanico **"modulado" por el entorno**: en sectores estrechos genera trayectorias más alineadas, en sectores abiertos abre la diversidad, sin mostrar modos separados artificiales.

### Jugar con los costos

![Trayectorias ejecutadas variando el costo de la clase grass](sem-navae/selvar-grass.png "Mismo modelo, mismo objetivo, distinto costo asignado al césped: 0, 1 y 2")

Acá está, para mí, la demostración más convincente de que la hipótesis H2 se cumple. Es el mismo sistema, el mismo modelo entrenado, el mismo objetivo. Lo único que cambia es **un número en una lista**. Y el robot cambia de ruta: con costo 0 corta por el pasto, con costo 2 da la vuelta larga por el pavimento.

![Mapas de costos al agregar la clase sand a la segmentación](sem-navae/selvar-sand.png "Agregar una clase nueva — 'sand' — y variar su costo, sin reentrenar nada")

Y en el segundo caso ni siquiera cambia un número: se **agrega una clase nueva** (`sand`, para distinguir los caminos de gravilla del pavimento) que el modelo nunca vio como categoría separada. Con el costo bajo los trata como pavimento normal; con el costo alto el robot prefiere caminos más abiertos aunque impliquen atravesar césped. Cero reentrenamiento. Eso es lo que da el vocabulario abierto.

### Cambiar el planificador local

![Trayectorias ejecutadas con dos planificadores locales distintos](sem-navae/lplanner_var.png "RL (rojo) vs DWA (azul): trayectorias similares en largo y en zonas transitadas")

Se probó el sistema completo con el planificador local basado en RL y con DWA del stack de navegación de ROS. Las trayectorias resultantes son similares en largo y en las zonas transitadas, lo que confirma que el módulo global es efectivamente agnóstico al nivel inferior — siempre que éste tenga buenas capacidades de evasión de obstáculos.

## Lo que no funciona tan bien

Algunas cosas que quedaron claramente como limitaciones:

**Movimientos oscilatorios.** En las trayectorias ejecutadas se ven zigzags en varios puntos. Son por dos razones: el momento en que se selecciona una trayectoria puede ser óptimo en costo pero no en la pose actual del robot, y el planificador local no está pensado para seguir *waypoints* perfectamente sino para llegar a ellos corrigiendo según el ambiente. NaVAE es el que menos zigzaguea de los tres, pero igual los tiene.

**Las trayectorias no son globalmente óptimas**, ni idénticas entre repeticiones de la misma prueba. Es consecuencia directa de no tener mapa: el planificador elige entre lo que generó localmente. Es un costo consciente del diseño, pero es un costo.

**Percepción local 2D.** Los dos planificadores locales probados usan percepción LiDAR bidimensional, y eso falla con escalones pequeños o terreno muy irregular. Y en esos mismos casos el filtro de colisiones también sufre. Una solución parcial sería orientar correctamente la nube de puntos usando la IMU; la buena sería tener odometría 3D robusta.

**El sistema es esencialmente reactivo.** El robot "olvida" lo que vio hace unos metros, aun usando información sensorial pasada. Eso puede hacer que las trayectorias vuelvan a un punto de inicio, o que nunca se alcance el objetivo. La solución natural sería un grafo topológico semántico ligero en el módulo de selección, con nodos asociados a características visuales clave — pero eso requiere un VLM más potente o un modelo entrenado para razonar sobre ese grafo, y complica bastante el sistema.

## Conclusiones

Las dos hipótesis se validan. El CVAE con *prior* aprendido efectivamente captura la multimodalidad y propone un abanico de rutas viables en vez de promediarlas. Y el VLM efectivamente permite discriminar entre ellas con criterios semánticos que ningún planificador geométrico puede expresar.

Más allá de las métricas, lo que creo que vale del trabajo es la **separación estricta entre generar y seleccionar**. Esa separación es la que permite las dos cosas al mismo tiempo: operación en tiempo real (porque el generador no tiene que razonar sobre semántica) y adaptabilidad *zero-shot* (porque el selector no tiene que entender geometría).

## Comentarios personales

Esta tesis me costó mucho, más de lo que esperaba cuando empecé.

La parte que más me frustró no fue la teoría sino el *gap* entre que algo funcione en evaluación offline y que funcione con el robot andando. Hay un montón de decisiones que en el paper ocupan media línea y que en la práctica fueron semanas: cuándo actualizar la trayectoria, qué hacer con las oclusiones, qué pasa cuando el robot se atora. La actualización asíncrona con histéresis nació literalmente de ver al Panther oscilando entre dos rutas casi iguales y no avanzar.

También aprendí harto sobre lo distinto que es trabajar con hardware real. Agradezco mucho a la gente del AMTC — especialmente a Michelle, que me ayudó con el Panther constantemente — y a los cabros del Laboratorio de Robótica, sobre todo a Ignacio y Mattías, con quienes discutí buena parte de esto.

El resultado directo de este trabajo fue un paper aceptado en RA-L, que cuento [acá](/projects/sem-navae). Y la recta final de la tesis coincidió con un momento personal difícil, del que salí en parte volviendo a la montaña; [esa historia la conté en otra parte](/ascents/algarrobo-traverse).
