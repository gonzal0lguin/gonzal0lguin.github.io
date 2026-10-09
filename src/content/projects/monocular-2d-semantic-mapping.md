---
title: Mapeo semántico 2D monocular
category: "academic-projects"
description: "Un proyecto de robótica."
pubDate: 2023-12-26 17:22:09 -03
author: Gonz
categories: [Robotics, Deep Learning]
tags: [ROS, mapping]
heroImage: /assets/img/headers/2dmapping.png
---


# Mapeo semántico 2D monocular

La idea de este proyecto es usar una cámara RGB monocular a bordo de una plataforma móvil para crear mapas de ocupación 2D (occupancy grids) y mapas semánticos, usando solo la odometría como fuente sensorial adicional.

Como se explica más abajo, el proceso consiste en obtener primero una imagen segmentada, luego aplicar una transformación a vista de pájaro (Bird's eye) y una conversión adecuada a grilla de ocupación o a un mapa local segmentado. Finalmente, usando la odometría, los mapas locales se concatenan para crear un mapa global.

La implementación completa y las instrucciones de uso están en mi perfil de GitHub: [gonzal0lguin/monocular-2d-mapping](https://github.com/gonzal0lguin/monocular-2d-mapping).

----

Este es el proyecto final del curso *Procesamiento Avanzado de Imágenes* del Departamento de Ingeniería Eléctrica de la Universidad de Chile.


> Dado el poco tiempo disponible para el proyecto, se hicieron varias simplificaciones para cumplir los objetivos. Estas son:
>- El proyecto es 100\% simulado en Gazebo y se basa en ROS Noetic, usando la simulación del robot [Panther](https://husarion.com/manuals/panther/) que provee Husarion.
>- No se usa ningún algoritmo de SLAM para corregir la deriva de la odometría, ni feature-matching o cierre de loops. Solo se usa la odometría para crear los mapas.
>- La segmentación la realiza una red entrenada a medida (más detalles abajo). Para obtener rápidamente pares de imágenes de entrenamiento, se usa una copia exacta de los mundos de simulación, pero que contiene solo colores planos para las clases objetivo.
 


## Mundos personalizados

Se decide crear tres entornos con objetos comunes de una ciudad, como casas, semáforos, señalética, conos, autos, árboles, etc. Para simplificar el entrenamiento, se crea una copia exacta de los mundos y se modifican los archivos .world para que a cada objeto se le asigne el color de una clase específica. Con esto se tienen las versiones originales de los mundos y una versión "segmentada", lo que agiliza la generación del dataset. La siguiente tabla presenta los colores usados para las clases elegidas, que suman 6 + 1.

<center>
<style type="text/css">
.tg  {border-collapse:collapse;border-spacing:0;}
.tg td{border-color:black;border-style:solid;border-width:1px;font-family:Arial, sans-serif;font-size:14px;
  overflow:hidden;padding:10px 5px;word-break:normal;}
.tg th{border-color:black;border-style:solid;border-width:1px;font-family:Arial, sans-serif;font-size:14px;
  font-weight:normal;overflow:hidden;padding:10px 5px;word-break:normal;}
.tg .tg-46ru{background-color:#96fffb;border-color:inherit;text-align:left;vertical-align:top}
.tg .tg-nto1{background-color:#000000;border-color:inherit;text-align:left;vertical-align:top}
.tg .tg-yj5y{background-color:#efefef;border-color:inherit;text-align:center;vertical-align:top}
.tg .tg-h8ck{background-color:#3fff00;border-color:inherit;text-align:left;vertical-align:top}
.tg .tg-zqsx{background-color:#ff00ff;border-color:inherit;text-align:left;vertical-align:top}
.tg .tg-c3ow{border-color:inherit;text-align:center;vertical-align:top}
.tg .tg-95rq{background-color:#0500ff;border-color:inherit;text-align:left;vertical-align:top}
.tg .tg-y698{background-color:#efefef;border-color:inherit;text-align:left;vertical-align:top}
.tg .tg-0pky{border-color:inherit;text-align:left;vertical-align:top}
.tg .tg-pyoy{background-color:#7f7f7f;border-color:inherit;text-align:left;vertical-align:top}
.tg .tg-lydr{background-color:#ff0000;border-color:inherit;text-align:left;vertical-align:top}
</style>
<table class="tg">
<thead>
  <tr>
    <th class="tg-yj5y">Clase</th>
    <th class="tg-yj5y">ID</th>
    <th class="tg-y698">RGB</th>
  </tr>
</thead>
<tbody>
  <tr>
    <td class="tg-0pky">Desconocido</td>
    <td class="tg-c3ow">0</td>
    <td class="tg-nto1"><span style="color:#FFF">Negro (0, 0, 0)</span></td>
  </tr>
  <tr>
    <td class="tg-0pky">Vehículos</td>
    <td class="tg-c3ow">1</td>
    <td class="tg-95rq">Azul (0, 0, 255)</td>
  </tr>
  <tr>
    <td class="tg-0pky">Calle/suelo</td>
    <td class="tg-c3ow">2</td>
    <td class="tg-h8ck">Verde (0, 255, 0)</td>
  </tr>
  <tr>
    <td class="tg-0pky">Edificios</td>
    <td class="tg-c3ow">3</td>
    <td class="tg-46ru">Cian (0, 255, 255)</td>
  </tr>
  <tr>
    <td class="tg-0pky">Cielo</td>
    <td class="tg-c3ow">4</td>
    <td class="tg-pyoy">Gris (127, 127, 127)</td>
  </tr>
  <tr>
    <td class="tg-0pky">Obstáculos</td>
    <td class="tg-c3ow">5</td>
    <td class="tg-lydr">Rojo (255, 0, 0)</td>
  </tr>
  <tr>
    <td class="tg-0pky">Personas</td>
    <td class="tg-c3ow">6</td>
    <td class="tg-zqsx">Magenta (255, 0, 255)</td>
  </tr>
</tbody>
</table>
</center>


Es importante mencionar que la clase 0 (desconocido) no existe directamente en los mundos, pero se considera para las transformaciones que se explican más adelante.

La siguiente figura muestra los tres mundos creados en Gazebo: los dos primeros, `circuit` y `small_city`, se usan para entrenamiento, y el mundo `test_city` contiene edificios y objetos que no están en los otros.

Circuit            |      Small city        |  Test city      
:-------------------------:|:-------------------------:|:-------------------------:
![](/assets/img/posts/2d-mapping/circuit.png)  |  ![](/assets/img/posts/2d-mapping/small_city.png) | ![](/assets/img/posts/2d-mapping/test_city.png)


### Obtención del dataset

Generar el dataset involucra tres pasos: primero, hay que obtener posiciones dentro de los mundos donde el robot podría estar durante su operación. Luego, hay que capturar imágenes en esas posiciones tanto en el mundo normal como en el segmentado. Finalmente, las imágenes del mundo segmentado se post-procesan para "aplanar" los colores; es decir, los colores con variaciones por las sombras del simulador se convierten en colores uniformes (rojo puro, azul puro, etc.).

Para el primer paso se crea un nodo de Python que se suscribe a la pose real (ground truth) del robot que reporta el plugin p3d de Gazebo y las guarda como un arreglo de numpy. El siguiente código muestra un fragmento de este proceso.

```python
def write_poses_to_file(self, msg):
        path = rospkg.RosPack().get_path('gazebo_sim')
        if not os.path.exists(os.path.join(path, 'trayectories')):
            os.makedirs(os.path.join(path, 'trayectories'))

        arr = np.asarray(self.poses_saved)
        if self.shuffle: np.random.shuffle(arr)

        np.save(os.path.join(path, f'trayectories/{self.poses_filename}.npy'), arr)
        
        rospy.loginfo(f'saved array with {len(arr)} poses.')
```


El siguiente paso, reproducir las poses, se hace con el servicio de Gazebo `/gazebo/set_model_state`, con el que se puede ubicar al robot dentro del mundo en una pose específica. Además, con OpenCV se guardan las imágenes de la cámara en la pose comandada. En resumen, el proceso para obtener las imágenes es:
1. Cargar una pose guardada.
2. Mover el robot a esa pose usando `SetModelState`.
3. Guardar la imagen de la cámara RGB en el directorio correspondiente.

Estos pasos se repiten en el mundo normal y en el segmentado, en las mismas poses, para obtener una imagen de entrada y su etiqueta de entrenamiento.

Finalmente, las imágenes del mundo segmentado pasan por un filtro que convierte las clases en colores uniformes. El código de abajo muestra, de forma resumida, el funcionamiento de la función. Se generan máscaras para cada color; por ejemplo, para el rojo: si un píxel tiene una intensidad en el canal rojo al menos 1,5 veces mayor que en el resto de los canales, se le asigna el valor máximo de rojo (255, 0, 0). Esto se repite para todos los píxeles y considerando los 6 colores posibles.

Si un píxel no cumple ninguna de las condiciones y queda en negro (valor inicial 0), se usa la función `nearest_non_black_color`, que le asigna el color predominante en una vecindad de 10 píxeles de radio.

```python
def flatten_colors_simple(image):
    # Split the image into color channels (R, G, B)
    red, green, blue = cv.split(image)

    # Create masks based on color conditions
    red_mask = (red > 1.5 * blue) & (red > 1.5 * green)
    ...
    segmented_image = np.zeros_like(image)

    # Assign colors to the segmented regions
    segmented_image[red_mask] = [0, 0, 255]  # Red
    ...

    black_pixels = np.all(segmented_image == [0, 0, 0], axis=-1)
    for i in range(segmented_image.shape[0]):
        for j in range(segmented_image.shape[1]):
            if black_pixels[i, j]:
                segmented_image[i, j] = nearest_non_black_color(segmented_image, i, j)

    return segmented_image
```


Con lo anterior se obtiene un dataset de 4192 imágenes con sus respectivas máscaras. En la siguiente figura se ve un ejemplo con la imagen de entrada en el mundo normal y en el segmentado, junto con la máscara de entrenamiento.

Original            |      Segmentada        |  Aplanada     
:-------------------------:|:-------------------------:|:-------------------------:
![](/assets/img/posts/2d-mapping/img_00010.png)  |  ![](/assets/img/posts/2d-mapping/img_00010_seg.png) | ![](/assets/img/posts/2d-mapping/img_00010_flat.png)


## U-Net

Se elige el modelo de segmentación U-Net por haberlo trabajado previamente en el curso. El repositorio [Pytorch-Unet](https://github.com/milesial/Pytorch-UNet) sirve de base, y se crea un dataset específico para esta implementación. Además, se agregan funciones para la inferencia durante la evaluación y para calcular la Intersección sobre Unión (IoU). El código (3) muestra una de las funciones para calcular el IoU y evaluar el desempeño de la red.

La siguiente tabla presenta los resultados de entrenar U-Net con el dataset de 4192 imágenes, con un 20% de validación, 200 épocas, un batch size de 16 y una tasa de aprendizaje de 1e-6. El entrenamiento duró 12 horas en una GPU RTX3060.

<style type="text/css">
.tg  {border-collapse:collapse;border-color:#ccc;border-spacing:0;}
.tg td{background-color:#fff;border-color:#ccc;border-style:solid;border-width:1px;color:#333;
  font-family:Arial, sans-serif;font-size:14px;overflow:hidden;padding:10px 5px;word-break:normal;}
.tg th{background-color:#f0f0f0;border-color:#ccc;border-style:solid;border-width:1px;color:#333;
  font-family:Arial, sans-serif;font-size:14px;font-weight:normal;overflow:hidden;padding:10px 5px;word-break:normal;}
.tg .tg-baqh{text-align:center;vertical-align:top}
.tg .tg-nrix{text-align:center;vertical-align:middle}
.tg .tg-dzk6{background-color:#f9f9f9;text-align:center;vertical-align:top}
.tg .tg-57iy{background-color:#f9f9f9;text-align:center;vertical-align:middle}
</style>
<table class="tg">
<thead>
  <tr>
    <th class="tg-nrix">Conjunto</th>
    <th class="tg-nrix">mIoU</th>
    <th class="tg-nrix">IoU 1</th>
    <th class="tg-nrix">IoU 2</th>
    <th class="tg-nrix">IoU 3</th>
    <th class="tg-nrix">IoU 4</th>
    <th class="tg-nrix">IoU 5</th>
    <th class="tg-nrix">IoU 6</th>
    <th class="tg-nrix">fps RTX3060<br>@640x480</th>
    <th class="tg-nrix">fps RTX3060<br>@320x240</th>
  </tr>
</thead>
<tbody>
  <tr>
    <td class="tg-baqh">Entrenamiento</td>
    <td class="tg-dzk6">0.796</td>
    <td class="tg-baqh">0.907</td>
    <td class="tg-dzk6">0.976</td>
    <td class="tg-baqh">0.969</td>
    <td class="tg-dzk6">0.973</td>
    <td class="tg-baqh">0.875</td>
    <td class="tg-dzk6">0.074</td>
    <td class="tg-nrix" rowspan="2">12.5<br></td>
    <td class="tg-57iy" rowspan="2">30</td>
  </tr>
  <tr>
    <td class="tg-baqh">Validación</td>
    <td class="tg-dzk6">0.797</td>
    <td class="tg-baqh"><span style="font-weight:400;font-style:normal">0.909</span></td>
    <td class="tg-dzk6"><span style="font-weight:400;font-style:normal">0.978</span></td>
    <td class="tg-baqh"><span style="font-weight:400;font-style:normal">0.974</span></td>
    <td class="tg-dzk6">0.978</td>
    <td class="tg-baqh"><span style="font-weight:400;font-style:normal">0.890</span></td>
    <td class="tg-dzk6"><span style="font-weight:400;font-style:normal">0.054</span></td>
  </tr>
</tbody>
</table>

Como se ve, se logran excelentes resultados para los obstáculos principales y el suelo, con un IoU sobre el 89%. Sin embargo, para la clase 6 (personas) se obtiene un IoU bajo, de menos del 8%, lo que se atribuye a la poca presencia de personas en las imágenes de entrenamiento. Si bien detectar personas es crucial durante la navegación, en la etapa de mapeo puede no ser tan relevante, ya que no son "obstáculos" estáticos.

## Transformación a vista de pájaro

La parte fundamental de la generación de mapas es realizar una transformación de perspectiva desde la cámara a una vista superior: la llamada transformación a vista de pájaro (bird's eye). Es importante mencionar que el correcto funcionamiento del cambio de perspectiva depende del supuesto de que el suelo es plano y no tiene variaciones considerables. Esto es razonable, ya que en el mundo simulado no hay desniveles ni cambios de altura.

Para obtener la matriz de transformación, se ubica un cuadrado de $1[m^2]$ en el suelo, 4 metros frente al robot, como muestra la figura de abajo. Con un detector de Harris de OpenCV se obtienen las coordenadas de las 4 esquinas, con las que se calcula la transformación mediante getPerspectiveTransform. Luego, el resultado se aplica a la imagen original, cambiando sus dimensiones de (640x480) a (480x480). La figura de la izquierda muestra la imagen con la perspectiva transformada.

Imagen de calibración            |      Perspectiva transformada        |
:-------------------------:|:-------------------------:|
![](/assets/img/posts/2d-mapping/perspective_calibration.png)  |  ![](/assets/img/posts/2d-mapping/warpedcalibration.png) 

Usando el tamaño conocido del cuadrado de calibración y las nuevas dimensiones de la imagen, se calcula una resolución de 80 píxeles por metro. Esto indica que el campo de visión de la cámara tras la transformación es de $6[m^2]$ a una distancia de $3.25[m]$ del centro del robot (base_link). La siguiente figura presenta la calibración, incluyendo el origen del robot y las dimensiones reales.

|           Perspectiva desde el `base_link`            |
:-------------------------:|
| ![](/assets/img/posts/2d-mapping/calaxes.png) |

## Mapeo con grillas de ocupación

### Mapeo local

BEV            |      Grilla de ocupación        |  Grilla de ocupación prob.     
:-------------------------:|:-------------------------:|:-------------------------:
![](/assets/img/posts/2d-mapping/bevocc1.png)  |  ![](/assets/img/posts/2d-mapping/bevocc2.png) | ![](/assets/img/posts/2d-mapping/bevocc3.png)

### Mapeo global

Esta parte consiste en generar un mapa global a partir de los mapas locales obtenidos en los pasos anteriores. Para esto, se parte del supuesto de la figura de abajo, donde hay un mapa global con un sistema de referencia $(X_G, Y_G)$. El robot se ubica en este mapa en una posición $(X_R, Y_R)$, discretizada según la resolución del mapa, y con una orientación θ. El mapa local está a una distancia $dx = 3.25[m]$ en la dirección $x$ y tiene dimensiones $(W, L) = (480, 480)$, con un sistema de referencia $(X_L, Y_L)$. Dada una celda $(x_l, y_l)$ del mapa local, se quiere conocer sus coordenadas $P = (x_g, y_g)$ en el mapa global.


Mapa local $(X_L, Y_L)$ dentro de un mapa global $(X_G, Y_G)$          |
:-------------------------:|
![](/assets/img/posts/2d-mapping/algorithm.png)  | 

A partir de la geometría del problema se derivan las ecuaciones (1) a (6), que permiten determinar las coordenadas de una celda del mapa local en el mapa global, considerando la pose del robot.

$$
\begin{align}
&R = \sqrt{(x_l -W/2)^2+(dx+L-y_l)^2}\\
&\phi = \arctan(dx+L-y_l, x_l-W/2)\\
&r_x = [R\cos(\theta-\phi)]\\
&r_y = [R\sin(\theta-\phi)]\\
&x_g = X_R + r_x\\
&y_g = Y_R - r_y\\
\end{align}
$$

El mapa de tipo Occupancy, durante su actualización, considera tres condiciones: si recibe un valor de -1, se ignora, ya que representa espacio sin información desde la transformación a vista de pájaro (BEV). Si recibe un valor de 0 (espacio libre), sobrescribe el valor anterior de la celda. Por último, los valores se recortan para que se mantengan entre 0 y 100, representando probabilidades de ocupación.

Como cada celda contiene probabilidades, durante la actualización se suman los valores anteriores con los actuales. Esto se debe a que observar una celda varias veces aumenta la certeza sobre su valor (por eso el valor se reasigna a 0 si se observa libre). Parte del algoritmo de actualización usado para los mapas de ocupación se muestra en el siguiente fragmento; la implementación completa está en [mapper.cpp](https://github.com/gonzal0lguin/monocular-2d-mapping/blob/main/ros/dev_ws/src/mono_perception/src/mapper.cpp).

```c++
void updateMap(std::vector<int8_t> &map, const std::vector<int8_t> &sensor_data, const std::vector<int> &position, double yaw)
{
    ...
    for (int x_ = 0; x_ < Ll; ++x_)
    {
        for (int y_ = 0; y_ < Hl; ++y_)
        {
            if (sensor_data[y_ * Hl + x_] == -1)
                continue;

            double R = sqrt(pow(Hl - y_ + dy, 2) + pow(x_ - dx, 2));
            double phi = atan2(x_ - dx, Hl - y_ + dy);

            int rx = static_cast<int>(R * cos(yawR - phi)) - map_size / 2;
            int ry = static_cast<int>(R * sin(yawR - phi)) - map_size / 2;

            int i = xR + rx;
            int j = yR - ry;

            if (sensor_data[y_ * Hl + x_] == 0)
            {
                map[j * map_size + i] = 0;
            }
            else
            {
                map[j * map_size + i] += static_cast<int>(sensor_data[y_ * Hl + x_]);
            }

            // clamp values to range 0, 100 (at this point we ignored unkown space "-1")
            map[j * map_size + i] = std::max(std::min(std::abs(map[j * map_size + i]), 100), 0);
        }
    }
}
```

Para los mapas semánticos se aplica la misma lógica, pero en este caso los valores de los píxeles no se suman, porque hay que mantener la clase observada en el mapa. En el siguiente fragmento solo se presenta la actualización relevante para este caso; el resto se mantiene igual.


```c++
...
for (int x_ = 0; x_ < Ll; ++x_)
{
  for (int y_ = 0; y_ < Hl; ++y_)
  {
    if (sensor_data[y_ * Hl + x_] == 0)
      continue;
    ...
    map[j * map_size + i] = static_cast<int>(sensor_data[y_ * Hl + x_] * 20);
    map[j * map_size + i] = std::max(std::min(std::abs(map[j * map_size + i]), 127), 0);
  }
}
```

## Resultados

En la figura de abajo se muestran los resultados de segmentación para algunas imágenes del conjunto de validación. El set de imágenes incluye la máscara de entrenamiento, la máscara predicha y la diferencia entre ambas. Como se esperaba, los resultados muestran una predicción correcta de la clase sobre el 90%, salvo por la persona, que se clasifica erróneamente como vehículo.

Predicciones de ejemplo y superposición en el conjunto de validación.          |
:-------------------------:|
![](/assets/img/posts/2d-mapping/unetval.png)  |


En la figura de abajo (izquierda) se muestra un mapa local obtenido en uno de los mundos de prueba. Las figuras de la derecha y del centro muestran las perspectivas desde Gazebo y RViz, respectivamente. En la imagen del centro también se destacan en rojo las lecturas del LiDAR. Como se ve, hay correspondencia entre las lecturas y el sector ocupado del mapa local, generado a partir de la imagen segmentada, lo que indica un funcionamiento y una alineación correctos.

Es importante notar que, al guardar los mapas, los valores de los píxeles se asignan según un umbral de ocupado/no ocupado, por lo que en la figura de la izquierda los píxeles más lejanos aparecen como libres, ya que tienen un valor bajo el umbral.

Gazebo            |      RViz (con escaneo láser)        |       Grilla de ocupación
:-------------------------:|:-------------------------:|:-------------------------:
![](/assets/img/posts/2d-mapping/local_grid_gz.png)  |  ![](/assets/img/posts/2d-mapping/local_grid_viz.png) | ![](/assets/img/posts/2d-mapping/local_grid.png)

---

Las siguientes imágenes muestran los resultados del mapeo con grillas de ocupación en los 3 mundos simulados. A la izquierda hay una vista superior del mundo en Gazebo y a la derecha, el mapa de ocupación.

Vista en Gazebo            |      Mapa de ocupación        |
:-------------------------:|:-------------------------:|
![](/assets/img/posts/2d-mapping/circuit-gz.png)  |  ![](/assets/img/posts/2d-mapping/circuit-occ.png)
![](/assets/img/posts/2d-mapping/small_city_gz.png)  |  ![](/assets/img/posts/2d-mapping/small_city_occ.png) 
![](/assets/img/posts/2d-mapping/test_city_gz.png)  |  ![](/assets/img/posts/2d-mapping/test_city_occ.png) 

Finalmente, las siguientes imágenes muestran los mapas semánticos obtenidos en los 3 mundos.

Circuit            |      Small city        |       Test city
:-------------------------:|:-------------------------:|:-------------------------:
![](/assets/img/posts/2d-mapping/circuit_sem.png)  |  ![](/assets/img/posts/2d-mapping/small_city_sem.png) | ![](/assets/img/posts/2d-mapping/test_city_sem.png)

Como se ve, en ambos casos los mapas conservan la forma general de los mundos, especialmente en el primero (circuit), que es el más simple y solo presenta las clases suelo y obstáculo. En general, los mapas logran representar los mundos desde una perspectiva "desde arriba".

Sin embargo, las clases de vehículos y obstáculos en los mundos small_city y test_city no logran representaciones coherentes de sus formas y tamaños desde una perspectiva vertical. Esto se nota especialmente en los mapas semánticos de las figuras anteriores, donde dichas clases aparecen en azul y rojo, respectivamente. Sobre todo en los autos, las formas se ven distorsionadas y más grandes que su tamaño normal. Esto se debe a dos razones: primero, la geometría de los vehículos varía en curvas, altura, etc., lo que hace que las distorsiones se exageren y difieran al verlos desde distintos ángulos. En el mapa semántico de `small_city` se acentúan las ruedas delanteras de los autos, y además se ve la micro de lado en la esquina inferior izquierda. La segunda razón tiene que ver con la forma en que se actualiza el mapa: como se explicó, se sobrescribe todo el tiempo, por lo que ver un objeto como un árbol o un auto desde dos posiciones distintas cambia por completo el resultado del mapa.

Como resultado, las geometrías planas y uniformes, como los edificios, se mapean mejor, ya que mantienen su forma al verse desde distintas posiciones. Los resultados obtenidos son lo suficientemente buenos para diferenciar entre espacio libre y ocupado, lo que permite un entendimiento amplio del entorno.

## Comentarios y trabajo futuro

Uno de los objetivos del proyecto era probar modelos distintos a U-Net en caso de que no funcionara de forma óptima. En particular, se intentó entrenar PSP-Net y E-Net; sin embargo, en ambos casos hubo problemas con CUDA o con la memoria de la GPU que impidieron entrenar estos modelos. También se intentó con YOLO-V8, pero su formato de máscaras de entrenamiento es texto (json), lo que habría requerido convertir todas las máscaras.

Hacia el final del proyecto se comenzó a trabajar con IC-Net, que tiene un modelo pre-entrenado en el dataset CityScapes. A futuro, se planea re-entrenar usando esta arquitectura, incluyendo el modelo pre-entrenado, para lograr una mejor segmentación con mayor velocidad y mayor resolución de imagen.

Por otro lado, un problema en la generación de mapas es la distancia entre el robot y la imagen, que es de $3.25[m]$. Esto es un problema porque crea un punto ciego justo delante del robot, lo que hace que la actualización del mapa sea menos precisa, ya que no puede cubrir puntos a distancias más cortas, como sí lo logra un LiDAR, por ejemplo. Esto también es riesgoso en un entorno de navegación dinámico, porque si aparece un objeto en el punto ciego, no habrá capacidad de reacción sin usar otros sensores. Una forma de abordar esto es usar una cámara con un campo de visión más amplio, como un lente ojo de pez, lo que implicaría re-generar el dataset.
