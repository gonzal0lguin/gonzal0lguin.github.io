---
title: Muestreo de trayectorias basado en curvatura
category: "academic-projects"
description: "Un proyecto de robótica."
pubDate: 2024-01-27 17:22:09 -03
author: Gonz
categories: [Robotics]
tags: [path, ROS]
heroImage: /assets/img/headers/path-sampling.png
---


Este pequeño proyecto nació de la necesidad de muestrear puntos de un planificador global clásico (como NavFn) para destrabar o entregar waypoints más precisos a un planificador local reactivo basado en RL. Esta implementación también sirve para otros planificadores locales que no necesitan la totalidad de un plan global complejo y que funcionan bien dentro de cierto rango.

En principio, la idea es obtener los waypoints más significativos de una trayectoria larga, como los puntos de máxima curvatura o los cambios de dirección, dado que los segmentos rectos normalmente no son un desafío para un planificador. Aun así, también se incluye un muestreo uniforme, ya que es la solución más simple. Se exploran otros dos enfoques, basados en los puntos de máxima curvatura y en los peaks de la magnitud del gradiente, que en la práctica son bastante similares, pero entregan resultados distintos.

Por último, se puede usar un método combinado, que muestrea puntos de forma uniforme y agrega los puntos que están dentro de un segmento más complejo, como las curvas pronunciadas.


La implementación completa y las instrucciones de uso están en mi perfil de GitHub: [gonzal0lguin/path-sampling](https://github.com/gonzal0lguin/path-sampling).

## Muestreo uniforme

En todos los ejemplos se usa un nodo independiente de navfn para obtener las trayectorias globales en un entorno previamente mapeado. Las trayectorias se obtienen llamando al servicio `/navfn/make_plan`, de tipo `MakeNavPlan`.

El primer enfoque es bastante simple: el largo de la trayectoria $L_p$ se calcula sumando la distancia euclidiana entre waypoints consecutivos, de la siguiente forma:

$$
L_p = \sum_{i=1}^{N} \lVert P_{i} - P_{i-1}\rVert
$$

Donde $P$ es la trayectoria que contiene $N$ waypoints.


Cuando la distancia $L_p$ llega a la distancia de muestreo $d_s$ (entregada como parámetro), se guarda el waypoint correspondiente. El proceso se repite hasta el final de la trayectoria, donde el último punto siempre se guarda (ya que es la meta).

El algoritmo descrito se presenta a continuación:

```python
from scipy.signal import savgoal_filter, find_peaks
...
def sample_plan_uniform(self, path: np.ndarray):
    L_p = 0
    waypoints = []
    for i in range(1, len(path)):
        segment_l = self.calculate_segment_lenght(
            path[i-1],
            path[i]
        ) 
        L_p += segment_l
        
        if L_p >= d_s:
            waypoints.append(path[i])
            L_p -= d_s

    if L_p < d_s and len(waypoints)>0:
        waypoints.pop()
    waypoints.append(path[-1])

    return waypoints
```

La imagen de abajo muestra un ejemplo de uso en un mundo simulado en Gazebo, donde la trayectoria completa está en verde y los waypoints muestreados son los puntos rojos.

<img src="/assets/img/posts/path-sampling/uniform-example.png" alt="center" width="700"/>


## Muestreo por curvatura

La curvatura se calcula con la siguiente ecuación, en términos de la representación paramétrica $(x(t), y(t))$.

$$
\kappa = \frac{\left\lvert \frac{dx}{dt}\left(\frac{d^2y}{dt^2}\right)^2-\frac{dy}{dt}\left(\frac{d^2x}{dt^2}\right)^2\right\rvert}{\left(\sqrt{\left(\frac{d^2x}{dt^2}\right)^2 + \left(\frac{d^2y}{dt^2}\right)^2}\right)^3}
$$

Como solo tenemos un conjunto de puntos $(x, y)$, las derivadas se calculan con `numpy.gradient`.

Al hacer pruebas, noté que las trayectorias tienen muchos segmentos no suaves al hacer zoom, lo que significa que el cálculo de la curvatura tiene mucho ripple. Para remediarlo, primero aplico un [filtro de Savitzky-Golay](https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.savgol_filter.html) a la trayectoria de entrada, lo que mejora los resultados.

Para detectar los peaks se usa [find peaks](https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.find_peaks.html) o [argrelextrema](https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.argrelextrema.html). Tras experimentar, `find_peaks` tuvo un mejor desempeño gracias a su mayor capacidad de ajuste, aunque eso también implica más parámetros que sintonizar.

## Muestreo por gradiente

El método anterior, incluso con la entrada filtrada, no siempre funciona como se espera. Este método usa la norma $\mathcal{L_1}$ del gradiente para encontrar los peaks, ya que la primera derivada tiene menos ruido que la segunda derivada usada para obtener $\kappa$.

La norma $\mathcal{L_1}$ se puede calcular como:

$$
\lVert \nabla P\rVert_{\mathcal{L_1}} =  \left|\frac{dx}{dt}\right| + \left|\frac{dy}{dt}\right|
$$

El siguiente fragmento muestra el proceso para obtener la norma y muestrear los puntos de interés. Nótese que igual uso el filtro de Savitzky-Golay sobre la trayectoria de entrada.

```python
from scipy.signal import savgoal_filter, find_peaks
...
def sample_curve_from_gradient(self, path, th):

    x_smooth = savgol_filter(path[:, 0], 51, 3)
    y_smooth = savgol_filter(path[:, 1], 51, 3)

    # Calculate the gradient in x and y directions
    dx = np.gradient(x_smooth)
    dy = np.gradient(y_smooth)

    # Calculate the L1 norm of the gradient
    gradient_magnitude = np.abs(dx) + np.abs(dy)
    # shift curve to positive y-axis for find_peaks
    gradient_magnitude -= np.min(gradient_magnitude)

    peaks, _ = find_peaks(gradient_magnitude, height, distance)

    return path[peaks]
```

A continuación se presenta una trayectoria de ejemplo con su norma $\mathcal{L_1}$ correspondiente. Los puntos rojos representan los puntos muestreados.

<img src="/assets/img/posts/path-sampling/curve_sample.png" alt="center" width="700"/>


La siguiente imagen muestra un ejemplo en Gazebo, donde la línea verde es la trayectoria original y los puntos rojos son los puntos muestreados.

<img src="/assets/img/posts/path-sampling/curve-example.png" alt="center" width="700"/>

## Muestreo combinado

Por definir... :)
