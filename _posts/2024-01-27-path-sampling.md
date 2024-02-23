---
title: Curvature based path sampling
date: 2024-01-27 17:22:09 -03
author: Gonz
categories: [Robotics]
tags: [path, ROS]
image:
    path: /assets/img/headers/path-sampling.png
---
{% include_relative _includes/head.html %}

This short project was created by a need to sample points from a classic global planner (such as NavFn) to unstack or give more accurate aypoints to a Reactive RL local planner. This implementation also serves purpose to other local planners that don't need the entirity of a complex global plan and can perform well within a certain range.

The idea in principle is to obtain the most significant waypoints of a long path, such as the maximum curvature point or the changes in direction, given that straight line segments normally are not a challenge to a planner. However, a uniform sampling approach is also provided given that it is the most simple solution. Two more approaches are explored, based on the maximum points of curvature and the peaks of the magnitude of the gradient, which in term are quite similar, but provide different results.

Lastly a combined method can be used, in order to sample points uniformly and aggregate the points that are within a more complex segment, such as big curves.


Full code implementation and usage instructions can be found in my github profile at [gonzal0lguin/path-sampling](https://github.com/gonzal0lguin/path-sampling).

## Uniform sampling

For all the examples a detached node of navfn is used to obtain the global paths in a pre-mapped environment. The paths can be obtained using the service call to `/navfn/make_plan` as a `NavFN/MakePlan` srv.

The first approach is quite simple, the obtained path length $L_p$ is calculated by summing the euclidean distance between consecutive waypoints as follows:

$
L_p = \sum_{i=1}^{N} \lVert P_{i} - P_{i-1}\rVert
$

Where $P$ is the path containing $N$ waypoints.


When the distance $L_p$ becomes equal to the sampling distance $d_s$ (passed as a parameter), then the corresponding waypoint is saved. The process is reapeted until the end of the path, where the last point is always saved (given that is the goal).

The described algorithm is presented bellow:

```python
    def sample_plan_uniform(self, path: np.ndarray):
        L_p = 0
        waypoints = []
        for i in range(1, len(path)):
            segment_l = self.calculate_segment_lenght(
                path[i-1],
                path[i]
            ) 
            L_p += segment_l
            
            if L_p >= self._waypoint_dst:
                waypoints.append(path[i])
                L_p -= self._waypoint_dst

        if L_p < self._waypoint_dst and len(waypoints)>0:
            waypoints.pop()
        waypoints.append(path[-1])

        return waypoints
```

The image below shows an example of usage on a simulated world in Gazebo, where the full path is in green and the sampled waypoints are the red dots.

<img src="/assets/img/posts/path-sampling/uniform-example.png" alt="center" width="700"/>


## Curvature sampling

## Graditent sampling


$$
P_{mag} =  \left|\frac{dx}{dt}\right| + \left|\frac{dy}{dt}\right|
$$

<img src="/assets/img/posts/path-sampling/curve_sample.png" alt="center" width="700"/>


<img src="/assets/img/posts/path-sampling/curve-example.png" alt="center" width="700"/>

## Combined sampling