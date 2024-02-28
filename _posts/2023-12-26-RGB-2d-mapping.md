---
title: Monocular 2D semantic mapping
date: 2023-12-26 17:22:09 -03
author: Gonz
categories: [Robotics, Deep Learning]
tags: [ROS, mapping]
image:
    path: /assets/img/headers/2dmapping.png
---
{% include_relative _includes/head.html %}

# Monocular 2d semantic mapping

The idea of this project is to use a 2D monocular RGB camera onboard a mobile platform and be able to create 2D occupancy grid maps and semantic maps using only odometry as a sensor source. 

As it will be explained below, the process will be to first obtain a segmented image, the perform a Bird's eye transform and an appropiate conversion to occupancy grid or a segmented local map. Finally, using odometry the local maps will be concatenated to create a global map.

Full code implementation and usage instructions can be found in my github profile at [gonzal0lguin/monocular-2d-mapping](https://github.com/gonzal0lguin/monocular-2d-mapping).

----

This is the final evaluation of the course *Advanced Image Procesing* of the Electrical Engineering Department of University of Chile.


> Given the limited timespan of the project, several simplifications were made to accomplish the objectives. These are the following:
>- The project will be 100\% simulated in Gazebo and be based in ROS Noetic, using the provided simulation of the [Panther](https://husarion.com/manuals/panther/) robot by Husarion. 
>- No SLAM algorithm will be used to correct for odometry drift and feature-matching or map closure. Only odometry will be used to create the maps.
>- Segmentation will be performed by a custom-trained network (more below). In order to rapidly obtain training image pairs, an exact copy of the simulation worlds will be used, but containing only plain colors of the target classes.
>- no se que mas for now


## Custom worlds

Circuit            |      Small city        |  Test city      
:-------------------------:|:-------------------------:|:-------------------------:
![](/assets/img/posts/2d-mapping/circuit.png)  |  ![](/assets/img/posts/2d-mapping/small_city.png) | ![](/assets/img/posts/2d-mapping/test_city.png)


### Gazebo

### Obtaining the dataset


Original            |      Segmented        |  Flattened     
:-------------------------:|:-------------------------:|:-------------------------:
![](/assets/img/posts/2d-mapping/img_00010.png)  |  ![](/assets/img/posts/2d-mapping/img_00010_seg.png) | ![](/assets/img/posts/2d-mapping/img_00010_flat.png)


## U-Net


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
    <th class="tg-nrix">Set</th>
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
    <td class="tg-baqh">Train</td>
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
    <td class="tg-baqh">Val</td>
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

## Bird's eye transform

The fundamental part of map generation involves performing a perspective transformation from the camera to a top-down view; the so-called bird's eye transformation. It is important to mention that the correct functioning of the perspective shift is subject to the assumption that the floor is flat and does not have considerable variations. This is reasonable since in the simulated world there are no bumps or changes in height.

To obtain the transformation matrix, a square of $1[m^2]$ is placed on the ground 4 meters in front of the robot, as shown in Figure (4a). Using an OpenCV Harris detector, the coordinates of the 4 corners are obtained, which can be used to obtain the transformation through getPerspectiveTransform. Then, the result is applied to the original image, changing the dimensions from (640x480) to (480x480). Figure (4b) shows the image with the transformed perspective.

Calibration image            |      Transformed perspective        |
:-------------------------:|:-------------------------:|
![](/assets/img/posts/2d-mapping/perspective_calibration.png)  |  ![](/assets/img/posts/2d-mapping/warpedcalibration.png) 

Using the known size of the calibration square and the new dimensions of the image, a resolution of 80 pixels per meter is calculated. This indicates that the field of view of the camera after the transformation is from $6[m^2]$ at a distance of $3.25[m]$ from the center of the robot (base_link). Figure (5) presents the calibration, including the origin of the robot and the actual dimensions.

|           Perspective from the `base_link`            |
:-------------------------:|
| ![](/assets/img/posts/2d-mapping/calaxes.png) |

## Occupancy grid mapping

### "Local Mapping"

BEV            |      Occ grid        |  Prob. Occ grid     
:-------------------------:|:-------------------------:|:-------------------------:
![](/assets/img/posts/2d-mapping/bevocc1.png)  |  ![](/assets/img/posts/2d-mapping/bevocc2.png) | ![](/assets/img/posts/2d-mapping/bevocc3.png)

### "Global Mapping"

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

Local Map $(X_L, Y_L)$ within a global map $(X_G, Y_G)$          |
:-------------------------:|
![](/assets/img/posts/2d-mapping/algorithm.png)  | 

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

## Semantic mapping

## Results

Sample predictions and overlap in the validation set.          |
:-------------------------:|
![](/assets/img/posts/2d-mapping/unetval.png)  |



Gazebo            |      RViz (with laser scan)        |       Occ grid
:-------------------------:|:-------------------------:|:-------------------------:
![](/assets/img/posts/2d-mapping/local_grid_gz.png)  |  ![](/assets/img/posts/2d-mapping/local_grid_viz.png) | ![](/assets/img/posts/2d-mapping/local_grid.png)


Gazebo View            |      Occupancy map        |
:-------------------------:|:-------------------------:|
![](/assets/img/posts/2d-mapping/circuit-gz.png)  |  ![](/assets/img/posts/2d-mapping/circuit-occ.png)
![](/assets/img/posts/2d-mapping/small_city_gz.png)  |  ![](/assets/img/posts/2d-mapping/small_city_occ.png) 
![](/assets/img/posts/2d-mapping/test_city_gz.png)  |  ![](/assets/img/posts/2d-mapping/test_city_occ.png) 


Circuit            |      Small city        |       Test city
:-------------------------:|:-------------------------:|:-------------------------:
![](/assets/img/posts/2d-mapping/circuit_sem.png)  |  ![](/assets/img/posts/2d-mapping/small_city_sem.png) | ![](/assets/img/posts/2d-mapping/test_city_sem.png)