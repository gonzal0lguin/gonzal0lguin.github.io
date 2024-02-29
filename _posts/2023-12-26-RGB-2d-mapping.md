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
>- 


## Custom worlds

It is decided to create three environments with common objects found in a city, such as houses, traffic lights, signage, cones, cars, trees, etc. To simplify training, an exact copy of the worlds is created, and the .world files are modified so that each object is assigned the color of a specific class. With this setup, there are the original versions of the worlds and a "segmented" version, which streamlines the dataset generation. Table (1) presents the colors used for the chosen classes, which total 6 + 1.

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
    <th class="tg-yj5y">Class</th>
    <th class="tg-yj5y">ID</th>
    <th class="tg-y698">RGB</th>
  </tr>
</thead>
<tbody>
  <tr>
    <td class="tg-0pky">Unknown</td>
    <td class="tg-c3ow">0</td>
    <td class="tg-nto1"><span style="color:#FFF">Black (0, 0, 0)</span></td>
  </tr>
  <tr>
    <td class="tg-0pky">Vehicles</td>
    <td class="tg-c3ow">1</td>
    <td class="tg-95rq">Blue (0, 0, 255)</td>
  </tr>
  <tr>
    <td class="tg-0pky">Street/ground</td>
    <td class="tg-c3ow">2</td>
    <td class="tg-h8ck">Green (0, 255, 0)</td>
  </tr>
  <tr>
    <td class="tg-0pky">Buildings</td>
    <td class="tg-c3ow">3</td>
    <td class="tg-46ru">Cyan (0, 255, 255)</td>
  </tr>
  <tr>
    <td class="tg-0pky">Sky</td>
    <td class="tg-c3ow">4</td>
    <td class="tg-pyoy">Gray (127, 127, 127)</td>
  </tr>
  <tr>
    <td class="tg-0pky">Obstacles</td>
    <td class="tg-c3ow">5</td>
    <td class="tg-lydr">Red (255, 0, 0)</td>
  </tr>
  <tr>
    <td class="tg-0pky">People</td>
    <td class="tg-c3ow">6</td>
    <td class="tg-zqsx">Magenta (255, 0, 255)</td>
  </tr>
</tbody>
</table>
</center>


It is important to mention that class 0 (unknown) does not directly exist in the worlds but is considered for the transformations that will be explained later.

The next figure displays the three worlds created in Gazebo, where the first two, `circuit` and `small_city`, will be used for training, and the world `test_city` contains buildings and objects not present in the others.

Circuit            |      Small city        |  Test city      
:-------------------------:|:-------------------------:|:-------------------------:
![](/assets/img/posts/2d-mapping/circuit.png)  |  ![](/assets/img/posts/2d-mapping/small_city.png) | ![](/assets/img/posts/2d-mapping/test_city.png)


### Obtaining the dataset

Generating the dataset involves three steps: firstly, positions within the worlds where the robot could be located during its operation must be obtained. Then, images at these positions must be acquired in both the normal and segmented worlds. Finally, the images in the segmented world need to be post-processed to "flatten" the colors. In other words, the colors with variations due to shadows within the simulator must be converted to uniform colors (pure red, pure blue, etc.).

For the first step, a Python node is created that subscribes to the ground truth pose of the robot reported by the p3d plugin in Gazebo and saves them as a numpy array. Code (1) shows a snippet of this process.

Para el primer punto, se crea un nodo de Python que se suscribe a la pose de la verdad terrestre del robot, reportada por el complemento p3d de Gazebo, y la guarda como un array de numpy. En el Código (1) se muestra un fragmento de este proceso.

-- codigo --

The next step, reproducing the poses, is done using the Gazebo service /gazebo/set_model_state, with which the robot can be positioned within the world in a specific pose. Additionally, using OpenCV, the camera images are saved at the commanded pose. In summary, the process to obtain the images is as follows:
1. Load a saved pose.
2. Move the robot to that pose using SetModelState.
3. Save the RGB camera image to the corresponding directory.

The above steps are repeated in both normal and segmented worlds in the same poses to obtain an input image and its training label.

Finally, the images from the segmented world are passed through a filter to convert the classes to uniform colors. Code (2) shows the functioning of the function in a summarized way. Masks are generated for each color, for example, red, such that if a pixel has intensity in the red channel greater than a quantity at least 1.5 times greater than the rest of the channels, then it is assigned the maximum red value (255, 0, 0). This is repeated for all pixels and considering the 6 possible colors.

In the case where a pixel does not meet any of the conditions and is assigned as black (initial value 0), the function nearest_non_black_color is used. This function assigns to the pixel the predominant color in a neighborhood with a radius of 10 pixels.

--codigo --

With the above, a dataset of 4192 images with their respective masks is obtained. In Figure (3), you can see an example with the input image in the normal and segmented worlds, along with the training mask.

Original            |      Segmented        |  Flattened     
:-------------------------:|:-------------------------:|:-------------------------:
![](/assets/img/posts/2d-mapping/img_00010.png)  |  ![](/assets/img/posts/2d-mapping/img_00010_seg.png) | ![](/assets/img/posts/2d-mapping/img_00010_flat.png)


## U-Net

The U-Net segmentation model is chosen for its prior work during the course. The [Pytorch-Unet](https://github.com/milesial/Pytorch-UNet) repository serves as a base, and a specific dataset is created for this implementation. Additionally, functions for inference during evaluation and for calculating Intersection over Union (IoU) are added. Code (3) demonstrates one of the functions for calculating IoU and evaluating the network's performance.

Table (2) presents the results of training U-Net on the dataset consisting of 4192 images with a validation size of 20%, 200 epochs, a batch size of 16, and a learning rate of 1e-6. The training lasted for 12 hours and was conducted on an RTX3060 GPU.

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

As is evident, excellent results are achieved for the main obstacles and the ground, with an IoU exceeding 89%. However, for class 6 (people), a low IoU of under 8% is obtained, which is attributed to the limited presence of people in the training images. While it is crucial to detect people during navigation, in the mapping stage, it may not be as relevant since they are not static "obstacles."

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

### Local Mapping

BEV            |      Occ grid        |  Prob. Occ grid     
:-------------------------:|:-------------------------:|:-------------------------:
![](/assets/img/posts/2d-mapping/bevocc1.png)  |  ![](/assets/img/posts/2d-mapping/bevocc2.png) | ![](/assets/img/posts/2d-mapping/bevocc3.png)

### Global Mapping

This part consists of generating a global map from the local maps obtained in the previous steps. For this, we start with the assumption in figure below, where there is a global map with a reference system $(X_G, Y_G)$. The robot is located in this map at a position $(X_R, Y_R)$, discretized based on the map resolution and with an orientation θ. The local map is at a distance $dx = 3.25[m]$ in the $x$ direction and has dimensions $(W, L) = (480, 480)$ with a reference system $(X_L, Y_L)$. Given a cell $(x_l, y_l)$ in the local map, we want to know its coordinates $P = (x_g, y_g)$ in the global map.


Local Map $(X_L, Y_L)$ within a global map $(X_G, Y_G)$          |
:-------------------------:|
![](/assets/img/posts/2d-mapping/algorithm.png)  | 

Based on the geometry of the problem, equations (1) to (6) are derived, which allow determining the coordinates of a cell in the local map in the global map, considering the robot's pose.

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

The Occupancy type map, during its updating process, considers three conditions: if it receives a value of -1, it is ignored as it represents space without information from the Bird's Eye View (BEV) transform. If it receives a value of 0 (free space), it overwrites the previous value of the cell. Lastly, values are clipped to ensure they remain between 0 and 100, representing probabilities of occupancy.

As there are probabilities in each cell, during the update, the previous values are summed with the current values. This is because observing a cell multiple times increases the certainty of its value (hence, the value is reassigned to 0 if observed). Part of the update algorithm used for occupancy maps is shown in the following snippet, full implementation can be found at [mapper.cpp](https://github.com/gonzal0lguin/monocular-2d-mapping/blob/main/ros/dev_ws/src/mono_perception/src/mapper.cpp).

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

For semantic maps, the same logic is applied, but in this case, the pixel values are not summed because the class observed in the map needs to be maintained. In the following snippet, only the relevant update for this case is presented, while the rest remains the same.


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

## Results

In Figure (8), segmentation results for some of the validation set images are displayed. The image set includes the training mask, the predicted mask, and the difference between the last two. As expected, the results show a correct class prediction of over 90%, except for the person being misclassified as a vehicle.

Sample predictions and overlap in the validation set.          |
:-------------------------:|
![](/assets/img/posts/2d-mapping/unetval.png)  |


In Figure (10c), a local map obtained in one of the test worlds is displayed. Figures (10a) and (10b) show perspectives from Gazebo and RViz, respectively. In (10b), LiDAR readings are also highlighted in red. As evident, there is a correspondence between the readings and the occupied sector in the local map, generated from the segmented image, indicating correct functionality and alignment.

It is important to note that when saving the maps, pixel values are assigned based on a threshold for occupied/non-occupied, so in Figure (10c), pixels farther away are shown as free since they have a value below the threshold.

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

As evident, the maps in both cases preserve the general shape of the worlds, especially in the first one (circuit), being the simplest and only presenting the ground and obstacle classes. Overall, the maps successfully represent the worlds from a "top-down" perspective.

However, the classes of vehicles and obstacles in the worlds small_city and test_city fail to have coherent representations of their shapes and sizes from a vertical perspective. This is particularly noticeable in the semantic maps in Figure (14b) and (14c), where the mentioned classes are highlighted in blue and red, respectively. It is especially observed in the cars that the shapes are distorted and larger than their normal size. This is due to two reasons: first, the geometry of the vehicles varies in curves, height, etc., causing the distortions to exaggerate and differ when viewed from different angles. In Figure (14b), the front wheels of the cars are accentuated, and also the bus is seen from the side in the bottom-left corner. The second reason is given by the way the map updates. As explained, it is overwritten at all times, so seeing an object like a tree or a car from two different positions completely changes the result of the map.

As a result, flat and uniform geometries like buildings map more effectively by maintaining their shape when viewed from different positions. The obtained results are good enough to differentiate between free and occupied space, allowing a broad understanding of the environment.