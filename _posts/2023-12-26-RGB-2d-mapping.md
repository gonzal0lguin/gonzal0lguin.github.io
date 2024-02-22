---
title: Monocular 2D semantic mapping
date: 2023-12-26 17:22:09 -03
author: Gonz
categories: [Robotics, Deep Learning]
tags: [ROS, mapping]
image:
    path: /assets/img/headers/2dmapping.png
---

# Monocular 2d semantic mapping

The idea of this project is to use a 2D monocular RGB camera onboard a mobile platform and be able to create 2D occupancy grid maps and semantic maps using only odometry as a sensor source. 

As it will be explained below, the process will be to first obtain a segmented image, the perform a Bird's eye transform and an appropiate conversion to occupancy grid or a segmented local map. Finillay, using odometry the local maps will be concatenated to create a global map.

Full code implementation and usage instructions can be found in my github profile at [gonzal0lguin/monocular-2d-mapping](https://github.com/gonzal0lguin/monocular-2d-mapping).

----

This is the final evaluation of the course *Advanced Image Procesing* of the Electrical Engineering Department of University of Chile.


> Given the limited timespan of the project, several simplifications were made to accomplish the objectives. These are the following:
>- The project will be 100\% simulated in Gazebo and be based in ROS Noetic, using the provided simulation of the [Panther](https://husarion.com/manuals/panther/) robot by Husarion. 
>- No SLAM algorithm will be used to correct for odometry drift and feature-matching or map closure. Only odometry will be used to create the maps.
>- Segmentation will be performed by a custom-trained network (more below). In order to rapidly obtain training image pairs, an exact copy of the simulation worlds will be used, but containing only plain colors of the target classes.
>- no se que mas for now


## Custom worlds

### Gazebo

### Obtaining the dataset


## Bird's eye transform

## Occupancy grid mapping

### "Local Mapping"

### "Gloabl Mapping"

## Semantic mapping