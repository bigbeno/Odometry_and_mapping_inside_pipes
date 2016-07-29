# Odometry and mapping inside pipes

## Synopsis

Software for running the SEA-snake inside pipe networks and estimate its motion as well as an high-level map of the traversed pipes.
The estimates come from the merging of inertial, visual and kinematics odometries.


## Code Example

The software is divided in three packages: ``` snake_in_pipes``` (MATLAB), ``` visual_odometry ``` (C++) and 	``` odometry_and_mapping ``` (MATLAB)

The proper use of them is in the following order:

1. Run an experiment, i.e. make the snake crawl inside a pipe while recording a video from the frontal monocular camera by running **`snake_in_pipes/snake_in_pipes_main.m`**
  
2. Estimate the robot motion from the video by running **`visual_odometry/src/visual_odometry.cpp`**
  
3. Feed the experiment log and the visual-odometry estimate to the inertial+kinematic estimators and run an EKF to merge all estimates by running **`odometry_and_mapping/odometry_and_mapping_in_pipes_main.m`**

## Motivation

This project was originally created for the completion of the Master Thesis of Elena Morara.
It would be useful for application of pipe visual inspections, in which the location of the detected fault must be retrieved.
Furthermore, it may be used to retrieve the map of an unknown pipe network.

## Installation

The MATLAB code requires a version greater or equal than 2014a and Symbolic Toolbox.  
The C++ code requires OpenCV 3.1 API.

## API Reference

See [Wiki page](https://github.com/biorobotics/Odometry_and_mapping_inside_pipes/wiki) for many more details on usage, as well as learned lessons and advices on future work directions.

## Tests

Just run `visual_odometry/src/visual_odometry.cpp` to perform visual odometry on a sample video recorded on the snake.


Just run `odometry_and_mapping/odometry_and_mapping_in_pipes_main.m ` to perform the full inertial+visual+kinematic estimation on a sample experiment log.  
The full estimation can be tested also on other provided sample logs, which are stored in ` odometry_and_mapping/sample_data/ `

## Contributors

So far, Elena Morara is the only contributor to this project.
