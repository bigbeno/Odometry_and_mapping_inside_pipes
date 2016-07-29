# Odometry_and_mapping_inside_pipes

## Synopsis

Software for running the SEA-snake inside pipe networks and estimate its motion as well as an high-level map of the traversed pipes.
The estimates come from the merging of inertial, visual and kinematics odometries.


## Code Example

The software is divided in three packages:
•	_snake_in_pipes_ (MATLAB)
•	_visual_odometry_(C++)
•	_odometry_and_mapping_ (MATLAB)

They are to be used in the given order. In details.

1. Run an experiment, i.e. make the snake crawl inside a pipe while recording a video from the frontal monocular camera:
```
snake_in_pipes/snake_in_pipes_main.m
```

2. Estimate the robot motion from the video:
```
visual_odometry/src/visual_odometry.cpp
```

3. Feed the experiment log and the visual-odometry estimate to the inertial+kinematic estimators and run an EKF to merge all estimates:
```
odometry_and_mapping/odometry_and_mapping_in_pipes_main.m
```

## Motivation

This project was originally created for the completion of the Master Thesis of Elena Morara.
It would be useful for application of pipe visual inspections, in which the location of the detected fault must be retrieved.
Furthermore, it may be used to retrieve the map of an unknown pipe network.

## Installation

The MATLAB code requires a version greater or equal than 2014a and Symbolic Toolbox.

The C++ code requires OpenCV 3.1 API for C++.

## API Reference

See DOCUMENTATION.odt for details on usage, as well as learned lessons and advices on future work directions.

## Tests

Just run
```
visual_odometry/src/visual_odometry.cpp
odometry_and_mapping/odometry_and_mapping_in_pipes_main.m
```
to perform the estimation on a sample experiment log and video.

The EKF (odometry_and_mapping_in_pipes_main.m) can be tested on different sample logs, which are stored in 
```
odometry_and_mapping/sample_data/
```

## Contributors

So far, Elena Morara is the only contributor to this project.