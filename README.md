# FastSLAM with GUI

This program is a pure C++ implementation of FastSLAM 1 and 2. It also integrated a GUI interface which draw current states and observations.

The source is based on yglee source code (https://github.com/yglee/FastSLAM), and orignal FastSLAM (http://www-personal.acfr.usyd.edu.au/tbailey/software/slam_simulations.htm) . We add Qt GUI and also fixed some bugs from yglee's implementation. 


## Requirements:
* Eigen3 ( sudo apt-get install libeigen3-dev)
* Qt4 (sudo apt-get install libqt4-core libqt4-dev)
* QCustomPlot (included, webpage: http://www.workslikeclockwork.com/) 


## Compile:
`make`


## Usage:
```
./FastSLAM
    -m                  [s] input map file name
    -mode               [s] runing mode
        waypoints   : following given waypoints
        interactive : use keyboard to control movement
    -method             [s] SLAM method
        EKF1        : EKF SLAM 1
        FAST1       : FastSLAM 1
        FAST2       : FastSLAM 2
    -h  (print usage)
```


examples:

`./FastSLAM -method FAST1 -mode interactive` (FastSLAM 1, user interactive)

`./FastSLAM -method FAST2 -mode waypoints -m data/example_webmap.mat` (FastSLAM 2, following waypoints, map is "example_webmap.mat")

`./FastSLAM -method EKF1 -mode waypoints -m data/example_loop1.mat` (EKF SLAM, following waypoints, map is "example_loop1.mat")



## Plateform:
Only test on Ubuntu 10.04 64-bit. 


## Issues:
* Low performance (Even worse than Matlab version)
* Crash occurs when zooming or moving plot (occasionally)


## Screenshot:
-![Screenshot 1](figures/Screenshot-2D-SLAM_1.png "Screenshot 1")
-![Screenshot 2](figures/Screenshot-2D-SLAM_2.png "Screenshot 2")


## Project homepage:
http://www.adv-ci.com/blog/source/fastslam-gui/
