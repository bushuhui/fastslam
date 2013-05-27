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
`./fastslam.e 1` (FastSLAM 1)

`./fastslam.e 2` (FastSLAM 2)


## Plateform:
Only test on Ubuntu 10.04 64-bit. 


## Issues:
* Low performance (Even worse than Matlab version)
* Crash occurs when zooming or moving plot (occasionally)
    

