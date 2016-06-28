TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG += qt


HEADERS += \
    ./src/fastslam_core.h \
    ./src/qcustomplot.h \
    ./src/SLAM_Plot.h \
    ./src/SLAM_Thread.h \
    ./src/fastslam_1.h \
    ./src/fastslam_2.h \
    ./src/ekfslam_1.h \
    ./src/utils.h

SOURCES += \
    ./src/fastslam_core.cpp \
    ./src/qcustomplot.cpp \
    ./src/SLAM_Plot.cpp \
    ./src/SLAM_Thread.cpp \
    ./src/fastslam_1.cpp \
    ./src/fastslam_2.cpp \
    ./src/ekfslam_1.cpp \
    ./src/main.cpp \
    ./src/test.cpp \
    ./src/utils.cpp

OTHER_FILES = Makefile README.md NOTES.txt

INCLUDEPATH +=  ./libs/eigen3

