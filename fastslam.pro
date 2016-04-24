QT += core gui printsupport
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle

HEADERS += \
    ./fastslam_core.h \
    ./qcustomplot.h \
    ./SLAM_Plot.h \
    ./SLAM_Thread.h \
    ./fastslam_1.h \
    ./fastslam_2.h \
    ./ekfslam_1.h \
    ./utils.h

SOURCES += \
    ./fastslam_core.cpp \
    ./qcustomplot.cpp \
    ./SLAM_Plot.cpp \
    ./SLAM_Thread.cpp \
    ./fastslam_1.cpp \
    ./fastslam_2.cpp \
    ./ekfslam_1.cpp \
    ./main.cpp \
    ./utils.cpp

OTHER_FILES = Makefile README.md NOTES.txt

QMAKE_LIB_FLAG += `pkg-config --libs opencv` \
                  `pkg-config --libs eigen3`

QMAKE_CXXFLAGS += `pkg-config --cflags opencv` \
                  `pkg-config --cflags eigen3`
