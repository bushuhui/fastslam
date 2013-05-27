TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG += qt


SOURCES += \
    ./fastslam_core.cpp \
    ./qcustomplot.cpp \
    ./SLAM_Plot.cpp \
    ./fastslam_1.cpp \
    ./fastslam_2.cpp \
    ./test.cpp \
    ./main.cpp \
    SLAM_Thread.cpp \
    utils.cpp

    

HEADERS += \
    ./fastslam_core.h \
    ./qcustomplot.h \
    ./SLAM_Plot.h \
    ./fastslam_1.h \
    ./fastslam_2.h \
    SLAM_Thread.h \
    utils.h

OTHER_FILES = Makefile



RTK_TOP     = /home/bushuhui/msdk/my_progs/uav/rtk++
OPENCV_TOP  = /opt/opencv-2.4.5

LIBS += -L$$OPENCV_TOP/lib \
        -lopencv_calib3d -lopencv_contrib -lopencv_core \
        -lopencv_features2d -lopencv_flann -lopencv_gpu \
        -lopencv_highgui -lopencv_imgproc -lopencv_legacy \
        -lopencv_ml -lopencv_nonfree -lopencv_objdetect \
        -lopencv_photo -lopencv_stitching -lopencv_ts \
        -lopencv_video -lopencv_videostab \
        -L$$RTK_TOP/lib \
        -lrtk_gui -lrtk_utils -lrtk_osa -lrtk_pr -lrtk_cv

INCLUDEPATH +=  /usr/include/eigen3 \
                $$OPENCV_TOP/include $$OPENCV_TOP/include/opencv \
                $$RTK_TOP/include
