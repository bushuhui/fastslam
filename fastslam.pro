TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG += qt



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
    ./test.cpp \
    ./main.cpp \
    ./utils.cpp


OTHER_FILES = Makefile README.md NOTES.txt



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

INCLUDEPATH +=  ./libs/eigen3 \
                $$OPENCV_TOP/include $$OPENCV_TOP/include/opencv 

