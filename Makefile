################################################################################
# Compiler settings
################################################################################
CC=gcc
CXX=g++
MOC=moc-qt4


################################################################################
# Qt settings
################################################################################
QT_CFLAGS = -DQT_NO_DEBUG -DQT_GUI_LIB -DQT_CORE_LIB -DQT_SHARED \
            -I/usr/share/qt4/mkspecs/linux-g++ \
            -I/usr/include/qt4/QtCore \
            -I/usr/include/qt4/QtGui \
            -I/usr/include/qt4
QT_LIBS   = -lQtGui -lQtCore 


################################################################################
# Eigen3
################################################################################
EIGEN3_CFLAGS = -I/usr/include/eigen3 
EIGEN3_LIBS   = 


################################################################################
# overall CFLAGS & LDFLAGS
################################################################################
CXXFLAGS = -D__STDC_CONSTANT_MACROS \
            $(QT_CFLAGS) $(EIGEN3_CFLAGS)
LDFLAGS  = -lz -lpthread \
            $(QT_LIBS)
MOC_CFLAGS = $(QT_CFLAGS)


#CXXFLAGS += -fopenmp
#LDFLAGS += -lgomp

#CXXFLAGS += $(FFMPEG_CFLAGS)

#CXXFLAGS += -g -rdynamic
CXXFLAGS += -O3 

################################################################################

################################################################################
src-all := $(wildcard *.cpp)
obj-all := $(patsubst %.cpp,%.o,$(src-all))
inc-all := $(wildcard *.h)

target_req = qcustomplot.o moc_qcustomplot.o \
    SLAM_Plot.o moc_SLAM_Plot.o \
    fastslam_1.o moc_fastslam_1.o \
    fastslam_2.o moc_fastslam_2.o \
    SLAM_Thread.o moc_SLAM_Thread.o \
    utils.o	fastslam_core.o main.o


all : fastslam.e test.e

moc_qcustomplot.o : qcustomplot.h
	$(MOC) qcustomplot.h -o moc_qcustomplot.cpp $(MOC_CFLAGS)
	$(CXX) -c moc_qcustomplot.cpp -o moc_qcustomplot.o $(CXXFLAGS)

moc_SLAM_Plot.o : SLAM_Plot.h
	$(MOC) SLAM_Plot.h -o moc_SLAM_Plot.cpp $(MOC_CFLAGS)
	$(CXX) -c moc_SLAM_Plot.cpp -o moc_SLAM_Plot.o $(CXXFLAGS)

moc_SLAM_Thread.o : SLAM_Thread.h
	$(MOC) SLAM_Thread.h -o moc_SLAM_Thread.cpp $(MOC_CFLAGS)
	$(CXX) -c moc_SLAM_Thread.cpp -o moc_SLAM_Thread.o $(CXXFLAGS)

moc_fastslam_1.o : fastslam_1.h
	$(MOC) fastslam_1.h -o moc_fastslam_1.cpp $(MOC_CFLAGS)
	$(CXX) -c moc_fastslam_1.cpp -o moc_fastslam_1.o $(CXXFLAGS)

moc_fastslam_2.o : fastslam_2.h
	$(MOC) fastslam_2.h -o moc_fastslam_2.cpp $(MOC_CFLAGS)
	$(CXX) -c moc_fastslam_2.cpp -o moc_fastslam_2.o $(CXXFLAGS)

fastslam.e : $(target_req)
	$(CXX) -o $@ $? $(LDFLAGS)

test.e : test.o fastslam_core.o
	$(CXX) -o $@ $? $(LDFLAGS)

%.e:%.cpp $(inc-all)
	$(CXX) $< -o $@ $(CFLAGS) $(LDFLAGS)

%.o:%.cpp
	$(CXX) -c $< -o $@ $(CXXFLAGS)

clean :
	rm -f *.e *.o moc_*

