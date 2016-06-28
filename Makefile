################################################################################
# Compiler settings
################################################################################
CC			= gcc
CXX			= g++
MOC			= moc-qt4


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
EIGEN3_CFLAGS = -I./libs/eigen3 
EIGEN3_LIBS   = 


################################################################################
# overall CFLAGS & LDFLAGS
################################################################################
CXXFLAGS 	= -D__STDC_CONSTANT_MACROS \
            	$(QT_CFLAGS) $(EIGEN3_CFLAGS) \
				-I./src
LDFLAGS  	= -lz -lpthread \
            	$(QT_LIBS)
MOC_CFLAGS 	= $(QT_CFLAGS)


CXXFLAGS   += -msse4
#CXXFLAGS += -fopenmp
#LDFLAGS += -lgomp

#CXXFLAGS += $(FFMPEG_CFLAGS)

CXXFLAGS   += -g -rdynamic
#CXXFLAGS += -O3 

################################################################################

################################################################################
src-all := $(wildcard *.cpp)
obj-all := $(patsubst %.cpp,%.o,$(src-all))
inc-all := $(wildcard *.h)

target_req = src/qcustomplot.o src/moc_qcustomplot.o \
    src/SLAM_Plot.o src/moc_SLAM_Plot.o \
    src/fastslam_1.o src/moc_fastslam_1.o \
    src/fastslam_2.o src/moc_fastslam_2.o \
    src/ekfslam_1.o src/moc_ekfslam_1.o \
    src/SLAM_Thread.o src/moc_SLAM_Thread.o \
    src/utils.o src/fastslam_core.o src/main.o


all : fastslam.e test.e

src/moc_qcustomplot.o : src/qcustomplot.h
	$(MOC) src/qcustomplot.h -o src/moc_qcustomplot.cpp $(MOC_CFLAGS)
	$(CXX) -c src/moc_qcustomplot.cpp -o src/moc_qcustomplot.o $(CXXFLAGS)

src/moc_SLAM_Plot.o : src/SLAM_Plot.h
	$(MOC) src/SLAM_Plot.h -o src/moc_SLAM_Plot.cpp $(MOC_CFLAGS)
	$(CXX) -c src/moc_SLAM_Plot.cpp -o src/moc_SLAM_Plot.o $(CXXFLAGS)

src/moc_SLAM_Thread.o : src/SLAM_Thread.h
	$(MOC) src/SLAM_Thread.h -o src/moc_SLAM_Thread.cpp $(MOC_CFLAGS)
	$(CXX) -c src/moc_SLAM_Thread.cpp -o src/moc_SLAM_Thread.o $(CXXFLAGS)

src/moc_fastslam_1.o : src/fastslam_1.h
	$(MOC) src/fastslam_1.h -o src/moc_fastslam_1.cpp $(MOC_CFLAGS)
	$(CXX) -c src/moc_fastslam_1.cpp -o src/moc_fastslam_1.o $(CXXFLAGS)

src/moc_fastslam_2.o : src/fastslam_2.h
	$(MOC) src/fastslam_2.h -o src/moc_fastslam_2.cpp $(MOC_CFLAGS)
	$(CXX) -c src/moc_fastslam_2.cpp -o src/moc_fastslam_2.o $(CXXFLAGS)

src/moc_ekfslam_1.o : src/ekfslam_1.h
	$(MOC) src/ekfslam_1.h -o src/moc_ekfslam_1.cpp $(MOC_CFLAGS)
	$(CXX) -c src/moc_ekfslam_1.cpp -o src/moc_ekfslam_1.o $(CXXFLAGS)

fastslam.e : $(target_req)
	$(CXX) -o $@ $? $(LDFLAGS)

test.e : src/test.o src/fastslam_core.o src/utils.o
	$(CXX) -o $@ $? $(LDFLAGS)

%.e:%.cpp $(inc-all)
	$(CXX) $< -o $@ $(CFLAGS) $(LDFLAGS)

%.o:%.cpp
	$(CXX) -c $< -o $@ $(CXXFLAGS)

clean :
	rm -f *.e src/*.o src/moc_*

