#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <pthread.h>

#include <QApplication>

//#include <rtk_debug.h>

#include "SLAM_Plot.h"
#include "SLAM_Thread.h"

#include "fastslam_1.h"
#include "fastslam_2.h"

////////////////////////////////////////////////////////////////////////////////
// global variables
////////////////////////////////////////////////////////////////////////////////
SlamPlot    *g_plot;


////////////////////////////////////////////////////////////////////////////////
// main function
////////////////////////////////////////////////////////////////////////////////
int main(int argc, char *argv[])
{
    int             ret = 0;
    int             slam_method = 1;
    SLAM_Thread     *slam_thread;

    //rtk::dbg_stacktrace_setup();

    // get slam version
    if( argc > 1 ) {
        slam_method = atoi(argv[1]);
    }

    QApplication a(argc, argv);

    // SLAM plot window
    SlamPlot w;
    g_plot = &w;
    w.show();
    w.setGeometry(10, 10, 950, 768);
    w.plot();

    // begin SLAM thread
    if( slam_method == 1 ) {
        slam_thread = new FastSLAM1_Thread;
    } else if ( slam_method == 2 ) {
        slam_thread = new FastSLAM2_Thread;
    }

    slam_thread->start();

    // begin GUI loop
    ret = a.exec();

    // stop SLAM thread
    slam_thread->stop();

    return ret;
}
