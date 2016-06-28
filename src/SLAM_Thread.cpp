
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <QtGui>

#include "SLAM_Thread.h"
#include "SLAM_Plot.h"

#include "utils.h"

// global variable
extern SlamPlot    *g_plot;


SLAM_Thread::SLAM_Thread(QObject *parent) : QThread(parent)
{
    isAlive = 1;

    commandID   = -1;
    commandTime = 0;

    qRegisterMetaType<QString>("QString");

    connect(this,   SIGNAL(replot()),               g_plot, SLOT(canvasReplot()));
    connect(this,   SIGNAL(showMessage(QString)),   g_plot, SLOT(canvasShowMessage(QString)));

    connect(g_plot, SIGNAL(commandSend(int)),       this,   SLOT(commandRecv(int)));
}

SLAM_Thread::~SLAM_Thread()
{
    wait();
}

void SLAM_Thread::stop(void)
{
    isAlive = 0;
}

void SLAM_Thread::commandRecv(int cmd)
{
    commandID   = cmd;
    commandTime = tm_get_millis();
}

void SLAM_Thread::getCommand(int *cmd)
{
    u_int64_t       time_now, dt;

    time_now = tm_get_millis();
    dt = time_now - commandTime;

    if( dt < 30 ) {
        *cmd = commandID;
    } else {
        *cmd = -1;
    }
}

// set run mode
void SLAM_Thread::setRunMode(RunMode mode)
{
    runMode = mode;
}

// set map filename
void SLAM_Thread::setMap(std::string &fname)
{
    fnMap = fname;
}
