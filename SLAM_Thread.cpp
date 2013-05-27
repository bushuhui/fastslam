
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

    command_id = 0;
    command_time = 0;

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
    command_id = cmd;
    command_time = tm_get_millis();
    printf("command = %d\n", command_id);
}

void SLAM_Thread::get_command(int *cmd)
{
    u_int64_t       time_now, dt;

    time_now = tm_get_millis();
    dt = time_now - command_time;

    if( dt < 30 ) {
        *cmd = command_id;
    } else {
        *cmd = -1;
    }
}
