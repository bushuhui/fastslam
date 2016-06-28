#ifndef __EKFSLAM_1_H__
#define __EKFSLAM_1_H__

#include <QtGui>
#include <QMutex>
#include <QSize>
#include <QThread>
#include <QWaitCondition>

#include <iostream>
#include <fstream>
#include <sstream>
#include <stdlib.h>
#include <vector>

#include <Eigen/Dense>

#include "fastslam_core.h"
#include "SLAM_Thread.h"

using namespace std;
using namespace Eigen;


class EKFSLAM1_Thread : public SLAM_Thread
{
    Q_OBJECT

public:
    EKFSLAM1_Thread(QObject *parent = 0);
    ~EKFSLAM1_Thread();

    void sim(MatrixXf &lm, MatrixXf &wp);

protected:
    void run();
};

#endif // __EKFSLAM_1_H__
