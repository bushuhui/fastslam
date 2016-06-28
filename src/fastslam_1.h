#ifndef __FASTSLAM_1_H__
#define __FASTSLAM_1_H__

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


class FastSLAM1_Thread : public SLAM_Thread
{
    Q_OBJECT

public:
    FastSLAM1_Thread(QObject *parent = 0);
    ~FastSLAM1_Thread();

    void predict(Particle &particle, float V, float G, MatrixXf &Q, float WB, float dt, int addrandom);
    float compute_weight(Particle &particle, vector<VectorXf> &z, vector<int> &idf, MatrixXf &R);
    vector<Particle> sim(MatrixXf &lm, MatrixXf &wp);


protected:
    void run();
};

#endif // __FASTSLAM_1_H__
