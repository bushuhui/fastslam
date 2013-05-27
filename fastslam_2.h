#ifndef __FASTSLAM_2_H__
#define __FASTSLAM_2_H__

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


class FastSLAM2_Thread : public SLAM_Thread
{
    Q_OBJECT

public:
    FastSLAM2_Thread(QObject *parent = 0);
    ~FastSLAM2_Thread();


    void sample_proposal(Particle &particle, vector<VectorXf> &z, vector<int> &idf, MatrixXf &R);
    float likelihood_given_xv(Particle &particle, vector<VectorXf> &z, vector<int> &idf, MatrixXf &R);
    VectorXf delta_xv(VectorXf &xv1, VectorXf &xv2);

    void predict(Particle &particle,float V,float G, MatrixXf &Q, float WB, float dt, int addrandom);
    void observe_heading(Particle &particle, float phi, int useheading);
    float gauss_evaluate(VectorXf &v, MatrixXf &S, int logflag);

    float compute_weight(Particle &particle, vector<VectorXf> &z, vector<int> &idf, MatrixXf &R);

    vector<Particle> sim(MatrixXf &lm, MatrixXf &wp);

protected:
    void run();
};

#endif // __FASTSLAM_2_H__
