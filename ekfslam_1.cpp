#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <algorithm>
#include <iterator>
#include <errno.h>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "fastslam_core.h"
#include "SLAM_Plot.h"
#include "ekfslam_1.h"


using namespace std;
using namespace Eigen;


// global variable
extern SlamPlot     *g_plot;
extern SLAM_Conf    *g_conf;


EKFSLAM1_Thread::EKFSLAM1_Thread(QObject *parent) : SLAM_Thread(parent)
{
}

EKFSLAM1_Thread::~EKFSLAM1_Thread()
{
    wait();
}


void EKFSLAM1_Thread::run()
{
    MatrixXf    lm; //landmark positions
    MatrixXf    wp; //way points

    printf("EKFSLAM 1\n\n");

    read_slam_input_file(fnMap, &lm, &wp);

    sim(lm, wp);
}


void EKFSLAM1_Thread::sim(MatrixXf &lm, MatrixXf &wp)
{
    int         pos_i = 0;
    double      time_all;

    int         m, n;
    int         i, j;

    QString             msgAll;

    QVector<double>     arrWaypoints_x, arrWaypoints_y;
    QVector<double>     arrLandmarks_x, arrLandmarks_y;
    double              x_min, x_max, y_min, y_max;

    int                 draw_skip = 4;

    g_conf->i("draw_skip", draw_skip);

    x_min =  1e30;
    x_max = -1e30;
    y_min =  1e30;
    y_max = -1e30;

    // draw waypoints
    if( runMode == SLAM_WAYPOINT ) {
        m = wp.rows();
        n = wp.cols();
        for(i=0; i<n; i++) {
            arrWaypoints_x.push_back(wp(0, i));
            arrWaypoints_y.push_back(wp(1, i));

            if( wp(0, i) > x_max ) x_max = wp(0, i);
            if( wp(0, i) < x_min ) x_min = wp(0, i);
            if( wp(1, i) > y_max ) y_max = wp(1, i);
            if( wp(1, i) < y_min ) y_min = wp(1, i);
        }
        g_plot->setWaypoints(arrWaypoints_x, arrWaypoints_y);
    }

    // draw landmarks
    m = lm.rows();
    n = lm.cols();
    for(i=0; i<n; i++) {
        arrLandmarks_x.push_back(lm(0, i));
        arrLandmarks_y.push_back(lm(1, i));

        if( lm(0, i) > x_max ) x_max = lm(0, i);
        if( lm(0, i) < x_min ) x_min = lm(0, i);
        if( lm(1, i) > y_max ) y_max = lm(1, i);
        if( lm(1, i) < y_min ) y_min = lm(1, i);
    }
    g_plot->setLandmarks(arrLandmarks_x, arrLandmarks_y);

    g_plot->setCarSize(g_conf->WHEELBASE, 0);
    g_plot->setCarSize(g_conf->WHEELBASE, 1);
    g_plot->setPlotRange(x_min-(x_max-x_min)*0.05, x_max+(x_max-x_min)*0.05,
                         y_min-(y_max-y_min)*0.05, y_max+(y_max-y_min)*0.05);


    //normally initialized configfile.h
    MatrixXf    Q(2,2), R(2,2);
    float       sigmaPhi;           // radians, heading uncertainty

    Q << pow(g_conf->sigmaV,2), 0,
         0 , pow(g_conf->sigmaG,2);

    R << g_conf->sigmaR*g_conf->sigmaR, 0,
         0, g_conf->sigmaB*g_conf->sigmaB;

    sigmaPhi = g_conf->sigmaT;

    VectorXf    xtrue(3);
    VectorXf    x(3, 1);
    MatrixXf    P(3, 3);

    xtrue.setZero(3);
    x.setZero(3);
    P.setZero(3, 3);

    float dt = g_conf->DT_CONTROLS; //change in time btw predicts
    float dtsum = 0; //change in time since last observation

    vector<int> ftag; //identifier for each landmark
    for (i=0; i<lm.cols(); i++)  ftag.push_back(i);

    //data ssociation table
    vector<int> da_table;
    for (i=0; i<lm.cols(); i++)  da_table.push_back(-1);


    int         iwp = 0;    //index to first waypoint
    int         nloop = g_conf->NUMBER_LOOPS;
    float       V = g_conf->V;  // default velocity
    float       G = 0;      //initial steer angle
    MatrixXf    plines;     //will later change to list of points
    MatrixXf    covLines;   // covariance ellipse lines

    if (g_conf->SWITCH_SEED_RANDOM !=0) {
        srand(g_conf->SWITCH_SEED_RANDOM);
    }

    MatrixXf Qe = MatrixXf(Q);
    MatrixXf Re = MatrixXf(R);

    if (g_conf->SWITCH_INFLATE_NOISE ==1) {
        Qe = 2*Q;
        Re = 8*R;
    }

    vector<int>         ftag_visible;
    vector<int>         idf;

    vector<VectorXf>    z;              //range and bearings of visible landmarks
    vector<VectorXf>    zf;
    vector<VectorXf>    zn;

    pos_i    = 0;
    time_all = 0.0;

    // initial position
    g_plot->addPos(xtrue(0), xtrue(1));
    g_plot->setCarPos(xtrue(0), xtrue(1), xtrue(2), 0);

    g_plot->addPosEst(xtrue(0), xtrue(1));
    g_plot->setCarPos(xtrue(0), xtrue(1), xtrue(2), 1);

    emit replot();

    float   *VnGn = new float[2];
    float   Vn, Gn;
    float   V_ori = V;
    int     cmd;

    //Main loop
    while ( isAlive ) {
        //printf("[%7d]\n", pos_i);

        if( runMode == SLAM_WAYPOINT ) {
            if( iwp == -1 ) break;

            compute_steering(xtrue, wp, iwp, g_conf->AT_WAYPOINT, G, g_conf->RATEG, g_conf->MAXG, dt);
            if (iwp ==-1 && nloop > 1) {
                iwp = 0;
                nloop --;
            }
        }
        if( runMode == SLAM_INTERACTIVE ) {
            getCommand(&cmd);

            // no commands then continue
            if( cmd == -1 ) continue;

            switch( cmd ) {
            case 1:
                // forward
                V = V_ori;
                G = 0.0;
                break;
            case 2:
                // backward
                V = -V_ori;
                G = 0.0;
                break;
            case 3:
                // turn left
                V = V_ori;
                G = 30.0*M_PI/180.0;
                break;
            case 4:
                // turn right
                V = V_ori;
                G = -30.0*M_PI/180.0;
                break;
            default:
                V = V_ori;
                G = 0.0;
            }
        }

        // get true position
        predict_true(xtrue, V, G, g_conf->WHEELBASE, dt);

        // add process noise
        add_control_noise(V, G, Q, g_conf->SWITCH_CONTROL_NOISE, VnGn);
        Vn = VnGn[0];
        Gn = VnGn[1];

        // predict position & heading
        ekf_predict(x, P, Vn, Gn, Qe, g_conf->WHEELBASE, dt);

        // correct x and P by other sensor (low noise, IMU ...)
        ekf_observe_heading(x, P,
                            xtrue(2)+g_conf->sigmaT*unifRand(),
                            g_conf->SWITCH_HEADING_KNOWN,
                            sigmaPhi);

        dtsum += dt;
        if( dtsum >= g_conf->DT_OBSERVE ) {
            dtsum = 0.0;

            ftag_visible = vector<int>(ftag); //modify the copy, not the ftag

            // z is the range and bearing of the observed landmark
            z = get_observations(xtrue, lm, ftag_visible, g_conf->MAX_RANGE);
            add_observation_noise(z, R, g_conf->SWITCH_SENSOR_NOISE);

            plines = make_laser_lines(z, xtrue);


            if( g_conf->SWITCH_ASSOCIATION_KNOWN )
                ekf_data_associate_known(x, z, ftag_visible,
                                         zf, idf, zn,
                                         da_table);
            else
                ekf_data_associate(x, P, z, Re,
                                   g_conf->GATE_REJECT, g_conf->GATE_AUGMENT,
                                   zf, idf, zn);

            ekf_update(x, P, zf, R, idf, g_conf->SWITCH_BATCH_UPDATE);

            ekf_augment(x, P, zn, Re);
        }

        // update status bar
        time_all = time_all + dt;
        pos_i ++;

        // accelate drawing speed
        if( pos_i % draw_skip != 0 ) continue;

        msgAll.sprintf("[%6d] %7.3f", pos_i, time_all);
        emit showMessage(msgAll);


        // add new position
        if( pos_i % 4 == 0 ) {
            g_plot->addPos(xtrue(0), xtrue(1));
            g_plot->addPosEst(x(0), x(1));
        }

        // draw current position
        g_plot->setCarPos(xtrue(0), xtrue(1), xtrue(2));
        g_plot->setCarPos(x(0), x(1), x(2), 1);

        // set laser lines
        g_plot->setLaserLines(plines);

        // set covanice ellipse lines
        MatrixXf x_(2, 1);
        MatrixXf P_ = P.block(0,0,2,2);
        x_(0) = x(0); x_(1) = x(1);
        make_covariance_ellipse(x_, P_, covLines);
        g_plot->setCovEllipse(covLines, 0);

        j = (x.size()-3)/2;
        for(i=0; i<j; i++) {
            x_(0) = x(3+i*2);
            x_(1) = x(3+i*2+1);
            P_ = P.block(3+i*2, 3+i*2, 2, 2);

            make_covariance_ellipse(x_, P_, covLines);
            g_plot->setCovEllipse(covLines, i+1);
        }

        emit replot();

        msleep(10);
    }

    if (VnGn) {
        delete[] VnGn;
    }
}

