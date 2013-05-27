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
#include "fastslam_1.h"


using namespace std;
using namespace Eigen;
using namespace config;


// global variable
extern SlamPlot    *g_plot;



FastSLAM1_Thread::FastSLAM1_Thread(QObject *parent) : SLAM_Thread(parent)
{
}

FastSLAM1_Thread::~FastSLAM1_Thread()
{
    wait();
}


void FastSLAM1_Thread::run()
{
    MatrixXf    lm; //landmark positions
    MatrixXf    wp; //way points

    printf("FastSLAM 1\n\n");

    read_slam_input_file("example_webmap.mat", &lm, &wp);

    vector<Particle> data = sim(lm,wp);

#if 0
    for (int i=0; i<data.size(); i++) {
        cout<<"particle i="<<i<<endl;
        cout<<endl;
        cout<<"xv (robot pose)"<<endl;
        cout<<data[i].xv()<<endl;
        cout<<endl;
        cout<<"Pv (controls)"<<endl;
        cout<<data[i].Pv()<<endl;
        cout<<endl;
        cout<<"xf (EFK means)"<<endl;
        for(int j=0; j<data[i].xf().size(); j++) {
            cout<<data[i].xf()[j]<<endl;
            cout<<endl;
        }
        cout<<endl;
        cout<<"Pf (covariance mat)"<<endl;
        for(int k=0; k<data[i].Pf().size(); k++) {
            cout<<data[i].Pf()[k]<<endl;
            cout<<endl;
        }
        cout<<endl;
        cout<<endl;
    }
#endif
}


vector<Particle> FastSLAM1_Thread::sim(MatrixXf &lm, MatrixXf &wp)
{
    int         pos_i = 0;
    double      time_all;

    int         m, n;
    int         i, j;

    QString             msgAll;

    QVector<double>     arrParticles_x, arrParticles_y;
    QVector<double>     arrParticlesFea_x, arrParticlesFea_y;

    QVector<double>     arrWaypoints_x, arrWaypoints_y;
    QVector<double>     arrLandmarks_x, arrLandmarks_y;
    double              x_min, x_max, y_min, y_max;

    double              w_max;
    double              x_mean, y_mean, t_mean;

    x_min =  1e30;
    x_max = -1e30;
    y_min =  1e30;
    y_max = -1e30;

    // draw waypoints
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

    g_plot->setCarSize(4, 0);
    g_plot->setCarSize(4, 1);
    g_plot->setPlotRange(x_min-(x_max-x_min)*0.05, x_max+(x_max-x_min)*0.05,
                         y_min-(y_max-y_min)*0.05, y_max+(y_max-y_min)*0.05);

    SWITCH_PREDICT_NOISE = 1;

    //normally initialized configfile.h
    Q << pow(sigmaV,2), 0,
         0 , pow(sigmaG,2);

    R << sigmaR*sigmaR, 0,
         0, sigmaB*sigmaB;


    //vector of particles (their count will change)
    vector<Particle> particles(NPARTICLES);
    for (int i=0; i<particles.size(); i++) {
        particles[i] = Particle();
    }

    //initialize particle weights as uniform
    float uniformw = 1.0/(float)NPARTICLES;
    for (unsigned int p = 0; p < NPARTICLES; p++) {
        particles[p].setW(uniformw);
    }

    VectorXf xtrue(3);
    xtrue.setZero();

    float dt = DT_CONTROLS; //change in time btw predicts
    float dtsum = 0; //change in time since last observation

    vector<int> ftag; //identifier for each landmark
    for (int i=0; i< lm.cols(); i++) {
        ftag.push_back(i);
    }

    //data ssociation table
    VectorXf da_table(lm.cols());
    for (int s=0; s<da_table.size(); s++) {
        da_table[s] = -1;
    }

    int iwp = 0; //index to first waypoint
    float G = 0; //initial steer angle
    MatrixXf plines; //will later change to list of points

    if (SWITCH_SEED_RANDOM !=0) {
        srand(SWITCH_SEED_RANDOM);
    }

    MatrixXf Qe = MatrixXf(Q);
    MatrixXf Re = MatrixXf(R);

    if (SWITCH_INFLATE_NOISE ==1) {
        Qe = 2*Q;
        Re = 2*R;
    }

    vector<int>         ftag_visible;
    vector<VectorXf>    z; //range and bearings of visible landmarks

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

    //Main loop
    while (iwp !=-1) {
        //printf("[%7d]\n", pos_i);

        compute_steering(xtrue, wp, iwp, AT_WAYPOINT, G, RATEG, MAXG, dt);
        if (iwp ==-1 && NUMBER_LOOPS > 1) {
            iwp = 0;
            NUMBER_LOOPS = NUMBER_LOOPS-1;
        }
        predict_true(xtrue, V, G, WHEELBASE, dt);

        //add process noise
        add_control_noise(V,G,Q,SWITCH_CONTROL_NOISE,VnGn);
        Vn = VnGn[0];
        Gn = VnGn[1];

        //Predict step
        for (i=0; i< NPARTICLES; i++) {
            predict(particles[i], Vn, Gn, Qe, WHEELBASE, dt, SWITCH_PREDICT_NOISE);
            if (SWITCH_HEADING_KNOWN) {
                for (int j=0; j< particles[i].xf().size(); j++) {
                    VectorXf xf_j = particles[i].xf()[j];
                    xf_j[2] = xtrue[2];
                    particles[i].setXfi(j,xf_j);
                }
            }
        }

        //Observe step
        dtsum = dtsum+dt;
        if (dtsum >= DT_OBSERVE) {
            dtsum=0;

            //Compute true data, then add noise
            ftag_visible = vector<int>(ftag); //modify the copy, not the ftag

            //z is the range and bearing of the observed landmark
            z = get_observations(xtrue,lm,ftag_visible,MAX_RANGE);
            add_observation_noise(z,R,SWITCH_SENSOR_NOISE);

            plines = make_laser_lines(z, xtrue);

            //Compute (known) data associations
            int Nf = particles[0].xf().size();
            vector<int>         idf;
            vector<VectorXf>    zf;
            vector<VectorXf>    zn;

            data_associate_known(z,ftag_visible,da_table,Nf,zf,idf,zn);

            //perform update
            for (int i =0; i<NPARTICLES; i++) {
                if (!zf.empty()) { //observe map features
                    float w = compute_weight(particles[i],zf,idf,R);
                    w = particles[i].w()*w;
                    particles[i].setW(w);
                    feature_update(particles[i],zf,idf,R);
                }
                if (!zn.empty()) {
                    add_feature(particles[i], zn, R);
                }
            }

            resample_particles(particles, NEFFECTIVE, SWITCH_RESAMPLE);
        }

        // get mean x, y
        x_mean = 0; y_mean = 0;  t_mean = 0;
        w_max = -1e30;
        for(i=0; i<NPARTICLES; i++) {
            if( particles[i].w() > w_max ) {
                w_max = particles[i].w();
                t_mean = particles[i].xv()(2);
            }
            x_mean += particles[i].xv()(0);
            y_mean += particles[i].xv()(1);
            //t_mean += pi_to_pi(particles[i].xv()(2));
        }

        x_mean = x_mean / NPARTICLES;
        y_mean = y_mean / NPARTICLES;
        //t_mean = t_mean / NPARTICLES;
        //printf("   x, y, t = %f %f %f\n", x_mean, y_mean, t_mean);

        // update status bar
        time_all = time_all + dt;
        pos_i ++;
        msgAll.sprintf("[%6d] %7.3f", pos_i, time_all);
        emit showMessage(msgAll);

        // Draw particles
        arrParticles_x.clear();
        arrParticles_y.clear();
        for(i=0; i<NPARTICLES; i++) {
            arrParticles_x.push_back( particles[i].xv()(0) );
            arrParticles_y.push_back( particles[i].xv()(1) );
        }
        g_plot->setParticles(arrParticles_x, arrParticles_y);

        // Draw feature particles
        arrParticlesFea_x.clear();
        arrParticlesFea_y.clear();
        for(i=0; i<NPARTICLES; i++) {
            for(j=0; j<particles[i].xf().size(); j++ ) {
                arrParticlesFea_x.push_back( particles[i].xf()[j](0) );
                arrParticlesFea_y.push_back( particles[i].xf()[j](1) );
            }
        }
        g_plot->setParticlesFea(arrParticlesFea_x, arrParticlesFea_y);

        // add new position
        if( pos_i % 10 == 0 ) {
            g_plot->addPos(xtrue(0), xtrue(1));
            g_plot->addPosEst(x_mean, y_mean);
        }

        // draw current position
        g_plot->setCarPos(xtrue(0), xtrue(1), xtrue(2));
        g_plot->setCarPos(x_mean, y_mean, t_mean, 1);

        // set laser lines
        g_plot->setLaserLines(plines);

        emit replot();

        msleep(10);

        // break main loop
        if( !isAlive ) break;
    }

    if (VnGn) {
        delete[] VnGn;
    }

    //cout<<"done with all functions and will return particles"<<endl<<flush;
    return particles;
}


void FastSLAM1_Thread::predict(Particle &particle, float V, float G,
                               MatrixXf &Q, float WB,float dt, int addrandom)
{
    //optional: add random noise to predicted state
    if (addrandom ==1) {
        VectorXf A(2);
        A(0) = V;
        A(1) = G;
        VectorXf VG(2);
        VG = multivariate_gauss(A,Q,1);
        V = VG(0);
        G = VG(1);
    }

    //predict state
    VectorXf xv = particle.xv();
    VectorXf xv_temp(3);
    xv_temp <<  xv(0) + V*dt*cos(G+xv(2)),
                xv(1) + V*dt*sin(G+xv(2)),
                pi_to_pi2(xv(2) + V*dt*sin(G/WB));
    particle.setXv(xv_temp);
}



//
//compute particle weight for sampling
//
float FastSLAM1_Thread::compute_weight(Particle &particle, vector<VectorXf> &z, vector<int> &idf, MatrixXf &R)
{
    vector<MatrixXf> Hv;
    vector<MatrixXf> Hf;
    vector<MatrixXf> Sf;
    vector<VectorXf> zp;

    //process each feature, incrementally refine proposal distribution
    compute_jacobians(particle,idf,R,zp,&Hv,&Hf,&Sf);

    vector<VectorXf> v;

    for (int j =0; j<z.size(); j++) {
        VectorXf v_j = z[j] - zp[j];
        v_j[1] = pi_to_pi(v_j[1]);
        v.push_back(v_j);
    }

    float w = 1.0;

    MatrixXf S;
    float den, num;
    for (int i=0; i<z.size(); i++) {
        S = Sf[i];
        den = 2*M_PI*sqrt(S.determinant());
        num = std::exp(-0.5 * v[i].transpose() * S.inverse() * v[i]);
        w = w*num/den;
    }
    return w;
}

