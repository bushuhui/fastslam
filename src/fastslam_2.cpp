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
#include "fastslam_2.h"


using namespace std;
using namespace Eigen;

// global variable
extern SlamPlot     *g_plot;
extern SLAM_Conf    *g_conf;


FastSLAM2_Thread::FastSLAM2_Thread(QObject *parent) : SLAM_Thread(parent)
{
}

FastSLAM2_Thread::~FastSLAM2_Thread()
{
    wait();
}


void FastSLAM2_Thread::run()
{
    MatrixXf    lm; //landmark positions
    MatrixXf    wp; //way points

    printf("FastSLAM 2\n\n");

    read_slam_input_file(fnMap, &lm, &wp);

    vector<Particle> data = sim(lm, wp);
}


vector<Particle> FastSLAM2_Thread::sim(MatrixXf &lm, MatrixXf &wp)
{
    int         pos_i = 0;
    double      time_all;

    int         m, n;
    int         i;

    QString             msgAll;

    QVector<double>     arrParticles_x, arrParticles_y;
    QVector<double>     arrParticlesFea_x, arrParticlesFea_y;

    QVector<double>     arrWaypoints_x, arrWaypoints_y;
    QVector<double>     arrLandmarks_x, arrLandmarks_y;
    double              x_min, x_max, y_min, y_max;

    double              w_max;
    double              x_mean, y_mean, t_mean;

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

    if (g_conf->SWITCH_PREDICT_NOISE) {
        printf("Sampling from predict noise usually OFF for FastSLAM 2.0\n");
    }
    if (g_conf->SWITCH_SAMPLE_PROPOSAL == 0) {
        printf("Sampling from optimal proposal is usually ON for FastSLAM 2.0\n");
    }

    //normally initialized configfile.h
    Eigen::MatrixXf Q(2,2), R(2,2);
    Q << pow(g_conf->sigmaV,2), 0,
         0 , pow(g_conf->sigmaG,2);

    R << g_conf->sigmaR*g_conf->sigmaR, 0,
         0, g_conf->sigmaB*g_conf->sigmaB;


    //vector of particles (their count will change)
    vector<Particle> particles(g_conf->NPARTICLES);
    for (unsigned long i=0; i<particles.size(); i++) {
        particles[i] = Particle();
    }

    //initialize particle weights as uniform
    float uniformw = 1.0/(float)g_conf->NPARTICLES;
    for (int p = 0; p <g_conf->NPARTICLES; p++) {
        particles[p].setW(uniformw);
    }

    VectorXf xtrue(3);
    xtrue.setZero();

    float dt = g_conf->DT_CONTROLS; //change in time btw predicts
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

    int                 iwp = 0;    //index to first waypoint
    int                 nloop = g_conf->NUMBER_LOOPS;
    float               V = g_conf->V;
    float               G = 0;      //initial steer angle
    MatrixXf            plines;     //will later change to list of points

    vector<int>         ftag_visible;
    vector<VectorXf>    z; //range and bearings of visible landmarks

    if (g_conf->SWITCH_SEED_RANDOM !=0) {
        srand(g_conf->SWITCH_SEED_RANDOM);
    }

    MatrixXf Qe = MatrixXf(Q);
    MatrixXf Re = MatrixXf(R);

    if (g_conf->SWITCH_INFLATE_NOISE ==1) {
        Qe = 2*Q;
        Re = 2*R;
    }

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
                nloop--;
            }
        }
        if( runMode == SLAM_INTERACTIVE ) {
            getCommand(&cmd);

            // no commands then continue
            if( cmd == -1 ) continue;

            if( cmd == 1 ) {
                // forward
                V = V_ori;
                G = 0.0;
            } else if ( cmd == 2 ) {
                // backward
                V = -V_ori;
                G = 0.0;
            } else if ( cmd == 3 ) {
                // turn left
                V = V_ori;
                G = 30.0*M_PI/180.0;
            } else if ( cmd == 4 ) {
                // turn right
                V = V_ori;
                G = -30.0*M_PI/180.0;
            }
        }

        // predict current position and angle
        predict_true(xtrue, V, G, g_conf->WHEELBASE, dt);

        //add process noise
        add_control_noise(V, G, Q, g_conf->SWITCH_CONTROL_NOISE, VnGn);
        Vn = VnGn[0];
        Gn = VnGn[1];

        //Predict step
        for (i=0; i< g_conf->NPARTICLES; i++) {
            predict(particles[i],Vn,Gn,Qe,g_conf->WHEELBASE,dt,g_conf->SWITCH_PREDICT_NOISE);
            observe_heading(particles[i], xtrue(2), g_conf->SWITCH_HEADING_KNOWN); //if heading known, observe heading
        }

        //Observe step
        dtsum = dtsum+dt;
        if (dtsum >= g_conf->DT_OBSERVE) {
            dtsum=0;

            //Compute true data, then add noise
            ftag_visible = vector<int>(ftag); //modify the copy, not the ftag

            //z is the range and bearing of the observed landmark
            z = get_observations(xtrue, lm, ftag_visible, g_conf->MAX_RANGE);
            add_observation_noise(z, R, g_conf->SWITCH_SENSOR_NOISE);

            plines = make_laser_lines(z, xtrue);

            //Compute (known) data associations
            int Nf = particles[0].xf().size();
            vector<int>         idf;
            vector<VectorXf>    zf;
            vector<VectorXf>    zn;

            data_associate_known(z,ftag_visible,da_table,Nf,zf,idf,zn);

            //observe map features
            if (!zf.empty()) {
                //sample from 'optimal' proposal distribution, then update map
                //#pragma omp parallel for private(i)
                for (i=0; i<g_conf->NPARTICLES; i++) {
                    sample_proposal(particles[i], zf, idf, Re);
                    feature_update(particles[i], zf, idf, Re);
                }

                //resample
                resample_particles(particles, g_conf->NEFFECTIVE, g_conf->SWITCH_RESAMPLE);
            }

            //Observe new features, augment map
            if (!zn.empty()) {

                //#pragma omp parallel for private(i)
                for (i=0; i<g_conf->NPARTICLES; i++) {
                    if (zf.empty()) {//sample from proposal distribution if we have not already done so above
                        VectorXf xv = multivariate_gauss(particles[i].xv(),
                                                         particles[i].Pv(), 1);
                        particles[i].setXv(xv);
                        MatrixXf pv(3,3);
                        pv.setZero();
                        particles[i].setPv(pv);
                    }
                    add_feature(particles[i], zn, Re);
                }
            }
        }


        // update status bar
        time_all = time_all + dt;
        pos_i ++;

        // accelate simulation speed
        if( pos_i % draw_skip != 0 ) continue;

        msgAll.sprintf("[%6d] %7.3f", pos_i, time_all);
        emit showMessage(msgAll);

        // get mean x, y
        x_mean = 0; y_mean = 0;  t_mean = 0;
        w_max = -1e30;
        for(i=0; i<g_conf->NPARTICLES; i++) {
            if( particles[i].w() > w_max ) {
                w_max = particles[i].w();
                t_mean = particles[i].xv()(2);
            }
            x_mean += particles[i].xv()(0);
            y_mean += particles[i].xv()(1);
            //t_mean += pi_to_pi(particles[i].xv()(2));
        }

        x_mean = x_mean / g_conf->NPARTICLES;
        y_mean = y_mean / g_conf->NPARTICLES;
        //t_mean = t_mean / NPARTICLES;
        //printf("   x, y, t = %f %f %f\n", x_mean, y_mean, t_mean);

        // Draw particles
        arrParticles_x.clear();
        arrParticles_y.clear();
        for(i=0; i<g_conf->NPARTICLES; i++) {
            arrParticles_x.push_back( particles[i].xv()(0) );
            arrParticles_y.push_back( particles[i].xv()(1) );
        }
        g_plot->setParticles(arrParticles_x, arrParticles_y);

        // Draw feature particles
        arrParticlesFea_x.clear();
        arrParticlesFea_y.clear();
        for(i=0; i<g_conf->NPARTICLES; i++) {
            for(unsigned long j=0; j<particles[i].xf().size(); j++ ) {
                arrParticlesFea_x.push_back( particles[i].xf()[j](0) );
                arrParticlesFea_y.push_back( particles[i].xf()[j](1) );
            }
        }
        g_plot->setParticlesFea(arrParticlesFea_x, arrParticlesFea_y);

        // add new position
        if( pos_i % 4 == 0 ) {
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
    }

    delete[] VnGn;

    //cout<<"done with all functions and will return particles"<<endl<<flush;
    return particles;
}

void FastSLAM2_Thread::predict(Particle &particle,float V,float G,MatrixXf &Q, float WB,float dt, int addrandom)
{
    VectorXf xv = particle.xv();
    MatrixXf Pv = particle.Pv();
#if 0
    cout<<"particle.xv() in predict"<<endl;
    cout<<particle.xv()<<endl;
    cout<<"particle.Pv() in predict"<<endl;
    cout<<particle.Pv()<<endl;
#endif
    //Jacobians
    float phi = xv(2);
    MatrixXf Gv(3,3);

    Gv <<   1,0,-V*dt*sin(G+phi),
            0,1,V*dt*cos(G+phi),
            0,0,1;
    MatrixXf Gu(3,2);
    Gu <<   dt*cos(G+phi), -V*dt*sin(G+phi),
            dt*sin(G+phi), V*dt*cos(G+phi),
            dt*sin(G)/WB, V*dt*cos(G)/WB;

    //predict covariance
    MatrixXf newPv;//(Pv.rows(),Pv.cols());

    //TODO: Pv here is somehow corrupted. Probably in sample_proposal
#if 0
    cout<<"Pv in predict"<<endl;
    cout<<Pv<<endl;
#endif

    newPv = Gv*Pv*Gv.transpose() + Gu*Q*Gu.transpose();
    particle.setPv(newPv);

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
    VectorXf xv_temp(3);
    xv_temp << xv(0) + V*dt*cos(G+xv(2)),
            xv(1) + V*dt*sin(G+xv(2)),
            pi_to_pi2(xv(2) + V*dt*sin(G/WB));
    particle.setXv(xv_temp);
}


void FastSLAM2_Thread::observe_heading(Particle &particle, float phi, int useheading)
{
    if (useheading ==0){
        return;
    }

    float sigmaPhi = 0.01*M_PI/180.0;
    // cout<<"sigmaPhi "<<sigmaPhi<<endl;
    VectorXf xv = particle.xv();
    // cout<<"xv"<<endl;
    // cout<<xv<<endl;
    MatrixXf Pv = particle.Pv();
    // cout<<"Pv"<<endl;
    // cout<<Pv<<endl;

    MatrixXf H(1,3);
    H<<0,0,1;

    float v = pi_to_pi(phi-xv(2));
    // cout<<"v"<<endl;
    // cout<<v<<endl;
    KF_joseph_update(xv,Pv,v,pow(sigmaPhi,2),H);
    // cout<<"KF_Joseph = xv"<<endl;
    // cout<<xv<<endl;
    // cout<<"KF_Joseph = Pv"<<endl;
    // cout<<Pv<<endl;

    particle.setXv(xv);
    particle.setPv(Pv);
}

float FastSLAM2_Thread::gauss_evaluate(VectorXf &v, MatrixXf &S, int logflag)
{
    int D = v.size();
    MatrixXf Sc = S.llt().matrixL();

    //normalised innovation
    VectorXf nin = Sc.jacobiSvd(ComputeThinU | ComputeThinV).solve(v);

    int s;

    float E=0;
    for (s=0; s<nin.size(); s++) {
        nin(s) = pow(nin(s),2);
        E += nin(s);
    }
    E=-0.5*E;
    //Note: above is a fast way to compute sets of inner-products

    unsigned i;
    float C,w;
    unsigned  m = min(Sc.rows(), Sc.cols());

    if (logflag !=1) {
        float prod = 1;
        for (i=0; i<m; i++) {
            prod=prod*Sc(i,i); //multiply the diagonals
        }
        C = pow((2*M_PI),(D/2))*prod; //normalizing term(makes Gaussian hyper volume =1
        w = exp(E)/C; //likelihood
   } else {
        float sum=0;
        for (i=0; i<m; i++) {
            sum+=log(Sc(i,i));
        }
        C = 0.5*D*log(2*M_PI) + sum; //log of normalising term
        w = E-C; //log-likelihood
    }
    return w;
}


//
//compute particle weight for sampling
//
float FastSLAM2_Thread::compute_weight(Particle &particle, vector<VectorXf> &z, vector<int> &idf, MatrixXf &R)
{
    vector<MatrixXf> Hv;
    vector<MatrixXf> Hf;
    vector<MatrixXf> Sf;
    vector<VectorXf> zp;

    //process each feature, incrementally refine proposal distribution
    compute_jacobians(particle,idf,R,zp,&Hv,&Hf,&Sf);

    vector<VectorXf> v;

    for (unsigned long j =0; j<z.size(); j++) {
        VectorXf v_j = z[j] - zp[j];
        v_j[1] = pi_to_pi(v_j[1]);
        v.push_back(v_j);
    }

    float w = 1.0;

    MatrixXf S;
    float den, num;
    for (unsigned long i=0; i<z.size(); i++) {
        S = Sf[i];
        den = 2*M_PI*sqrt(S.determinant());
        num = std::exp(-0.5 * v[i].transpose() * S.inverse() * v[i]);
        w = w*num/den;
    }
    return w;
}

//compute proposal distribution, then sample from it, and compute new particle weight
//void sample_proposal(Particle &particle, MatrixXf z, vector<int> idf, MatrixXf R)
void FastSLAM2_Thread::sample_proposal(Particle &particle, vector<VectorXf> &z, vector<int> &idf, MatrixXf &R)
{
    VectorXf xv(particle.xv()); //robot position
    MatrixXf Pv(particle.Pv()); //controls (motion command)

    VectorXf xv0(xv);
    MatrixXf Pv0(Pv);

    vector<MatrixXf> Hv;
    vector<MatrixXf> Hf;
    vector<MatrixXf> Sf;

    // FIXME: z is std vector, get first item's rows?
    vector<VectorXf>    zp;
    MatrixXf            Hvi;
    MatrixXf            Hfi;
    MatrixXf            Sfi;

    // FIXME: z is std vector, get first item's rows?
    VectorXf vi(z[0].rows());

    //process each feature, incrementally refine proposal distribution
    unsigned long i;

    for (i =0; i<idf.size(); i++) {
        vector<int>     j;
        j.push_back(idf[i]);
        zp.clear();

        compute_jacobians(particle, j, R, zp, &Hv, &Hf, &Sf);
        //assert(zp.size() == 1);

        Hvi = Hv[0];
        Hfi = Hf[0];
        Sfi = Sf[0].inverse();

        vi = z[i] - zp[0];
        vi[1] = pi_to_pi(vi[1]);

        //proposal covariance
        MatrixXf Pv_inv = Pv.llt().solve(MatrixXf::Identity(Pv.rows(), Pv.cols()));
        Pv = Hvi.transpose() * Sfi * Hvi + Pv_inv; //Pv.inverse();
        Pv = Pv.llt().solve(MatrixXf::Identity(Pv.rows(), Pv.cols()));//Pv.inverse();

        //proposal mean
        xv = xv + Pv * Hvi.transpose() * Sfi *vi;
        particle.setXv(xv);
        particle.setPv(Pv);
    }

    //sample from proposal distribution
    VectorXf xvs = multivariate_gauss(xv, Pv, 1);
    particle.setXv(xvs);
    MatrixXf zeros(3,3);
    zeros.setZero();
    particle.setPv(zeros);

    //compute sample weight: w = w* p(z|xk) p(xk|xk-1) / proposal
    float       like, prior, prop, w;
    VectorXf    v1, v2;

    v1 = delta_xv(xv0,xvs);
    v2 = delta_xv(xv, xvs);

    like = likelihood_given_xv(particle, z, idf, R);
    prior = gauss_evaluate(v1, Pv0, 0);
    prop = gauss_evaluate(v2, Pv, 0);
    w = particle.w() * like * prior / prop;

    particle.setW(w);
}

float FastSLAM2_Thread::likelihood_given_xv(Particle &particle, vector<VectorXf> &z, vector<int> &idf, MatrixXf &R)
{
    float w = 1;
    vector<int> idfi;

    vector<MatrixXf> Hv;
    vector<MatrixXf> Hf;
    vector<MatrixXf> Sf;

    vector<VectorXf> zp;
    VectorXf v(z[0].rows());

    unsigned i,k;

    for (i=0; i<idf.size(); i++){
        idfi.clear();
        idfi.push_back(idf[i]);
        zp.clear();

        compute_jacobians(particle,idfi,R,zp,&Hv,&Hf,&Sf);

        for (k=0; k<z[0].rows(); k++) {
            v(k) = z[i][k]-zp[0][k];
        }
        v(1) = pi_to_pi(v(1));

        w = w*gauss_evaluate(v,Sf[0],0);
    }
    return w;
}

VectorXf FastSLAM2_Thread::delta_xv(VectorXf &xv1, VectorXf &xv2)
{
    //compute innovation between two xv estimates, normalising the heading component
    VectorXf dx = xv1-xv2;
    dx(2) = pi_to_pi(dx(2));
    return dx;
}

