
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>

#include <string>
#include <vector>
#include <algorithm>
#include <iterator>

#include <iostream>
#include <fstream>

#include <Eigen/Dense>

#include "fastslam_core.h"

using namespace std;
using namespace Eigen;


void add_control_noise(float V, float G, MatrixXf &Q, int addnoise, float *VnGn)
{
	if (addnoise ==1) {
		VectorXf A(2);
		A(0) = V;
		A(1) = G;
		VectorXf C(2);
		C = multivariate_gauss(A,Q,1);
		VnGn[0] = C(0);
		VnGn[1] = C(1);
	}
}

//
// add new features
//
void add_feature(Particle &particle, vector<VectorXf> &z, MatrixXf &R)
{
    int lenz = z.size();
    vector<VectorXf> xf;
	vector<MatrixXf> Pf;
    VectorXf xv = particle.xv();

    float r,b,s,c;
    MatrixXf Gz(2,2);

    for (int i=0; i<lenz; i++) {
        r = z[i][0];
        b = z[i][1];
        s = sin(xv(2)+b);
        c = cos(xv(2)+b);
		
		VectorXf measurement(2);
		measurement(0) = xv(0) + r*c;
		measurement(1) = xv(1) + r*s;
 		xf.push_back(measurement);
        Gz <<c,-r*s,s,r*c;

        Pf.push_back(Gz*R*Gz.transpose()); 	
    }

    int lenx = particle.xf().size();
    vector<int> ii;
    for (int i=lenx; i<lenx+lenz; i++) {
        ii.push_back(i);
    }	

	for(int j=0; j<ii.size(); j++) {
		particle.setXfi(ii[j],xf[j]);
	}

	for(int i=0; i<ii.size(); i++) {
		particle.setPfi(ii[i],Pf[i]);
	}
}


//http://moby.ihme.washington.edu/bradbell/mat2cpp/randn.cpp.xml
MatrixXf nRandMat::randn(int m, int n) 
{	
	// use formula 30.3 of Statistical Distributions (3rd ed)
	// Merran Evans, Nicholas Hastings, and Brian Peacock
	int urows = m * n + 1;
    VectorXf u(urows);

	//u is a random matrix
	for (int r=0; r<urows; r++) {
        // FIXME: better way?
        u(r) = std::rand() * 1.0/RAND_MAX;
	}
	
	MatrixXf x(m,n);

    int     i, j, k;
    float   square, amp, angle;

	k = 0;
    for(i = 0; i < m; i++) {
        for(j = 0; j < n; j++) {
            if( k % 2 == 0 ) {
                square = - 2. * std::log( u(k, 0) );
				if( square < 0. )
					square = 0.;
				amp = sqrt(square);
                angle = 2. * M_PI * u(k+1, 0);
				x(i, j) = amp * std::sin( angle );
			}
			else	
				x(i, j) = amp * std::cos( angle );
			k++;
		}
	}

	return x;
}

MatrixXf nRandMat::rand(int m, int n) 
{	
	MatrixXf x(m,n);	
	int i, j;
	float rand_max = float(RAND_MAX);

	for(i = 0; i < m; i++) {	
		for(j = 0; j < n; j++)
			x(i, j) = float(std::rand()) / rand_max;
	} 
	return x;
}

//add random measurement noise. We assume R is diagnoal matrix
void add_observation_noise(vector<VectorXf> &z, MatrixXf &R, int addnoise)
{
	if (addnoise == 1){
		int len = z.size();	
		if (len > 0) {
			MatrixXf randM1 = nRandMat::randn(1,len);
			MatrixXf randM2 = nRandMat::randn(1,len);

			for (int c=0; c<len; c++) {
				z[c][0] = z[c][0] + randM1(0,c)*sqrt(R(0,0));
				z[c][1] = z[c][1] + randM2(0,c)*sqrt(R(1,1));
			}
		}
	}	
}


void compute_jacobians(
        Particle &particle,
        vector<int> &idf,
        MatrixXf &R,
        vector<VectorXf> &zp,   // measurement (range, bearing)
        vector<MatrixXf> *Hv,   // jacobians of function h (deriv of h wrt pose)
        vector<MatrixXf> *Hf,   // jacobians of function h (deriv of h wrt mean)
        vector<MatrixXf> *Sf)   // measurement covariance
{
	VectorXf xv = particle.xv();

	vector<VectorXf> xf;
	vector<MatrixXf> Pf;

    unsigned int i;
	for (unsigned i=0; i<idf.size(); i++) {
		xf.push_back(particle.xf()[idf[i]]);
		Pf.push_back((particle.Pf())[idf[i]]); //particle.Pf is a array of matrices
	}

	float dx,dy,d2,d;
	MatrixXf HvMat(2,3);
	MatrixXf HfMat (2,2);

	for (i=0; i<idf.size(); i++) {
		dx = xf[i](0) - xv(0);
		dy = xf[i](1) - xv(1);
		d2 = pow(dx,2) + pow(dy,2);	
		d = sqrt(d2);

		VectorXf zp_vec(2);
		
		//predicted observation
		zp_vec[0] = d;
        zp_vec[1] = pi_to_pi(atan2(dy,dx) - xv(2));
		zp.push_back(zp_vec);
		
		//Jacobian wrt vehicle states
		HvMat<< -dx/d, -dy/d,  0, 
                dy/d2, -dx/d2,-1;

		//Jacobian wrt feature states
        HfMat<< dx/d,   dy/d,
                -dy/d2, dx/d2;

		Hv->push_back(HvMat);
		Hf->push_back(HfMat);

		//innovation covariance of feature observation given the vehicle'
		MatrixXf SfMat = HfMat*Pf[i]*HfMat.transpose() + R; 
		Sf->push_back(SfMat);      
	}			
}

void compute_steering(VectorXf &x, MatrixXf &wp, int &iwp, float minD,
                      float &G, float rateG, float maxG, float dt)
{
    /*
        % INPUTS:
        %   x - true position
        %   wp - waypoints
        %   iwp - index to current waypoint
        %   minD - minimum distance to current waypoint before switching to next
        %   G - current steering angle
        %   rateG - max steering rate (rad/s)
        %   maxG - max steering angle (rad)
        %   dt - timestep
    */

    //determine if current waypoint reached
    Vector2d cwp;
    cwp[0] = wp(0,iwp); //-1 since indexed from 0
    cwp[1] = wp(1,iwp);

    float d2 = pow((cwp[0] - x[0]),2) + pow((cwp[1]-x[1]),2);

    if (d2 < minD*minD) {
        iwp++; //switch to next
        if (iwp >= wp.cols()) {
            iwp =-1;
            return;
        }

        cwp[0] = wp(0,iwp); //-1 since indexed from 0
        cwp[1] = wp(1,iwp);
    }

    //compute change in G to point towards current waypoint
    float deltaG = atan2(cwp[1]-x[1], cwp[0]-x[0]) - x[2] - G;
    deltaG = pi_to_pi(deltaG);

    //limit rate
    float maxDelta = rateG*dt;
    if (abs(deltaG) > maxDelta) {
        int sign = (deltaG > 0) ? 1 : ((deltaG < 0) ? -1 : 0);
        deltaG = sign*maxDelta;
    }

    //limit angle
    G = G+deltaG;
    if (abs(G) > maxG) {
        int sign2 = (G > 0) ? 1: ((G < 0) ? -1 : 0);
        G = sign2*maxG;
    }
}


//z is range and bearing of visible landmarks
// find associations (zf) and new features (zn)
void data_associate_known(vector<VectorXf> &z, vector<int> &idz, VectorXf &table, int Nf, \
                          vector<VectorXf> &zf, vector<int> &idf, vector<VectorXf> &zn)
{
    idf.clear();
    vector<int> idn;

    unsigned i,ii,r;

    for (i =0; i< idz.size(); i++) {
        ii = idz[i];
        VectorXf z_i;
        if (table(ii) ==-1) { //new feature
            z_i = z[i];
            zn.push_back(z_i);
            idn.push_back(ii);
        }
        else {
            z_i = z[i];
            zf.push_back(z_i);
            idf.push_back(table(ii));
        }
    }

    assert(idn.size() == zn.size());
    for (int i=0; i<idn.size(); i++) {
        table(idn[i]) = Nf+i;
    }
}


//z is the list of measurements conditioned on the particle.
void feature_update(Particle &particle, vector<VectorXf> &z, vector<int> &idf, MatrixXf &R)
{
    //Having selected a new pose from the proposal distribution, this pose is assumed perfect and each feature update maybe computed independently and without pose uncertainty
    vector<VectorXf> xf;    //updated EKF means
    vector<MatrixXf> Pf;    //updated EKF covariances

	for (unsigned i=0; i<idf.size(); i++) {
		xf.push_back(particle.xf()[idf[i]]); //means
		Pf.push_back(particle.Pf()[idf[i]]); //covariances
	}	
	
	vector<VectorXf> zp;
    vector<MatrixXf> Hv;
    vector<MatrixXf> Hf;
    vector<MatrixXf> Sf;
    
	compute_jacobians(particle,idf,R,zp,&Hv,&Hf,&Sf);

	vector<VectorXf> v; //difference btw two measurements (used to update mean)
	for (int i=0; i<z.size(); i++) {
		VectorXf measure_diff = z[i] - zp[i];
		measure_diff[1] = pi_to_pi(measure_diff[1]);
		v.push_back(measure_diff);
	}

    VectorXf vi; 
    MatrixXf Hfi;
    MatrixXf Pfi;
    VectorXf xfi; 

	for (int i=0; i<idf.size(); i++) {
		vi = v[i];
		Hfi = Hf[i];
		Pfi = Pf[i];
		xfi = xf[i];
        KF_cholesky_update(xfi,Pfi,vi,R,Hfi);
		xf[i] = xfi;
		Pf[i] = Pfi;
	}

	for (int i=0; i<idf.size(); i++) {
		particle.setXfi(idf[i],xf[i]);
		particle.setPfi(idf[i],Pf[i]);
	}
}

vector<VectorXf> get_observations(VectorXf &x, MatrixXf lm, vector<int> &idf, float rmax)
{
	get_visible_landmarks(x,lm,idf,rmax);
	return compute_range_bearing(x,lm);	
}

void get_visible_landmarks(VectorXf &x, MatrixXf &lm, vector<int> &idf, float rmax)
{
	//select set of landmarks that are visible within vehicle's 
	//semi-circular field of view
	vector<float> dx;
	vector<float> dy;

	for (int c=0; c<lm.cols(); c++) {
		dx.push_back(lm(0,c) - x(0));
		dy.push_back(lm(1,c) - x(1));
	}

	float phi = x(2);

	//distant points are eliminated
	vector<int> ii = find2(dx,dy,phi,rmax); 

	MatrixXf lm_new (lm.rows(), ii.size());
	unsigned j,k;
	for (j=0; j<lm.rows(); j++){
		for(k=0; k< ii.size(); k++){
			lm_new(j,k) = lm(j,ii[k]);
		}
	}
	lm = MatrixXf(lm_new); 

	vector<int> idf_backup(idf);
	idf.clear();
	for(int i=0; i<ii.size(); i++) {
		idf.push_back(idf_backup[ii[i]]);
	}
}

vector<VectorXf> compute_range_bearing(VectorXf &x, MatrixXf &lm)
{
	vector<float> dx; 
	vector<float> dy;

	for (int c=0; c<lm.cols(); c++) {
		dx.push_back(lm(0,c) - x(0));
		dy.push_back(lm(1,c) - x(1));
	}	

	assert(dx.size() == lm.cols());
	assert(dy.size() == lm.cols());

	float phi = x(2);
	vector<VectorXf> z;

	for (int i =0; i<lm.cols(); i++) {
		VectorXf zvec(2);
		zvec<< sqrt(pow(dx[i],2) + pow(dy[i],2)), atan2(dy[i],dx[i]) - phi;	
		z.push_back(zvec);
	}

	return z; 
}

vector<int> find2(vector<float> &dx, vector<float> &dy, float phi, float rmax)
{
	vector<int> index;
	//incremental tests for bounding semi-circle
	for (int j=0; j<dx.size(); j++) {
		if ((abs(dx[j]) < rmax) && (abs(dy[j]) < rmax)
				&& ((dx[j]* cos(phi) + dy[j]* sin(phi)) > 0.0)
				&& (((float)pow(dx[j],2) + (float)pow(dy[j],2)) < (float)pow(rmax,2))){
			index.push_back(j);			
		}
	}
	return index;				
}

void KF_cholesky_update(VectorXf &x, MatrixXf &P,VectorXf &v,MatrixXf &R,MatrixXf &H)
{
    MatrixXf PHt = P*H.transpose();
    MatrixXf S = H*PHt + R;
    
    S = (S+S.transpose()) * 0.5; //make symmetric
    MatrixXf SChol = S.llt().matrixL();
    SChol.transpose();
    SChol.conjugate();

    MatrixXf SCholInv = SChol.inverse(); //tri matrix
    MatrixXf W1 = PHt * SCholInv;
    MatrixXf W = W1 * SCholInv.transpose();

    x = x + W*v;
    P = P - W1*W1.transpose();
}


void KF_joseph_update(VectorXf &x, MatrixXf &P,float v,float R, MatrixXf &H)
{
    VectorXf PHt = P*H.transpose();
    MatrixXf S = H*PHt;
    S(0,0) += R;
    MatrixXf Si = S.inverse();
    //Si = make_symmetric(Si);
    make_symmetric(Si);
    MatrixXf PSD_check = Si.llt().matrixL(); //chol of scalar is sqrt
    PSD_check.transpose();
    PSD_check.conjugate();

    VectorXf W = PHt*Si;
    x = x+W*v;
    
    //Joseph-form covariance update
    MatrixXf eye(P.rows(), P.cols());
    eye.setIdentity();
    MatrixXf C = eye - W*H;
    P = C*P*C.transpose() + W*R*W.transpose();  

    float eps = 2.2204*pow(10.0,-16); //numerical safety 
    P = P+eye*eps;

    PSD_check = P.llt().matrixL();
    PSD_check.transpose();
    PSD_check.conjugate(); //for upper tri
}

MatrixXf make_symmetric(MatrixXf &P)
{
    return (P + P.transpose())*0.5;
}


MatrixXf line_plot_conversion(MatrixXf &lnes)
{
	int len = lnes.cols()*3 -1;			
	MatrixXf p(2,len);

	for (int j=0; j<len; j+=3) {
		int k = floor((j+1)/3); //reverse of len calculation
		p(0,j) = lnes(0,k);
		p(1,j) = lnes(1,k);
		p(0,j+1) = lnes(2,k);
		p(1,j+1) = lnes(3,k);
		if (j+2 <len) {
			p(0,j+2) = NULL;
			p(1,j+2) = NULL;
		}
	}
	return p;
}

//rb is measurements
//xv is robot pose
MatrixXf make_laser_lines(vector<VectorXf> &rb, VectorXf &xv)
{
    if ( rb.empty() ) {
        return MatrixXf(0,0);
    }

    int len = rb.size();
    MatrixXf lnes(4,len);

    MatrixXf globalMat(2,rb.size());
    int j;
    for (j=0; j<globalMat.cols();j++) {
        globalMat(0,j) = rb[j][0]*cos(rb[j][1]);
        globalMat(1,j) = rb[j][0]*sin(rb[j][1]);
    }

    TransformToGlobal(globalMat, xv);

    for (int c=0; c<lnes.cols();c++) {
        lnes(0,c) = xv(0);
        lnes(1,c) = xv(1);
        lnes(2,c) = globalMat(0,c);
        lnes(3,c) = globalMat(1,c);
    }

    //return line_plot_conversion(lnes);
    return lnes;
}


// FIXME: this function not correct!
VectorXf multivariate_gauss(VectorXf &x, MatrixXf &P, int n)
{
	int len = x.size();
	//choleksy decomposition
	MatrixXf S = P.llt().matrixL();
    MatrixXf X;
	X = nRandMat::randn(len,n);	
    	
	//VectorXf ones = //VectorXf::Ones(n).transpose();
	MatrixXf ones = MatrixXf::Ones(1,n);	
	return S*X + x*ones;
}


void pi_to_pi(VectorXf &angle) 
{
    int n;
    for (int i=0; i<angle.size(); i++) {
        if ((angle[i] < (-2*M_PI)) || (angle[i] > (2*M_PI))) {
            n=floor(angle[i]/(2*M_PI));
            angle[i] = angle[i]-n*(2*M_PI);
        }

        if (angle[i] > M_PI) {
            angle[i] = angle[i] - (2*M_PI);
        }
        if (angle[i] < -M_PI) {
            angle[i] = angle[i] + (2*M_PI);
        }

    }
}

float pi_to_pi(float ang) 
{
    int n;
    if ( (ang < -2*M_PI) || (ang > 2*M_PI) ) {
        n=floor(ang/(2*M_PI));
        ang = ang-n*(2*M_PI);
    }

    if (ang > M_PI) {
        ang = ang - (2*M_PI);
    }
    if (ang < -M_PI) {
        ang = ang + (2*M_PI);
    }


    return ang;
}

float pi_to_pi2(float ang)
{
    if (ang > M_PI) {
        ang = ang - (2*M_PI);
    }
    if (ang < -M_PI) {
        ang = ang + (2*M_PI);
    }

    return ang;
}


void read_slam_input_file(const string s, MatrixXf *lm, MatrixXf *wp)
{
    if(access(s.c_str(), R_OK) == -1) {
        std::cerr << "Unable to read input file" << s << std::endl;
        exit(EXIT_FAILURE);
    }

    ifstream in(s.c_str());

    int lineno = 0;
    int lm_rows =0;
    int lm_cols =0;
    int wp_rows =0;
    int wp_cols =0;

    while(in) {
        lineno++;
        string str;
        getline(in,str);
        istringstream line(str);

        vector<string> tokens;
        copy(istream_iterator<string>(line),
             istream_iterator<string>(),
             back_inserter<vector<string> > (tokens));

        if(tokens.size() ==0) {
            continue;
        }
        else if (tokens[0][0] =='#') {
            continue;
        }
        else if (tokens[0] == "lm") {
            if(tokens.size() != 3) {
                std::cerr<<"Wrong args for lm!"<<std::endl;
                std::cerr<<"Error occuredon line"<<lineno<<std::endl;
                std::cerr<<"line:"<<str<<std::endl;
                exit(EXIT_FAILURE);
            }
            lm_rows = strtof(tokens[1].c_str(),NULL);
            lm_cols = strtof(tokens[2].c_str(),NULL);

            lm->resize(lm_rows,lm_cols);
            for (int c =0; c<lm_cols; c++) {
                lineno++;
                if (!in) {
                    std::cerr<<"EOF after reading" << std::endl;
                    exit(EXIT_FAILURE);
                }
                getline(in,str);
                istringstream line(str);
                vector<string> tokens;
                copy(istream_iterator<string>(line),
                     istream_iterator<string>(),
                     back_inserter<vector<string> > (tokens));
                if(tokens.size() < lm_rows) {
                    std::cerr<<"invalid line for lm coordinate!"<<std::endl;
                    std::cerr<<"Error occured on line "<<lineno<<std::endl;
                    std::cerr<<"line: "<<str<<std::endl;
                    exit(EXIT_FAILURE);
                }

                for (unsigned r=0; r< lm_rows; r++) {
                    (*lm)(r,c) = strtof(tokens[r].c_str(),NULL);
                }
            }
        }
        else if (tokens[0] == "wp") {
            if(tokens.size() != 3) {
                std::cerr<<"Wrong args for wp!"<<std::endl;
                std::cerr<<"Error occured on line"<<lineno<<std::endl;
                std::cerr<<"line:"<<str<<std::endl;
                exit(EXIT_FAILURE);
            }
            wp_rows = strtof(tokens[1].c_str(),NULL);
            wp_cols = strtof(tokens[2].c_str(),NULL);
            wp->resize(wp_rows, wp_cols);
            for (int c =0; c<wp_cols; c++) {
                lineno++;
                if (!in) {
                    std::cerr<<"EOF after reading" << std::endl;
                    exit(EXIT_FAILURE);
                }
                getline(in,str);
                istringstream line(str);
                std::vector<string> tokens;
                copy(istream_iterator<string>(line),
                     istream_iterator<string>(),
                     back_inserter<std::vector<string> > (tokens));
                if(tokens.size() < wp_rows) {
                    std::cerr<<"invalid line for wp coordinate!"<<std::endl;
                    std::cerr<<"Error occured on line "<<lineno<<std::endl;
                    std::cerr<<"line: "<<str<<std::endl;
                    exit(EXIT_FAILURE);
                }

                for (int r=0; r< lm_rows; r++) {
                    (*wp)(r,c) = strtof(tokens[r].c_str(),NULL);
                }
            }
        }
        else {
            std::cerr<<"Unkwown command"<<tokens[0] <<std::endl;
            std::cerr<<"Error occured on line"<<lineno<<std::endl;
            std::cerr<<"line: "<<str<<std::endl;
            exit(EXIT_FAILURE);
        }
    }
}



void predict_true(VectorXf &xv, float V, float G, int WB, float dt) 
{
	xv(0) = xv(0) + V*dt*cos(G+xv(2));		
	xv(1) = xv(1) + V*dt*sin(G+xv(2));
	xv(2) = pi_to_pi(xv(2) + V*dt*sin(G)/WB);		
}

void resample_particles(vector<Particle> &particles, int Nmin, int doresample) 
{
    int             i;
    unsigned int    N = particles.size();
    VectorXf        w(N);

    for (i=0; i<N; i++) {
        w(i) = particles[i].w();
    }

    float ws = w.sum();
    for (i=0; i<N; i++) {
        particles[i].setW(w(i)/ws);
    }

    float       Neff=0;
    vector<int> keep;

    stratified_resample(w, keep, Neff);

    vector<Particle> old_particles = vector<Particle>(particles);
    particles.resize(keep.size());	

    if ((Neff < Nmin) && (doresample == 1)) {
        for(i=0; i< keep.size(); i++) {
            particles[i] = old_particles[keep[i]]; 	
        }

        for (i=0; i<N; i++) {
			float new_w = 1.0f/(float)N;
            particles[i].setW(new_w);
        }
    }
}

void stratified_random(int N, vector<float> &di)
{ 
    float k = 1.0/(float)N;
   
    //deterministic intervals
    float temp = k/2;
    while (temp < (1-k/2)) {
        di.push_back(temp);
        temp = temp+k;
    }
   
    // FIXME: when set NPARTICLES = 30, this whill failed
    assert(di.size() == N); 
    
    //dither within interval
    vector<float>::iterator diter; 
    for (diter = di.begin(); diter != di.end(); diter++) {
        *diter = (*diter) + unifRand() * k - (k/2);
    }
}

//
// Generate a random number between 0 and 1
// return a uniform number in [0,1].
double unifRand()
{
    return rand() / double(RAND_MAX);
}

// FIXME: input w will be modified?
void stratified_resample(VectorXf w, vector<int> &keep, float &Neff)
{
    VectorXf    wsqrd(w.size());
    double      w_sum = w.sum();

    for (int i=0; i<w.size(); i++) {
        // FIXME: matlab version is: w = w / sum(w)
        w(i) = w(i)/w_sum;
        wsqrd(i) = pow(w(i),2);
    }
    Neff = 1/wsqrd.sum();

    int len = w.size();
    keep.resize(len);
    for (int i=0; i<len; i++) {
        keep[i] = -1;
    }

    vector<float> select;
    stratified_random(len, select);
    cumsum(w);    

    int ctr=0;
    for (int i=0; i<len; i++) {
        while ((ctr<len) && (select[ctr]<w(i))) {
            keep[ctr] = i;
            ctr++;
        }
    }
}

//
//returns a cumulative sum array
//
void cumsum(VectorXf &w) 
{
    VectorXf csumVec(w.size());
    for (int i=0; i< w.size(); i++) {
        float sum =0;
        for (int j=0; j<=i; j++) {
            sum+=w(j);
        }			
        csumVec(i) = sum;
    }

    w = VectorXf(csumVec); //copy constructor. Double check
}


void TransformToGlobal(MatrixXf &p, VectorXf &b)
{
	//rotate
	MatrixXf rot(2,2);
	rot<<cos(b(2)), -sin(b(2)), sin(b(2)), cos(b(2));
	
	MatrixXf p_resized;
	p_resized = MatrixXf(p);
	p_resized.conservativeResize(2,p_resized.cols());
	p_resized = rot*p_resized;		
	
	//translate
	int c;
	for (c=0;c<p_resized.cols();c++) {
		p(0,c) = p_resized(0,c)+b(0); 				
		p(1,c) = p_resized(1,c)+b(1); 				
	}

	float input;
	//if p is a pose and not a point
	if (p.rows() ==3){
		for (int k=0; k<p_resized.cols();k++) {
			input = p(2,k) +b(2);
   			p(2,k) = pi_to_pi(input);
		}		
	}	
}


////////////////////////////////////////////////////////////////////////////////
// particle class
////////////////////////////////////////////////////////////////////////////////
Particle::Particle() 
{
	_w = 1.0; 
	_xv = VectorXf(3);
        _xv.setZero();
	_Pv = MatrixXf(3,3);
        _Pv.setZero();
	_da = NULL;
}

Particle::Particle(float &w, VectorXf &xv, MatrixXf &Pv, vector<VectorXf> &xf, vector<MatrixXf> &Pf, float* da)
{
	_w = w;
	_xv = xv;
	_Pv = Pv;
	_xf = xf;
	_Pf = Pf;
	_da = da;		
}

Particle::~Particle()
{
}

//getters
float& Particle::w()
{
	return _w;	
}

VectorXf& Particle::xv()
{
	return _xv;	
}

MatrixXf& Particle::Pv()
{
	return _Pv;	
}

vector<VectorXf>& Particle::xf()
{
	return _xf;	
}

vector<MatrixXf>& Particle::Pf()
{
	return _Pf;	
}

float* Particle::da()
{
	return _da;	
}

//setters
void Particle::setW(float w)
{
	_w = w;
}

void Particle::setXv(VectorXf &xv)
{
	_xv = xv;
}

void Particle::setPv(MatrixXf &Pv)
{
	_Pv = Pv;
}

void Particle::setXf(vector<VectorXf> &xf)
{
	_xf = xf;
}

void Particle::setXfi(int i, VectorXf &vec) 
{
	if (i >= _xf.size()){
		_xf.resize(i+1);
	}
	_xf[i] = vec;
}

void Particle::setPf(vector<MatrixXf> &Pf)
{
	_Pf = Pf;
}

void Particle::setPfi(int i, MatrixXf &m) 
{
	if(i >= _Pf.size()) {
		_Pf.resize(i+1);
	}
	_Pf[i] = m;
}

void Particle::setDa(float* da)
{
	_da = da;
}



////////////////////////////////////////////////////////////////////////////////
// fastslam config Variables
//      Configuration file
//      Permits various adjustments to parameters of the FastSLAM algorithm.
//      See fastslam_sim.h for more information
////////////////////////////////////////////////////////////////////////////////

// control parameters
float config::V= 3.0; // m/s
float config::MAXG= 30*M_PI/180; // radians, maximum steering angle (-MAXG < g < MAXG)
float config::RATEG= 20*M_PI/180; // rad/s, maximum rate of change in steer angle
int config::WHEELBASE= 4; // metres, vehicle wheel-base
float config::DT_CONTROLS= 0.025; // seconds, time interval between control signals

// control noises
float config::sigmaV= 0.3; // m/s
float config::sigmaG= (3.0*M_PI/180); // radians

Eigen::MatrixXf config::Q(2,2);

// observation parameters
float config::MAX_RANGE= 30.0; // metres
float config::DT_OBSERVE= 8* config::DT_CONTROLS; // seconds, time interval between observations

// observation noises
float config::sigmaR= 0.1; // metres
float config::sigmaB= (1.0*M_PI/180); // radians

Eigen::MatrixXf config::R(2,2);

// waypoint proximity
float config::AT_WAYPOINT= 1.0; // metres, distance from current waypoint at which to switch to next waypoint
int config::NUMBER_LOOPS= 2; // number of loops through the waypoint list

// resampling
unsigned int config::NPARTICLES= 100;
float config::NEFFECTIVE= 0.75* config::NPARTICLES; // minimum number of effective particles before resampling

// switches
int config::SWITCH_CONTROL_NOISE= 1;
int config::SWITCH_SENSOR_NOISE = 1;
int config::SWITCH_INFLATE_NOISE= 0;
int config::SWITCH_PREDICT_NOISE = 0;   // sample noise from predict (usually 1 for fastslam1.0 and 0 for fastslam2.0)
int config::SWITCH_SAMPLE_PROPOSAL = 1; // sample from proposal (no effect on fastslam1.0 and usually 1 for fastslam2.0)
int config::SWITCH_HEADING_KNOWN= 0;
int config::SWITCH_RESAMPLE= 1; 
int config::SWITCH_PROFILE= 1;
int config::SWITCH_SEED_RANDOM= 0; // if not 0, seed the randn() with its value at beginning of simulation (for repeatability)

