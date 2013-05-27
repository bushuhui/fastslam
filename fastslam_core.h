#ifndef __FASTSLAM_CORE_H__
#define __FASTSLAM_CORE_H__

#include <iostream>
#include <vector>
#include <string>

#include <Eigen/Dense>


using namespace std;
using namespace Eigen;


////////////////////////////////////////////////////////////////////////////////
// particle class
////////////////////////////////////////////////////////////////////////////////
class Particle {
public:
	Particle();
	Particle(float &w, VectorXf &xv, MatrixXf &Pv, vector<VectorXf> &xf, vector<MatrixXf> &Pf, float* da);
	~Particle();
        
	//getters	
    float &w();
    VectorXf &xv();             //robot pose: x,y,theta (heading dir)
    MatrixXf &Pv();             //controls: velocities
    vector<VectorXf> &xf();     //2d means of EKF
    vector<MatrixXf> &Pf();     //covariance matrices for EKF
    float *da();                //

	//setters
    void setW(float w);
	void setXv(VectorXf &xv);
	void setPv(MatrixXf &Pv);
	void setXf(vector<VectorXf> &xf);
	void setXfi(int i, VectorXf &vec);
	void setPf(vector<MatrixXf> &Pf);
	void setPfi(int i, MatrixXf &m);
	void setDa(float* da);
	
private:
	float _w;
	VectorXf _xv;
	MatrixXf _Pv;		
	vector<VectorXf> _xf;
	vector<MatrixXf> _Pf;
	float* _da;
};

////////////////////////////////////////////////////////////////////////////////
// fastslam config Variables
////////////////////////////////////////////////////////////////////////////////
namespace config {
    extern float V;
    extern float MAXG;
    extern float RATEG;
    extern int WHEELBASE;
    extern float DT_CONTROLS;

    extern float sigmaV;
    extern float sigmaG;

    extern Eigen::MatrixXf Q;

    extern float MAX_RANGE;
    extern float DT_OBSERVE;

    extern float sigmaR;
    extern float sigmaB;

    extern Eigen::MatrixXf R;

    extern float AT_WAYPOINT;
    extern int NUMBER_LOOPS;

    extern unsigned int NPARTICLES;
    extern float NEFFECTIVE;

    extern int SWITCH_CONTROL_NOISE;
    extern int SWITCH_SENSOR_NOISE;
    extern int SWITCH_INFLATE_NOISE;
    extern int SWITCH_PREDICT_NOISE;

    extern int SWITCH_SAMPLE_PROPOSAL;
    extern int SWITCH_HEADING_KNOWN;
    extern int SWITCH_RESAMPLE;
    extern int SWITCH_PROFILE;
    extern int SWITCH_SEED_RANDOM;
}

////////////////////////////////////////////////////////////////////////////////
// Functions
////////////////////////////////////////////////////////////////////////////////
void compute_jacobians(Particle &particle,
                        vector<int> &idf,
                        MatrixXf &R,
                        vector<VectorXf> &zp,
						vector<MatrixXf> *Hv, 
                        vector<MatrixXf> *Hf, 
                        vector<MatrixXf> *Sf);
                        
void KF_cholesky_update(VectorXf &x,MatrixXf &P,VectorXf &v,MatrixXf &R,MatrixXf &H);

void KF_joseph_update(VectorXf &x,MatrixXf &P,float v,float R, MatrixXf &H);
MatrixXf make_symmetric(MatrixXf &P);

void resample_particles(vector<Particle> &particles, int Nmin, int doresample); 
void stratified_resample(VectorXf w, vector<int> &keep, float &Neff);
void cumsum(VectorXf &w);

void predict_true(VectorXf &xv,float V,float G,int WB,float dt); 

void compute_steering(VectorXf &x, MatrixXf &wp, int &iwp, float minD,
                        float &G, float rateG, float maxG, float dt);

void data_associate_known(vector<VectorXf> &z, vector<int> &idz, VectorXf &table, int Nf, \
						  vector<VectorXf> &zf, vector<int> &idf, vector<VectorXf> &zn); 
						  
void add_feature(Particle &particle, vector<VectorXf> &z, MatrixXf &R);
void feature_update(Particle &particle, vector<VectorXf> &z, vector<int> &idf, MatrixXf &R);

vector<VectorXf> get_observations(VectorXf &x, MatrixXf lm, vector<int> &idf,float rmax);
void get_visible_landmarks(VectorXf &x, MatrixXf &lm, vector<int> &idf, float rmax);
vector<VectorXf> compute_range_bearing(VectorXf &x, MatrixXf &lm);
vector<int> find2(vector<float> &dx, vector<float> &dy, float phi, float rmax);

void add_control_noise(float V, float G, MatrixXf &Q, int addnoise,float *VnGn);
void add_observation_noise(vector<VectorXf> &z, MatrixXf &R, int addnoise);



void pi_to_pi(VectorXf &angle); //takes in array of floats, returna array 
float pi_to_pi(float ang);
float pi_to_pi2(float ang);


void read_slam_input_file(const string s, MatrixXf *lm, MatrixXf *wp);


void TransformToGlobal(MatrixXf &p, VectorXf &b);
MatrixXf make_laser_lines(vector<VectorXf> &rb, VectorXf &xv);
MatrixXf line_plot_conversion(MatrixXf &lnes);


void stratified_random(int N, vector<float> &di);
double unifRand();

VectorXf multivariate_gauss(VectorXf &x, MatrixXf &P, int n);

namespace nRandMat{
	MatrixXf randn(int m, int n);
	MatrixXf rand(int m, int n); 
}


#endif // end of __FASTSLAM_CORE_H__

