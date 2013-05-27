
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <unistd.h>

#include <iostream>

#include <Eigen/Dense>

#include "fastslam_core.h"

using namespace std;
using namespace Eigen;

int test_multivariate_gauss(int argc, char *argv[])
{
    VectorXf x(2);
    MatrixXf P(2, 2);
    int n;

    VectorXf    res;

    x(0) = 3.000000000000000;
    x(1) = -0.008726646259972;

    P(0, 0) = 0.090000000000000; P(0, 1) = 0.0;
    P(1, 0) = 0.0;               P(1, 1) = 0.002741556778080;

    n = 1;


    int len = x.size();
    //choleksy decomposition
    MatrixXf S = P.llt().matrixL();
    MatrixXf X;

    X = nRandMat::randn(len,n);

    cout << "S = " << S << endl;
    cout << "X = " << X << endl;

    //VectorXf ones = //VectorXf::Ones(n).transpose();
    MatrixXf ones = MatrixXf::Ones(1,n);

    res = S*X + x*ones;

    cout << "res = " << res << endl;
}

int test_randn(int argc, char *argv[])
{
    MatrixXf    r;
    int         n;

    n = 10;
    r.resize(n, n);
    r = nRandMat::randn(n, n);
    cout << "r = " << r << endl;

    int         i, j;

    for(j=0; j<n; j++) {
        for(i=0; i<n; i++) {
            printf("%12.6g ", r(j, i));
        }
        printf("\n");
    }
}


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

#define TEST_ROUTINE(name, desc) \
    printf("------------------------------\n"); \
    printf(#name" - "desc"\n"); \
    printf("------------------------------\n"); \
    name(argc, argv); \
    printf("\n\n");


int main(int argc, char *argv[])
{
    TEST_ROUTINE(test_randn,                "randn");

    TEST_ROUTINE(test_multivariate_gauss,   "multivariate_gauss");


    return 0;
}
