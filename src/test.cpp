
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <unistd.h>

#include <iostream>

#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

#include "utils.h"
#include "fastslam_core.h"

using namespace std;
using namespace Eigen;


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

int eigen3_01(CParamArray *pa)
{
    MatrixXd m(2,2);
    m(0,0) = 3;
    m(1,0) = 2.5;
    m(0,1) = -1;
    m(1,1) = m(1,0) + m(0,1);
    std::cout << "m=\n" << m << std::endl;
}

int eigen3_02(CParamArray *pa)
{
    MatrixXd m = MatrixXd::Random(3,3);
    m = (m + MatrixXd::Constant(3,3,1.2)) * 50;
    cout << "m =" << endl << m << endl;
    VectorXd v(3);

    v << 1, 2, 3;
    cout << "m * v =" << endl << m * v << endl;

    cout << "v.rows() = " << v.rows() << endl;
    cout << "v.cols() = " << v.cols() << endl;
}

int eigen3_03(CParamArray *pa)
{
    Matrix3d m = Matrix3d::Random();
    m = (m + Matrix3d::Constant(1.2)) * 50;
    cout << "m =\n" << endl << m << endl;
    Vector3d v(1,2,3);
    cout << "m * v =\n" << endl << m * v << endl;
}

int eigen3_04(CParamArray *pa)
{
    VectorXf a(50), b(50), c(50), d(50);
    a = 3*b + 4*c + 5*d;


    MatrixXd m = MatrixXd::Random(10,10);
    cout << "m = \n" << m << endl;

    m = ((m.array()+1.0)/2.0).matrix();

    cout << "m = \n" << m << "\n\n";
}

int eigen3_05(CParamArray *pa)
{
    Matrix3f m;
    m << 1, 2, 3,
         4, 5, 6,
         7, 8, 9;
    std::cout << m;
}

int eigen3_06(CParamArray *pa)
{
    MatrixXd m(2,5);

    m = MatrixXd::Random(2, 5);
    cout << "m (before resizing) = \n" << m << endl;
    m.resize(4,3);
    cout << "m (after resizing) = \n" << m << "\n\n";

    std::cout << "The matrix m is of size "
              << m.rows() << "x" << m.cols() << "\n";
    std::cout << "It has " << m.size() << " coefficients" << "\n\n";

    m.conservativeResize(4, 4);
    cout << "\nm (after conservativeResize) = \n" << m << "\n\n";


    VectorXd v(2);

    v = VectorXd::Random(2);
    cout << "v (before resizing) = \n" << v << endl;
    v.resize(5);
    cout << "v (after resizing) = \n" << v << "\n\n";
    std::cout << "The vector v is of size " << v.size() << std::endl;
    std::cout << "As a matrix, v is of size "
              << v.rows() << "x" << v.cols() << std::endl;

    v.conservativeResize(10);
    std::cout << "v (after conservativeResize) = \n" << v << "\n\n";
}

int eigen3_07(CParamArray *pa)
{
    MatrixXf a(2,2);
    std::cout << "a is of size " << a.rows() << "x" << a.cols() << std::endl;
    MatrixXf b(3,3);
    a = b;
    std::cout << "a is now of size " << a.rows() << "x" << a.cols() << std::endl;
}

int eigen3_block(CParamArray *pa)
{
    MatrixXf    a(3, 3), b;

    a.setZero(4, 4);
    a << 1, 2, 3, 4,
         5, 6, 7, 8,
         9, 10, 11, 12,
         13, 14, 15, 16;
    cout << "a = \n" << a << "\n\n";

    b = a;

    a.setZero(6, 6);
    a.block(0,0,4,4) = b;
    cout << "a = \n" << a << "\n\n";

    a.conservativeResize(8, 8);
    a.block(6, 6, 2, 2) << 9, 8, 7, 6;
    a.block(6, 0, 2, 6) << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12;
    a.block(0, 6, 6, 2) = a.block(6, 0, 2, 6).transpose();
    cout << "a = \n" << a << "\n\n";


    VectorXf    x(4), x_;
    x << 1, 2, 3, 4;

    x_ = x.block(0,0,2,1);
    cout << "x  = \n" << x << "\n\n";
    cout << "x_ = \n" << x_ << "\n\n";
}

int add_sub(CParamArray *pa)
{
    Matrix2d a;
    a << 1, 2,
            3, 4;
    MatrixXd b(2,2);
    b << 2, 3,
            1, 4;
    std::cout << "a + b =\n" << a + b << std::endl;
    std::cout << "a - b =\n" << a - b << std::endl;
    std::cout << "Doing a += b;" << std::endl;
    a += b;
    std::cout << "Now a =\n" << a << std::endl;
    Vector3d v(1,2,3);
    Vector3d w(1,0,0);
    std::cout << "-v + w - v =\n" << -v + w - v << std::endl;
}

int trans_conj(CParamArray *pa)
{
    MatrixXcf a = MatrixXcf::Random(2,2);
    cout << "Here is the matrix a\n" << a << endl;
    cout << "Here is the matrix a^T\n" << a.transpose() << endl;
    cout << "Here is the conjugate of a\n" << a.conjugate() << endl;
    cout << "Here is the matrix a^*\n" << a.adjoint() << endl;
}

int test_chol(CParamArray *pa)
{
    MatrixXd A(3,3);
    A << 4,-1,2, -1,6,0, 2,0,5;
    cout << "The matrix A is" << endl << A << endl;
    LLT<MatrixXd> lltOfA(A); // compute the Cholesky decomposition of A
    MatrixXd L = lltOfA.matrixL(); // retrieve factor L in the decomposition
    // The previous two lines can also be written as "L = A.llt().matrixL()"
    cout << "The Cholesky factor L is" << endl << L << endl;
    cout << "To check this, let us compute L * L.transpose()" << endl;
    cout << L * L.transpose() << endl;
    cout << "This should equal the matrix A" << endl;

    cout << "\n\n";

    MatrixXd B=A.llt().matrixU();
    cout << "chol(B) = \n" << B << "\n\n";
}



int test_sqrtm(CParamArray *pa)
{
    const double pi = std::acos(-1.0);
    MatrixXf A(2,2);
    A << cos(pi/3), -sin(pi/3),
            sin(pi/3), cos(pi/3);
    std::cout << "The matrix A is:\n" << A << "\n\n";
    std::cout << "The matrix square root of A is:\n" << A.sqrt() << "\n\n";
    std::cout << "The square of the last matrix is:\n" << A.sqrt() * A.sqrt() << "\n";
}


int test_expm(CParamArray *pa)
{
    const double pi = std::acos(-1.0);
    MatrixXd A(2,2);
    A << cos(pi/3), -sin(pi/3),
            sin(pi/3), cos(pi/3);
    std::cout << "The matrix A is:\n" << A << "\n\n";
    std::cout << "The matrix exp of A is:\n" << A.exp() << "\n\n";
}


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

int test_randn(CParamArray *pa)
{
    MatrixXf    r;
    int         n;

    n = 10;
    r.resize(n, n);
    r = nRandMat::randn(n, n);
    cout << "r = \n" << r << endl;
}


int test_multivariate_gauss(CParamArray *pa)
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

    cout << "S = \n" << S << "\n\n";
    cout << "X = \n" << X << "\n\n";

    //VectorXf ones = //VectorXf::Ones(n).transpose();
    MatrixXf ones = MatrixXf::Ones(1,n);

    res = S*X + x;

    cout << "res = \n" << res << "\n\n";

    VectorXf res2;

    res2 = multivariate_gauss(x, P, n);
    cout << "res2 = \n" << res2 << "\n\n";
}



////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////


struct RTK_TestFunctionArray   g_arrFunc[] =
{
    RTK_FUNC_TEST_DEF(eigen3_01,         "First sample"),
    RTK_FUNC_TEST_DEF(eigen3_02,         "Matrix and Vector"),
    RTK_FUNC_TEST_DEF(eigen3_03,         "Simple operation"),
    RTK_FUNC_TEST_DEF(eigen3_04,         "Random init"),
    RTK_FUNC_TEST_DEF(eigen3_05,         "Stream setting"),
    RTK_FUNC_TEST_DEF(eigen3_06,         "matrix resize"),
    RTK_FUNC_TEST_DEF(eigen3_07,         "matrix assignment and resizing"),
    RTK_FUNC_TEST_DEF(eigen3_block,      "matrix block"),

    RTK_FUNC_TEST_DEF(add_sub,           "Addition and subtraction"),
    RTK_FUNC_TEST_DEF(trans_conj,        "Transposition and conjugation"),

    RTK_FUNC_TEST_DEF(test_chol,         "Cholesky decomposition"),
    RTK_FUNC_TEST_DEF(test_sqrtm,        "matrix sqrt"),
    RTK_FUNC_TEST_DEF(test_expm,         "matrix expm"),


    RTK_FUNC_TEST_DEF(test_randn,                "randn"),
    RTK_FUNC_TEST_DEF(test_multivariate_gauss,   "multivariate_gauss"),

    {NULL,  "NULL",  "NULL"},
};

int main(int argc, char *argv[])
{
    CParamArray     pa;

    rtk_test_main(argc, argv, g_arrFunc, pa);
}
