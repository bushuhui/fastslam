#ifndef SLAMPLOT_H
#define SLAMPLOT_H

#include <QtCore>
#include <QtGui>
#include <QMainWindow>
#include <QTimer>

#include <Eigen/Dense>

#include "qcustomplot.h"


class SlamPlot : public QMainWindow
{
    Q_OBJECT

public:
    explicit SlamPlot(QWidget *parent = 0);
    ~SlamPlot();

    void setupCanvas(void);
    void setupInitData(void);

    void setLandmarks(QVector<double> &arrX, QVector<double> &arrY);
    void setWaypoints(QVector<double> &arrX, QVector<double> &arrY);

    void setParticles(QVector<double> &arrX, QVector<double> &arrY);
    void setParticlesFea(QVector<double> &arrX, QVector<double> &arr);

    void setLaserLines(Eigen::MatrixXf &lnes);
    void setCovEllipse(Eigen::MatrixXf &lnes, int idx);
    void clearCovEllipse(void);

    void addPos(double x, double y);
    void addPosEst(double x, double y);

    void setCarPos(double x, double y, double t, int id=0);
    void setCarSize(double s, int id=0);
    void setCarModel(double *parm, int id=0);

    void setPlotRange(double xmin, double xmax, double ymin, double ymax);

    void clear(void);
    void plot(void);

    void setScreenShot_fname(std::string &fnBase);

    void showMessage(QString &msg);


protected slots:
    void canvsMousePressEvent(QMouseEvent *event);
    void canvasMouseMoveEvent(QMouseEvent *event);
    void canvasReplot(void);
    void canvasShowMessage(QString msg);

    void plotBegin(void);                   // slot for QCustomPlot begin plot
    void plotEnd(void);                     // slot for QCustomPlot end plot

    void covEllipseAdd(int n);              // slot for add new covariance ellipse

signals:
    void commandSend(int cmd);
    void addCovEllipse(int n);


protected:
    void keyPressEvent(QKeyEvent *event);
    void mousePressEvent(QMouseEvent *event);
    void resizeEvent(QResizeEvent *event);
    void timerEvent(QTimerEvent *event);

private:
    QCustomPlot         *customPlot;

    QVector<double>     arrWaypoint_x, arrWaypoint_y;
    QVector<double>     arrPos_x, arrPos_y;
    QVector<double>     arrEstPos_x, arrEstPos_y;
    QVector<double>     arrCurrPos_x, arrCurrPos_y;
    QVector<double>     arrParticles_x, arrParticles_y;
    QVector<double>     arrLandmarks_x, arrLandmarks_y;
    QVector<double>     arrParticlesFea_x, arrParticlesFea_y;

    double              parmCarModel[4];        // 0 - pos x
                                                // 1 - pos y
                                                // 2 - theta
                                                // 3 - size
    double              parmCarEst[4];          // 0 - pos x
                                                // 1 - pos y
                                                // 2 - theta
                                                // 3 - size

    QCPGraph            *plotParticles;
    QCPGraph            *plotParticlesFea;
    QCPGraph            *plotLandmarks;
    QCPCurve            *curvWayPoint;
    QCPCurve            *curvRealPos;
    QCPCurve            *curvEstPos;
    QCPCurve            *curvCar;
    QCPCurve            *curvCarEst;

    QVector<QCPGraph*>  arrLaserLines;                  // laser line graph
    QVector<QCPCurve*>  arrCovLines;                    // cov ellipse graph

    QString             msgString1, msgString2;

    std::string         fnScreenShot_base;

    QMutex              *muxData;

private:
    void drawCar(int idx=0);
};

#endif // SLAMPLOT_H
