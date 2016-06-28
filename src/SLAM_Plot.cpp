#include <QtGui>
#include <QDebug>
#include <QDesktopWidget>
#include <QScreen>
#include <QMessageBox>
#include <QMetaEnum>


#include "SLAM_Plot.h"


SlamPlot::SlamPlot(QWidget *parent) :
    QMainWindow(parent)
{
    muxData = new QMutex(QMutex::NonRecursive);

    customPlot = new QCustomPlot(this);
    connect(customPlot, SIGNAL(beforeReplot()),     this, SLOT(plotBegin()));
    connect(customPlot, SIGNAL(afterReplot()),      this, SLOT(plotEnd()));
    connect(this,       SIGNAL(addCovEllipse(int)), this, SLOT(covEllipseAdd(int)));

    setCentralWidget(customPlot);

    setupInitData();
    setupCanvas();

    // window title
    setWindowTitle("2D SLAM");

    // status bar
    msgString1 = "";
    msgString2 = "";
    statusBar()->clearMessage();

    customPlot->replot();

    //startTimer(30);
}

SlamPlot::~SlamPlot()
{
    delete muxData;
}

void SlamPlot::keyPressEvent(QKeyEvent *event)
{
    int key;
    int cmd = -1;

    key  = event->key();

    if( key == Qt::Key_Up )     cmd = 1;
    if( key == Qt::Key_Down )   cmd = 2;
    if( key == Qt::Key_Left )   cmd = 3;
    if( key == Qt::Key_Right )  cmd = 4;


    QCPRange    rng_x, rng_y;
    rng_x = customPlot->xAxis->range();
    rng_y = customPlot->yAxis->range();

    // Pan
    if( key == Qt::Key_A ) {
        customPlot->xAxis->moveRange(-rng_x.size()/8.0);
    }

    if( key == Qt::Key_D ) {
        customPlot->xAxis->moveRange(rng_x.size()/8.0);
    }

    if( key == Qt::Key_W ) {
        customPlot->yAxis->moveRange(rng_y.size()/8.0);
    }

    if( key == Qt::Key_S ) {
        customPlot->yAxis->moveRange(-rng_y.size()/8.0);
    }

    // zoom
    if( key == Qt::Key_Comma) {
        customPlot->xAxis->scaleRange(0.8, rng_x.center());
        customPlot->yAxis->scaleRange(0.8, rng_y.center());
    }

    if( key == Qt::Key_Period) {
        customPlot->xAxis->scaleRange(1.25, rng_x.center());
        customPlot->yAxis->scaleRange(1.25, rng_y.center());
    }

    if( cmd > 0 ) emit commandSend(cmd);
}

void SlamPlot::mousePressEvent(QMouseEvent *event)
{
    // 1 - left
    // 2 - right
    // 4 - middle
    printf("window pressed, %d, %d, %d\n", event->button(), event->pos().x(), event->pos().y());

    if( event->button() == 1 ) {

    }
}

void SlamPlot::resizeEvent(QResizeEvent *event)
{
    customPlot->xAxis->setScaleRatio(customPlot->yAxis, 1.0);
}

void SlamPlot::timerEvent(QTimerEvent *event)
{
    //plot();
}

void SlamPlot::canvasMouseMoveEvent(QMouseEvent *event)
{
    double      fx, fy;
    QString     msg;

    fx = customPlot->xAxis->pixelToCoord(event->pos().x());
    fy = customPlot->yAxis->pixelToCoord(event->pos().y());

    msgString2.sprintf("pos: %7.3f %7.3f\n", fx, fy);

    msg = msgString1 + " | " + msgString2;
    statusBar()->showMessage(msg);
}


void SlamPlot::canvsMousePressEvent(QMouseEvent *event)
{
#if 0
    // 1 - left
    // 2 - right
    // 4 - middle
    if( event->button() != 2 ) return;

    double      fx, fy;

    fx = customPlot->xAxis->pixelToCoord(event->pos().x());
    fy = customPlot->yAxis->pixelToCoord(event->pos().y());

    // add to estimated position
    arrEstPos_x.push_back(fx);
    arrEstPos_y.push_back(fy);
    curvEstPos->setData(arrEstPos_x, arrEstPos_y);

    // generate particles
    int n_particles, i;
    double  x1, y1;
    n_particles = 10;
    arrParticles_x.clear();
    arrParticles_y.clear();
    for(i=0; i<n_particles; i++) {
        x1 = fx + 2.0*(rand()%10000)/10000.0-1.0;
        y1 = fy + 2.0*(rand()%10000)/10000.0-1.0;

        arrParticles_x.push_back(x1);
        arrParticles_y.push_back(y1);
    }
    plotParticles->setData(arrParticles_x, arrParticles_y);

    // draw car
    if( arrEstPos_x.size() > 1 ) {
        int         n;
        double      x1, y1, x2, y2, dx, dy;
        n = arrEstPos_x.size();

        x1 = arrEstPos_x[n-2];
        y1 = arrEstPos_y[n-2];
        x2 = arrEstPos_x[n-1];
        y2 = arrEstPos_y[n-1];

        dx = x2 - x1;
        dy = y2 - y1;
        parmCarModel[2] = atan2(dy, dx);
    }
    parmCarModel[0] = fx;
    parmCarModel[1] = fy;
    drawCar();

    // replot canvas
    customPlot->replot();
#endif
}


void SlamPlot::canvasReplot(void)
{
    plot();    
}

void SlamPlot::plotBegin(void)
{
    muxData->lock();
}

void SlamPlot::plotEnd(void)
{
    muxData->unlock();
}


void SlamPlot::canvasShowMessage(QString msg)
{
    showMessage(msg);
}



void SlamPlot::setupInitData(void)
{
    if ( 0 ) {
        double      t, x, y, x1, y1;
        int         t_step = 60;
        double      dt, R=0, dR=0.2;
        int         i;
        int         n_particles;

        // generate waypoints
        dt = 2*M_PI/t_step;
        for(i = 0; i<t_step; i++) {
            t = i*dt;
            x = R*cos(t);
            y = R*sin(t);
            arrWaypoint_x.push_back(x);
            arrWaypoint_y.push_back(y);

            R = R + dR;
        }

        // generate estimated position
        arrEstPos_x.push_back(0);
        arrEstPos_y.push_back(0);

        // generate current position
        arrCurrPos_x.push_back(0);
        arrCurrPos_y.push_back(0);

        // generate particles
        n_particles = 10;
        x = 0;
        y = 0;
        for(i=0; i<n_particles; i++) {
            x1 = x + 2.0*(rand()%10000)/10000.0-1.0;
            y1 = y + 2.0*(rand()%10000)/10000.0-1.0;

            arrParticles_x.push_back(x1);
            arrParticles_y.push_back(y1);
        }
    }

    // setupt car parameters
    parmCarModel[0] = 0;            // pos - x
    parmCarModel[1] = 0;            // pos - y
    parmCarModel[2] = 0;            // angle
    parmCarModel[3] = 1;            // size

    parmCarEst[0] = 0;              // pos - x
    parmCarEst[1] = 0;              // pos - y
    parmCarEst[2] = 0;              // angle
    parmCarEst[3] = 1;              // size
}

void SlamPlot::setupCanvas(void)
{
    connect(customPlot, SIGNAL(mousePress(QMouseEvent*)), this, SLOT(canvsMousePressEvent(QMouseEvent*)));
    connect(customPlot, SIGNAL(mouseMove(QMouseEvent*)), this, SLOT(canvasMouseMoveEvent(QMouseEvent*)));

    customPlot->xAxis->setScaleRatio(customPlot->yAxis, 1.0);

    customPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);

    if( 0 ) {
        customPlot->setLocale(QLocale(QLocale::English, QLocale::UnitedKingdom)); // period as decimal separator and comma as thousand separator
        customPlot->legend->setVisible(true);
        QFont legendFont = font();  // start out with SlamPlot's font..
        legendFont.setPointSize(9); // and make a bit smaller for legend
        customPlot->legend->setFont(legendFont);
        customPlot->legend->setBrush(QBrush(QColor(255,255,255,230)));

        // by default, the legend is in the inset layout of the main axis rect. So this is how we access it to change legend placement:
        customPlot->axisRect()->insetLayout()->setInsetAlignment(0, Qt::AlignTop|Qt::AlignRight);
    }


    // setup waypoint plot
    curvWayPoint = new QCPCurve(customPlot->xAxis, customPlot->yAxis);
    customPlot->addPlottable(curvWayPoint);
    curvWayPoint->setPen(QPen(QColor(0, 255, 0)));
    curvWayPoint->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCross, QColor(255, 0, 0), QColor(255, 0, 0), 7));
    curvWayPoint->setLineStyle(QCPCurve::lsLine);
    curvWayPoint->setName("Waypoint");

    // setup real positions
    curvRealPos = new QCPCurve(customPlot->xAxis, customPlot->yAxis);
    customPlot->addPlottable(curvRealPos);
    curvRealPos->setPen(QPen(QColor(0, 0, 255)));
    curvRealPos->setLineStyle(QCPCurve::lsLine);
    curvRealPos->setName("Real Position");

    // setup estimated positions
    curvEstPos = new QCPCurve(customPlot->xAxis, customPlot->yAxis);
    customPlot->addPlottable(curvEstPos);
    curvEstPos->setPen(QPen(QColor(255, 0, 0)));
    curvEstPos->setLineStyle(QCPCurve::lsLine);
    curvEstPos->setName("Estimated Position");

    // setup particles (scatter)
    plotParticles = customPlot->addGraph();
    plotParticles->setPen(QColor(0, 0, 0, 255));
    plotParticles->setLineStyle(QCPGraph::lsNone);
    plotParticles->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCircle, 2));
    plotParticles->setName("Particles");

    // setup particles feature (scatter)
    plotParticlesFea = customPlot->addGraph();
    plotParticlesFea->setPen(QColor(255, 0, 0));
    plotParticlesFea->setLineStyle(QCPGraph::lsNone);
    plotParticlesFea->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCircle, 2));
    plotParticlesFea->setName("Feature Particles");

    // setupt landmarks
    plotLandmarks = customPlot->addGraph();
    plotLandmarks->setPen(QColor(0, 0, 255, 255));
    plotLandmarks->setBrush(QBrush(QColor(0, 0, 255, 80)));
    plotLandmarks->setLineStyle(QCPGraph::lsNone);
    plotLandmarks->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssStar, 5));
    plotLandmarks->setName("Landmarks");

    // real car position
    curvCar = new QCPCurve(customPlot->xAxis, customPlot->yAxis);
    customPlot->addPlottable(curvCar);
    curvCar->setPen(QPen(QColor(0, 0, 255)));
    curvCar->setLineStyle(QCPCurve::lsLine);
    curvCar->setBrush(QBrush(QColor(0, 0, 255, 80)));
    curvCar->setName("Real car position");

    // estimated car position
    curvCarEst = new QCPCurve(customPlot->xAxis, customPlot->yAxis);
    customPlot->addPlottable(curvCarEst);
    curvCarEst->setPen(QPen(QColor(255, 0, 0)));
    curvCarEst->setLineStyle(QCPCurve::lsLine);
    curvCarEst->setBrush(QBrush(QColor(255, 0, 0, 80)));
    curvCarEst->setName("Estimated car position");

    // laser lines
    for(int i=0; i<5; i++) {
        QCPGraph *gLaserLine = customPlot->addGraph();
        gLaserLine->setPen(QColor(255, 0, 0));
        gLaserLine->setLineStyle(QCPGraph::lsLine);
        gLaserLine->setName("Laser lines");

        arrLaserLines.push_back(gLaserLine);
    }

    // Cov ellipse lines
    for(int i=0; i<25; i++) {
        QCPCurve *gCovEllipse = new QCPCurve(customPlot->xAxis, customPlot->yAxis);
        customPlot->addPlottable(gCovEllipse);
        gCovEllipse->setPen(QColor(128, 0, 128));
        gCovEllipse->setLineStyle(QCPCurve::lsLine);
        gCovEllipse->setName("Covanice ellipse");

        arrCovLines.push_back(gCovEllipse);
    }

    // set ranges appropriate to show data:
    customPlot->xAxis->setRange(-20, 20);
    customPlot->yAxis->setRange(-20, 20);
    // set labels
    customPlot->xAxis->setLabel("X [m]");
    customPlot->yAxis->setLabel("Y [m]");
    // make ticks on both axis go outward:
    customPlot->xAxis->setTickLength(0, 5);
    customPlot->xAxis->setSubTickLength(0, 3);
    customPlot->yAxis->setTickLength(0, 5);
    customPlot->yAxis->setSubTickLength(0, 3);

    // configure right and top axis to show ticks but no labels:
    customPlot->xAxis2->setVisible(true);
    customPlot->xAxis2->setTickLabels(false);
    customPlot->yAxis2->setVisible(true);
    customPlot->yAxis2->setTickLabels(false);
    customPlot->xAxis2->setTickLength(0, 0);
    customPlot->xAxis2->setSubTickLength(0, 0);
    customPlot->yAxis2->setTickLength(0, 0);
    customPlot->yAxis2->setSubTickLength(0, 0);
}

void SlamPlot::setLandmarks(QVector<double> &arrX, QVector<double> &arrY)
{
    muxData->lock();

#if 0
    arrLandmarks_x = arrX;
    arrLandmarks_y = arrY;
    plotLandmarks->setData(arrLandmarks_x, arrLandmarks_y);
#else
    plotLandmarks->setData(arrX, arrY);
#endif

    muxData->unlock();
}

void SlamPlot::setWaypoints(QVector<double> &arrX, QVector<double> &arrY)
{
    muxData->lock();

#if 0
    arrWaypoint_x = arrX;
    arrWaypoint_y = arrY;
    curvWayPoint->setData(arrWaypoint_x, arrWaypoint_y);
#else
    curvWayPoint->setData(arrX, arrY);
#endif

    muxData->unlock();
}

void SlamPlot::setParticles(QVector<double> &arrX, QVector<double> &arrY)
{
    muxData->lock();

#if 0
    arrParticles_x = arrX;
    arrParticles_y = arrY;
    plotParticles->setData(arrParticles_x, arrParticles_y);
#else
    plotParticles->setData(arrX, arrY);
#endif

    muxData->unlock();
}

void SlamPlot::setParticlesFea(QVector<double> &arrX, QVector<double> &arrY)
{
    muxData->lock();

#if 0
    arrParticlesFea_x = arrX;
    arrParticlesFea_y = arrY;
    plotParticlesFea->setData(arrParticlesFea_x, arrParticlesFea_y);
#else
    plotParticlesFea->setData(arrX, arrY);
#endif

    muxData->unlock();
}


void SlamPlot::setLaserLines(Eigen::MatrixXf &lnes)
{
    muxData->lock();

    int                 i;
    int                 nGraph, nLines;
    QVector<double>     arrX, arrY;

    nGraph = arrLaserLines.size();
    nLines = lnes.cols();

    // add new graph
    if( nLines > nGraph ) {
        for(i=0; i<nLines-nGraph; i++) {
            QCPGraph *gLaserLine = customPlot->addGraph();
            gLaserLine->setPen(QColor(255, 0, 0));
            gLaserLine->setLineStyle(QCPGraph::lsLine);
            gLaserLine->setName("Laser lines");

            arrLaserLines.push_back(gLaserLine);
        }
    }

    // set laser line data
    for(i=0; i<nLines; i++) {
        arrX.clear();   arrY.clear();
        arrX.push_back(lnes(0, i));
        arrX.push_back(lnes(2, i));
        arrY.push_back(lnes(1, i));
        arrY.push_back(lnes(3, i));

        arrLaserLines[i]->setData(arrX, arrY);
    }
    for(i=nLines; i<nGraph; i++) {
        arrX.clear();   arrY.clear();
        arrX.push_back(0.0);    arrX.push_back(0.0);
        arrY.push_back(0.0);    arrY.push_back(0.0);
        arrLaserLines[i]->setData(arrX, arrY);
    }

    muxData->unlock();
}

// slot for add new covariance ellipse
void SlamPlot::covEllipseAdd(int n)
{
    muxData->lock();

    int                 i;
    int                 nGraph;

    nGraph = arrCovLines.size();

    // add new graph
    if( n > nGraph ) {
        for(i=0; i<n-nGraph+1; i++) {
            QCPCurve *gCovEllipse = new QCPCurve(customPlot->xAxis, customPlot->yAxis);
            customPlot->addPlottable(gCovEllipse);
            gCovEllipse->setPen(QColor(128, 0, 128));
            gCovEllipse->setLineStyle(QCPCurve::lsLine);
            gCovEllipse->setName("Covanice ellipse");

            arrCovLines.push_back(gCovEllipse);
        }
    }

    muxData->unlock();
}


void SlamPlot::setCovEllipse(Eigen::MatrixXf &lnes, int idx)
{
    muxData->lock();

    int                 i, nSeg;
    int                 nGraph;

    QVector<double>     arrX, arrY;

    nSeg   = lnes.cols();
    nGraph = arrCovLines.size();

    if( idx >= nGraph ) {
        muxData->unlock();
        emit addCovEllipse(nGraph*2);
        return;
    }

    // set covanice ellipse line data
    arrX.clear();
    arrY.clear();

    for(i=0; i<nSeg; i++) {
        arrX.push_back(lnes(0, i));
        arrY.push_back(lnes(1, i));

        arrCovLines[idx]->setData(arrX, arrY);
    }

    muxData->unlock();
}

void SlamPlot::clearCovEllipse(void)
{
    muxData->lock();

    int                 i, nGraph;
    QVector<double>     arrX, arrY;

    nGraph = arrCovLines.size();

    arrX.clear();
    arrY.clear();
    for(i=0; i<nGraph; i++)
        arrCovLines[i]->setData(arrX, arrY);

    muxData->unlock();
}

void SlamPlot::addPos(double x, double y)
{
    muxData->lock();

    arrPos_x.push_back(x);
    arrPos_y.push_back(y);

    curvRealPos->setData(arrPos_x, arrPos_y);

    muxData->unlock();
}

void SlamPlot::addPosEst(double x, double y)
{
    muxData->lock();

    arrEstPos_x.push_back(x);
    arrEstPos_y.push_back(y);

    curvEstPos->setData(arrEstPos_x, arrEstPos_y);

    // draw car
    if( arrEstPos_x.size() > 1 ) {
        int         n;
        double      x1, y1, x2, y2, dx, dy;
        n = arrEstPos_x.size();

        x1 = arrEstPos_x[n-2];
        y1 = arrEstPos_y[n-2];
        x2 = arrEstPos_x[n-1];
        y2 = arrEstPos_y[n-1];

        dx = x2 - x1;
        dy = y2 - y1;
        parmCarEst[2] = atan2(dy, dx);
    }
    parmCarEst[0] = x;
    parmCarEst[1] = y;
    drawCar(1);

    muxData->unlock();
}


void SlamPlot::setCarPos(double x, double y, double t, int idx)
{
    muxData->lock();

    if( idx == 0 ) {
        parmCarModel[0] = x;
        parmCarModel[1] = y;
        parmCarModel[2] = t;
    } else {
        parmCarEst[0] = x;
        parmCarEst[1] = y;
        parmCarEst[2] = t;
    }

    drawCar(idx);

    muxData->unlock();
}

void SlamPlot::setCarSize(double s, int idx)
{
    muxData->lock();

    if( idx == 0 )
        parmCarModel[3] = s;
    else if( idx == 1 )
        parmCarEst[3] = s;

    drawCar(idx);

    muxData->unlock();
}

void SlamPlot::setCarModel(double *parm, int idx)
{
    muxData->lock();

    for(int i=0; i<4; i++) {
        if( idx == 0 )
            parmCarModel[i] = parm[i];
        else if( idx == 1 )
            parmCarEst[i] = parm[i];
    }

    drawCar(idx);

    muxData->unlock();
}

void SlamPlot::setPlotRange(double xmin, double xmax, double ymin, double ymax)
{
    muxData->lock();

    // set ranges appropriate to show data:
    customPlot->xAxis->setRange(xmin, xmax);
    customPlot->yAxis->setRange(ymin, ymax);

    muxData->unlock();
}

void SlamPlot::showMessage(QString &msg)
{
    QString     _msg;

    msgString1 = msg;
    _msg = msgString1 + " | " + msgString2;
    statusBar()->showMessage(_msg);
}

void SlamPlot::setScreenShot_fname(std::string &fnBase)
{
    fnScreenShot_base = fnBase;
}


void SlamPlot::clear(void)
{
    muxData->lock();

    arrWaypoint_x.clear();
    arrWaypoint_y.clear();
    curvWayPoint->setData(arrWaypoint_x, arrWaypoint_y);

    arrParticles_x.clear();
    arrParticles_y.clear();
    plotParticles->setData(arrParticles_x, arrParticles_y);

    arrEstPos_x.clear();
    arrEstPos_y.clear();
    curvEstPos->setData(arrEstPos_x, arrEstPos_y);

    arrCurrPos_x.clear();
    arrCurrPos_y.clear();

    muxData->unlock();
}

void SlamPlot::plot(void)
{
    static int  idx = 0;
    QString     fnOut;

    customPlot->replot();

    if( fnScreenShot_base.size() > 0 ) {
        fnOut.sprintf("%s_%05d.png", fnScreenShot_base.c_str(), idx++);
        customPlot->savePng(fnOut);
    }
}

void SlamPlot::drawCar(int idx)
{
    double  x, y, t, s;
    double  x1, y1, x2, y2;

    QVector<double> arrX, arrY;

    if( idx == 0 ) {
        x = parmCarModel[0];
        y = parmCarModel[1];
        t = parmCarModel[2];
        s = parmCarModel[3];
    } else {
        x = parmCarEst[0];
        y = parmCarEst[1];
        t = parmCarEst[2];
        s = parmCarEst[3];
    }

    x2 = x+s/3*cos(M_PI/2.0+t);
    y2 = y+s/3*sin(M_PI/2.0+t);
    arrX.push_back(x2);
    arrY.push_back(y2);

    x1 = x+s/3*cos(3.0*M_PI/2.0+t);
    y1 = y+s/3*sin(3.0*M_PI/2.0+t);
    arrX.push_back(x1);
    arrY.push_back(y1);

    x1 = x+s*1.3*cos(t);
    y1 = y+s*1.3*sin(t);
    arrX.push_back(x1);
    arrY.push_back(y1);

    arrX.push_back(x2);
    arrY.push_back(y2);

    if( idx == 0 )
        curvCar->setData(arrX, arrY);
    else if( idx == 1 )
        curvCarEst->setData(arrX, arrY);
}
