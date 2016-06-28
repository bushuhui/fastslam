#ifndef SLAM_THREAD_H
#define SLAM_THREAD_H

#include <stdio.h>
#include <stdint.h>
#include <string>
#include <QtGui>

class SLAM_Thread : public QThread
{
    Q_OBJECT

public:
    enum RunMode {
        SLAM_WAYPOINT,                  // move along waypoints
        SLAM_INTERACTIVE                // user interactive
    };

    SLAM_Thread(QObject *parent = 0);
    ~SLAM_Thread();

    void stop(void);

    void getCommand(int *cmd);          // 1 - Forward
                                        // 2 - Backward
                                        // 3 - Turn Left
                                        // 4 - Turn Right

    void setRunMode(RunMode mode);      // set run mode
    void setMap(std::string &fname);        // set map filename


signals:
    void replot();
    void showMessage(QString msg);

public slots:
    virtual void commandRecv(int cmd);

protected:
    virtual void run() = 0;

    int         isAlive;                // is finished?

    int         commandID;              // command id
    uint64_t    commandTime;            // command receive time-stamp
    RunMode     runMode;                // running mode

    std::string fnMap;                  // map filename
};

#endif // SLAM_THREAD_H
