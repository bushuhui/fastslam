#ifndef SLAM_THREAD_H
#define SLAM_THREAD_H

#include <stdio.h>
#include <QtGui>

class SLAM_Thread : public QThread
{
    Q_OBJECT

public:
    SLAM_Thread(QObject *parent = 0);
    ~SLAM_Thread();

    void stop(void);

    void get_command(int *cmd);         // 1 - Forward
                                        // 2 - Backward
                                        // 3 - Turn Left
                                        // 4 - Turn Right

signals:
    void replot();
    void showMessage(QString msg);

public slots:
    virtual void commandRecv(int cmd);

protected:
    virtual void run() = 0;

    int         isAlive;
    int         command_id;
    u_int64_t   command_time;
};

#endif // SLAM_THREAD_H
