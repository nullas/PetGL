#ifndef PROJECTION_THREAD_H
#define PROJECTION_THREAD_H

#include <QMutexLocker>
#include <QThread>
#include <QMutex>
#include "elastic.h"
#include "eigen_hamilton.h"


class ProjectionThreading : public QThread
{
    Q_OBJECT

public:
    explicit ProjectionThreading(Elastic* p);
    ~ProjectionThreading();
    void stop();
    void pause();
    void resume();

protected:
    void run();

signals:
    void UpdateView();

public slots:

private:
    HamiltonProjection hp_;
    enum ThreadState {TS_RUNNING, TS_PAUSED, TS_STOPPED};
    ThreadState status();
    QMutex status_mutex_;
    ThreadState status_;
};

#endif // PROJECTION_THREAD_H
