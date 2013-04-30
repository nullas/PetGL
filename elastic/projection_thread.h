#ifndef PROJECTION_THREAD_H
#define PROJECTION_THREAD_H

#include <QMutexLocker>
#include <QThread>
#include <QMutex>
#include <fstream>
#include "elastic.h"
#include "projection.h"
#include "PetMesh.h"

class ProjectionThreading : public QThread
{
    Q_OBJECT

public:
    explicit ProjectionThreading(Elastic* p, QString _fin);
    ~ProjectionThreading();
    PetCurve *curve_;
    void stop();
    void pause();
    void resume();
    Constraints constraints_;
    typedef PetMesh::Point Point;

    void ClearAllConstraints();
protected:
    void run();

signals:
    void UpdateView();

public slots:

private:
    enum ThreadState {TS_RUNNING, TS_PAUSED, TS_STOPPED};
    ThreadState status();
    QMutex status_mutex_;
    ThreadState status_;
    QString qf_;
    Elastic *p_;
};

#endif // PROJECTION_THREAD_H
