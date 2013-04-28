#include "projection_thread.h"

ProjectionThreading::ProjectionThreading(Elastic *p) :
    QThread(NULL), hp_(p), status_(TS_RUNNING)
{
    int i=0, rlt=0;
    while (status() != TS_STOPPED)
    {
        if (status() == TS_RUNNING)
        {
            for (; i < 5; i++)
            {
                rlt = hp_.Next();
                if (rlt != 0)
                {
                    stop();
                    break;
                }
            }
            emit UpdateView();
            msleep(20);
        }
    }
}

ProjectionThreading::~ProjectionThreading()
{

}

ProjectionThreading::ThreadState ProjectionThreading::status()
{
    QMutexLocker locker(&status_mutex_);
    ThreadState rlt;
    rlt = status_;
    return rlt;
}

void ProjectionThreading::stop()
{
    QMutexLocker locker(&status_mutex_);
    status_ = TS_STOPPED;
}

void ProjectionThreading::pause()
{
    QMutexLocker locker(&status_mutex_);
    if (status_ == TS_RUNNING)
        status_ = TS_PAUSED;
}

void ProjectionThreading::resume()
{
    QMutexLocker locker(&status_mutex_);
    if (status_ == TS_PAUSED)
        status_ = TS_RUNNING;
}
