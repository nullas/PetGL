#include "projection2_thread.h"
#include <QFileInfo>
#include <QDir>
#include <fstream>

ProjectionThreading2::ProjectionThreading2(Elastic *p, QString _fin) :
    QThread(NULL), curve_(p->curveToOptimize),status_(TS_RUNNING),
    qf_(_fin), p_(p)
{

}

ProjectionThreading2::~ProjectionThreading2()
{

}

void ProjectionThreading2::run()
{
    int i=0, rlt=0;
    ifstream fin;
    fin.open(qf_.toAscii(), ios::in);
    if (!fin.is_open())
    {
        cout<< "Read file error" << qf_.toStdString() <<endl;
        return;
    }
    std::string fname;
    fin >> fname;
    if (fin.eof()) return;
    int iteration = 1;
    int idx;
    char cType;
    int n;
    std::vector<pair<char, int> > filestruct;
    fin >> cType;
    if (cType == 'I')
    {
        fin >> iteration;
        fin >> cType;
    }
    for (i = 0; i < iteration; ++i)
    {
        filestruct.clear();
        while (cType != 'E' && !fin.eof())
        {
            if (cType == 'S')
            {
                fin >> cType;
                continue;
            }
            fin >> n;
            filestruct.push_back(pair<char, int>(cType, n));
            fin >> cType;
        }

        ClearAllConstraints();
        Point::value_type x, y, z, w;
        Point P;
        std::vector<pair<char, int> >::const_iterator it = filestruct.begin(),
                it_end = filestruct.end();
        for (; it != it_end; ++it)
        {
            switch ((*it).first)
            {
            case 'P':
                for (int i = 0; i < (*it).second; ++i)
                {
                    fin >> idx >> x >> y >> z;
                    constraints_.PositionConstraints.push_back(pair<PetCurve::VertexHandle, Point>
                                                  (curve_->vertex_handle(idx),
                                                   Point(x,y,z)));
                }
                break;
            case 'T':
                for (int i = 0; i < (*it).second; ++i)
                {
                    fin >> idx >> x >> y >> z;
                    constraints_.TangentConstraints.push_back(pair<PetCurve::EdgeHandle, Point>
                                                 (curve_->edge_handle(idx),
                                                  Point(x,y,z)));
                }
                break;
            case 'A':
                for (int i = 0; i < (*it).second; ++i)
                {
                    fin >> idx >> x >> y >> z >> w;
                    P = Point(x, y, z);
                    constraints_.PlaneConstraintsInfo.push_back(pair<PetCurve::Point, double>(P, w));
                    constraints_.PlaneConstraints.push_back(curve_->vertex_handle(idx));
                }
                break;
            case 'V':
                for (int i =0; i < (*it).second; ++i)
                {
                    fin >> idx;
                    constraints_.VerticesToOptimize.push_back(curve_->vertex_handle(idx));
                }
                break;
            default:
                return;
            }
        }

        TangentProjection2 hp(curve_, constraints_, p_->pO);
        hp.check_derivative(p_->pO.check_derivatives);
        while (status() != TS_STOPPED)
        {
            if (status() == TS_RUNNING)
            {
                for (i = 0; i < 5; i++)
                {
                    rlt = hp.Next();
                    if (rlt != 0)
                    {
                        stop();
                        break;
                    }
                }
                emit UpdateView();
                msleep(10);
            }
        }
        fin >> cType;
        while (!fin.eof() && cType != 'S') fin >> cType;
        if (fin.eof()) break;
    }
    if (fin.is_open())
        fin.close();
}

ProjectionThreading2::ThreadState ProjectionThreading2::status()
{
    ThreadState rlt;
    status_mutex_.lock();
    rlt = status_;
    status_mutex_.unlock();
    return rlt;
}

void ProjectionThreading2::stop()
{
    status_mutex_.lock();
    status_ = TS_STOPPED;
    status_mutex_.unlock();
}

void ProjectionThreading2::pause()
{
    status_mutex_.lock();
    if (status_ == TS_RUNNING)
        status_ = TS_PAUSED;
    status_mutex_.unlock();
}

void ProjectionThreading2::resume()
{
    status_mutex_.lock();
    if (status_ == TS_PAUSED)
        status_ = TS_RUNNING;
    status_mutex_.unlock();
}

void ProjectionThreading2::ClearAllConstraints()
{
    constraints_.PositionConstraints.clear();
    constraints_.TangentConstraints.clear();
    constraints_.PlaneConstraints.clear();
    constraints_.PlaneConstraintsInfo.clear();
    constraints_.VerticesToOptimize.clear();
}

