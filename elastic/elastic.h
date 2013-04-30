#ifndef ELASTIC_H
#define ELASTIC_H

#include <QObject>
#include <QtPlugin>
#include <QWidget>
#include <QString>
#include <QScrollArea>
#include <QGridLayout>
#include <QThread>
#include <coin/IpTNLP.hpp>

#include "defs.h"
#include "PluginInterface.h"
#include "PetGL.h"

#include "coin/IpIpoptApplication.hpp"



namespace Ui {
class ElasticPanel;
}

#define PLUGIN_NAME "elastic"

class Elastic : public PetPluginInterface
{
    Q_OBJECT
    Q_INTERFACES(PetPluginInterface)
public:
    Elastic();
    ~Elastic();
    void initial(QWidget* parent);
    void drawExtra();
    double r;
    typedef typename OpenMesh::Vec3d Point;
    std::vector<PetCurve::VertexHandle> selectedVertices;
    std::vector<PetCurve::EdgeHandle> selectedEdges;
    std::vector<PetCurve::CurveHandle> selectedCurves;

    std::map<PetCurve::VertexHandle, unsigned int> idxMapping;
    std::vector<PetCurve::EdgeHandle> edgesToOptimize;
    std::vector<PetCurve::VertexHandle> verticesToOptimize;

    std::vector<pair<PetCurve::VertexHandle, Point> > PositionConstraints;
    std::vector<pair<PetCurve::EdgeHandle, Point> > TangentConstraints;
    std::vector<pair<PetCurve::Point, double> > PlaneConstraintsInfo;
    std::vector<PetCurve::VertexHandle> PlaneConstraints;
    std::vector<Point> MaterialFrameConstraints;

    PetCurve* curveToOptimize;

    typedef struct
    {
        double BendingEnergyCoef;
        double TwistingEnergyCoef;
        double PositionConstraintsWeight;
        double TangentConstraintsCoef;
        double PlaneConstraintsCoef;
        double r;
        int extension;
        int twisting_times;
        double twisting_fraction;
        double dt;
        int itertations;
        int max_steps;
    }pOptimize;

    pOptimize pO;

signals:

private slots:
    void on_selectcurve_clicked();

    void on_clearcurves_clicked();

    void on_clearvertices_clicked();

    void on_doubleSpinBox_collisionradius_valueChanged(double arg1);

    void on_pushButton_intersection_clicked();

    void on_pushButton_VerticesFromCurves_clicked();

    void on_pushButton_PointsInvolved_clicked();

    void on_pushButton_wholeCurvesVertices_clicked();

    void on_pushButton_doitright_clicked();

    void on_pushButton_setInvolved_clicked();

    void on_doubleSpinBox_PositionConstraintsWeight_editingFinished();

    void on_doubleSpinBox_BendingEnergyCoef_editingFinished();

    void on_pushButton_test_clicked();

    void on_pushButton_wholeCurvesInteriorVertices_clicked();

    void on_doubleSpinBox_TangentConstraintsCoef_editingFinished();

    void on_pushButton_optimize_clicked();

    void on_pushButton_exprotSelection_clicked();

    void on_pushButton_clear_clicked();

    void on_spinBox_extension_editingFinished();

    void on_doubleSpinBox_editingFinished();

    void on_doubleSpinBox_PlaneConstraintsCoef_editingFinished();

    void on_pushButton_doItRightIter_clicked();

    void on_pushButton_computeRotation_clicked();

    void on_pushButton_twist_clicked();

    void on_doubleSpinBox_TwistingEnergyCoef_editingFinished();

    void on_spinBox_twist_times_editingFinished();

    void on_pushButton_hamiltonProj_clicked();

    void on_doubleSpinBox_dt_editingFinished();

    void on_doubleSpinBox_twist_fraction_editingFinished();

    void on_spinBox_ProjectionIter_editingFinished();

    void on_spinBox_max_step_editingFinished();

    void on_pushButton_projectionThread_clicked();

    void DeleteProjectionThreading();

public:
    PetGL* pgl;
    QWidget* tabPluginWidget;
    QTabWidget* tabPlugin;
    Ui::ElasticPanel *ui;
    QScrollArea* tabPluginScrollArea;
    int tabPluginIndex;

    void addTabWidget(const char*);
    void setupTabWidget();

    PetMesh* pmesh;
    PetCurve* pcurve;

    std::vector<pair<Point, Point> > axisExtra;

    bool getCurrentCurve();
    void clearAllConstraints();

    double Mepsilon;

    //fast collision detection to filter out line segments far away.
    bool LineSegmentsCollide(const Point& p0,
                                       const Point& p1,
                                       const Point& q0,
                                       const Point& q1);

    bool LineSegmentsCollide(const unsigned int, const unsigned int);

    bool LineSegmentsCollide(const PetCurve::EdgeHandle i, const PetCurve::EdgeHandle j);

    //Line segments distance
    double LineSegmentsSqDistance(const Point& p0,
                                       const Point& p1,
                                       const Point& q0,
                                       const Point& q1);

    double LineSegmentsDistance(const Point& p0,
                                       const Point& p1,
                                       const Point& q0,
                                       const Point& q1);

    double LineSegmentsDistance(const unsigned int i, const unsigned int j);

    double LineSegmentsDistance(const PetCurve::EdgeHandle, const PetCurve::EdgeHandle);

    double CrossLineSegmentsDistance(const Point& p0,
                                     const Point& p1,
                                     const Point& q0,
                                     const Point& q1);

    double CrossLineSegmentsSqDistance(const Point& p0,
                                     const Point& p1,
                                     const Point& q0,
                                     const Point& q1);

    void EdgesIntersections(const std::vector<unsigned int>& idx,
                           std::vector<std::pair<unsigned int, unsigned int> >& inter);

    void EdgesIntersections(const std::vector<PetCurve::EdgeHandle>& idx,
                           std::vector<std::pair<PetCurve::EdgeHandle, PetCurve::EdgeHandle> >& inter);

    void setPointsFixed(const std::vector<PetCurve::VertexHandle>& hnd);

    void computeRotation();
    void SetupIpoptOptions(Ipopt::SmartPtr<Ipopt::IpoptApplication> &);
    QThread *pt_;
};

#endif // ELASTIC_H
