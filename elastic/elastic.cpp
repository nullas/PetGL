#include "elastic.h"

#include <cmath>

#include <QFileDialog>

#include "coin/IpIpoptApplication.hpp"
#include "optimize.h"
#include "ui_elasticpanel.h"


Q_EXPORT_PLUGIN2(PLUGIN_NAME, Elastic)

Elastic::Elastic() : ui(new Ui::ElasticPanel), Mepsilon(_Mepsilon)
{
    tabPluginWidget = NULL;
    curveToOptimize = NULL;
    pmesh = NULL;
    pcurve = NULL;
}

Elastic::~Elastic()
{
    delete ui;
    delete tabPluginScrollArea;
}

void Elastic::initial(QWidget* _parent)
{
    pgl = dynamic_cast<PetGL*>(_parent);
    tabPlugin = pgl->getPluginTab();
    addTabWidget(PLUGIN_NAME);
}

void Elastic::addTabWidget(const char* pluginName)
{
    if(!tabPluginWidget)
    {
        tabPluginWidget = new QWidget();
        ui->setupUi(tabPluginWidget);

        tabPluginScrollArea = new QScrollArea;
        tabPluginScrollArea->setWidget(tabPluginWidget);
        tabPluginIndex = tabPlugin->addTab(tabPluginScrollArea,tr(pluginName));
        tabPlugin->setCurrentIndex(tabPluginIndex);

        tabPluginScrollArea->setWidgetResizable(true);

        setupTabWidget();
    }
}

void Elastic::setupTabWidget()
{
    connect(ui->selectcurve,SIGNAL(clicked()),this,SLOT(on_selectcurve_clicked()));
    connect(ui->clearcurves,SIGNAL(clicked()),this,SLOT(on_clearcurves_clicked()));
    connect(ui->clearvertices,SIGNAL(clicked()),this,SLOT(on_clearvertices_clicked()));
    connect(ui->doubleSpinBox_collisionradius,SIGNAL(valueChanged(double)),
            this,SLOT(on_doubleSpinBox_collisionradius_valueChanged(double)));
    connect(ui->pushButton_intersection,SIGNAL(clicked()),this,SLOT(on_pushButton_intersection_clicked()));
    connect(ui->pushButton_VerticesFromCurves,SIGNAL(clicked()),
            this,SLOT(on_pushButton_VerticesFromCurves_clicked()));
    connect(ui->pushButton_wholeCurvesVertices,SIGNAL(clicked()),
            this,SLOT(on_pushButton_wholeCurvesVertices_clicked()));
    connect(ui->pushButton_setInvolved,SIGNAL(clicked()),
            this,SLOT(on_pushButton_setInvolved_clicked()));
    connect(ui->pushButton_doitright,SIGNAL(clicked()),
            this,SLOT(on_pushButton_doitright_clicked()));
    connect(ui->doubleSpinBox_PositionConstraintsWeight,SIGNAL(editingFinished()),
            this,SLOT(on_doubleSpinBox_PositionConstraintsWeight_editingFinished()));
    connect(ui->doubleSpinBox_TangentConstraintsCoef, SIGNAL(editingFinished()),
            this, SLOT(on_doubleSpinBox_TangentConstraintsCoef_editingFinished()));
    connect(ui->spinBox_extension, SIGNAL(editingFinished()),
            this, SLOT(on_spinBox_extension_editingFinished()));
    connect(ui->pushButton_test, SIGNAL(clicked()),
            this, SLOT(on_pushButton_test_clicked()));
    connect(ui->pushButton_wholeCurvesInteriorVertices, SIGNAL(clicked()),
            this, SLOT(on_pushButton_wholeCurvesInteriorVertices_clicked()));
    connect(ui->pushButton_optimize, SIGNAL(clicked()),
            this, SLOT(on_pushButton_optimize_clicked()));
    connect(ui->pushButton_exprotSelection,SIGNAL(clicked()),
            this, SLOT(on_pushButton_exprotSelection_clicked()));
    connect(ui->pushButton_clear, SIGNAL(clicked()),
            this, SLOT(on_pushButton_clear_clicked()));


    r = ui->doubleSpinBox_collisionradius->value();
    pO.r = r;
    pO.PositionConstraintsWeight = ui->doubleSpinBox_PositionConstraintsWeight->value();
    pO.BendingEnergyCoef = ui->doubleSpinBox_BendingEnergyCoef->value();
    pO.TangentConstraintsCoef = ui->doubleSpinBox_TangentConstraintsCoef->value();
    pO.extension = ui->spinBox_extension->value();
}



void Elastic::on_selectcurve_clicked()
{
    PetMesh* pmesh = pgl->getCurrentMesh();
    if (!pmesh) return;
    if (!pmesh->iscurve()) return;
    PetCurve* pcurve = dynamic_cast<PetCurve*>(pmesh);
    std::vector<unsigned int> idx, f_idx;
    pcurve->getSelectedVertices(idx);
    PetCurve::VertexFaceIter vf_it;
    std::vector<unsigned int>::const_iterator it = idx.begin(), it_end = idx.end();
    for (; it != it_end; ++it)
    {
        for (vf_it = pcurve->vf_iter(pcurve->vertex_handle(*it)); vf_it; ++vf_it)
            f_idx.push_back(vf_it.handle().idx());
    }
    pcurve->setCurvesSelected(f_idx);
    pgl->updateView();
}

void Elastic::on_clearcurves_clicked()
{
    pmesh = pgl->getCurrentMesh();
    if (!pmesh) return;
    if (!pmesh->iscurve()) return;
    pcurve = dynamic_cast<PetCurve*>(pmesh);
    pcurve->clearSelectedCurves();
    pgl->updateView();
}

void Elastic::on_clearvertices_clicked()
{
    pmesh = pgl->getCurrentMesh();
    if (!pmesh) return;
    if (!pmesh->iscurve()) return;
    pcurve = dynamic_cast<PetCurve*>(pmesh);
    pcurve->clearSelectedVertices();
    pgl->updateView();
}


bool Elastic::LineSegmentsCollide(const Point& p1,
                                   const Point& p2,
                                   const Point& q1,
                                   const Point& q2)
{
    OpenMesh::Vec3d p_center((p1 + p2)/2), q_center((q1+q2)/2);
    if ((p_center - q_center).norm() < ((p1-p2).norm() + (q1 - q2).norm())/2 + r)
        return true;
    else
        return false;
}



bool Elastic::getCurrentCurve()
{
    pmesh = pgl->getCurrentMesh();
    if (!pmesh) return false;
    if (!pmesh->iscurve()) return false;
    pcurve = dynamic_cast<PetCurve*>(pmesh);
    return true;
}

bool Elastic::LineSegmentsCollide(const unsigned int i, const unsigned int j)
{
    OpenMesh::HalfedgeHandle he_i(pcurve->halfedge_handle(pcurve->edge_handle(i),0)),
            he_j(pcurve->halfedge_handle(pcurve->edge_handle(j),0));
    OpenMesh::VertexHandle p1(pcurve->from_vertex_handle(he_i)),
            p2(pcurve->to_vertex_handle(he_i)),
            p3(pcurve->from_vertex_handle(he_j)),
            p4(pcurve->to_vertex_handle(he_j));
    return LineSegmentsCollide(
                pcurve->point(p1),
                pcurve->point(p2),
                pcurve->point(p3),
                pcurve->point(p4));

}

bool Elastic::LineSegmentsCollide(const PetCurve::EdgeHandle i, const PetCurve::EdgeHandle j)
{
    OpenMesh::HalfedgeHandle he_i(pcurve->halfedge_handle(i, 0)),
            he_j(pcurve->halfedge_handle(j, 0));
    OpenMesh::VertexHandle p1(pcurve->from_vertex_handle(he_i)),
            p2(pcurve->to_vertex_handle(he_i)),
            p3(pcurve->from_vertex_handle(he_j)),
            p4(pcurve->to_vertex_handle(he_j));
    return LineSegmentsCollide(
                pcurve->point(p1),
                pcurve->point(p2),
                pcurve->point(p3),
                pcurve->point(p4));

}

void Elastic::on_doubleSpinBox_collisionradius_valueChanged(double d)
{
    r = d;
    pO.r = r;
}

double Elastic::LineSegmentsSqDistance(const Point &p0, const Point &p1, const Point &q0, const Point &q1)
{
    Point u = p1 - p0, v = q1 - q0, w = p0 - q0;
    double a = u | u;
    double b = u | v;
    double c = v | v;
    double d = u | w;
    double e = v | w;
    double D = a * c - b * b;
    double sc, sN, sD = D;
    double tc, tN, tD = D;
    if (D < Mepsilon)
    {
        sN = 0.0;
        sD = 1.0;
        tN = e;
        tD = c;
    }
    else
    {
        sN = (b * e - c * d);
        tN = (a * e - b * d);
        if (sN < 0.0)
        {
            sN = 0.0;
            tN = e;
            tD = c;
        }
        else if (sN > sD)
        {
            sN = sD;
            tN = e + b;
            tD = c;
        }
    }

    if (tN < 0.0)
    {
        tN = 0.0;
        if ( -d < 00)
            sN = 0.0;
        else if ( -d > a)
            sN = sD;
        else
        {
            sN = -d;
            sD = a;
        }
    }
    else if (tN > tD)
    {
        tN = tD;
        if ((-d + b ) < 0.0)
            sN = 0;
        else if ((-d + b) > a)
            sN = sD;
        else
        {
            sN = (-d + b);
            sD = a;
        }
    }
    sc = (fabs(sN) < Mepsilon ? 0.0 : sN / sD);
    tc = (fabs(sN) < Mepsilon ? 0.0 : tN / tD);
    Point dP = w + (sc * u) - (tc * v);
    return dP.sqrnorm();
}

double Elastic::CrossLineSegmentsSqDistance(const Point &p0, const Point &p1, const Point &q0, const Point &q1)
{
    Point u = p1 - p0;
    Point v = q1 - q0;
    Point w = (p0 + p1) / 2 - (q0 + q1) / 2;

    Point cross = u % v;

    if (cross.norm() < Mepsilon)
    {
        return (w % v).sqrnorm() / v.sqrnorm();
    }
    else
    {
        double d = cross | w;
        return d * d / cross.sqrnorm();
    }
}

double Elastic::CrossLineSegmentsDistance(const Point &p0, const Point &p1, const Point &q0, const Point &q1)
{
    return sqrt(CrossLineSegmentsSqDistance(p0,p1,q0,q1));
}

double Elastic::LineSegmentsDistance(const Point &p0, const Point &p1, const Point &q0, const Point &q1)
{
    double d = LineSegmentsSqDistance(p0,p1,q0,q1);
    return sqrt(d);
}


double Elastic::LineSegmentsDistance(const unsigned int i, const unsigned int j)
{
    OpenMesh::HalfedgeHandle he_i(pcurve->halfedge_handle(pcurve->edge_handle(i),0)),
            he_j(pcurve->halfedge_handle(pcurve->edge_handle(j),0));
    OpenMesh::VertexHandle p1(pcurve->from_vertex_handle(he_i)),
            p2(pcurve->to_vertex_handle(he_i)),
            p3(pcurve->from_vertex_handle(he_j)),
            p4(pcurve->to_vertex_handle(he_j));
    return LineSegmentsDistance(
                pcurve->point(p1),
                pcurve->point(p2),
                pcurve->point(p3),
                pcurve->point(p4));
}

double Elastic::LineSegmentsDistance(const PetCurve::EdgeHandle i, const PetCurve::EdgeHandle j)
{
    OpenMesh::HalfedgeHandle he_i(pcurve->halfedge_handle(i, 0)),
            he_j(pcurve->halfedge_handle(j, 0));
    OpenMesh::VertexHandle p1(pcurve->from_vertex_handle(he_i)),
            p2(pcurve->to_vertex_handle(he_i)),
            p3(pcurve->from_vertex_handle(he_j)),
            p4(pcurve->to_vertex_handle(he_j));
    return LineSegmentsDistance(
                pcurve->point(p1),
                pcurve->point(p2),
                pcurve->point(p3),
                pcurve->point(p4));
}



void Elastic::EdgesIntersections(const std::vector<unsigned int> &idx,
                                 std::vector<std::pair<unsigned int, unsigned int> > &inter)
{
    if(!getCurrentCurve()) return;
    std::vector<unsigned int>::const_iterator it = idx.begin(), it_end = idx.end(), it2;
    PetMesh::HalfedgeHandle he_hnd, he2_hnd;
    PetMesh::EdgeHandle e_hnd, prev_e_hnd, next_e_hnd;
    for (; it != it_end; ++it)
    {
        e_hnd = pcurve->edge_handle(*it);
        he_hnd = pcurve->halfedge_handle(e_hnd, 0);
        he2_hnd = pcurve->next_halfedge_handle(he_hnd);
        next_e_hnd = pcurve->edge_handle(he2_hnd);
        he2_hnd = pcurve->prev_halfedge_handle(he_hnd);
        prev_e_hnd = pcurve->edge_handle(he2_hnd);
        for (it2 = it; it2 != it_end; ++it2)
        {
            if (int(*it2) == e_hnd.idx()
                    || int(*it2) == next_e_hnd.idx()
                    || int(*it2) == prev_e_hnd.idx()) continue;
            if (LineSegmentsCollide(*it,*it2) && LineSegmentsDistance(*it, *it2) < r)
                inter.push_back(pair<unsigned int, unsigned int>(*it,*it2));
        }
    }
}


void Elastic::EdgesIntersections(const std::vector<PetCurve::EdgeHandle>& idx,
                                 std::vector<std::pair<PetCurve::EdgeHandle, PetCurve::EdgeHandle> >& inter)
{
    if(!getCurrentCurve()) return;
    std::vector<PetCurve::EdgeHandle>::const_iterator it = idx.begin(), it_end = idx.end(), it2;
    PetMesh::HalfedgeHandle he_hnd, he2_hnd;
    PetMesh::EdgeHandle prev_e_hnd, next_e_hnd;
    for (; it != it_end; ++it)
    {
        he_hnd = pcurve->halfedge_handle(*it, 0);
        he2_hnd = pcurve->next_halfedge_handle(he_hnd);
        next_e_hnd = pcurve->edge_handle(he2_hnd);
        he2_hnd = pcurve->prev_halfedge_handle(he_hnd);
        prev_e_hnd = pcurve->edge_handle(he2_hnd);
        for (it2 = it; it2 != it_end; ++it2)
        {
            if (*it2 == *it || *it2 == next_e_hnd || *it2 == prev_e_hnd)
                continue;
            if (LineSegmentsCollide(*it,*it2) && LineSegmentsDistance(*it, *it2) < r)
                inter.push_back(pair<PetCurve::EdgeHandle, PetCurve::EdgeHandle>(*it,*it2));
        }
    }
}

void Elastic::on_pushButton_intersection_clicked()
{
    if(!getCurrentCurve()) return;
    std::vector<PetCurve::EdgeHandle> idxSelectedEdges;
    std::vector<pair<PetCurve::EdgeHandle, PetCurve::EdgeHandle> > inter;
    pcurve->getSelectedEdges(idxSelectedEdges);
    pcurve->clearSelectedEdges();
    EdgesIntersections(idxSelectedEdges, inter);
    idxSelectedEdges.clear();
    std::vector<pair<PetCurve::EdgeHandle, PetCurve::EdgeHandle> >::const_iterator it = inter.begin(),
            it_end = inter.end();
    for (; it != it_end; ++it)
    {
        idxSelectedEdges.push_back((*it).first);
        idxSelectedEdges.push_back((*it).second);
    }
    pcurve->setEdgesSelected(idxSelectedEdges);
    pgl->updateView();
}

void Elastic::on_pushButton_VerticesFromCurves_clicked()
{
    if (!getCurrentCurve()) return;
    pcurve->getSelectedCurves(selectedCurves);
    pcurve->setVerticesSelectedByCurves(selectedCurves);
    pgl->updateView();
}


void Elastic::on_pushButton_wholeCurvesVertices_clicked()
{
    PetMesh* pmesh = pgl->getCurrentMesh();
    if (!pmesh) return;
    if (!pmesh->iscurve()) return;
    PetCurve* pcurve = dynamic_cast<PetCurve*>(pmesh);
    std::vector<unsigned int> idx;
    std::set<int> set_f_idx;
    pcurve->getSelectedVertices(idx);
    PetCurve::VertexFaceIter vf_it;
    std::vector<unsigned int>::const_iterator it = idx.begin(), it_end = idx.end();
    for (; it != it_end; ++it)
    {
        for (vf_it = pcurve->vf_iter(pcurve->vertex_handle(*it)); vf_it; ++vf_it)
            set_f_idx.insert(vf_it.handle().idx());
    }
    std::vector<PetCurve::VertexHandle> vecVertices;
    PetCurve::CurveVertexIter cv_it;
    std::set<int>::const_iterator f_idx_it = set_f_idx.begin(),
            f_idx_it_end = set_f_idx.end();
    for (; f_idx_it != f_idx_it_end; ++f_idx_it)
    {
        for (cv_it = pcurve->fv_iter(pcurve->face_handle(*f_idx_it)); cv_it; ++cv_it)
        {
            vecVertices.push_back(cv_it.handle());
        }
    }
    pcurve->setVerticesSelected(vecVertices);
    pgl->updateView();
}

void Elastic::on_pushButton_setInvolved_clicked()
{
    if (!getCurrentCurve()) return;
    pcurve->getSelectedEdges(edgesToOptimize);
    pcurve->getSelectedVertices(verticesToOptimize);
    curveToOptimize = pcurve;
}

void Elastic::on_pushButton_doitright_clicked()
{
    QString filename = \
            QFileDialog::getOpenFileName(tabPlugin, \
                                         tr("Optimize"), \
                                         "/home/nullas/workspace/PetGL/meshes", \
                                         tr("Optimization (*.opt)"));
    if (filename.isEmpty()) return;
    QFileInfo finfo = QFileInfo(filename);
    QDir dir = finfo.absoluteDir();
    ifstream fin;
    fin.open(filename.toAscii(), ios::in);
    if (!fin.is_open())
    {
        cout<< "Read file error" << filename.toStdString() <<endl;
        return;
    }

    std::string fname;
    fin >> fname;
    if (fin.eof()) return;
    fname = dir.filePath(QString(fname.c_str())).toStdString();
    PetCurve *mesh = new PetCurve();
    if(!mesh->read_curve(QString(fname.c_str()))) return;

    pgl->AddPetMesh(mesh);

    PetMesh* pmesh = mesh;
    if (!pmesh) return;
    if (!pmesh->iscurve()) return;
    curveToOptimize = dynamic_cast<PetCurve*>(pmesh);

    int idx;
    char cType;
    int n;
    std::vector<pair<char, int> > filestruct;
    fin >> cType;
    while (cType != 'E' && !fin.eof())
    {
        fin >> n;
        filestruct.push_back(pair<char, int>(cType, n));
        fin >> cType;
    }

    Point::value_type x, y, z;
    PositionConstraints.clear();
    TangentConstraints.clear();
    verticesToOptimize.clear();
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
                PositionConstraints.push_back(pair<PetCurve::VertexHandle, Point>
                                              (curveToOptimize->vertex_handle(idx),
                                               Point(x,y,z)));
            }
            break;
        case 'T':
            for (int i = 0; i < (*it).second; ++i)
            {
                fin >> idx >> x >> y >> z;
                TangentConstraints.push_back(pair<PetCurve::EdgeHandle, Point>
                                             (curveToOptimize->edge_handle(idx),
                                              Point(x,y,z)));
            }
            break;
        case 'V':
            for (int i =0; i < (*it).second; ++i)
            {
                fin >> idx;
                verticesToOptimize.push_back(curveToOptimize->vertex_handle(idx));
            }
            break;
        default:
            return;
        }
    }

    Ipopt::SmartPtr<Ipopt::TNLP> mynlp = new Optimize(this);
    Ipopt::SmartPtr<Ipopt::IpoptApplication> app = IpoptApplicationFactory();

    app->Options()->SetNumericValue("tol", 1e-7);
    app->Options()->SetStringValue("mu_strategy", "adaptive");
    app->Options()->SetStringValue("output_file", "ipopt.out");
//    app->Options()->SetStringValue("derivative_test","second-order");
//    app->Options()->SetStringValue("linear_solver","ma57");

    Ipopt::ApplicationReturnStatus status;
    status = app->Initialize();
    if (status != Ipopt::Solve_Succeeded)
    {
        std::cout << "\n\n*** Error during initialization!\n" << std::endl;
    }

    // Ask Ipopt to solve the problem
    status = app->OptimizeTNLP(mynlp);

    if (status == Ipopt::Solve_Succeeded)
    {
        std::cout << "\n\n*** The problem solved!\n" << std::endl;
    }
    else
    {
        std::cout << "\n\n*** The problem FAILED!\n" << std::endl;
    }

    pgl->updateView(1);
}


void Elastic::on_doubleSpinBox_PositionConstraintsWeight_editingFinished()
{
    pO.PositionConstraintsWeight = ui->doubleSpinBox_PositionConstraintsWeight->value();
}

void Elastic::on_doubleSpinBox_BendingEnergyCoef_editingFinished()
{
    pO.BendingEnergyCoef = ui->doubleSpinBox_BendingEnergyCoef->value();
}

void Elastic::on_pushButton_test_clicked()
{
    getCurrentCurve();
    curveToOptimize = pcurve;
    verticesToOptimize.clear();
    verticesToOptimize.push_back(curveToOptimize->vertex_handle(2));
    PositionConstraints.clear();
//    PositionConstraints.push_back(pair<PetCurve::VertexHandle, PetCurve::Point>(pcurve->vertex_handle(0),
//                                                                                pcurve->point(vertex_handle(1))));
    Ipopt::SmartPtr<Ipopt::TNLP> mynlp = new Optimize(this);
    Ipopt::SmartPtr<Ipopt::IpoptApplication> app = IpoptApplicationFactory();
    app->Options()->SetNumericValue("tol", 1e-4);
    app->Options()->SetStringValue("mu_strategy", "adaptive");
    app->Options()->SetStringValue("output_file", "ipopt.out");
//    app->Options()->SetStringValue("derivative_test","second-order");
    app->Options()->SetStringValue("linear_solver","ma57");
    Ipopt::ApplicationReturnStatus status;
    status = app->Initialize();
    if (status != Ipopt::Solve_Succeeded) {
    std::cout << "\n\n*** Error during initialization!\n" << std::endl;
    }

    // Ask Ipopt to solve the problem
    status = app->OptimizeTNLP(mynlp);

    if (status == Ipopt::Solve_Succeeded) {
    std::cout << "\n\n*** The problem solved!\n" << std::endl;
    }
    else {
    std::cout << "\n\n*** The problem FAILED!\n" << std::endl;
    }

    pgl->updateView(1);
}

void Elastic::on_pushButton_wholeCurvesInteriorVertices_clicked()
{
    PetMesh* pmesh = pgl->getCurrentMesh();
    if (!pmesh) return;
    if (!pmesh->iscurve()) return;
    PetCurve* pcurve = dynamic_cast<PetCurve*>(pmesh);
    std::vector<unsigned int> idx;
    std::set<int> set_f_idx;
    pcurve->getSelectedVertices(idx);
    PetCurve::VertexFaceIter vf_it;
    std::vector<unsigned int>::const_iterator it = idx.begin(), it_end = idx.end();
    for (; it != it_end; ++it)
    {
        for (vf_it = pcurve->vf_iter(pcurve->vertex_handle(*it)); vf_it; ++vf_it)
            set_f_idx.insert(vf_it.handle().idx());
    }
    std::vector<PetCurve::VertexHandle> vecVertices;
    PetCurve::CurveVertexIter cv_it;
    std::set<int>::const_iterator f_idx_it = set_f_idx.begin(),
            f_idx_it_end = set_f_idx.end();
    for (; f_idx_it != f_idx_it_end; ++f_idx_it)
    {
        for (cv_it = pcurve->fv_iter(pcurve->face_handle(*f_idx_it)); cv_it; ++cv_it)
        {
            vecVertices.push_back(cv_it.handle());
        }
    }
    pcurve->setVerticesSelected(vecVertices);
    vecVertices.clear();
    PetCurve::VertexHandle v_hnd;
    PetCurve::CurveHalfedgeIter ch_it;
    for (f_idx_it = set_f_idx.begin(); f_idx_it != f_idx_it_end; ++f_idx_it)
    {
        for (ch_it = pcurve->fh_iter(pcurve->face_handle(*f_idx_it)); ch_it; ++ch_it)
        {
            if (pcurve->property(pcurve->isCurveHalfEdge, ch_it.handle())) continue;
            v_hnd = pcurve->from_vertex_handle(ch_it.handle());
            vecVertices.push_back(v_hnd);
            v_hnd = pcurve->to_vertex_handle(ch_it.handle());
            vecVertices.push_back(v_hnd);
        }
    }
    pcurve->setVerticesUnelected(vecVertices);
    pgl->updateView();
}

void Elastic::on_doubleSpinBox_TangentConstraintsCoef_editingFinished()
{
    pO.TangentConstraintsCoef = ui->doubleSpinBox_TangentConstraintsCoef->value();
}

void Elastic::on_pushButton_optimize_clicked()
{
    if (verticesToOptimize.empty()) return;
    Ipopt::SmartPtr<Ipopt::TNLP> mynlp = new Optimize(this);
    Ipopt::SmartPtr<Ipopt::IpoptApplication> app = IpoptApplicationFactory();
    app->Options()->SetNumericValue("tol", 1e-9);
    app->Options()->SetStringValue("mu_strategy", "adaptive");
    app->Options()->SetStringValue("output_file", "ipopt.out");
//    app->Options()->SetStringValue("derivative_test","second-order");
//    app->Options()->SetNumericValue("derivative_test_perturbation", 1e-8);
//    app->Options()->SetStringValue("linear_solver","ma57");
    Ipopt::ApplicationReturnStatus status;
    status = app->Initialize();
    if (status != Ipopt::Solve_Succeeded) {
    std::cout << "\n\n*** Error during initialization!\n" << std::endl;
    }

    // Ask Ipopt to solve the problem
    status = app->OptimizeTNLP(mynlp);

    if (status == Ipopt::Solve_Succeeded) {
    std::cout << "\n\n*** The problem solved!\n" << std::endl;
    }
    else {
    std::cout << "\n\n*** The problem FAILED!\n" << std::endl;
    }

    pgl->updateView(1);
}

void Elastic::on_pushButton_exprotSelection_clicked()
{
    if (curveToOptimize == NULL) return;
    QString filename = QFileDialog::getSaveFileName(tabPlugin, "save to", \
                                                    "/home/nullas/workspace/PetGL/meshes", \
                                                    tr("Optimization (*.opt)"));
    if (filename.isEmpty()) return;
    std::ofstream fout;

    fout.open(filename.toAscii(), ios::out);

    QString curvename = curveToOptimize->name;
    fout << curvename.toStdString() << std::endl;

    if (verticesToOptimize.size())
    {
        fout << "V " << verticesToOptimize.size() << std::endl;
    }
    std::vector<unsigned int> idxVertices;
    curveToOptimize->getSelectedVertices(idxVertices);
    if (idxVertices.size())
    {
        fout << "P " << idxVertices.size() << std::endl;
    }
    std::vector<unsigned int> idxEdges;
    curveToOptimize->getSelectedEdges(idxEdges);
    if (idxEdges.size())
    {
        fout << "T " << idxEdges.size() << std::endl;
    }

    fout << 'E' << std::endl;

    for (unsigned int i = 0; i < verticesToOptimize.size(); ++i)
    {
        fout << verticesToOptimize[i].idx() << " ";
    }
    fout << std::endl;
    for (unsigned int i = 0; i < idxVertices.size(); ++i)
    {
        fout << idxVertices[i] << " " << std::endl;
    }
    for (unsigned int i = 0; i < idxEdges.size(); ++i)
    {
        fout << idxEdges[i] << " " << std::endl;
    }
    fout.close();
}

void Elastic::on_pushButton_clear_clicked()
{
    verticesToOptimize.clear();
    TangentConstraints.clear();
    PositionConstraints.clear();
}

void Elastic::on_spinBox_extension_editingFinished()
{
    pO.extension = ui->spinBox_extension->value();
}
