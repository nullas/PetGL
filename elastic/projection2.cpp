#include "projection2.h"
#include "eigen_hamilton.h"
#include <limits>
#include <Eigen/SPQRSupport>
#include <Eigen/UmfPackSupport>
#include <Eigen/Geometry>
#include <stdlib.h>
#include <ctime>
#include <iomanip>

#define M_EP 1e-13

TangentProjection2::TangentProjection2(PetCurve *curve, const Constraints& _cons, const Elastic::pOptimize& pO) :
    check_derivative_(false), tangent_gradient_(true), initialized_(false)
{
    curve_ = curve;
    constraints_ = _cons;
    for (unsigned int i = 0; i < constraints_.PlaneConstraints.size(); ++i)
    {
        constraints_.PlaneConstraintsInfo[i].first.normalize();
    }
    vertices_ = constraints_.VerticesToOptimize;
    parameter_ = pO;
    n_variables_ = 3 * vertices_.size();
    rhs_.setZero(n_variables_);
    lhs_.setZero(n_variables_);
    x_.setZero(n_variables_);
    v_.setZero(n_variables_);
    gradients_.setZero(n_variables_);
    careful_ = true;
    dt_ = 0;
    tol_ = 1e-8;
    for (unsigned int i = 0; i < vertices_.size(); ++i)
    {
        idx_mapping_.insert(std::map<PetCurve::VertexHandle,int>::value_type(vertices_[i], i));
    }
    GetStartPoint();
    GetHalfedges();
    GetEdgeLengths();
    GetCrossings();
    InitialTwisting();
    n_constraints_ = halfedges_.size() + crossings_.size();
    g_ = Eigen::VectorXd(n_constraints_);
    grad_g_ = SpMat(n_variables_, n_constraints_);
    run_.step = -1;
    run_.timing_dt = 0;
    run_.timing_projection = 0;
    run_.timing_tangent_optimal = 0;
    srand(time(NULL));
    fout.open("projection.out");
    if (!fout.is_open())
        std::cout << "open projection.out fail\n" << std::endl;
}

TangentProjection2::~TangentProjection2()
{
    fout.close();
}

int TangentProjection2::GetStartPoint()
{
    Point po;
    for (unsigned int i = 0; i < vertices_.size(); ++i)
    {
        po = curve_->point(vertices_[i]);
        x_(3*i) = po[0];
        x_(3*i + 1) = po[1];
        x_(3*i + 2) = po[2];
    }
    InitialTwisting();
    return 0;
}

int TangentProjection2::point_index(const PetCurve::VertexHandle &v_hnd)
{
    if (idx_mapping_.count(v_hnd))
    {
        return idx_mapping_[v_hnd];
    }
    else
    {
        return -1;
    }
}

Eigen::Vector3d TangentProjection2::point(const PetCurve::VertexHandle& v_hnd, const Eigen::VectorXd& x)
{
    int i = point_index(v_hnd);
    if (i >= 0)
    {
        return Eigen::Vector3d(&x(3 * i));
    }
    else
    {
        return Eigen::Vector3d((curve_->point(v_hnd)).data());
    }
}

void TangentProjection2::GetHalfedges()
{
    PetCurve::CurveIter c_it = curve_->faces_begin(), c_end = curve_->faces_end();
    PetCurve::CurveHalfedgeIter ch_it;
    PetCurve::VertexHandle v0, v1;
    for (; c_it != c_end; ++c_it)
    {
        for (ch_it  = curve_->fh_iter(c_it.handle()); ch_it; ++ch_it)
        {
            v0 = curve_->from_vertex_handle(ch_it.handle());
            v1 = curve_->to_vertex_handle(ch_it.handle());
            if (point_index(v0) >= 0 || point_index(v1) >= 0)
            {
                halfedges_.push_back(ch_it.handle());
                edge_length_.push_back(ComputeEdge(ch_it.handle(), x_).squaredNorm());
            }
        }
    }
}

void TangentProjection2::GetEdgeLengths()
{
    PetCurve::CurveIter c_it = curve_->faces_begin(), c_end = curve_->faces_end();
    PetCurve::CurveHalfedgeIter ch_it;
    PetCurve::VertexHandle v0, v1;
    PetCurve::Point p0, p1;
    double l;
    for (; c_it != c_end; ++c_it)
    {
        l = 0;
        for (ch_it = curve_->fh_iter(c_it.handle()); ch_it; ++ch_it)
        {
            v0 = curve_->from_vertex_handle(ch_it.handle());
            p0 = curve_->point(v0);
            v1 = curve_->to_vertex_handle(ch_it.handle());
            p1 = curve_->point(v1);
            edge_length_mapping_.insert(std::map<PetCurve::HalfedgeHandle,double>::value_type(ch_it.handle(), (p1-p0).norm()));
            l += (p1-p0).norm();
        }
        curve_length_mapping_.insert(std::map<PetCurve::CurveHandle, double>::value_type(c_it.handle(), l));
    }
}

void TangentProjection2::GetCrossings()
{
    PetCurve::HalfedgeHandle prev_h_hnd, next_h_hnd;
    double r_sq = parameter_.r * parameter_.r;
    for (unsigned int i  = 0; i < halfedges_.size(); ++i)
    {
        prev_h_hnd = curve_->prev_halfedge_handle(halfedges_[i]);
        next_h_hnd = curve_->next_halfedge_handle(halfedges_[i]);
        for (unsigned int j = i + 1; j < halfedges_.size(); ++j)
        {
            if (halfedges_[j] == halfedges_[i] || halfedges_[j] == next_h_hnd || halfedges_[j] == prev_h_hnd)
                continue;
            if (LineSegmentsSqDistance(halfedges_[i], halfedges_[j]) < r_sq)
                crossings_.push_back(pair<PetCurve::HalfedgeHandle, PetCurve::HalfedgeHandle>(halfedges_[i], halfedges_[j]));
        }
    }
}

int TangentProjection2::UpdateCrossings(const Eigen::VectorXd& x)
{
    PetCurve::HalfedgeHandle first = first, second;
    int idx = 0;
    int idx_end = crossings_.size();
    double min_dist;
    double distance;
    pair<PetCurve::HalfedgeHandle, PetCurve::HalfedgeHandle> update;
    for (idx = 0; idx < idx_end; ++idx)
    {
        first = crossings_[idx].first;
        second = crossings_[idx].second;
        update = pair<PetCurve::HalfedgeHandle, PetCurve::HalfedgeHandle>(first, second);
        min_dist = LineSegmentsSqDistance(first, second, x);
        for (first = curve_->prev_halfedge_handle(crossings_[idx].first); first != curve_->next_halfedge_handle(crossings_[idx].first); first = curve_->next_halfedge_handle(first))
        {
            for (second = curve_->prev_halfedge_handle(crossings_[idx].second); second != curve_->next_halfedge_handle(crossings_[idx].second); second = curve_->next_halfedge_handle(second))
            {
                distance = LineSegmentsSqDistance(first, second, x);
                if (distance < min_dist)
                {
                    update = pair<PetCurve::HalfedgeHandle, PetCurve::HalfedgeHandle>(first, second);
                    min_dist = distance;
                }
            }
        }
        crossings_[idx] = update;
    }
    return 0;
}

double TangentProjection2::LineSegmentsSqDistance(const Point &p0, const Point &p1, const Point &q0, const Point &q1)
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
    if (D < M_EP)
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
    sc = (fabs(sN) < M_EP ? 0.0 : sN / sD);
    tc = (fabs(sN) < M_EP ? 0.0 : tN / tD);
    Point dP = w + (sc * u) - (tc * v);
    return dP.sqrnorm();
}

double TangentProjection2::LineSegmentsSqDistance(const Eigen::Vector3d& p0 ,const Eigen::Vector3d& p1, const Eigen::Vector3d& q0, const Eigen::Vector3d& q1)
{
    Point pp0(&p0.x());
    Point pp1(&p1.x());
    Point pq0(&q0.x());
    Point pq1(&q1.x());
    return LineSegmentsSqDistance(pp0, pp1, pq0, pq1);
}

double TangentProjection2::LineSegmentsSqDistance(const PetCurve::HalfedgeHandle& h_hnd1, const PetCurve::HalfedgeHandle& h_hnd2)
{
    Point v0 = curve_->point(curve_->from_vertex_handle(h_hnd1));
    Point v1 = curve_->point(curve_->to_vertex_handle(h_hnd1));
    Point v2 = curve_->point(curve_->from_vertex_handle(h_hnd2));
    Point v3 = curve_->point(curve_->to_vertex_handle(h_hnd2));
    return LineSegmentsSqDistance(v0, v1, v2, v3);
}

double TangentProjection2::LineSegmentsSqDistance(const PetCurve::HalfedgeHandle& h_hnd1, const PetCurve::HalfedgeHandle& h_hnd2, const Eigen::VectorXd& x)
{
    Vec3d v0 = point(curve_->from_vertex_handle(h_hnd1), x);
    Vec3d v1 = point(curve_->to_vertex_handle(h_hnd1), x);
    Vec3d v2 = point(curve_->from_vertex_handle(h_hnd2), x);
    Vec3d v3 = point(curve_->to_vertex_handle(h_hnd2), x);
    return LineSegmentsSqDistance(v0, v1, v2, v3);
}

int TangentProjection2::Next()
{
    int rlt = 0;
    if (!initialized_)
    {
        if (check_derivative_)
            rlt = CheckDerivative();
        if (rlt < 0)
            return rlt;
        dt_ = 0;
        OutputStatus();
        initialized_ = true;
    }
    time_t tic = clock(), toc;
    rlt = SymplecticEuler();
    toc = clock();
    run_.timing_tangent_optimal = (double)(toc - tic) / CLOCKS_PER_SEC;
    if (rlt < 0)
        return rlt;
    tic = clock();
    rlt += Projection();
    toc = clock();
    run_.timing_projection = (double)(toc - tic) / CLOCKS_PER_SEC;
    if (rlt < 0)
        return rlt;
    if (run_.step % 10 == 0 || rlt == 1)
    {
        rlt += UpdateMesh();
        if (rlt < 0)
            return rlt;
    }
    rlt += OutputStatus();
    if (rlt < 0)
        return rlt;
    if (rlt > 0)
    {
        fout << "Succceed!!!" << std::endl;
    }
    return rlt;
}

int TangentProjection2::SymplecticEuler()
{
    static Eigen::VectorXd v(n_constraints_);
    static Eigen::SPQR<SpMat> qr;
    ComputeGradients(x_, gradients_);
    ComputeGradientG(x_, grad_g_);
    qr.compute(grad_g_);
    if (qr.info() != 0)
        return -1;
    v = qr.solve(gradients_);
    lhs_ = grad_g_ * v;
    gradients_ -= lhs_;
//    double d = gradients_.dot(lhs_);
//    d *= d;
    double sd = 100;
    sd = std::max(OneNorm(v) / v.rows(), sd) / sd;
    if (MaxNorm(gradients_) < tol_ / sd)
        return 1;
    time_t tic = clock();
    dt_ = dt(x_, gradients_);
    if (dt_ == 0)
    {
        if (!careful_)
            return 1;
        else
        {
            dt_ = dt_full(x_, gradients_);
            if (dt_ == 0)
                return 1;
        }
    }
    time_t toc = clock();
    run_.timing_dt = (double)(toc - tic) / CLOCKS_PER_SEC;
    x_ -= gradients_ * dt_;
    return 0;
}

double TangentProjection2::dt(const Eigen::VectorXd& x, const Eigen::VectorXd& grads)
{
    static const double gr = 0.618;
    static int typical_start = 0;
    double dt = 0;
    double min_energy;
    int rlt = ComputeEnergy(x, min_energy);
    if (rlt != 0)
        return rlt;
    double temp_energy, temp_g;
    Eigen::VectorXd g(n_constraints_);
    int start = std::max(0, typical_start - 1);
    double temp_dt = std::pow(gr, start) * parameter_.dt;
    int i;
    for (i = start; i < parameter_.itertations; ++i)
    {
        lhs_ = x - grads * temp_dt;
        if (ComputeEnergy(lhs_, temp_energy) == 0 && ComputeG(lhs_, g) == 0)
        {
            temp_g = MaxNorm(g);
            if (temp_energy < min_energy && std::abs(temp_energy - min_energy) < std::abs(min_energy) / 20 && temp_g < 1e-6)
            {
                dt = temp_dt;
                min_energy = temp_energy;
                break;
            }
        }
        temp_dt *= gr;
    }
    typical_start = i;
    return dt;
}

double TangentProjection2::dt_full(const VecXd& x, const VecXd& grads)
{
    static const double gr = 0.618;
    double dt = 0;
    double temp_dt = 1e-8;
    double min_energy;
    int rlt = ComputeEnergy(x, min_energy);
    if (rlt != 0)
        return rlt;
    double temp_energy, temp_g;
    Eigen::VectorXd g(n_constraints_);
    for (int i = 0; i < 5; ++i)
    {
        lhs_ = x - grads * temp_dt;
        if (ComputeEnergy(lhs_, temp_energy) == 0 && ComputeG(lhs_, g) == 0)
        {
            temp_g = MaxNorm(g);
            if (temp_energy < min_energy && std::abs(temp_energy - min_energy) < std::abs(min_energy) / 20 && temp_g < 1e-6)
            {
                dt = temp_dt;
                min_energy = temp_energy;
            }
        }
        temp_dt *= gr;
    }
    return dt;
}

Eigen::Vector3d TangentProjection2::ComputeEdge(const PetCurve::HalfedgeHandle &h_hnd, const Eigen::VectorXd x)
{
    PetCurve::VertexHandle v0, v1;
    v0 = curve_->from_vertex_handle(h_hnd);
    v1 = curve_->to_vertex_handle(h_hnd);
    Eigen::Vector3d p0 = point(v0, x);
    Eigen::Vector3d p1 = point(v1, x);
    return p1 - p0;
}

int TangentProjection2::ComputeEnergy(const Eigen::VectorXd& x, double& energy)
{
    energy = 0;
    // bending energy
    PetCurve::CurveIter c_it = curve_->faces_begin(), c_end = curve_->faces_end();
    PetCurve::CurveHalfedgeIter ch_it;
    PetCurve::HalfedgeHandle h_e, h_f;
    PetCurve::VertexHandle v0, v1, v2;
    Eigen::Vector3d e, f;
    double l, l_e, l_f;
    for (; c_it != c_end; ++c_it)
    {
        for (ch_it = curve_->fh_iter(c_it.handle()); ch_it; ++ch_it)
        {
            h_e = ch_it.handle();
            h_f = curve_->next_halfedge_handle(h_e);
            e = ComputeEdge(h_e, x);
            f = ComputeEdge(h_f, x);
            v0 = curve_->from_vertex_handle(h_e);
            v1 = curve_->to_vertex_handle(h_e);
            v2 = curve_->to_vertex_handle(h_f);
            l_e = edge_length_mapping_[h_e];
            l_f = edge_length_mapping_[h_f];
            l = l_e + l_f;
            l /= 2;
            energy += 2*parameter_.BendingEnergyCoef / l * (l_e*l_f- e.dot(f)) / (l_e*l_f + e.dot(f));
        }
    }
    // twisting energy
    c_it = curve_->faces_begin(), c_end = curve_->faces_end();
    double theta;
    for (; c_it != c_end; ++c_it)
    {
        l = curve_length_mapping_[c_it.handle()];
        theta = UpdateTheta(c_it.handle(), x);
        energy += parameter_.TwistingEnergyCoef * theta * theta / l;
    }
    // position constraints
    std::vector<pair<PetCurve::VertexHandle, PetCurve::Point> >::const_iterator pc_it = constraints_.PositionConstraints.begin(),
            pc_end = constraints_.PositionConstraints.end();
    PetCurve::VertexHandle v;
    Vec3d p, p_dest;
    for(; pc_it != pc_end; ++pc_it)
    {
        v = pc_it->first;
        p = point(v, x);
        p_dest = Vec3d((pc_it->second).data());
        energy += parameter_.PositionConstraintsWeight * (p - p_dest).squaredNorm();
    }
    // plane constraints
    int i = 0, end = constraints_.PlaneConstraints.size();
    Vec3d axis;
    double plane_energy;
    for (; i < end; ++i)
    {
        v = constraints_.PlaneConstraints[i];
        p = point(v, x);
        axis = Vec3d(constraints_.PlaneConstraintsInfo[i].first.data());
        plane_energy = p.dot(axis) + constraints_.PlaneConstraintsInfo[i].second;
        energy += parameter_.PlaneConstraintsCoef * plane_energy * plane_energy;
    }
    return 0;
}

int TangentProjection2::ComputeG(const Eigen::VectorXd& x, Eigen::VectorXd& g)
{
    Eigen::Vector3d e, f;
    unsigned int i = 0;
    for (; i < halfedges_.size(); ++i)
    {
        e = ComputeEdge(halfedges_[i], x);
        g[i] = e.squaredNorm() - edge_length_[i];
    }
    UpdateCrossings(x);
    unsigned int base = i;
    for (i = 0; i < crossings_.size(); ++i)
    {
        g[base + i] = CrossLineSegmentDistance(i, x);
    }
    return 0;
}

double TangentProjection2::CrossLineSegmentDistance(unsigned int i, const Eigen::VectorXd& x)
{
    Eigen::Vector3d p0 = point(curve_->from_vertex_handle(crossings_[i].first), x);
    Eigen::Vector3d p1 = point(curve_->to_vertex_handle(crossings_[i].first), x);
    Eigen::Vector3d q0 = point(curve_->from_vertex_handle(crossings_[i].second), x);
    Eigen::Vector3d q1 = point(curve_->to_vertex_handle(crossings_[i].second), x);
    Eigen::Vector3d w = (p0 - p1).cross(q0 - q1);
    return w.dot((p0 + p1) / 2 - (q0 + q1) / 2);
}

int TangentProjection2::ComputeGradientG(const Eigen::VectorXd& x, SpMat &g)
{
    g.setZero();
    triplet_.clear();
    unsigned int i = 0;
    int j;
    Vec3d p;
    PetCurve::VertexHandle v;
    for (; i < halfedges_.size(); ++i)
    {
        p = ComputeEdge(halfedges_[i], x);
        v = curve_->from_vertex_handle(halfedges_[i]);
        j = point_index(v);
        if (j >= 0)
        {
            InsertTriplet(j, i, -2 * p, triplet_);
        }
        v = curve_->to_vertex_handle(halfedges_[i]);
        j = point_index(v);
        if (j >= 0)
        {
            InsertTriplet(j, i, 2 * p, triplet_);
        }
    }
    int base = i;
    for (i = 0; i < crossings_.size(); ++i)
    {
        ComputeGradientCrossing(i, base, x, triplet_);
    }
    g.setFromTriplets(triplet_.begin(), triplet_.end());
    if (!g.isCompressed())
        g.makeCompressed();
    return 0;
}

void TangentProjection2::ComputeGradientCrossing(const int idx_cross, const int base, const Eigen::VectorXd& x, std::vector<Tri>& triplet)
{
    /*------------------------------------------
                   p0_  q1
                    |\ /
                      /
                    |/ \
                   q0-  p1
    ------------------------------------------*/
    PetCurve::VertexHandle h_p0 = curve_->from_vertex_handle(crossings_[idx_cross].first);
    PetCurve::VertexHandle h_p1 = curve_->to_vertex_handle(crossings_[idx_cross].first);
    PetCurve::VertexHandle h_q0 = curve_->from_vertex_handle(crossings_[idx_cross].second);
    PetCurve::VertexHandle h_q1 = curve_->to_vertex_handle(crossings_[idx_cross].second);
    Vec3d p0 = point(h_p0, x);
    Vec3d q0 = point(h_q0, x);
    Vec3d p1 = point(h_p1, x);
    Vec3d q1 = point(h_q1, x);
    int idx_p0 = point_index(h_p0);
    int idx_p1 = point_index(h_p1);
    int idx_q0 = point_index(h_q0);
    int idx_q1 = point_index(h_q1);
    Vec3d p;
    if (idx_p0 >= 0)
    {
        p = (q1 - p1).cross(q0 - q1);
        InsertTriplet(idx_p0, idx_cross + base, p, triplet);
    }
    if (idx_p1 >= 0)
    {
        p = (p0 - q1).cross(q0 - q1);
        InsertTriplet(idx_p1, idx_cross + base, p, triplet);
    }
    if (idx_q0 >= 0)
    {
        p = (p0 - p1).cross(q1 - p1);
        InsertTriplet(idx_q0, idx_cross + base, p, triplet);
    }
    if (idx_q1 >= 0)
    {
        p = (p0 - p1).cross(p1 - q0);
        InsertTriplet(idx_q1, idx_cross + base, p, triplet);
    }
}

void TangentProjection2::InsertTriplet(const int i, const int j, const Vec3d& v, std::vector<Tri>& triplet)
{
    triplet.push_back(Tri(3*i, j, v(0)));
    triplet.push_back(Tri(3*i + 1, j, v(1)));
    triplet.push_back(Tri(3*i + 2, j, v(2)));
}

double TangentProjection2::CrossingDistance(const unsigned int i, const Eigen::VectorXd& x)
{
    Eigen::Vector3d p0 = point(curve_->from_vertex_handle(crossings_[i].first), x);
    Eigen::Vector3d p1 = point(curve_->to_vertex_handle(crossings_[i].first), x);
    Eigen::Vector3d q0 = point(curve_->from_vertex_handle(crossings_[i].second), x);
    Eigen::Vector3d q1 = point(curve_->to_vertex_handle(crossings_[i].second), x);
    return LineSegmentsSqDistance(p0, p1, q0, q1);
}

int TangentProjection2::ComputeGradients(const Eigen::VectorXd& x, Eigen::VectorXd& gradients)
{
    gradients.setZero(n_variables_);
    // bending energy
    PetCurve::CurveIter c_it = curve_->faces_begin(), c_end = curve_->faces_end();
    PetCurve::CurveHalfedgeIter ch_it;
    PetCurve::HalfedgeHandle h_e, h_f;
    PetCurve::VertexHandle v0, v1, v2;
    Eigen::Vector3d e, f;
    int i;
    double l, l_e, l_f, d, coef;
    for (; c_it != c_end; ++c_it)
    {
        for (ch_it = curve_->fh_iter(c_it.handle()); ch_it; ++ch_it)
        {
            h_e = ch_it.handle();
            h_f = curve_->next_halfedge_handle(h_e);
            e = ComputeEdge(h_e, x);
            f = ComputeEdge(h_f, x);
            v0 = curve_->from_vertex_handle(h_e);
            v1 = curve_->to_vertex_handle(h_e);
            v2 = curve_->to_vertex_handle(h_f);
            l_e = edge_length_mapping_[h_e];
            l_f = edge_length_mapping_[h_f];
            l = l_e + l_f;
            d = l_e * l_f;
            l /= 2;
            coef = 4*parameter_.BendingEnergyCoef / l * d / pow(d + e.dot(f),2);
            i = point_index(v0);
            if (i >= 0)
            {
                gradients.segment(3 * i, 3) += coef * f;
            }
            i = point_index(v1);
            if (i >= 0)
            {
                gradients.segment(3 * i, 3) += coef * (e - f);
            }
            i = point_index(v2);
            if (i >= 0)
            {
                gradients.segment(3 * i, 3) += -coef * e;
            }
        }
    }
    // twisting energy
    c_it = curve_->faces_begin(), c_end = curve_->faces_end();
    double theta;
    Vec3d Kb;
    for (; c_it != c_end; ++c_it)
    {
        l = curve_length_mapping_[c_it.handle()];
        theta = UpdateTheta(c_it.handle(), x);
        for (ch_it = curve_->fh_iter(c_it.handle()); ch_it; ++ch_it)
        {
            h_e = ch_it.handle();
            h_f = curve_->next_halfedge_handle(h_e);
            e = ComputeEdge(h_e, x);
            f = ComputeEdge(h_f, x);
            v0 = curve_->from_vertex_handle(h_e);
            v1 = curve_->to_vertex_handle(h_e);
            v2 = curve_->to_vertex_handle(h_f);
            Kb = ComputeKb(e, f);
            i = point_index(v0);
            if (i >= 0)
            {
                gradients.segment(3 * i, 3) -= parameter_.TwistingEnergyCoef * theta / l / e.norm() * Kb;
            }
            i = point_index(v1);
            if (i >= 0)
            {
                gradients.segment(3 * i, 3) -= parameter_.TwistingEnergyCoef * theta / l * (1/f.norm() - 1/e.norm()) * Kb;
            }
            i = point_index(v2);
            if (i >= 0)
            {
                gradients.segment(3 * i, 3) -= parameter_.TwistingEnergyCoef * theta / l / -f.norm() * Kb;
            }
        }
    }
    // position constraints
    std::vector<pair<PetCurve::VertexHandle, PetCurve::Point> >::const_iterator pc_it = constraints_.PositionConstraints.begin(),
            pc_end = constraints_.PositionConstraints.end();
    PetCurve::VertexHandle v;
    Vec3d p, p_dest;
    int idx;
    for(; pc_it != pc_end; ++pc_it)
    {
        v = pc_it->first;
        idx = point_index(v);
        if (idx >= 0)
        {
            p = point(v, x);
            p_dest = Vec3d((pc_it->second).data());
            gradients.segment(3*idx, 3) += 2*parameter_.PositionConstraintsWeight * (p - p_dest);
        }
    }
    // plane constraints
    int end = constraints_.PlaneConstraints.size();
    Vec3d axis;
    double plane_energy;
    for (i = 0; i < end; ++i)
    {
        v = constraints_.PlaneConstraints[i];
        idx = point_index(v);
        if (idx >= 0)
        {
            p = point(v, x);
            axis = Vec3d(constraints_.PlaneConstraintsInfo[i].first.data());
            plane_energy = p.dot(axis) + constraints_.PlaneConstraintsInfo[i].second;
            gradients.segment(3*idx,3) += 2*parameter_.PlaneConstraintsCoef * plane_energy * axis;
        }
    }
    return 0;
}

TangentProjection2::Vec3d TangentProjection2::ComputeGradientBending(const Vec3d& e, const Vec3d f, int i)
{
    double r = 2 * e.dot(f) / (e.norm()*f.norm() + e.dot(f));
    double deno = -e.norm()*f.norm() - e.dot(f);
    Vec3d rlt;
    rlt.setZero(3);
    if (i == -1)
    {
        rlt = -2*f - r*(-e/e.norm()*f.norm() - f);
    }
    if (i == 0)
    {
        rlt = 2*f - 2*e + r*(-e/e.norm()*f.norm() - f + f/f.norm()*e.norm() + e);
    }
    if (i == 1)
    {
        rlt = 2*e - r*(f/f.norm()*e.norm() + e);
    }
    rlt /= deno;
    return rlt;
}

double TangentProjection2::UpdateTheta(const PetCurve::CurveHandle& curve, const Eigen::VectorXd& x)
{
    double writhe_fraction = ComputeWritheFraction(curve, x);
    double diff = numeric_limits<double>::max();
    double writhe = curve_twisting_[curve];
    int twist_times = curve_twisting_times_[curve];
    int offset, temp_writhe;
    for (offset = -1; offset <= 1; ++offset)
    {
        if (std::abs(writhe_fraction + (twist_times + offset) * 2 * M_PI - writhe) <= diff)
        {
            temp_writhe = twist_times + offset;
            diff = std::abs(writhe_fraction + (twist_times + offset) * 2 * M_PI - writhe);
        }
    }
    curve_twisting_times_[curve] = temp_writhe;
    writhe = writhe_fraction + temp_writhe * 2 * M_PI;
    curve_twisting_[curve] = writhe;
    double theta = curve_material_[curve];
    theta -= writhe;
    return theta;
}

double TangentProjection2::ComputeWritheFraction(const PetCurve::CurveHandle& curve, const Eigen::VectorXd& x)
{
    // find a vector perpendicular to t_0;
    PetCurve::CurveHalfedgeIter ch_it;
    ch_it = curve_->fh_iter(curve);
    Vec3d t_0 = ComputeEdge(ch_it.handle(), x);
    t_0.normalize();
    Vec3d ref = t_0.cross(Vec3d(1,0,0));
    if (ref.norm() < 1e-8)
        ref = t_0.cross(Vec3d(0,0,1));
    ref.normalize();
    // parallel transport
    Vec3d p = ref;
    PetCurve::HalfedgeHandle h_e, h_f;
    Vec3d e, f;
    for (ch_it = curve_->fh_iter(curve); ch_it; ++ch_it)
    {
        h_e = ch_it.handle();
        h_f = curve_->next_halfedge_handle(h_e);
        e = ComputeEdge(h_e, x);
        f = ComputeEdge(h_f, x);
        p = ParallelTransport(e, f, p);
    }
    double writhe_fraction = ComputeDifferenceAngle(t_0, ref, p);
    return writhe_fraction;
}

int TangentProjection2::InitialTwisting()
{
    PetCurve::CurveIter c_it = curve_->faces_begin(),
            c_end = curve_->faces_end();
    double material = (parameter_.twisting_times + parameter_.twisting_fraction) * 2 * M_PI;
    double writhe;
    for (; c_it != c_end; ++c_it)
    {
        curve_material_[c_it.handle()] = material;
        writhe = ComputeWritheFraction(c_it.handle(), x_);
        curve_twisting_[c_it.handle()] = writhe;
        curve_twisting_times_[c_it.handle()] = 0;
    }
    return 0;
}

double TangentProjection2::ComputeDifferenceAngle(const Vec3d &axis, const Vec3d &up, const Vec3d &x)
{
    Vec3d axis_normed = axis.normalized();
    Vec3d up_normed = up.normalized();
    Vec3d normal = axis_normed.cross(up_normed);
    double pr_y = normal.dot(x);
    double pr_x = up_normed.dot(x);
    return atan2(pr_y, pr_x);
}

Eigen::Vector3d TangentProjection2::ParallelTransport(Vec3d t1, Vec3d t2, const Vec3d& x)
{
    t1.normalize();
    t2.normalize();
    Vec3d cross = t1.cross(t2);
    if (cross.norm() < M_EP)
    {
        return x;
    }
    else
    {
        cross.normalize();
        Vec3d n1 = cross.cross(t1);
        Vec3d n2 = cross.cross(t2);
        double y = x.dot(n1);
        double z = x.dot(cross);
        return n2 * y + cross * z;
    }
}

Eigen::Vector3d TangentProjection2::ComputeKb(const Eigen::Vector3d& e, const Eigen::Vector3d& f)
{
    Eigen::Vector3d rlt = e.cross(f);
    rlt *= 2 / (e.norm() * f.norm() + e.dot(f));
    return rlt;
}

Eigen::Matrix3d TangentProjection2::ComputeJKb(const Eigen::Vector3d& e, const Eigen::Vector3d& f, int i)
{
    Eigen::Vector3d Kb = ComputeKb(e, f);
    Eigen::Matrix3d m = Eigen::Matrix3d::Zero();
    if (i == -1)
        m += CrossMatrix(f * 2) + Kb * (f + e/e.norm()*f.norm()).transpose();
    else if (i == 0)
        m += CrossMatrix((e + f) * -2) + Kb * (e + f/f.norm()*e.norm() - f - e/e.norm()*f.norm()).transpose();
    else if (i == 1)
        m += CrossMatrix(e * 2) - Kb * (e + f/f.norm()*e.norm()).transpose();
    m /= e.norm()*f.norm() + e.dot(f);
    return m;
}

Eigen::Matrix3d TangentProjection2::CrossMatrix(const Eigen::Vector3d &p)
{
    Eigen::Matrix3d m = Eigen::Matrix3d::Zero();
    m(0,1) = -p[2];
    m(0,2) = p[1];
    m(1,2) = -p[0];
    m(1,0) = p[2];
    m(2,0) = -p[1];
    m(2,1) = p[0];
    return m;
}

int TangentProjection2::Projection()
{
    static Eigen::VectorXd lambda(n_constraints_), d_lambda(n_constraints_);
    static SpMat grad_g_T(n_constraints_, n_variables_), GTG(n_constraints_, n_constraints_);
    int rlt = ComputeGradientG(x_, grad_g_);
    if (rlt != 0)
        return rlt;
    lambda.setZero(n_constraints_);
    grad_g_T = grad_g_.transpose();
    GTG = grad_g_T * grad_g_;
//    Eigen::SPQR<SpMat> solver2(GTG);
//    int i2 = solver2.rank();
    Eigen::SPQR<SpMat> solver(GTG);
    int i = parameter_.itertations;
    do
    {
        lhs_ = x_ + grad_g_ * lambda;
        rlt = ComputeG(lhs_, g_);
        if (rlt != 0)
            return -1;
        d_lambda = solver.solve(g_);
        lambda -= d_lambda;
        i--;
    }
    while (MaxNorm(d_lambda) > 1e-8 && i > 0 && MaxNorm(g_) > 1e-11);
    if (i == 0)
        return -1;
    lambda -= d_lambda;
    x_ += grad_g_ * lambda;
    rlt = ComputeG(x_, g_);
    if (rlt != 0)
        return -1;
    if (MaxNorm(g_) > 1e-8)
        return -1;
    return 0;
}

double TangentProjection2::MaxNorm(const Eigen::VectorXd& x)
{
    double low = x.minCoeff();
    double up = x.maxCoeff();
    low = std::abs(low);
    up = std::abs(up);
    return low < up ? up : low;
}

double TangentProjection2::OneNorm(const VecXd &v)
{
    double rlt = 0;
    for (int i = 0; i < v.rows(); ++i)
    {
        rlt += std::abs(v(i));
    }
    return rlt;
}

int TangentProjection2::UpdateMesh()
{
    int i = 0, end = vertices_.size();
    Vec3d p;
    PetCurve::VertexHandle v;
    for (; i < end; ++i)
    {
        v = vertices_[i];
        p = point(v, x_);
        curve_->set_point(v, Point(p(0), p(1), p(2)));
    }
    return 0;
}

void TangentProjection2::check_derivative(bool check)
{
    check_derivative_ = check;
}

int TangentProjection2::CheckDerivative()
{
    fout << "checking first derivatives..." << std::endl;
    fout << std::scientific;
    GetStartPoint();
    int rlt = ComputeGradients(x_, gradients_);
    if (rlt != 0)
        return rlt;
    double base_energy, energy, err;
    rlt = ComputeEnergy(x_, base_energy);
    if (rlt != 0)
        return rlt;
    int err_number = 0;
    double random_number;
    for (int i = 0; i < x_.rows(); ++i)
    {
        do
            random_number = 1e-8 * (Rand() - 0.5);
        while (std::abs(random_number) < M_EP);
        x_[i] += random_number;
        rlt = ComputeEnergy(x_, energy);
        if (rlt != 0)
            return rlt;
        energy -= base_energy;
        energy /= random_number;
        err = RelativeError(gradients_[i], energy);
        if (err != 0)
        {
            ++err_number;
            fout << "grad_f" << "\t[" << i << "\t]" << "\t" << gradients_[i] << "\t" << energy << "\t" << err << std::endl;
        }
        GetStartPoint();
    }
    rlt = ComputeGradientG(x_, grad_g_);
    if (rlt != 0)
        return rlt;
    Eigen::VectorXd base_constraints(n_constraints_), constraints(n_constraints_);
    rlt = ComputeG(x_, base_constraints);
    if (rlt != 0)
        return rlt;
    SpMat GgT = grad_g_.transpose();
    for (int i = 0; i < x_.rows(); ++i)
    {

        x_[i] += random_number;
        rlt = ComputeG(x_, constraints);
        if (rlt != 0)
            return rlt;
        constraints -= base_constraints;
        constraints /= random_number;
        SpMat::InnerIterator it(GgT, i);
        for (int j = 0; j < constraints.rows(); ++j)
        {
            if (it && it.row() == j)
            {
                err = RelativeError(it.value(), constraints(j));
                if (err != 0)
                {
                    ++err_number;
                    fout << "grad_g" << "\t[" << i << "\t," << j << "\t]" << "\t" << it.value() << "\t" << constraints(j) << "\t" << err << std::endl;
                }
                ++it;
            }
            else
            {
                err = RelativeError(0, constraints(j));
                if (err != 0)
                {
                    ++err_number;
                    fout << "grad_g*" << "\t[" << i << "\t," << j << "\t]" << "\t" << std::setw(12) << 0 << "\t" << constraints(j) << "\t" << err << std::endl;
                }
            }
        }
        GetStartPoint();
    }
    if (err_number == 0)
        fout << "no error found in derivatives" << std::endl;
    else
        fout << "find " << err_number << " errors" << std::endl;
    fout << "finish checking.." << std::endl;
    return 0;
}

double TangentProjection2::RelativeError(const double x, const double y)
{
    if (x - y < M_EP)
        return 0;
    if (x < 1e-4)
        return std::abs(x - y);
    else
        return std::abs(x - y) / std::abs(x);
}

double TangentProjection2::Rand()
{
    return (double)rand() / 26681;
}

int TangentProjection2::OutputStatus()
{
    static time_t rawtime;
    static struct tm *timeinfo;
    static char timestring[100];
    static clock_t tic = clock();
    ++run_.step;
    double energy;
    int rlt = ComputeEnergy(x_, energy);
    if (rlt != 0)
        return rlt;
    rlt = ComputeG(x_, g_);
    if (rlt != 0)
        return rlt;
    if (run_.step % 5 == 0)
    {
        time(&rawtime);
        timeinfo = localtime(&rawtime);
        strftime(timestring, 100, "%T", timeinfo);
        fout << setw(15) << "step" << setw(15) << "dt" << setw(15) << "energy" << setw(15) << "violation" << setw(15) << "timing";
        fout << setw(15) << "tan optimal" << setw(15) << "find dt" << setw(15) << "projection\t";
        fout << timestring << std::endl;
    }
    clock_t toc = clock();
    fout << setw(15) << run_.step << setw(15) << dt_;
    fout << setw(15) << energy << setw(15) << MaxNorm(g_) << setw(12) << (double)(toc - tic) / CLOCKS_PER_SEC;
    fout << setw(15) << run_.timing_tangent_optimal << setw(15) << run_.timing_dt << setw(15) << run_.timing_projection << std::endl;
    tic = toc;
    if (run_.step >= parameter_.max_steps)
        return 1;
    return 0;
}

void TangentProjection2::tol(double _tol)
{
    tol_ = _tol;
}

void TangentProjection2::careful(bool _c)
{
    careful_ = _c;
}

