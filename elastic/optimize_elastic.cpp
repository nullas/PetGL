#include "optimize_elastic.h"

#include <cmath>
#include <cblas.h>
#include <limits>

#ifndef UNUSED
#define UNUSED(x) ((void)x)
#endif

#ifndef M_EP
#define M_EP 1e-8
#endif


OptimizeElastic::OptimizeElastic(Elastic* p) : pElastic(p), vertices(p->verticesToOptimize)
{
    n_variables = pElastic->verticesToOptimize.size() * 3;
    curve = p->curveToOptimize;
    assert(curve != NULL);
    pOp = &(p->pO);
    idxMapping.clear();
    std::vector<PetCurve::VertexHandle>::const_iterator it = vertices.begin(), it_end = vertices.end();
    PetCurve::VertexHandle v_hnd, v0_hnd, v1_hnd;
    int i = 0;
    for (; it != it_end; ++it)
    {
        idxMapping.insert(std::map<PetCurve::VertexHandle,int>::value_type(*it, i));
        ++i;
    }
    additionalPoints.clear();
    additionalPoints.push_back(OpenMesh::Vec3d(0,0,0));
    std::set<PetCurve::VertexHandle> setvertices(vertices.begin(), vertices.end());
    std::set<PetCurve::VertexHandle>::const_iterator v_hnd_end = setvertices.end();
    PetCurve::CurveIter c_it = curve->faces_begin(), c_end = curve->faces_end();
    int first, second;
    Point p0, p1, pc;
    PetCurve::CurveHalfedgeIter ch_it;
    PetCurve::HalfedgeHandle tmp_e_hnd;
    for (; c_it != c_end; ++c_it)
    {
        for (ch_it = curve->fh_iter(c_it.handle()); ch_it; ++ch_it)
        {
            if (!curve->property(curve->isCurveHalfEdge, ch_it.handle())) continue;
            v0_hnd = curve->from_vertex_handle(ch_it.handle());
            v_hnd = curve->to_vertex_handle(ch_it.handle());
            if (setvertices.find(v0_hnd) != v_hnd_end || setvertices.find(v_hnd) != v_hnd_end)
            {
                p0 = curve->point(v0_hnd);
                pc = curve->point(v_hnd);
                edges.push_back(curve->edge_handle(ch_it.handle()));
                first = insertAdditionalPoint(v0_hnd);
                second = insertAdditionalPoint(v_hnd);
                idxVertexEdges.push_back(pair<int,int>(first,second));
                edgesSqLength.push_back((pc - p0).sqrnorm());
            }
            tmp_e_hnd = curve->next_halfedge_handle(ch_it.handle());
            if (!curve->property(curve->isCurveHalfEdge, tmp_e_hnd)) continue;
            v1_hnd = curve->to_vertex_handle(tmp_e_hnd);
            if (setvertices.find(v0_hnd) != v_hnd_end || setvertices.find(v_hnd) != v_hnd_end
                    || setvertices.find(v1_hnd) != v_hnd_end)
            {
                p0 = curve->point(v0_hnd);
                pc = curve->point(v_hnd);
                p1 = curve->point(v1_hnd);
                EnergyVertices.push_back(v_hnd);
                i = insertAdditionalPoint(v0_hnd);
                idxPrevVertex.push_back(i);
                i = insertAdditionalPoint(v_hnd);
                idxTheVertex.push_back(i);
                i = insertAdditionalPoint(v1_hnd);
                idxNextVertex.push_back(i);
                VerticesWeight.push_back(((p0 - pc).norm() + (p1 - pc).norm())/2);
                EdgesLengthProduct.push_back( (p0 - pc).norm() * (p1 - pc).norm());
            }

        }
    }
    double tmp;
    std::vector<pair<PetCurve::VertexHandle, PetCurve::Point> >::const_iterator p_it = p->PositionConstraints.begin(),
            p_it_end = p->PositionConstraints.end();
    for(; p_it != p_it_end; ++p_it)
    {
        if (idxMapping.count((*p_it).first) && idxMapping[(*p_it).first] >= 0)
        {
            positions.push_back((*p_it).second);
            PositionConstraints.push_back(idxMapping[(*p_it).first]);
        }
    }
    Point p0d, p1d, pcd;
    std::vector<pair<PetCurve::EdgeHandle, PetCurve::Point> >::const_iterator t_it = p->TangentConstraints.begin(),
            t_it_end = p->TangentConstraints.end();
    for (; t_it != t_it_end; ++t_it)
    {
        tmp_e_hnd = curve->halfedge_handle(t_it->first, 0);
        v0_hnd = curve->from_vertex_handle(tmp_e_hnd);
        v1_hnd = curve->to_vertex_handle(tmp_e_hnd);
        if ((idxMapping.count(v0_hnd) && idxMapping[v0_hnd] >= 0) || (idxMapping.count(v1_hnd) && idxMapping[v1_hnd] >= 0))
        {
            first = insertAdditionalPoint(v0_hnd);
            second = insertAdditionalPoint(v1_hnd);
            p0d = curve->point(v0_hnd);
            p1d = curve->point(v1_hnd);
            tmp = (p1d - p0d).norm();
            TangentConstraints.push_back(pair<int,int>(first, second));
            pcd = t_it->second;
            pcd /= tmp * pcd.norm();
            tangents.push_back(pcd);
        }
    }
    int end;
    end = p->PlaneConstraints.size();
    i = 0;
    for (; i < end; ++i)
    {
        if (idxMapping.count(p->PlaneConstraints[i]) && idxMapping[p->PlaneConstraints[i]] >= 0)
        {
            tmp = p->PlaneConstraintsInfo[i].first.norm();
            PlaneConstraintsInfo.push_back(pair<Point, double>(p->PlaneConstraintsInfo[i].first / tmp,
                                                          p->PlaneConstraintsInfo[i].second / tmp));
            PlaneConstraints.push_back(idxMapping[p->PlaneConstraints[i]]);
        }
    }

    // material frames at two ends
    material_frames_at_ends_.first = p->MaterialFrameConstraints[0];
    material_frames_at_ends_.second = p->MaterialFrameConstraints[1];
    c_it = curve->faces_begin(), c_end = curve->faces_end();
    for (; c_it != c_end; ++c_it)
    {
        ch_it = curve->fh_iter(c_it.handle());
        PetCurve::CurveHalfedgeIter tmp_ch_it = ch_it;
        while (ch_it && !curve->property(curve->isCurveHalfEdge, ch_it.handle()))
        {
            ++ch_it;
        }
        halfedge_at_ends_.first = ch_it.handle();
        while (ch_it && curve->property(curve->isCurveHalfEdge, ch_it.handle()))
        {
            tmp_ch_it = ch_it;
            ++ch_it;
        }
        halfedge_at_ends_.second = tmp_ch_it.handle();
        if(curve->next_halfedge_handle(tmp_ch_it) == halfedge_at_ends_.first)
        {
            halfedge_at_ends_.second = halfedge_at_ends_.first;
        }
    }
    RectifyMaterialFrameAtEnds(halfedge_at_ends_.first, material_frames_at_ends_.first);
    RectifyMaterialFrameAtEnds(halfedge_at_ends_.second, material_frames_at_ends_.second);
    p0 = curve->point(curve->from_vertex_handle(halfedge_at_ends_.first));
    p1 = curve->point(curve->to_vertex_handle(halfedge_at_ends_.first));
    previous_end_positions_.first = p1 - p0;
    p0 = curve->point(curve->from_vertex_handle(halfedge_at_ends_.second));
    p1 = curve->point(curve->to_vertex_handle(halfedge_at_ends_.second));
    previous_end_positions_.second = p1 - p0;
    c_it = curve->faces_begin(), c_end = curve->faces_end();
    total_length_ = ComputeTotalLength();

    // num of constraints
    EdgesIntersections();
    n_constraints = edges.size() + crosses.size();
}


OptimizeElastic::~OptimizeElastic()
{

}


//only cares the additional Point
int OptimizeElastic::insertAdditionalPoint(const PetCurve::VertexHandle &v_hnd)
{
    if (idxMapping.count(v_hnd))
    {
        return idxMapping[v_hnd];
    }
    else
    {
        int i = -additionalPoints.size();
        idxMapping.insert(std::map<PetCurve::VertexHandle, int>::value_type(v_hnd,i));
        additionalPoints.push_back(curve->point(v_hnd));
        return i;
    }
}

bool OptimizeElastic::get_nlp_info(Ipopt::Index &n, Ipopt::Index &m,
                            Ipopt::Index &nnz_jac_g, Ipopt::Index &nnz_h_lag,
                            Ipopt::TNLP::IndexStyleEnum &index_style)
{
    n = n_variables;
    m = n_constraints;
    nnz_jac_g = edges.size() * 6 + crosses.size()  * n_variables;
    nnz_h_lag = (n_variables + 1) * n_variables / 2;
    index_style = Ipopt::TNLP::C_STYLE;
    return true;
}

bool OptimizeElastic::get_bounds_info(Ipopt::Index n, Ipopt::Number *x_l, Ipopt::Number *x_u,
                               Ipopt::Index m, Ipopt::Number *g_l, Ipopt::Number *g_u)
{
    assert(n == n_variables);
    assert(m == n_constraints);
    for ( int i = 0; i < n; i++)
    {
        x_l[i] = -2e19;
        x_u[i] =  2e19;
    }
    int end = edges.size();
    for (int i = 0; i < end; ++i)
    {
        g_l[i] = g_u[i] = edgesSqLength[i];
    }
    end = edges.size() + crosses.size();
    for (int i = edges.size(); i < end; ++i)
    {
        g_l[i] = 0;
        g_u[i] = 0;
    }
    return true;
}


bool OptimizeElastic::get_starting_point(Ipopt::Index n, bool init_x, Ipopt::Number *x,
                        bool init_z, Ipopt::Number *z_L, Ipopt::Number *z_U,
                        Ipopt::Index m, bool init_lambda, Ipopt::Number *lambda)
{
    assert(n == n_variables);
    assert(m == n_constraints);
    assert(init_z == false);
    assert(init_lambda == false);
    UNUSED(init_x);
    UNUSED(z_L);
    UNUSED(z_U);
    UNUSED(lambda);
    PointArrayEdit px = x;
    Point point;
    int n_points = vertices.size();
    for (int i  = 0; i < n_points; ++i)
    {
        point = curve->point(vertices[i]);
        px(i,0) = point[0];
        px(i,1) = point[1];
        px(i,2) = point[2];
    }
    Point t1 = ComputeEdge(halfedge_at_ends_.first, x);
    t1 /= t1.norm();
    Point t2 = ComputeEdge(halfedge_at_ends_.second, x);
    t2 /= t2.norm();

    bishop_at_end_ = ComputeParallelTransportation(x);
    point = ComputeEdge(halfedge_at_ends_.second, x);
    difference_material_frame_ = ComputeDifferenceAngle(point, material_frames_at_ends_.second, bishop_at_end_);
    difference_material_frame_ -= pOp->twisting_times * 2 * M_PI;
    return true;
}


bool OptimizeElastic::eval_f(Ipopt::Index n, const Ipopt::Number *x, bool new_x, Ipopt::Number &obj_value)
{
    UNUSED(n);
    if (new_x && !updateCross(x)) return false;
    obj_value = 0;
    PointArray px = PointArray(x);
    Ipopt::Number t[3];
    Ipopt::Number r = 0;
    double tmp;
    int i = 0, size = EnergyVertices.size();
    for (; i < size; i++)
    {
        obj_value += computeBeta(i, x) / VerticesWeight[i];
    }
    obj_value *= 2 * pOp->BendingEnergyCoef;

    UpdateBishopFrameAtEnd(x);
    Point reference = ComputeParallelTransportation(x);
    difference_material_frame_ += ComputeDifferenceAngle(reference, x);
    obj_value += pOp->TwistingEnergyCoef * difference_material_frame_ * difference_material_frame_ / total_length_;

    i = 0;
    size = PositionConstraints.size();
    for (; i < size; i++)
    {
        sub(px(PositionConstraints[i]), positions[i].data(), t);
        r += sqrnorm(t);
    }
    r *= pOp->PositionConstraintsWeight;
    obj_value += r;

    r = 0;
    i = 0;
    size = TangentConstraints.size();
    for (; i < size; ++i)
    {
        computeTangentEdge(i, x, t);
        r += 1 - dot(t, tangents[i].data());
    }
    obj_value += r * pOp->TangentConstraintsCoef;

    r = 0;
    i = 0;
    size = PlaneConstraints.size();
    for (; i < size; ++i)
    {
        tmp = (getPoint(PlaneConstraints[i], x) | PlaneConstraintsInfo[i].first) + PlaneConstraintsInfo[i].second;
        r += tmp * tmp;
    }
    obj_value += r * pOp->PlaneConstraintsCoef;
    return true;
}


bool OptimizeElastic::eval_grad_f(Ipopt::Index n, const Ipopt::Number *x, bool new_x, Ipopt::Number *grad_f)
{
    assert(n == n_variables);
    if (new_x && !updateCross(x)) return false;
    Ipopt::Number t1[3], t2[3], t[3], e[3], f[3], tmp;
    Ipopt::Number* pV;
    PointArrayEdit pf = grad_f;
    PointArray px = x;
    setZeros(grad_f, n);
    int idx;
    int i = 0, size = EnergyVertices.size();
    for (; i < size; i++)
    {
        computeEF(i, x, e, f);
        tmp = EdgesLengthProduct[i] + dot(e,f);
        tmp *= tmp;
        tmp = 4 * pOp->BendingEnergyCoef * EdgesLengthProduct[i]/ VerticesWeight[i] / tmp;
        idx = idxPrevVertex[i];
        multiplyByScale(f, tmp, t1);
        if (idx >= 0)
            addTo(t1, pf(idx));
        multiplyByScale(e, -tmp, t2);
        idx = idxNextVertex[i];
        if (idx >= 0)
            addTo(t2, pf(idx));
        idx = idxTheVertex[i];
        if (idx >= 0)
        {
            add(t1, t2, t);
            multiplyByScaleTo(t, -1, pf(idx));
        }
    }

    PetCurve::HalfedgeHandle h_hnd_e = halfedge_at_ends_.first;
    PetCurve::HalfedgeHandle h_hnd_f = curve->next_halfedge_handle(h_hnd_e);
    Point E, F, G;
    while (h_hnd_e != halfedge_at_ends_.second)
    {
        E = ComputeEdge(h_hnd_e, x);
        F = ComputeEdge(h_hnd_f, x);
        idx = insertAdditionalPoint(curve->from_vertex_handle(h_hnd_e));
        if (idx >= 0)
        {
            G = ComputeTwistGradient(E, F, 0);
            multiplyByScaleTo(G.data(), -2 * difference_material_frame_ / total_length_, pf(idx));
        }
        idx = insertAdditionalPoint(curve->to_vertex_handle(h_hnd_e));
        if (idx >= 0)
        {
            G = ComputeTwistGradient(E, F, 1);
            multiplyByScaleTo(G.data(), -2 * difference_material_frame_ / total_length_, pf(idx));
        }
        idx = insertAdditionalPoint(curve->to_vertex_handle(h_hnd_f));
        if (idx >= 0)
        {
            G = ComputeTwistGradient(E, F, 2);
            multiplyByScaleTo(G.data(), -2 * difference_material_frame_ / total_length_, pf(idx));
        }
        h_hnd_e = h_hnd_f;
        h_hnd_f = curve->next_halfedge_handle(h_hnd_e);
    }

    i = 0;
    size = PositionConstraints.size();
    for (; i < size; i++)
    {
        idx = PositionConstraints[i];
        pV = positions[i].data();
        sub(px(idx), pV, t);
        multiplyByScaleTo(t, 2 * pOp->PositionConstraintsWeight, pf(idx));
    }
    i = 0;
    size = TangentConstraints.size();
    for (; i < size; ++i)
    {
        pV = tangents[i].data();
        idx = TangentConstraints[i].first;
        if (idx >= 0)
            multiplyByScaleTo(pV, pOp->TangentConstraintsCoef, pf(idx));
        idx = TangentConstraints[i].second;
        if (idx >= 0)
            multiplyByScaleTo(pV, -pOp->TangentConstraintsCoef, pf(idx));
    }

    Point P;
    i = 0;
    size = PlaneConstraints.size();
    for (; i < size; ++i)
    {
        idx = PlaneConstraints[i];
        P = PlaneConstraintsInfo[i].first;
        tmp = 2 * ((P | getPoint(idx, x)) + PlaneConstraintsInfo[i].second);
        multiplyByScaleTo(P.data(), tmp * pOp->PlaneConstraintsCoef, pf(idx));
    }
    return true;
}

bool OptimizeElastic::eval_g(Ipopt::Index n, const Ipopt::Number *x, bool new_x, Ipopt::Index m, Ipopt::Number *g)
{
    UNUSED(n);
    UNUSED(m);
    if (new_x && !updateCross(x)) return false;
    int i = 0, size = edges.size();
    for (; i < size; i++)
        g[i] = computeEdgeLength(i, x);
    i = size;
    int end = edges.size() + crosses.size();
    for (; i < end; ++i)
    {
        g[i] = computeCrossDiagDistance(i - size, x);
    }
    return true;
}

bool OptimizeElastic::eval_jac_g(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                        Ipopt::Index m, Ipopt::Index nele_jac,
                        Ipopt::Index *iRow, Ipopt::Index *jCol, Ipopt::Number *values)
{
    UNUSED(n);
    UNUSED(m);
    if (new_x && !updateCross(x)) return false;
    Ipopt::Number e[3];
    int idx_nv, idx_pv;
    int idx = 0;
    if (values == NULL)
    {

        int i = 0, size = edges.size();
        for (; i < size; i++)
        {
            idx_pv = idxVertexEdges[i].first;
            idx_nv = idxVertexEdges[i].second;
            if (idx_pv >= 0)
            {
                iRow[idx] = i;
                iRow[idx + 1] = i;
                iRow[idx + 2] = i;
                jCol[idx] = 3 * idx_pv;
                jCol[idx + 1] = 3 * idx_pv + 1;
                jCol[idx + 2] = 3 * idx_pv + 2;
                idx += 3;
            }
            if (idx_nv >= 0)
            {
                iRow[idx] = i;
                iRow[idx + 1] = i;
                iRow[idx + 2] = i;
                jCol[idx] = 3 * idx_nv;
                jCol[idx + 1] = 3 * idx_nv + 1;
                jCol[idx + 2] = 3 * idx_nv + 2;
                idx += 3;
            }
        }
        int end = crosses.size();
        i = 0;
        for (; i < end; ++i)
        {
            for (int j = 0; j < n_variables; ++j)
            {
                iRow[idx] = size + i;
                jCol[idx] = j;
                ++idx;
            }
        }
        for (; idx < nele_jac; idx++)
        {
            iRow[idx] = 0;
            jCol[idx] = 0;
        }
    }
    else
    {
        setZeros(values, nele_jac);
        int i = 0, size = edges.size();
        for (; i < size; i++)
        {
            computeEdge(i, x, e);
            idx_pv = idxVertexEdges[i].first;
            idx_nv = idxVertexEdges[i].second;
            if (idx_pv >= 0)
            {
                multiplyByScale(e, -2, values + idx);
                idx += 3;
            }
            if (idx_nv >= 0)
            {
                multiplyByScale(e, 2, values + idx);
                idx += 3;
            }
        }
        int end = crosses.size();
        i = 0;
        for (; i < end; ++i)
        {
            ComputeGradientCross(i, idx, values, x);
        }
    }
    assert(idx <= nele_jac);
    return true;
}

bool OptimizeElastic::eval_h(Ipopt::Index n, const Ipopt::Number *x,
                              bool new_x, Ipopt::Number obj_factor,
                              Ipopt::Index m, const Ipopt::Number *lambda, bool new_lambda,
                              Ipopt::Index nele_hess, Ipopt::Index *iRow, Ipopt::Index *jCol, Ipopt::Number *values)
{
    UNUSED(n);
    UNUSED(m);
    UNUSED(new_lambda);
    if (new_x && !updateCross(x)) return false;
    int idx_pv, idx_nv, idx_cv;
    Ipopt::Number e[3];
    int i = 0;
    int j = 0;
    int size = 0;
    int base = 0;
    int idx_pc = 0;
    if (values == NULL)
    {
        for (i = 0; i < n_variables; ++i)
        {
            for (j = 0; j <= i; ++j)
            {
                iRow[base+j] = i;
                jCol[base+j] = j;
            }
            base += i + 1;
        }
    }
    else
    {
        double tmp = 0;
        setZeros(values, nele_hess);
        i = 0, size = edges.size();
        for (; i < size; i++)
        {
            computeEdge(i, x, e);
            idx_pv = idxVertexEdges[i].first;
            idx_nv = idxVertexEdges[i].second;
            if (idx_pv >= 0)
            {
               AddHessianAtDiagonal(idx_pv, idx_pv, values, 2 * lambda[i]);
                if (idx_nv > idx_pv)
                {
                    AddHessianAtDiagonal(idx_nv, idx_pv, values, -2 * lambda[i]);
                }
            }
            if (idx_nv >= 0)
            {
                AddHessianAtDiagonal(idx_nv, idx_nv, values, 2 * lambda[i]);
                if (idx_pv > idx_nv)
                {
                    AddHessianAtDiagonal(idx_pv, idx_nv, values, -2 * lambda[i]);
                }
            }
        }

        i = 0;
        size = PositionConstraints.size();
        tmp = 2 * pOp->PositionConstraintsWeight * obj_factor;
        for (; i < size; i++)
        {
            idx_pc = PositionConstraints[i];
            AddHessianAtDiagonal(idx_pc, idx_pc, values, tmp);
        }

        // Plane constraints
        Point P;
        i = 0;
        size = PlaneConstraints.size();
        for (; i < size; ++i)
        {
            P = PlaneConstraintsInfo[i].first;
            idx_pc = PlaneConstraints[i];
            AddHessianAtEntriesBendingHelper(idx_pc, idx_pc, P * 2 * obj_factor * pOp->PlaneConstraintsCoef, P, values);
        }

        //Bending energy
        i = 0;
        size = EnergyVertices.size();
        for (; i < size; i++)
        {
            tmp = 2 * obj_factor * pOp->BendingEnergyCoef / VerticesWeight[i];
            idx_pv = idxPrevVertex[i];
            idx_nv = idxNextVertex[i];
            idx_cv = idxTheVertex[i];
            if (idx_pv >= 0)
            {
                AddHessianBending(i, idx_pv, idx_pv, values, x, tmp);
                if (idx_cv > idx_pv)
                {
                    AddHessianBending(i, idx_cv, idx_pv, values, x, tmp);
                }
                if (idx_nv > idx_pv)
                {
                    AddHessianBending(i, idx_nv, idx_pv, values, x, tmp);
                }
            }
            if (idx_nv >= 0)
            {
                AddHessianBending(i, idx_nv, idx_nv, values, x, tmp);
                if (idx_pv > idx_nv)
                {
                    AddHessianBending(i, idx_pv, idx_nv, values, x, tmp);
                }
                if (idx_cv > idx_nv)
                {
                    AddHessianBending(i, idx_cv, idx_nv, values, x, tmp);
                }
            }
            if (idx_cv >= 0)
            {
                AddHessianBending(i, idx_cv, idx_cv, values, x, tmp);
                if (idx_pv > idx_cv)
                {
                    AddHessianBending(i, idx_pv, idx_cv, values, x, tmp);
                }
                if (idx_nv > idx_cv)
                {
                    AddHessianBending(i, idx_nv, idx_cv, values, x, tmp);
                }
            }
        }

        //crossing constraints
        i = 0;
        size = crosses.size();
        int base = edges.size();
        for (; i < size; ++i)
        {
            int p0_idx = insertAdditionalPoint(curve->from_vertex_handle(crosses[i].first));
            int p1_idx = insertAdditionalPoint(curve->to_vertex_handle(crosses[i].first));
            int q0_idx = insertAdditionalPoint(curve->from_vertex_handle(crosses[i].second));
            int q1_idx = insertAdditionalPoint(curve->to_vertex_handle(crosses[i].second));
            Point p0 = getPoint(p0_idx, x);
            Point p1 = getPoint(p1_idx, x);
            Point q0 = getPoint(q0_idx, x);
            Point q1 = getPoint(q1_idx, x);
            if (p0_idx >= 0)
            {
                if (p1_idx > p0_idx)
                    AddHessianCrossing(p1_idx, p0_idx, (q1 - q0) * lambda[base+i], values); // H_(p0)(p1)
                if (q0_idx > p0_idx)
                    AddHessianCrossing(q0_idx, p0_idx, (p1 - q1) * lambda[base+i], values); // H_(p0)(q0)
                if (q1_idx > p0_idx)
                    AddHessianCrossing(q1_idx, p0_idx, (q0 - p1) * lambda[base+i], values); // H_(p0)(q1)
            }
            if (p1_idx >= 0)
            {
                if (p0_idx > p1_idx)
                    AddHessianCrossing(p0_idx, p1_idx, (q0 - q1) * lambda[base+i], values); // H_(p1)(p0)
                if (q0_idx > p1_idx)
                    AddHessianCrossing(q0_idx, p1_idx, (q1 - p0) * lambda[base+i], values); // H_(p1)(q0)
                if (q1_idx > p1_idx)
                    AddHessianCrossing(q1_idx, p1_idx, (p0 - q0) * lambda[base+i], values); // H_(p1)(q1)
            }
            if (q0_idx >= 0)
            {
                if (p0_idx > q0_idx)
                    AddHessianCrossing(p0_idx, q0_idx, (q1 - p1) * lambda[base+i], values); // H_(q0)(p0)
                if (p1_idx > q0_idx)
                    AddHessianCrossing(p1_idx, q0_idx, (p0 - q1) * lambda[base+i], values); // H_(q0)(p1)
                if (q1_idx > q0_idx)
                    AddHessianCrossing(q1_idx, q0_idx, (p1 - p0) * lambda[base+i], values); // H_(q0)(q1)
            }
            if (q1_idx >= 0)
            {
                if (p0_idx > q1_idx)
                    AddHessianCrossing(p0_idx, q1_idx, (p1 - q0) * lambda[base+i], values); // H_(q1)(p0)
                if (p1_idx > q1_idx)
                    AddHessianCrossing(p1_idx, q1_idx, (q0 - p0) * lambda[base+i], values); // H_(q1)(p1)
                if (q0_idx > q1_idx)
                    AddHessianCrossing(p0_idx, q1_idx, (p0 - p1) * lambda[base+i], values); // H_(q1)(q0)
            }
        }
        // twisting energy
        AddHessianTwistingStep1(values, x, 2 * pOp->TwistingEnergyCoef * difference_material_frame_ * obj_factor / total_length_);
        AddHessianTwistingStep2(values, x, -2 * pOp->TwistingEnergyCoef * obj_factor / total_length_);
    }
    return true;
}

void OptimizeElastic::finalize_solution(Ipopt::SolverReturn status, Ipopt::Index n, const Ipopt::Number *x,
                               const Ipopt::Number *z_L, const Ipopt::Number *z_U,
                               Ipopt::Index m, const Ipopt::Number *g, const Ipopt::Number *lambda,
                               Ipopt::Number obj_value,
                               const Ipopt::IpoptData *ip_data, Ipopt::IpoptCalculatedQuantities *ip_cq)
{
    UNUSED(status);
    UNUSED(z_L);
    UNUSED(z_U);
    UNUSED(m);
    UNUSED(g);
    UNUSED(lambda);
    UNUSED(obj_value);
    UNUSED(ip_data);
    UNUSED(ip_cq);
    assert(n == int(vertices.size() * 3));
    int idx = 0, size = vertices.size();
    for (; idx < size; idx++)
    {
        curve->set_point(vertices[idx],OpenMesh::vector_cast<PetCurve::Point>(OpenMesh::Vec3d(x + 3 * idx)));
    }
}


void OptimizeElastic::setHessianPos_SkewSym(const int i, const int j, Ipopt::Index* iRow,Ipopt::Index* jCol, int& idx)
{
    iRow[idx] = 3 * i;
    jCol[idx] = 3 * j + 1;
    ++idx;
    iRow[idx] = 3 * i;
    jCol[idx] = 3 * j + 2;
    ++idx;
    iRow[idx] = 3 * i + 1;
    jCol[idx] = 3 * j + 2;
    ++idx;
    iRow[idx] = 3 * i + 1;
    jCol[idx] = 3 * j;
    ++idx;
    iRow[idx] = 3 * i + 2;
    jCol[idx] = 3 * j;
    ++idx;
    iRow[idx] = 3 * i + 2;
    jCol[idx] = 3 * j + 1;
    ++idx;
}


//fill J_col Nabla_row Dist
void OptimizeElastic::setHessianValues_SkewSym(const int i, const int row, const int col,
                                        Ipopt::Number *values, int& idx, const double *x, double weight)
{
    int idx_p0, idx_p1, idx_q0, idx_q1;
    idx_p0 = crossE[i][crossRef[i].first];
    idx_p1 = crossE[i][crossRef[i].first + 1];
    idx_q0 = crossF[i][crossRef[i].second];
    idx_q1 = crossF[i][crossRef[i].second + 1];
    Point q0 = getPoint(idx_q0, x);
    Point q1 = getPoint(idx_q1, x);
    Point p0 = getPoint(idx_p0, x);
    Point p1 = getPoint(idx_p1, x);
    if (row == idx_p0)
    {
        if (col == idx_p1) // H_p1 p0
        {
            setHessianValues_SkewSym_helper((q0 - q1) * weight, values, idx);
        }
        else if (col == idx_q0) //H_q0 p0
        {
            setHessianValues_SkewSym_helper((q1 - p1) * weight, values, idx);
        }
        else if (col == idx_q1) //H_q1 p0
        {
            setHessianValues_SkewSym_helper((p1 - q0) * weight, values, idx);
        }
        else
            idx += 6;
    }
    else if (row == idx_p1)
    {
        if (col == idx_p0) // H_p0 p1
        {
            setHessianValues_SkewSym_helper((q1 - q0) * weight, values, idx);
        }
        else if (col == idx_q0) //H_q0 p1
        {
            setHessianValues_SkewSym_helper((p0 - q1) * weight, values, idx);
        }
        else if (col == idx_q1) //H_q1 p1
        {
            setHessianValues_SkewSym_helper((q0 - p0) * weight, values, idx);
        }
        else
            idx += 6;
    }
    else if (row == idx_q0)
    {
        if (col == idx_p0) // H_p0 q0
        {
            setHessianValues_SkewSym_helper((p1 - q1) * weight, values, idx);
        }
        else if (col == idx_p1) //H_p1 q0
        {
            setHessianValues_SkewSym_helper((q1 - p0) * weight, values, idx);
        }
        else if (col == idx_q1) //H_q1 q0
        {
            setHessianValues_SkewSym_helper((p0 - p1) * weight, values, idx);
        }
        else
            idx += 6;
    }
    else if (row == idx_q1)
    {
        if (col == idx_p0) // H_p0 q1
        {
            setHessianValues_SkewSym_helper((q0 - p1) * weight, values, idx);
        }
        else if (col == idx_p1) //H_p1 q1
        {
            setHessianValues_SkewSym_helper((p0 - q0) * weight, values, idx);
        }
        else if (col == idx_q0) //H_q0 q1
        {
            setHessianValues_SkewSym_helper((p1 - p0) * weight, values, idx);
        }
        else
            idx += 6;
    }
    else
        idx += 6;
}

void OptimizeElastic::setHessianValues_SkewSym_helper(const Point& p, Ipopt::Number *values, int& idx)
{
    values[idx++] = -p[2];
    values[idx++] = p[1];
    values[idx++] = -p[0];
    values[idx++] = p[2];
    values[idx++] = -p[1];
    values[idx++] = p[0];
}

void OptimizeElastic::setJacGPos_cross(const int ref, const int base, const int i, int& idx, int* iRow, int* jCol)
{
    int first = crossE[i][ref];
    int second = crossF[i][ref];
    if (first >= 0)
    {
        iRow[idx] = base + i;
        iRow[idx + 1] = base + i;
        iRow[idx + 2] = base + i;
        jCol[idx] = 3 * first;
        jCol[idx + 1] = 3 * first + 1;
        jCol[idx + 2] = 3 * first + 2;
        idx += 3;
    }
    if (second >= 0)
    {
        iRow[idx] = base + i;
        iRow[idx + 1] = base + i;
        iRow[idx + 2] = base + i;
        jCol[idx] = 3 * second;
        jCol[idx + 1] = 3 * second + 1;
        jCol[idx + 2] = 3 * second + 2;
        idx += 3;
    }
}

void OptimizeElastic::ComputeGradientCross(const int idx_cross, int &idx, double *values, const double *x)
{
    /*------------------------------------------
                   p0_  q1
                    |\ /
                      /
                    |/ \
                   q0-  p1
    ------------------------------------------*/
    int idx_p0 = insertAdditionalPoint(curve->from_vertex_handle(crosses[idx_cross].first));
    int idx_p1 = insertAdditionalPoint(curve->to_vertex_handle(crosses[idx_cross].first));
    int idx_q0 = insertAdditionalPoint(curve->from_vertex_handle(crosses[idx_cross].second));
    int idx_q1 = insertAdditionalPoint(curve->to_vertex_handle(crosses[idx_cross].second));
    Point p0 = getPoint(idx_p0, x);
    Point q0 = getPoint(idx_q0, x);
    Point p1 = getPoint(idx_p1, x);
    Point q1 = getPoint(idx_q1, x);
    Point p;
    if (idx_p0 >= 0)
    {
        p = (q1 - p1) % (q0 - q1);
        copy(p.data(), values + idx + 3 * idx_p0);
    }
    if (idx_p1 >= 0)
    {
        p = (p0 - q1) % (q0 - q1);
        copy(p.data(), values + idx + 3 * idx_p1);
    }
    if (idx_q0 >= 0)
    {
        p = (p0 - p1) % (q1 - p1);
        copy(p.data(), values + idx + 3 * idx_q0);
    }
    if (idx_q1 >= 0)
    {
        p = (p0 - p1) % (p1 - q0);
        copy(p.data(), values + idx + 3 * idx_q1);
    }
    idx += n_variables;
}


void OptimizeElastic::computeEF(const int i, const Ipopt::Number *x, Point &e, Point &f)
{
    Point p = getPoint(idxPrevVertex[i], x);
    Point c = getPoint(idxTheVertex[i], x);
    Point n = getPoint(idxNextVertex[i], x);
    e = c - p;
    f = n - c;
}


void OptimizeElastic::computeEF(const int i, const Ipopt::Number *x, double *e, double *f)
{
    const double *p = prevVertices(i, x);
    const double *c = theVertices(i, x);
    const double *n = nextVertices(i, x);
    sub(c, p, e);
    sub(n, c, f);
}


double OptimizeElastic::computeBeta(int i, const Ipopt::Number *x)
{
    double e[3], f[3], d;
    computeEF(i, x, e, f);
    d = dot(e, f);
    return (EdgesLengthProduct[i] - d) / (EdgesLengthProduct[i] + d);
}

void OptimizeElastic::computeEdge(int i, const Ipopt::Number *x, Ipopt::Number *r)
{
    const Ipopt::Number *pv, *nv;
    pv = EdgePrevVertex(i, x);
    nv = EdgeNextVertex(i, x);
    sub(nv, pv, r);
}

Ipopt::Number OptimizeElastic::computeEdgeLength(int i, const Ipopt::Number *x)
{
    Ipopt::Number e[3];
    const Ipopt::Number *pv, *nv;
    pv = EdgePrevVertex(i, x);
    nv = EdgeNextVertex(i, x);
    sub(nv, pv, e);
    return sqrnorm(e);
}

const Ipopt::Number* OptimizeElastic::EdgePrevVertex(int i, const Ipopt::Number *x)
{
    int idx = idxVertexEdges[i].first;
    if (idx >= 0)
        return x + idx * 3;
    else
        return additionalPoints[-idx].data();
}

const Ipopt::Number* OptimizeElastic::EdgeNextVertex(int i, const Ipopt::Number *x)
{
    int idx = idxVertexEdges[i].second;
    if (idx >= 0)
        return x + idx * 3;
    else
        return additionalPoints[-idx].data();
}

const Ipopt::Number* OptimizeElastic::TangentPrevVertex(int i, const Ipopt::Number *x)
{
    int idx = TangentConstraints[i].first;
    if (idx >= 0)
        return x + 3 * idx;
    else
        return additionalPoints[-idx].data();
}

const Ipopt::Number* OptimizeElastic::TangentNextVertex(int i, const Ipopt::Number *x)
{
    int idx = TangentConstraints[i].second;
    if (idx >= 0)
        return x + 3 * idx;
    else
        return additionalPoints[-idx].data();
}


PetCurve::Point OptimizeElastic::getPoint(const int i, const double* x)
{
    if (i >= 0)
        return PetCurve::Point(x + 3 * i);
    else
        return additionalPoints[-i];
}


void OptimizeElastic::computeTangentEdge(int i, const Ipopt::Number *x, Ipopt::Number* r)
{
    const Ipopt::Number* pv, *nv;
    pv = TangentPrevVertex(i, x);
    nv = TangentNextVertex(i, x);
    sub(nv, pv, r);
}

inline void OptimizeElastic::setZeros(Ipopt::Number *r)
{
    r[0] = r[1] = r[2] = 0;
}

inline void OptimizeElastic::setZeros(Ipopt::Number *r, int n)
{
    double tmp = 0;
    cblas_dcopy(n, &tmp, 0, r, 1);
}

inline void OptimizeElastic::setValues(Ipopt::Number *r, Ipopt::Number v)
{
    r[0] = r[1] = r[2] = v;
}

void OptimizeElastic::setHessianPos_bending(const int i, const int j,
                                Ipopt::Index* iRow,Ipopt::Index* jCol,
                                int& idx)
{
    if(i == j)
    {
        iRow[idx] = 3 * i;
        iRow[idx + 1] = 3 * i + 1;
        iRow[idx + 2] = 3 * i + 1;
        iRow[idx + 3] = 3 * i + 2;
        iRow[idx + 4] = 3 * i + 2;
        iRow[idx + 5] = 3 * i + 2;
        jCol[idx] = 3 * j;
        jCol[idx + 1] = 3 * j;
        jCol[idx + 2] = 3 * j + 1;
        jCol[idx + 3] = 3 * j;
        jCol[idx + 4] = 3 * j + 1;
        jCol[idx + 5] = 3 * j + 2;
        idx += 6;
    }
    else
    {
        int row, col;
        for (row = 0; row < 3; ++row)
        {
            for (col = 0; col < 3; ++col)
            {
                iRow[idx] = 3 * i + row;
                jCol[idx++] = 3 * j + col;
            }
        }
    }

}

void OptimizeElastic::setHessianPos(const int i, const int j,
                                Ipopt::Index* iRow,Ipopt::Index* jCol,
                                int& idx)
{
    iRow[idx] = 3 * i;
    iRow[idx + 1] = 3 * i + 1;
    iRow[idx + 2] = 3 * i + 2;
    jCol[idx] = 3 * j;
    jCol[idx + 1] = 3 * j + 1;
    jCol[idx + 2] = 3 * j + 2;
    idx += 3;
}


inline void OptimizeElastic::setHessianValues(int& idx, Ipopt::Number* x, const Ipopt::Number v)
{
    x[idx] = x[idx + 1] = x[idx + 2] = v;
    idx += 3;
}


void OptimizeElastic::setHessianValues_bending(const int i, const int row, const int col,
                                        int& idx, double *values, const double* x, const double v)
{
    Point e, f;
    computeEF(i, x, e, f);
    double P = EdgesLengthProduct[i];
    double deno = P + (e | f);
    double tmp = deno * deno * deno;
    tmp = v / tmp;
    int p, c, n;
    p = idxPrevVertex[i];
    c = idxTheVertex[i];
    n = idxNextVertex[i];
    if (row == col)
    {
        if (row == p)
        {
            setHessianVal_bending_helper(4 * P * f, f, values + idx, 0);
            multiplyByScale(values + idx, tmp, 6);
            idx += 6;
        }
        else if (row == c)
        {
            setHessianVal_bending_diag(4 * P * deno, values + idx, 0);
            setHessianVal_bending_helper(4 * P * (e - f), e - f, values + idx, 0);
            multiplyByScale(values + idx, tmp, 6);
            idx += 6;
        }
        else if (row == n)
        {
            setHessianVal_bending_helper(4 * P * e, e, values + idx, 0);
            multiplyByScale(values + idx, tmp, 6);
            idx += 6;
        }

    }
    else
    {
        if (row == p)
        {

            if (col == c)//H_(i)(i-1)
            {
                setHessianVal_bending_diag(-2 * P * deno, values + idx);
                setHessianVal_bending_helper(4 * P * f, e - f, values + idx);
                multiplyByScale(values + idx, tmp, 9);
                idx += 9;
            }
            else if (col == n)//H_(i+1)(i-1)
            {
                setHessianVal_bending_diag(2 * P * deno, values + idx);
                setHessianVal_bending_helper(-4 * P * f, e, values + idx);
                multiplyByScale(values + idx, tmp, 9);
                idx += 9;
            }
        }
        else if (row == c)
        {
            if (col == p)//H_(i-1)(i)
            {
                setHessianVal_bending_diag(-2 * P * deno, values + idx);
                setHessianVal_bending_helper(4 * P * (e - f), f, values + idx);
                multiplyByScale(values + idx, tmp, 9);
                idx += 9;
            }
            else if (col == n)//H_(i+1)(i)
            {
                setHessianVal_bending_diag(-2 * P * deno, values + idx);
                setHessianVal_bending_helper(4 * P * (f - e), e, values + idx);
                multiplyByScale(values + idx, tmp, 9);
                idx += 9;
            }
        }
        else if (row == n)
        {
            if (col == p)//H_(i-1)(i+1)
            {
                setHessianVal_bending_diag(2 * P * deno, values + idx);
                setHessianVal_bending_helper(-4 * P * e, f, values + idx);
                multiplyByScale(values + idx, tmp, 9);
                idx += 9;
            }

            else if (col == c)//H_(i)(i+1)
            {
                setHessianVal_bending_diag(-2 * P * deno, values + idx);
                setHessianVal_bending_helper(4 * P * e, f - e, values + idx);
                multiplyByScale(values + idx, tmp, 9);
                idx += 9;
            }
        }
    }
}

void OptimizeElastic::setHessianVal_bending_helper(const Point& p, const Point& q, double *values, const int s)
{
    if (s == 1)
    {
        multiplyByScaleTo(q.data(), p[0], values);
        multiplyByScaleTo(q.data(), p[1], values + 3);
        multiplyByScaleTo(q.data(), p[2], values + 6);
    }
    if (s == 0)
    {
        multiplyByScaleTo(q.data(), p[0], values, 1);
        multiplyByScaleTo(q.data(), p[1], values + 1, 2);
        multiplyByScaleTo(q.data(), p[2], values + 3, 3);
    }
}

void OptimizeElastic::setHessianVal_bending_diag(const double v, double *values, const int s)
{
    if (s == 1)
    {
        values[0] += v;
        values[4] += v;
        values[8] += v;
    }
    if (s == 0)
    {
        values[0] += v;
        values[2] += v;
        values[5] += v;
    }
}

inline void OptimizeElastic::cross(const Ipopt::Number* p, const Ipopt::Number* q, Ipopt::Number* r)
{
    r[0] = p[1] * q[2] - p[2] * q[1];
    r[1] = p[2] * q[0] - p[0] * q[2];
    r[2] = p[0] * q[1] - p[1] * q[0];
}

inline void OptimizeElastic::add(const Ipopt::Number* p, const Ipopt::Number* q, Ipopt::Number* r)
{
    r[0] = p[0] + q[0];
    r[1] = p[1] + q[1];
    r[2] = p[2] + q[2];
}

inline void OptimizeElastic::sub(const Ipopt::Number* p, const Ipopt::Number* q, Ipopt::Number* r)
{
    r[0] = p[0] - q[0];
    r[1] = p[1] - q[1];
    r[2] = p[2] - q[2];
}

inline void OptimizeElastic::addTo(const Ipopt::Number *p, Ipopt::Number* dst)
{
    dst[0] += p[0];
    dst[1] += p[1];
    dst[2] += p[2];
}

inline void OptimizeElastic::copy(const Ipopt::Number* src, Ipopt::Number* dst)
{
    dst[0] = src[0];
    dst[1] = src[1];
    dst[2] = src[2];
}

inline void copy(const Ipopt::Number* src, Ipopt::Number* dst, const Ipopt::Index n)
{
    cblas_dcopy(n, src, 1, dst, 1);
}

inline Ipopt::Number OptimizeElastic::dot(const Ipopt::Number* p, const Ipopt::Number* q)
{
    return p[0] * q[0] + p[1] * q[1] + p[2] * q[2];
}

inline Ipopt::Number OptimizeElastic::sqrnorm(const Ipopt::Number* p)
{
    return p[0] * p[0] + p[1] * p[1] + p[2] * p[2];
}

inline Ipopt::Number OptimizeElastic::norm(const Ipopt::Number* p)
{
    return sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]);
}

inline const Ipopt::Number* OptimizeElastic::prevVertices(const int i, const Ipopt::Number *x)
{
    int idx = idxPrevVertex[i];
    if (idx >= 0)
        return x + idx * 3;
    else
        return (additionalPoints[-idx]).data();
}

inline const Ipopt::Number* OptimizeElastic::theVertices(const int i, const Ipopt::Number *x)
{
    int idx = idxTheVertex[i];
    if (idx >= 0)
        return x + idx * 3;
    else
        return (additionalPoints[-idx]).data();
}

inline const Ipopt::Number* OptimizeElastic::nextVertices(const int i, const Ipopt::Number *x)
{
    int idx = idxNextVertex[i];
    if (idx >= 0)
        return x + idx * 3;
    else
        return (additionalPoints[-idx]).data();
}
inline void OptimizeElastic::multiplyByScale(Ipopt::Number *x, Ipopt::Number dn)
{
    x[0] *= dn;
    x[1] *= dn;
    x[2] *= dn;
}

inline void OptimizeElastic::divideByScale(Ipopt::Number *x, Ipopt::Number dn)
{
    x[0] /= dn;
    x[1] /= dn;
    x[2] /= dn;
}

inline void OptimizeElastic::multiplyByScale(const Ipopt::Number *x, Ipopt::Number dn, Ipopt::Number *r)
{
    r[0] = x[0] * dn;
    r[1] = x[1] * dn;
    r[2] = x[2] * dn;
}

inline void OptimizeElastic::multiplyByScale(Ipopt::Number *x, Ipopt::Number dn, int n)
{
    cblas_dscal(n, dn, x, 1);
}

inline void OptimizeElastic::multiplyByScaleTo(const Ipopt::Number *x, Ipopt::Number dn, Ipopt::Number *r)
{
    r[0] += x[0] * dn;
    r[1] += x[1] * dn;
    r[2] += x[2] * dn;
}

inline void OptimizeElastic::multiplyByScaleTo(const Ipopt::Number *x, Ipopt::Number dn, Ipopt::Number *r, int n)
{
    cblas_daxpy(n, dn, x, 1, r, 1);
}

inline void OptimizeElastic::divideByScale(const Ipopt::Number *x, Ipopt::Number dn, Ipopt::Number *r)
{
    r[0] = x[0] / dn;
    r[1] = x[1] / dn;
    r[2] = x[2] / dn;
}

void OptimizeElastic::crossNormal(const double* p0,
                 const double* p1,
                 const double* q0,
                 const double* q1,
                 double* r)
{
    double u[3],v[3];
    sub(p0, p1, u);
    sub(q0, q1, v);
    cross(u, v, r);
}


void OptimizeElastic::crossDiag(const double *p0, const double *p1, const double *q0, const double *q1, double *r)
{
    double p_center[3], q_center[3];
    add(p0, p1, p_center);
    multiplyByScale(p_center, 1/2);
    add(q0, q1, q_center);
    multiplyByScale(q_center, 1/2);
    sub(p_center, q_center, r);
}

bool OptimizeElastic::LineSegmentsCollide(const Point& p1,
                                   const Point& p2,
                                   const Point& q1,
                                   const Point& q2)
{
    OpenMesh::Vec3d p_center((p1 + p2)/2), q_center((q1+q2)/2);
    if ((p_center - q_center).norm() < ((p1-p2).norm() + (q1 - q2).norm())/2 + pOp->r)
        return true;
    else
        return false;
}

bool OptimizeElastic::LineSegmentsCollide(const PetCurve::HalfedgeHandle& he_i, const PetCurve::HalfedgeHandle& he_j)
{
    OpenMesh::VertexHandle p1(curve->from_vertex_handle(he_i)),
            p2(curve->to_vertex_handle(he_i)),
            p3(curve->from_vertex_handle(he_j)),
            p4(curve->to_vertex_handle(he_j));
    return LineSegmentsCollide(
                curve->point(p1),
                curve->point(p2),
                curve->point(p3),
                curve->point(p4));
}


double OptimizeElastic::LineSegmentsSqDistance(const PetCurve::HalfedgeHandle& he_i, const PetCurve::HalfedgeHandle& he_j)
{
    OpenMesh::VertexHandle p1(curve->from_vertex_handle(he_i)),
            p2(curve->to_vertex_handle(he_i)),
            p3(curve->from_vertex_handle(he_j)),
            p4(curve->to_vertex_handle(he_j));
    return LineSegmentsSqDistance(
                curve->point(p1),
                curve->point(p2),
                curve->point(p3),
                curve->point(p4));
}


double OptimizeElastic::LineSegmentsSqDistance(const PetCurve::HalfedgeHandle& he_i, const PetCurve::HalfedgeHandle& he_j, const double *x)
{
    OpenMesh::VertexHandle p1(curve->from_vertex_handle(he_i)),
            p2(curve->to_vertex_handle(he_i)),
            p3(curve->from_vertex_handle(he_j)),
            p4(curve->to_vertex_handle(he_j));
    return LineSegmentsSqDistance(
                getPoint(insertAdditionalPoint(p1), x),
                getPoint(insertAdditionalPoint(p2), x),
                getPoint(insertAdditionalPoint(p3), x),
                getPoint(insertAdditionalPoint(p4), x));
}


double OptimizeElastic::LineSegmentsSqDistance(const int idx, const int ref_i, const int ref_j, const double* x)
{
    return LineSegmentsSqDistance(
                getPoint(crossE[idx][ref_i], x),
                getPoint(crossE[idx][ref_i+1], x),
                getPoint(crossF[idx][ref_j], x),
                getPoint(crossF[idx][ref_j+1], x));
}


double OptimizeElastic::CrossLineSegmentsDistance(const Point &p0, const Point &p1, const Point &q0, const Point &q1)
{
    Point w = (p0 - p1) % (q0 - q1);
    return ((p0 + p1) / 2 - (q0 + q1) / 2) | w;
}

double OptimizeElastic::computeCrossDiagDistance(const int i, const double* x)
{
    return CrossLineSegmentsDistance(
            getPoint(crossE[i][crossRef[i].first],x),
            getPoint(crossE[i][crossRef[i].first+1],x),
            getPoint(crossF[i][crossRef[i].second],x),
            getPoint(crossF[i][crossRef[i].second+1],x)
            );
}


double OptimizeElastic::LineSegmentsSqDistance(const Point &p0, const Point &p1, const Point &q0, const Point &q1)
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


double OptimizeElastic::LineSegmentsSqDistance(const int p0,
                                   const int p1,
                                   const int q0,
                                   const int q1,
                              const double* x)
{
    return LineSegmentsSqDistance(getPoint(p0,x), getPoint(p1,x), getPoint(q0,x), getPoint(q1,x));
}


void OptimizeElastic::EdgesIntersections()
{
//    std::set<PetCurve::EdgeHandle> setEdges(edges.begin(), edges.end());

    std::vector<PetCurve::EdgeHandle>::const_iterator it = edges.begin(), it_end = edges.end(), it2;
    PetCurve::HalfedgeHandle he_hnd, he2_hnd;
    PetCurve::EdgeHandle prev_e_hnd, next_e_hnd;
    double r_sq = pOp->r * pOp->r;
    for (; it != it_end; ++it)
    {
        he_hnd = curve->halfedge_handle(*it, 0);
        he2_hnd = curve->next_halfedge_handle(he_hnd);
        next_e_hnd = curve->edge_handle(he2_hnd);
        he2_hnd = curve->prev_halfedge_handle(he_hnd);
        prev_e_hnd = curve->edge_handle(he2_hnd);
        for (it2 = it; it2 != it_end; ++it2)
        {
            if (*it2 == *it || *it2 == next_e_hnd || *it2 == prev_e_hnd)
                continue;
            he2_hnd = curve->halfedge_handle(*it2, 0);
            if (LineSegmentsCollide(he_hnd, he2_hnd) && LineSegmentsSqDistance(he_hnd, he2_hnd) < r_sq)
                crosses.push_back(pair<PetCurve::HalfedgeHandle, PetCurve::HalfedgeHandle>(he_hnd, he2_hnd));
        }
    }
    PetCurve::VertexHandle v0, v1;
    int from, to;
    std::vector<pair<PetCurve::HalfedgeHandle, PetCurve::HalfedgeHandle> >::const_iterator it_cross = crosses.begin(),
            it_end_cross = crosses.end();
    for (; it_cross != it_end_cross; ++it_cross)
    {
        deque<int> E, F;
        v0 = curve->from_vertex_handle((*it_cross).first);
        v1 = curve->to_vertex_handle((*it_cross).first);
        from = insertAdditionalPoint(v0);
        to = insertAdditionalPoint(v1);
        E.push_back(from);
        E.push_back(to);
        he2_hnd = he_hnd = (*it_cross).first;
        for (int i = 0; i <= pOp->extension; ++i)
        {
            he_hnd = curve->next_halfedge_handle(he_hnd);
            to = insertAdditionalPoint(curve->to_vertex_handle(he_hnd));
            E.push_back(to);
            he2_hnd = curve->prev_halfedge_handle(he2_hnd);
            from = insertAdditionalPoint(curve->from_vertex_handle(he2_hnd));
            E.push_front(from);
        }
        crossE.push_back(E);

        v0 = curve->from_vertex_handle((*it_cross).second);
        v1 = curve->to_vertex_handle((*it_cross).second);
        from = insertAdditionalPoint(v0);
        to = insertAdditionalPoint(v1);
        F.push_back(from);
        F.push_back(to);
        he2_hnd = he_hnd = (*it_cross).second;
        for (int i = 0; i <= pOp->extension; ++i)
        {
            he_hnd = curve->next_halfedge_handle(he_hnd);
            to = insertAdditionalPoint(curve->to_vertex_handle(he_hnd));
            F.push_back(to);
            he2_hnd = curve->prev_halfedge_handle(he2_hnd);
            from = insertAdditionalPoint(curve->from_vertex_handle(he2_hnd));
            F.push_front(from);
        }
        crossF.push_back(F);

        crossRef.push_back(pair<int, int>(pOp->extension + 1, pOp->extension + 1));
    }
    refSize = 2 * (2 + pOp->extension) - 1;
}


bool OptimizeElastic::updateCross(const double* x)
{
    PetCurve::HalfedgeHandle first, second;
    int idx = 0;
    int idx_end = crosses.size();
    double min_dist;
    double distance;
    pair<PetCurve::HalfedgeHandle, PetCurve::HalfedgeHandle> update;
    for (idx = 0; idx < idx_end; ++idx)
    {
        min_dist = numeric_limits<double>::max();
        for (first = curve->prev_halfedge_handle(crosses[idx].first); first != curve->next_halfedge_handle(crosses[idx].first); first = curve->next_halfedge_handle(first))
        {
            for (second = curve->prev_halfedge_handle(crosses[idx].second); second != curve->next_halfedge_handle(crosses[idx].second); second = curve->next_halfedge_handle(second))
            {
                distance = LineSegmentsSqDistance(first, second, x);
                if (distance < min_dist)
                {
                    update = pair<PetCurve::HalfedgeHandle, PetCurve::HalfedgeHandle>(first, second);
                    min_dist = distance;
                }
            }
        }
        crosses[idx] = update;
    }
    return true;
}


void OptimizeElastic::RectifyMaterialFrameAtEnds(const PetCurve::HalfedgeHandle &h_hnd, Point &p)
{
    Point p0, p1, t;
    p0 = curve->point(curve->from_vertex_handle(h_hnd));
    p1 = curve->point(curve->to_vertex_handle(h_hnd));
    t = p1 - p0;
    t /= t.norm();
    double tmp = t | p;
    p = p - tmp * t;
    p /= p.norm();
}


void OptimizeElastic::UpdateMaterialFrameAtEnds(const double *x)
{
    int idx0 = insertAdditionalPoint(curve->from_vertex_handle(halfedge_at_ends_.first));
    int idx1 = insertAdditionalPoint(curve->to_vertex_handle(halfedge_at_ends_.first));
    Point p0 = getPoint(idx0 ,x);
    Point p1 = getPoint(idx1, x);
    material_frames_at_ends_.first = ParallelTransportation(previous_end_positions_.first, p1 - p0, material_frames_at_ends_.first);
    idx0 = insertAdditionalPoint(curve->from_vertex_handle(halfedge_at_ends_.first));
    idx1 = insertAdditionalPoint(curve->to_vertex_handle(halfedge_at_ends_.first));
    p0 = getPoint(idx0 ,x);
    p1 = getPoint(idx1, x);
    material_frames_at_ends_.second = ParallelTransportation(previous_end_positions_.second, p1 - p0, material_frames_at_ends_.second);
}


void OptimizeElastic::UpdateBishopFrameAtEnd(const double *x)
{
    int idx0 = insertAdditionalPoint(curve->from_vertex_handle(halfedge_at_ends_.second));
    int idx1 = insertAdditionalPoint(curve->to_vertex_handle(halfedge_at_ends_.second));
    Point p0 = getPoint(idx0 ,x);
    Point p1 = getPoint(idx1, x);
    bishop_at_end_ = ParallelTransportation(previous_end_positions_.second, p1 - p0, bishop_at_end_);
}


OptimizeElastic::Point OptimizeElastic::ComputeEdge(const PetCurve::HalfedgeHandle &h_hnd, const Ipopt::Number *x)
{
    int idx0 = insertAdditionalPoint(curve->from_vertex_handle(h_hnd));
    int idx1 = insertAdditionalPoint(curve->to_vertex_handle(h_hnd));
    Point p0 = getPoint(idx0 ,x);
    Point p1 = getPoint(idx1, x);
    return p1 - p0;
}


OptimizeElastic::Point OptimizeElastic::ParallelTransportation(Point t1, Point t2, const Point &x)
{
    t1 /= t1.norm();
    t2 /= t2.norm();
    Point cross = t1 % t2;
    if (cross.norm() < M_EP)
    {
        return x;
    }
    else
    {
        cross /= cross.norm();
        Point n1 = cross % t1;
        Point n2 = cross % t2;
        double y = x | n1;
        double z = x | cross;
        return n2 * y + cross * z;
    }
}


OptimizeElastic::Point OptimizeElastic::ComputeParallelTransportation(const double *x)
{
    PetCurve::HalfedgeHandle h_hnd = halfedge_at_ends_.first;
    PetCurve::HalfedgeHandle tmp_h_hnd = curve->next_halfedge_handle(h_hnd);
    Point v = material_frames_at_ends_.first;
    while (h_hnd != halfedge_at_ends_.second)
    {
        Point t1 = ComputeEdge(h_hnd, x);
        Point t2 = ComputeEdge(tmp_h_hnd, x);
        v = ParallelTransportation(t1, t2, v);
        h_hnd = tmp_h_hnd;
        tmp_h_hnd = curve->next_halfedge_handle(h_hnd);
    }
    return v;
}


double OptimizeElastic::ComputeDifferenceAngle(const Point &axis, const Point &up, const Point &x)
{
    Point axis_normed = axis / axis.norm();
    Point up_normed = up / up.norm();
    Point normal = axis_normed % up_normed;
    double pr_y = normal | x;
    double pr_x = up_normed | x;
    return atan2(pr_y, pr_x);
}


double OptimizeElastic::ComputeDifferenceAngle(Point r ,const double *x)
{
    Point point = ComputeEdge(halfedge_at_ends_.second, x);
    return ComputeDifferenceAngle(point, bishop_at_end_, r);
}


OptimizeElastic::Point OptimizeElastic::ComputeTwistGradient(const Point &E, const Point &F, const int i)
{
    Point Kb = ComputeKb(E, F);
    if (i == 0)
    {
        return Kb / E.norm() / 2;
    }
    else if (i == 2)
    {
        return Kb / F.norm() / -2;
    }
    else
    {
        return Kb * (-1 / 2 / E.norm() + 1 / 2 / F.norm());
    }
}


OptimizeElastic::Point OptimizeElastic::ComputeKb(const Point &E, const Point &F)
{
    return (E % F) / (E.norm() * F.norm() + (E | F));
}


OptimizeElastic::Point OptimizeElastic::ComputeKb(const int i, const double *x)
{
    Point e, f;
    computeEF(i, x, e, f);
    return ComputeKb(e, f);
}


inline void OptimizeElastic::AddHessianAtEntry(const int i, const int j, const int c_i, const int c_j, double *x, const double value)
{
    x[(i*3 + c_i)*(i*3 + c_i + 1)/2 + j*3 + c_j] += value;
}


inline void OptimizeElastic::AddHessianAtDiagonal(const int i, const int j, double *x, double value)
{
    AddHessianAtEntry(i, j, 0, 0, x, value);
    AddHessianAtEntry(i, j, 1, 1, x, value);
    AddHessianAtEntry(i, j, 2, 2, x, value);
}


void OptimizeElastic::AddHessianAtEntries(const int i, const int j, double *x, const double *values)
{
    if (i == j)
    {
        for (int c_i = 0; c_i < 3; ++c_i)
        {
            for (int c_j = 0; c_j <= c_i; ++c_j)
            {
                x[(i*3 + c_i)*(i*3 + c_i + 1)/2 + j*3 + c_j] += values[3*c_i + c_j];
            }
        }
    }
    else
    {
        for (int c_i = 0; c_i < 3; ++c_i)
        {
            for (int c_j = 0; c_j < 3; ++c_j)
            {
                x[(i*3 + c_i)*(i*3 + c_i + 1)/2 + j*3 + c_j] += values[3*c_i + c_j];
            }
        }
    }

}


inline void OptimizeElastic::AddHessianAtEntries(const int i, const int j, double *x, const Matrix3 &values)
{
    if (i == j)
    {
        for (int c_i = 0; c_i < 3; ++c_i)
        {
            for (int c_j = 0; c_j <= c_i; ++c_j)
            {
                x[(i*3 + c_i)*(i*3 + c_i + 1)/2 + j*3 + c_j] += values(c_i, c_j);
            }
        }
    }
    else
    {
        for (int c_i = 0; c_i < 3; ++c_i)
        {
            for (int c_j = 0; c_j < 3; ++c_j)
            {
                x[(i*3 + c_i)*(i*3 + c_i + 1)/2 + j*3 + c_j] += values(c_i, c_j);
            }
        }
    }

}


void OptimizeElastic::AddHessianAtEntriesBendingHelper(const int i, const int j, const Point &p, const Point &q, double *values)
{
    double m[9];
    setZeros(m, 9);
    multiplyByScale(q.data(), p[0], m);
    multiplyByScale(q.data(), p[1], m + 3);
    multiplyByScale(q.data(), p[2], m + 6);
    AddHessianAtEntries(i, j, values, m);
}


void OptimizeElastic::AddHessianAtEntriesBendingHelper(const Point &p, const Point &q, double m[9])
{
    setZeros(m, 9);
    multiplyByScale(q.data(), p[0], m);
    multiplyByScale(q.data(), p[1], m + 3);
    multiplyByScale(q.data(), p[2], m + 6);
}


// m = p * q^T
void OptimizeElastic::AddHessianAtEntriesBendingHelper(const Point &p, const Point &q, Matrix3 &m)
{
    Eigen::Vector3d v_p(p.data());
    Eigen::Vector3d v_q(q.data());
    m = v_p * v_q.transpose();
}


OptimizeElastic::Matrix3 OptimizeElastic::ComputePQT(const Point &p, const Point &q)
{
    Eigen::Vector3d v_p(p.data());
    Eigen::Vector3d v_q(q.data());
    return v_p * v_q.transpose();
}


void OptimizeElastic::AddHessianBending(const int i, const int row, const int col,
                                        double *values, const double *x, const double coef)
{
    Point e, f;
    computeEF(i, x, e, f);
    double P = EdgesLengthProduct[i];
    double deno = P + (e | f);
    double tmp = deno * deno * deno;
    tmp = coef / tmp;
    int p, c, n;
    p = idxPrevVertex[i];
    c = idxTheVertex[i];
    n = idxNextVertex[i];
    Matrix3 m = Matrix3::Zero();
    if (row == p)
    {
        if (col == c)//H_(i)(i-1)
        {
            m += Matrix3::Identity() * (-2 * P * deno);
            m += ComputePQT(4 * P * f, e - f);
        }
        else if (col == n)//H_(i+1)(i-1)
        {
            m += Matrix3::Identity() * (2 * P * deno * tmp);
            m += ComputePQT(-4 * P * f, e);
        }
        else if (col == p)//H_(i-1)(i-1)
        {
            m += ComputePQT(4 * P *f, f);
        }
    }
    else if (row == c)
    {
        if (col == p)//H_(i-1)(i)
        {
            m += Matrix3::Identity() * (-2 * P * deno);
            m += ComputePQT(4 * P * (e - f), f);
        }
        else if (col == n)//H_(i+1)(i)
        {
            m += Matrix3::Identity() * (-2 * P * deno);
            m += ComputePQT(4 * P * (f - e), e);
        }
        else if (col == c)//H_(i)(i)
        {
            m += Matrix3::Identity() * (4 * P * deno);
            m += ComputePQT(4 * P * (e - f), e - f);
        }
    }
    else if (row == n)
    {
        if (col == p)//H_(i-1)(i+1)
        {
            m += Matrix3::Identity() * (2 * P * deno);
            m += ComputePQT(-4 * P * e, f);
        }
        else if (col == c)//H_(i)(i+1)
        {
            m += Matrix3::Identity() * (-2 * P * deno);
            m += ComputePQT(4 * P * e, f - e);
        }
        else if (col == n)// H_(i+1)(i+1)
        {
            m += ComputePQT(4 * P * e, e);
        }
    }
    m *= tmp;
    AddHessianAtEntries(row, col, values, m);
}


void OptimizeElastic::AddHessianCrossing(const int row, const int col, const Point &p, double *values)
{
    Matrix3 m = Matrix3::Zero();
    m(0,1) = -p[2];
    m(0,2) = p[1];
    m(1,2) = -p[0];
    m(1,0) = p[2];
    m(2,0) = -p[1];
    m(2,1) = p[0];
    AddHessianAtEntries(row, col, values, m);
}


void OptimizeElastic::AddHessianTwistingStep1(double *values, const double *x, const double coef)
{
    PetCurve::HalfedgeHandle he_hnd = halfedge_at_ends_.first;
    PetCurve::VertexHandle pv, pc, pn;
    Point e, f, P(0,0,0), Q(0,0,0);
    int idx_pv, idx_pc, idx_pn;
    std::vector<Eigen::Vector3d> gradients(vertices.size(), Eigen::Vector3d(0,0,0));
    while (he_hnd != halfedge_at_ends_.second)
    {
        pv = curve->from_vertex_handle(he_hnd);
        pc = curve->to_vertex_handle(he_hnd);
        he_hnd = curve->next_halfedge_handle(he_hnd);
        pn = curve->to_vertex_handle(he_hnd);
        idx_pv = insertAdditionalPoint(pv);
        idx_pc = insertAdditionalPoint(pc);
        idx_pn = insertAdditionalPoint(pn);
        e = getPoint(idx_pc, x) - getPoint(idx_pv, x);
        f = getPoint(idx_pn, x) - getPoint(idx_pc, x);
        if (idx_pv >= 0)
        {
            gradients[idx_pv] += Eigen::Vector3d((ComputeKb(e,f)).data()) / (2 * e.norm());
        }
        if (idx_pn >= 0)
        {
            gradients[idx_pn] += Eigen::Vector3d((ComputeKb(e,f)).data()) / (2 * f.norm());
        }
        if (idx_pc >= 0)
        {
            gradients[idx_pc] += Eigen::Vector3d((ComputeKb(e, f)).data()) * (-1 / (2 * e.norm()) + 1 / (2 * f.norm()));
        }
    }
    for (unsigned int i = 0; i < vertices.size(); ++i)
    {
        for (unsigned int j = 0; j <= i; ++j)
        {
            AddHessianAtEntries(i, j, values, gradients[i] * coef * gradients[j].transpose());
        }
    }
}


void OptimizeElastic::AddHessianTwistingStep2(double *values, const double *x, const double coef)
{
    PetCurve::HalfedgeHandle he_hnd = halfedge_at_ends_.first;
    PetCurve::VertexHandle pv, pc, pn;
    Point e, f;
    int idx_pv, idx_pc, idx_pn;
    while (he_hnd != halfedge_at_ends_.second)
    {
        pv = curve->from_vertex_handle(he_hnd);
        pc = curve->to_vertex_handle(he_hnd);
        he_hnd = curve->next_halfedge_handle(he_hnd);
        pn = curve->to_vertex_handle(he_hnd);
        idx_pv = insertAdditionalPoint(pv);
        idx_pc = insertAdditionalPoint(pc);
        idx_pn = insertAdditionalPoint(pn);
        e = getPoint(idx_pc, x) - getPoint(idx_pv, x);
        f = getPoint(idx_pn, x) - getPoint(idx_pc, x);
        if (idx_pv >= 0)
        {
            AddHessianAtEntries(idx_pv, idx_pv, values, ComputeGradientKb(e, f, -1) * (coef / (2 * e.norm())));
            if (idx_pc > idx_pv)
                AddHessianAtEntries(idx_pc, idx_pv, values, ComputeGradientKb(e, f, -1) * (coef * (-1 / (2 * e.norm()) + 1 / (2 * f.norm()))));
            if (idx_pn > idx_pv)
                AddHessianAtEntries(idx_pn, idx_pv, values, ComputeGradientKb(e, f, -1) * (coef / (2 * f.norm())));
        }
        if (idx_pn >= 0)
        {
            AddHessianAtEntries(idx_pn, idx_pn, values, ComputeGradientKb(e, f, 1) * (coef / (2 * f.norm())));
            if (idx_pv > idx_pn)
                AddHessianAtEntries(idx_pv, idx_pn, values, ComputeGradientKb(e, f, 1) * (coef / (2 * e.norm())));
            if (idx_pc > idx_pn)
                AddHessianAtEntries(idx_pc, idx_pn, values, ComputeGradientKb(e, f, 1) * (coef * (-1 / (2 * e.norm()) + 1 / (2 * f.norm()))));
        }
        if (idx_pc >= 0)
        {
            AddHessianAtEntries(idx_pc, idx_pc, values, ComputeGradientKb(e, f, 0) * (coef * (-1 / (2 * e.norm()) + 1 / (2 * f.norm()))));
            if (idx_pn > idx_pc)
                AddHessianAtEntries(idx_pn, idx_pc, values, ComputeGradientKb(e, f, 0) * (coef / (2 * f.norm())));
            if (idx_pv > idx_pc)
                AddHessianAtEntries(idx_pv, idx_pc, values, ComputeGradientKb(e, f, 0) * (coef / (2 * e.norm())));
        }
    }
}


OptimizeElastic::Matrix3 OptimizeElastic::ComputeGradientKb(const Point &e, const Point &f, const int i)
{
    Point Kb = ComputeKb(e, f);
    Matrix3 m = Matrix3::Zero();
    if (i == -1)
        m += 2 * ComputeCrossMatrix(f) + ComputePQT(Kb, f);
    else if (i == 0)
        m += -2 * ComputeCrossMatrix(e + f) + ComputePQT(Kb, e -f);
    else if (i == 1)
        m += 2 * ComputeCrossMatrix(e) - ComputePQT(Kb, e);
    m /= e.norm()*f.norm() + (e|f);
    return m;
}


OptimizeElastic::Matrix3 OptimizeElastic::ComputeCrossMatrix(const Point &p)
{
    Matrix3 m = Matrix3::Zero();
    m(0,1) = -p[2];
    m(0,2) = p[1];
    m(1,2) = -p[0];
    m(1,0) = p[2];
    m(2,0) = -p[1];
    m(2,1) = p[0];
    return m;
}


double OptimizeElastic::ComputeTotalLength()
{
    double total_length = 0;
    Point pc, pv, pn;
    PetCurve::HalfedgeHandle h_hnd = halfedge_at_ends_.first;
    while (h_hnd != halfedge_at_ends_.second)
    {
        pv = curve->point(curve->from_vertex_handle(h_hnd));
        pc = curve->point(curve->to_vertex_handle(h_hnd));
        h_hnd = curve->next_halfedge_handle(h_hnd);
        pn = curve->point(curve->to_vertex_handle(h_hnd));
        total_length += (pc - pv).norm() / 2;
        total_length += (pn - pc).norm() / 2;
    }
    return total_length;
}
