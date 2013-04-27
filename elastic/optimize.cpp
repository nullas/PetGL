#include "optimize.h"

#include <cmath>
#include <cblas.h>
#include <limits>

#define UNUSED(x) ((void)x)
#define Mep 1e-8

Optimize::Optimize(Elastic* p) : pElastic(p), vertices(p->verticesToOptimize)
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
    EdgesIntersections();
    n_constraints = edges.size() + crosses.size();
}


Optimize::~Optimize()
{

}


//only cares the additional Point
int Optimize::insertAdditionalPoint(const PetCurve::VertexHandle &v_hnd)
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

bool Optimize::get_nlp_info(Ipopt::Index &n, Ipopt::Index &m,
                            Ipopt::Index &nnz_jac_g, Ipopt::Index &nnz_h_lag,
                            Ipopt::TNLP::IndexStyleEnum &index_style)
{
    n = n_variables;
    m = n_constraints;
    int tmp = 12 * (1 + pOp->extension);
    nnz_jac_g = edges.size() * 6 + crosses.size()  * tmp;
//    tmp = 4 * (1 + pOp->extension);
//    tmp = 3 * tmp * (tmp - 1);
    tmp = 12 * (refSize - 2) + (refSize - 1) * (refSize - 1) * 6;
    nnz_h_lag = EnergyVertices.size() * 45 + PositionConstraints.size() * 3 + PlaneConstraints.size() * 9
            + edges.size() * 9 + crosses.size() * tmp;
    index_style = Ipopt::TNLP::C_STYLE;
    return true;
}

bool Optimize::get_bounds_info(Ipopt::Index n, Ipopt::Number *x_l, Ipopt::Number *x_u,
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

bool Optimize::get_starting_point(Ipopt::Index n, bool init_x, Ipopt::Number *x,
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
    return true;
}

bool Optimize::eval_f(Ipopt::Index n, const Ipopt::Number *x, bool new_x, Ipopt::Number &obj_value)
{
    UNUSED(n);
    UNUSED(new_x);
    if (updateCross(x) == false) return false;
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

bool Optimize::eval_grad_f(Ipopt::Index n, const Ipopt::Number *x, bool new_x, Ipopt::Number *grad_f)
{
    assert(n == n_variables);
    UNUSED(new_x);
    if (updateCross(x) == false) return false;
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

bool Optimize::eval_g(Ipopt::Index n, const Ipopt::Number *x, bool new_x, Ipopt::Index m, Ipopt::Number *g)
{
    UNUSED(n);
    UNUSED(m);
    UNUSED(new_x);
    if (updateCross(x) == false) return false;
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

bool Optimize::eval_jac_g(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                        Ipopt::Index m, Ipopt::Index nele_jac,
                        Ipopt::Index *iRow, Ipopt::Index *jCol, Ipopt::Number *values)
{
    UNUSED(n);
    UNUSED(m);
    UNUSED(new_x);

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
        int ref;
        int end = crosses.size();
        i = 0;
        for (; i < end; ++i)
        {
            for (ref = 1; ref < refSize; ++ref)
            {
                setJacGPos_cross(ref, size, i, idx, iRow, jCol);
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
        if (updateCross(x) == false) return false;
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
        int ref;
        int end = crosses.size();
        i = 0;
        for (; i < end; ++i)
        {
            for (ref = 1; ref < refSize; ++ref)
            {
                setJacGVal_cross(ref, i, idx, values, x);
            }
        }
    }
    assert(idx <= nele_jac);

    return true;
}

bool Optimize::eval_h(Ipopt::Index n, const Ipopt::Number *x,
                              bool new_x, Ipopt::Number obj_factor,
                              Ipopt::Index m, const Ipopt::Number *lambda, bool new_lambda,
                              Ipopt::Index nele_hess, Ipopt::Index *iRow, Ipopt::Index *jCol, Ipopt::Number *values)
{
    UNUSED(n);
    UNUSED(m);
    UNUSED(new_lambda);
    UNUSED(new_x);
    int idx_pv, idx_nv, idx_cv;
    Ipopt::Number e[3];
    int idx = 0;
    int i, size;
    int idx_pc;
    if (values == NULL)
    {
        i = 0, size = edges.size();
        for (; i < size; i++)
        {
            idx_pv = idxVertexEdges[i].first;
            idx_nv = idxVertexEdges[i].second;
            if (idx_pv >= 0)
            {
                setHessianPos(idx_pv, idx_pv, iRow, jCol, idx);
                if (idx_nv > idx_pv)
                {
                    setHessianPos(idx_nv, idx_pv, iRow, jCol, idx);
                }
            }
            if (idx_nv >= 0)
            {
                setHessianPos(idx_nv, idx_nv, iRow, jCol, idx);
                if (idx_pv > idx_nv)
                {
                    setHessianPos(idx_pv, idx_nv, iRow, jCol, idx);
                }
            }
        }

        i = 0;
        size = PositionConstraints.size();
        for (; i < size; i++)
        {
            idx_pc = PositionConstraints[i];
            setHessianPos(idx_pc, idx_pc, iRow, jCol, idx);
        }

        //Plane Constraints
        i = 0;
        size = PlaneConstraints.size();
        for (; i < size; ++i)
        {
            idx_pc = PlaneConstraints[i];
            setHessianPos_bending(idx_pc, idx_pc, iRow, jCol, idx);
        }

        //Bending energy
        i = 0;
        size = EnergyVertices.size();
        for (; i < size; ++i)
        {
            idx_pv = idxPrevVertex[i];
            idx_nv = idxNextVertex[i];
            idx_cv = idxTheVertex[i];
            if (idx_pv >= 0)
            {
                setHessianPos_bending(idx_pv, idx_pv, iRow, jCol, idx);
                if (idx_cv > idx_pv)
                {
                    setHessianPos_bending(idx_cv, idx_pv, iRow, jCol, idx);
                }
                if (idx_nv > idx_pv)
                {
                    setHessianPos_bending(idx_nv, idx_pv, iRow, jCol, idx);
                }
            }
            if (idx_nv >= 0)
            {
                setHessianPos_bending(idx_nv, idx_nv, iRow, jCol, idx);
                if (idx_pv > idx_nv)
                {
                    setHessianPos_bending(idx_pv, idx_nv, iRow, jCol, idx);
                }
                if (idx_cv > idx_nv)
                {
                    setHessianPos_bending(idx_cv, idx_nv, iRow, jCol, idx);
                }
            }
            if (idx_cv >= 0)
            {
                setHessianPos_bending(idx_cv, idx_cv, iRow, jCol, idx);
                if (idx_pv > idx_cv)
                {
                    setHessianPos_bending(idx_pv, idx_cv, iRow, jCol, idx);
                }
                if (idx_nv > idx_cv)
                {
                    setHessianPos_bending(idx_nv, idx_cv, iRow, jCol, idx);
                }
            }
        }
        int ref_E, ref_F, p, q;
        i = 0;
        size = crosses.size();
        for (; i < size; ++i)
        {
            for (ref_E = 1; ref_E < refSize - 1; ++ref_E)
            {
                if (crossE[i][ref_E] >= 0 && crossE[i][ref_E + 1] >= 0)
                {
                    setHessianPos_SkewSym(
                            crossE[i][ref_E],
                            crossE[i][ref_E + 1],
                            iRow, jCol, idx);
                }
            }
            for (ref_F = 1; ref_F < refSize - 1; ++ref_F)
            {
                if (crossF[i][ref_F] >= 0 && crossF[i][ref_F + 1] >= 0)
                {
                    setHessianPos_SkewSym(
                            crossF[i][ref_F],
                            crossF[i][ref_F + 1],
                            iRow, jCol, idx);
                }
            }
            for (ref_E = 1; ref_E < refSize; ++ref_E)
            {
                p = crossE[i][ref_E];
                if (p < 0) continue;
                for (ref_F = 1; ref_F < refSize; ++ref_F)
                {
                    q = crossF[i][ref_F];
                    if (q >= 0)
                    {
                        setHessianPos_SkewSym(p, q, iRow, jCol, idx);
                    }
                }
            }
        }
        for (; idx < nele_hess; idx++)
        {
            iRow[idx] = 0;
            jCol[idx] = 0;
        }
    }
    else
    {
        if (updateCross(x) == false) return false;
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
                setHessianValues(idx, values, 2 * lambda[i]);
                if (idx_nv > idx_pv)
                {
                    setHessianValues(idx, values, -2 * lambda[i]);
                }
            }
            if (idx_nv >= 0)
            {
                setHessianValues(idx, values, 2 * lambda[i]);
                if (idx_pv > idx_nv)
                {
                    setHessianValues(idx, values, -2 * lambda[i]);
                }
            }
        }

        i = 0;
        size = PositionConstraints.size();
        tmp = 2 * pOp->PositionConstraintsWeight * obj_factor;
        for (; i < size; i++)
        {
            setHessianValues(idx, values, tmp);
        }

        //Plane constraints
        Point P;
        i = 0;
        size = PlaneConstraints.size();
        for (; i < size; ++i)
        {
            P = PlaneConstraintsInfo[i].first;
            setHessianVal_bending_helper(P * 2 * obj_factor * pOp->PlaneConstraintsCoef, P, values + idx, 0);
            idx += 6;
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
                setHessianValues_bending(i, idx_pv, idx_pv, idx, values, x, tmp);
                if (idx_cv > idx_pv)
                {
                    setHessianValues_bending(i, idx_cv, idx_pv, idx, values, x, tmp);
                }
                if (idx_nv > idx_pv)
                {
                    setHessianValues_bending(i, idx_nv, idx_pv, idx, values, x, tmp);
                }
            }
            if (idx_nv >= 0)
            {
                setHessianValues_bending(i, idx_nv, idx_nv, idx, values, x, tmp);
                if (idx_pv > idx_nv)
                {
                    setHessianValues_bending(i, idx_pv, idx_nv, idx, values, x, tmp);
                }
                if (idx_cv > idx_nv)
                {
                    setHessianValues_bending(i, idx_cv, idx_nv, idx, values, x, tmp);
                }
            }
            if (idx_cv >= 0)
            {
                setHessianValues_bending(i, idx_cv, idx_cv, idx, values, x, tmp);
                if (idx_pv > idx_cv)
                {
                    setHessianValues_bending(i, idx_pv, idx_cv, idx, values, x, tmp);
                }
                if (idx_nv > idx_cv)
                {
                    setHessianValues_bending(i, idx_nv, idx_cv, idx, values, x, tmp);
                }
            }
        }

        //crossing constraints
        int ref_E, ref_F, p, q;
        int base = edges.size();
        i = 0;
        size = crosses.size();
        for (; i < size; ++i)
        {
            for (ref_E = 1; ref_E < refSize - 1; ++ref_E)
            {
                if (crossE[i][ref_E] >= 0 && crossE[i][ref_E + 1] >= 0)
                {
                    setHessianValues_SkewSym(i, crossE[i][ref_E], crossE[i][ref_E + 1], values, idx, x, lambda[base + i]);
                }
            }
            for (ref_F = 1; ref_F < refSize - 1; ++ref_F)
            {
                if (crossF[i][ref_F] >= 0 && crossF[i][ref_F + 1] >= 0)
                {
                    setHessianValues_SkewSym(i, crossF[i][ref_F], crossF[i][ref_F + 1], values, idx, x, lambda[base + i]);
                }
            }
            for (ref_E = 1; ref_E < refSize; ++ref_E)
            {
                p = crossE[i][ref_E];
                if (p < 0) continue;
                for (ref_F = 1; ref_F < refSize; ++ref_F)
                {
                    q = crossF[i][ref_F];
                    if (q >= 0)
                    {
                        setHessianValues_SkewSym(i, p, q, values, idx, x, lambda[base + i]);
                    }
                }
            }
        }

    }
    assert(idx <= nele_hess);

    return true;
}

void Optimize::finalize_solution(Ipopt::SolverReturn status, Ipopt::Index n, const Ipopt::Number *x,
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


void Optimize::setHessianPos_SkewSym(const int i, const int j, Ipopt::Index* iRow,Ipopt::Index* jCol, int& idx)
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
void Optimize::setHessianValues_SkewSym(const int i, const int row, const int col,
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

void Optimize::setHessianValues_SkewSym_helper(const Point& p, Ipopt::Number *values, int& idx)
{
    values[idx++] = -p[2];
    values[idx++] = p[1];
    values[idx++] = -p[0];
    values[idx++] = p[2];
    values[idx++] = -p[1];
    values[idx++] = p[0];
}

void Optimize::setJacGPos_cross(const int ref, const int base, const int i, int& idx, int* iRow, int* jCol)
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

void Optimize::setJacGVal_cross(const int ref, const int i, int& idx, double *values, const double *x)
{
    int first = crossE[i][ref];
    int second = crossF[i][ref];
    Point p;
    if (first >= 0)
    {
        if (ref == crossRef[i].first)
        {
            p = JacG_cross(i, 0, x);
            copy(p.data(), values + idx);
        }
        else if (ref == crossRef[i].first + 1)
        {
            p = JacG_cross(i, 1, x);
            copy(p.data(), values + idx);
        }
        else
        {
            setZeros(values + idx);
        }
        idx += 3;
    }
    if (second >= 0)
    {
        if (ref == crossRef[i].second)
        {
            p = JacG_cross(i, 2, x);
            copy(p.data(), values + idx);
        }
        else if (ref == crossRef[i].second + 1)
        {
            p = JacG_cross(i, 3, x);
            copy(p.data(), values + idx);
        }
        else
        {
            setZeros(values + idx);
        }
        idx += 3;
    }
}



Optimize::Point Optimize::JacG_cross(const int idx_cross, int vertexposition, const double* x)
{
    /*------------------------------------------
                   p0_  q1
                    |\ /
                      /
                    |/ \
                   q0-  p1
    ------------------------------------------*/

    Point p0,p1,q0,q1,result;
    p0 = getPoint(crossE[idx_cross][crossRef[idx_cross].first], x);
    q0 = getPoint(crossF[idx_cross][crossRef[idx_cross].second], x);
    p1 = getPoint(crossE[idx_cross][crossRef[idx_cross].first+1], x);
    q1 = getPoint(crossF[idx_cross][crossRef[idx_cross].second+1], x);

    switch (vertexposition)
    {
    case 0:
        //p0
        result = (q1 - p1) % (q0 - q1);
        break;
    case 1:
        //p1
        result = (p0 - q1) % (q0 - q1);
        break;
    case 2:
        //q0
        result = (p0 - p1) % (q1 - p1);
        break;
    case 3:
        //q1
        result = (p0 - p1) % (p1 - q0);
        break;

    }
    return result;
}



void Optimize::computeEF(const int i, const Ipopt::Number *x, Point& e, Point& f)
{
    Point p = getPoint(idxPrevVertex[i], x);
    Point c = getPoint(idxTheVertex[i], x);
    Point n = getPoint(idxNextVertex[i], x);
    e = c - p;
    f = n - c;
}

void Optimize::computeEF(const int i, const Ipopt::Number *x, double *e, double *f)
{
    const double *p = prevVertices(i, x);
    const double *c = theVertices(i, x);
    const double *n = nextVertices(i, x);
    sub(c, p, e);
    sub(n, c, f);
}


double Optimize::computeBeta(int i, const Ipopt::Number *x)
{
    double e[3], f[3], d;
    computeEF(i, x, e, f);
    d = dot(e, f);
    return (EdgesLengthProduct[i] - d) / (EdgesLengthProduct[i] + d);
}

void Optimize::computeEdge(int i, const Ipopt::Number *x, Ipopt::Number *r)
{
    const Ipopt::Number *pv, *nv;
    pv = EdgePrevVertex(i, x);
    nv = EdgeNextVertex(i, x);
    sub(nv, pv, r);
}

Ipopt::Number Optimize::computeEdgeLength(int i, const Ipopt::Number *x)
{
    Ipopt::Number e[3];
    const Ipopt::Number *pv, *nv;
    pv = EdgePrevVertex(i, x);
    nv = EdgeNextVertex(i, x);
    sub(nv, pv, e);
    return sqrnorm(e);
}

const Ipopt::Number* Optimize::EdgePrevVertex(int i, const Ipopt::Number *x)
{
    int idx = idxVertexEdges[i].first;
    if (idx >= 0)
        return x + idx * 3;
    else
        return additionalPoints[-idx].data();
}

const Ipopt::Number* Optimize::EdgeNextVertex(int i, const Ipopt::Number *x)
{
    int idx = idxVertexEdges[i].second;
    if (idx >= 0)
        return x + idx * 3;
    else
        return additionalPoints[-idx].data();
}

const Ipopt::Number* Optimize::TangentPrevVertex(int i, const Ipopt::Number *x)
{
    int idx = TangentConstraints[i].first;
    if (idx >= 0)
        return x + 3 * idx;
    else
        return additionalPoints[-idx].data();
}

const Ipopt::Number* Optimize::TangentNextVertex(int i, const Ipopt::Number *x)
{
    int idx = TangentConstraints[i].second;
    if (idx >= 0)
        return x + 3 * idx;
    else
        return additionalPoints[-idx].data();
}


PetCurve::Point Optimize::getPoint(int i, const double* x)
{
    if (i >= 0)
        return PetCurve::Point(x + 3 * i);
    else
        return additionalPoints[-i];
}


void Optimize::computeTangentEdge(int i, const Ipopt::Number *x, Ipopt::Number* r)
{
    const Ipopt::Number* pv, *nv;
    pv = TangentPrevVertex(i, x);
    nv = TangentNextVertex(i, x);
    sub(nv, pv, r);
}

inline void Optimize::setZeros(Ipopt::Number *r)
{
    r[0] = r[1] = r[2] = 0;
}

inline void Optimize::setZeros(Ipopt::Number *r, int n)
{
    double tmp = 0;
    cblas_dcopy(n, &tmp, 0, r, 1);
}

inline void Optimize::setValues(Ipopt::Number *r, Ipopt::Number v)
{
    r[0] = r[1] = r[2] = v;
}

void Optimize::setHessianPos_bending(const int i, const int j,
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

void Optimize::setHessianPos(const int i, const int j,
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


inline void Optimize::setHessianValues(int& idx, Ipopt::Number* x, const Ipopt::Number v)
{
    x[idx] = x[idx + 1] = x[idx + 2] = v;
    idx += 3;
}


void Optimize::setHessianValues_bending(const int i, const int row, const int col,
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

void Optimize::setHessianVal_bending_helper(const Point& p, const Point& q, double *values, const int s)
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

void Optimize::setHessianVal_bending_diag(const double v, double *values, const int s)
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

inline void Optimize::cross(const Ipopt::Number* p, const Ipopt::Number* q, Ipopt::Number* r)
{
    r[0] = p[1] * q[2] - p[2] * q[1];
    r[1] = p[2] * q[0] - p[0] * q[2];
    r[2] = p[0] * q[1] - p[1] * q[0];
}

inline void Optimize::add(const Ipopt::Number* p, const Ipopt::Number* q, Ipopt::Number* r)
{
    r[0] = p[0] + q[0];
    r[1] = p[1] + q[1];
    r[2] = p[2] + q[2];
}

inline void Optimize::sub(const Ipopt::Number* p, const Ipopt::Number* q, Ipopt::Number* r)
{
    r[0] = p[0] - q[0];
    r[1] = p[1] - q[1];
    r[2] = p[2] - q[2];
}

inline void Optimize::addTo(const Ipopt::Number *p, Ipopt::Number* dst)
{
    dst[0] += p[0];
    dst[1] += p[1];
    dst[2] += p[2];
}

inline void Optimize::copy(const Ipopt::Number* src, Ipopt::Number* dst)
{
    dst[0] = src[0];
    dst[1] = src[1];
    dst[2] = src[2];
}

inline void copy(const Ipopt::Number* src, Ipopt::Number* dst, const Ipopt::Index n)
{
    cblas_dcopy(n, src, 1, dst, 1);
}

inline Ipopt::Number Optimize::dot(const Ipopt::Number* p, const Ipopt::Number* q)
{
    return p[0] * q[0] + p[1] * q[1] + p[2] * q[2];
}

inline Ipopt::Number Optimize::sqrnorm(const Ipopt::Number* p)
{
    return p[0] * p[0] + p[1] * p[1] + p[2] * p[2];
}

inline Ipopt::Number Optimize::norm(const Ipopt::Number* p)
{
    return sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]);
}

inline const Ipopt::Number* Optimize::prevVertices(const int i, const Ipopt::Number *x)
{
    int idx = idxPrevVertex[i];
    if (idx >= 0)
        return x + idx * 3;
    else
        return (additionalPoints[-idx]).data();
}

inline const Ipopt::Number* Optimize::theVertices(const int i, const Ipopt::Number *x)
{
    int idx = idxTheVertex[i];
    if (idx >= 0)
        return x + idx * 3;
    else
        return (additionalPoints[-idx]).data();
}

inline const Ipopt::Number* Optimize::nextVertices(const int i, const Ipopt::Number *x)
{
    int idx = idxNextVertex[i];
    if (idx >= 0)
        return x + idx * 3;
    else
        return (additionalPoints[-idx]).data();
}
inline void Optimize::multiplyByScale(Ipopt::Number *x, Ipopt::Number dn)
{
    x[0] *= dn;
    x[1] *= dn;
    x[2] *= dn;
}

inline void Optimize::divideByScale(Ipopt::Number *x, Ipopt::Number dn)
{
    x[0] /= dn;
    x[1] /= dn;
    x[2] /= dn;
}

inline void Optimize::multiplyByScale(const Ipopt::Number *x, Ipopt::Number dn, Ipopt::Number *r)
{
    r[0] = x[0] * dn;
    r[1] = x[1] * dn;
    r[2] = x[2] * dn;
}

inline void Optimize::multiplyByScale(Ipopt::Number *x, Ipopt::Number dn, int n)
{
    cblas_dscal(n, dn, x, 1);
}

inline void Optimize::multiplyByScaleTo(const Ipopt::Number *x, Ipopt::Number dn, Ipopt::Number *r)
{
    r[0] += x[0] * dn;
    r[1] += x[1] * dn;
    r[2] += x[2] * dn;
}

inline void Optimize::multiplyByScaleTo(const Ipopt::Number *x, Ipopt::Number dn, Ipopt::Number *r, int n)
{
    cblas_daxpy(n, dn, x, 1, r, 1);
}

inline void Optimize::divideByScale(const Ipopt::Number *x, Ipopt::Number dn, Ipopt::Number *r)
{
    r[0] = x[0] / dn;
    r[1] = x[1] / dn;
    r[2] = x[2] / dn;
}

void Optimize::crossNormal(const double* p0,
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


void Optimize::crossDiag(const double *p0, const double *p1, const double *q0, const double *q1, double *r)
{
    double p_center[3], q_center[3];
    add(p0, p1, p_center);
    multiplyByScale(p_center, 1/2);
    add(q0, q1, q_center);
    multiplyByScale(q_center, 1/2);
    sub(p_center, q_center, r);
}

bool Optimize::LineSegmentsCollide(const Point& p1,
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

bool Optimize::LineSegmentsCollide(const PetCurve::HalfedgeHandle& he_i, const PetCurve::HalfedgeHandle& he_j)
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


double Optimize::LineSegmentsSqDistance(const PetCurve::HalfedgeHandle& he_i, const PetCurve::HalfedgeHandle& he_j)
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

double Optimize::LineSegmentsSqDistance(const int idx, const int ref_i, const int ref_j, const double* x)
{
    return LineSegmentsSqDistance(
                getPoint(crossE[idx][ref_i], x),
                getPoint(crossE[idx][ref_i+1], x),
                getPoint(crossF[idx][ref_j], x),
                getPoint(crossF[idx][ref_j+1], x));
}


double Optimize::CrossLineSegmentsDistance(const Point &p0, const Point &p1, const Point &q0, const Point &q1)
{
    Point w = (p0 - p1) % (q0 - q1);
    if ( w.norm() / (p0 - p1).norm() / (q0 - q1).norm() < 5e-2)
    {
        std::cout << "really?" << std::endl;
    }
    return ((p0 + p1) / 2 - (q0 + q1) / 2) | w;
}

double Optimize::computeCrossDiagDistance(const int i, const double* x)
{
    return CrossLineSegmentsDistance(
            getPoint(crossE[i][crossRef[i].first],x),
            getPoint(crossE[i][crossRef[i].first+1],x),
            getPoint(crossF[i][crossRef[i].second],x),
            getPoint(crossF[i][crossRef[i].second+1],x)
            );
}


double Optimize::LineSegmentsSqDistance(const Point &p0, const Point &p1, const Point &q0, const Point &q1)
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
    if (D < Mep)
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
    sc = (fabs(sN) < Mep ? 0.0 : sN / sD);
    tc = (fabs(sN) < Mep ? 0.0 : tN / tD);
    Point dP = w + (sc * u) - (tc * v);
    return dP.sqrnorm();
}


double Optimize::LineSegmentsSqDistance(const int p0,
                                   const int p1,
                                   const int q0,
                                   const int q1,
                              const double* x)
{
    return LineSegmentsSqDistance(getPoint(p0,x), getPoint(p1,x), getPoint(q0,x), getPoint(q1,x));
}


void Optimize::EdgesIntersections()
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


bool Optimize::updateCross(const double* x)
{
//    return true;
    int first, second;
    int idx_ref;
    int idx_end = crosses.size();
    double min_dist;
    double distance;
    bool result = true;
    pair<int, int> update;
    for (idx_ref = 0; idx_ref < idx_end; ++idx_ref)
    {
        min_dist = numeric_limits<double>::max();
        for (first = crossRef[idx_ref].first - 1; first <= crossRef[idx_ref].first + 1; ++first)
        {
            for (second = crossRef[idx_ref].second - 1; second <= crossRef[idx_ref].second + 1; ++second)
            {
                distance = LineSegmentsSqDistance(idx_ref, first, second, x);
                if (distance < min_dist)
                {
                    update = pair<int, int>(first, second);
                    min_dist = distance;
                }
            }
        }
        if(update.first == 0 || update.second == 0 || update.first == refSize-1 || update.second >= refSize-1)
        {
            std::cout << "Extension touched!\n" << std::endl;
            result = false;
            break;
        }
        crossRef[idx_ref] = update;
    }
    return result;
}
