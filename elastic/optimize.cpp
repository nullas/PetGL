#include "optimize.h"

#include <cmath>

#define _unused(x) ((void)x)
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
    PetCurve::Point p0, p1, pc;
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
                VerticesWeight.push_back(2./((p0 - pc).norm() + (p1 - pc).norm()));
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
            positions.push_back(OpenMesh::vector_cast<OpenMesh::Vec3d>((*p_it).second));
            PositionConstraints.push_back(idxMapping[(*p_it).first]);
        }
    }
    OpenMesh::Vec3d p0d, p1d, pcd;
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
            tmp = (p0 - p1).norm();
            TangentConstraints.push_back(pair<int,int>(first, second));
            pcd = OpenMesh::vector_cast<OpenMesh::Vec3d>(t_it->second);
            pcd /= tmp * pc.norm();
            tangents.push_back(pcd);
        }
    }
    n_constraints = edges.size();
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
        additionalPoints.push_back(OpenMesh::vector_cast<OpenMesh::Vec3d>(curve->point(v_hnd)));
        return i;
    }
}

bool Optimize::get_nlp_info(Ipopt::Index &n, Ipopt::Index &m,
                            Ipopt::Index &nnz_jac_g, Ipopt::Index &nnz_h_lag,
                            Ipopt::TNLP::IndexStyleEnum &index_style)
{
    n = n_variables;
    m = n_constraints;
    nnz_jac_g = edges.size() * 6;
    nnz_h_lag = EnergyVertices.size() * 18 + PositionConstraints.size() * 3 + edges.size() * 9;
    index_style = Ipopt::TNLP::C_STYLE;
    return true;
}

bool Optimize::get_bounds_info(Ipopt::Index n, Ipopt::Number *x_l, Ipopt::Number *x_u,
                               Ipopt::Index m, Ipopt::Number *g_l, Ipopt::Number *g_u)
{
    assert(n == n_variables);
    assert(m == n_constraints);
//    PointArrayEdit xl = x_l, xu = x_u;
//    PetCurve::Point point;
//    int n_points = vertices.size();
//    for (int i  = 0; i < n_points; ++i)
//    {
//        point = curve->point(vertices[i]);
////        xl(i,0) = point[0] - 0.1;
////        xl(i,1) = point[1] - 0.1;
////        xl(i,2) = point[2] - 0.1;
////        xu(i,0) = point[0] + 0.1;
////        xu(i,1) = point[1] + 0.1;
////        xu(i,2) = point[2] + 0.1;
    for ( int i = 0; i < n; i++)
    {
        x_l[i] = -2e19;
        x_u[i] =  2e19;
    }
    for (int i = 0; i < n_constraints; ++i)
    {
        g_l[i] = g_u[i] = edgesSqLength[i];
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
    assert(init_x == true);
    _unused(z_L);
    _unused(z_U);
    _unused(lambda);
    PointArrayEdit px = x;
    PetCurve::Point point;
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
    _unused(n);
    _unused(new_x);
    obj_value = 0;
    PointArray px = PointArray(x);
    Ipopt::Number t[3];
    Ipopt::Number r = 0;
    int i = 0, size = EnergyVertices.size();
    for (; i < size; i++)
    {
        obj_value += computeSqK(i, x) * VerticesWeight[i];
    }
    obj_value *= pOp->BendingEnergyCoef;
    i = 0;
    size = PositionConstraints.size();
    for (; i < size; i++)
    {
        sub(px(PositionConstraints[i]), positions[i].data(), t);
        r += sqrnorm(t);
    }
    r *= pOp->PositionConstraintsWeight;
    obj_value += r;
    return true;
}

bool Optimize::eval_grad_f(Ipopt::Index n, const Ipopt::Number *x, bool new_x, Ipopt::Number *grad_f)
{
    assert(n == n_variables);
    _unused(new_x);
    Ipopt::Number t1[3], t2[3], t[3], e[3], f[3], tmp;
    Ipopt::Number* pV;
    PointArrayEdit pf = grad_f;
    PointArray px = x;
    setZeros(grad_f, n);
    int idx;
    int i = 0, size = EnergyVertices.size();
    for (; i < size; i++)
    {
        tmp = pOp->BendingEnergyCoef * VerticesWeight[i] / EdgesLengthProduct[i];
        computeEF(i, x, e, f);
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
    return true;
}

bool Optimize::eval_g(Ipopt::Index n, const Ipopt::Number *x, bool new_x, Ipopt::Index m, Ipopt::Number *g)
{
    _unused(n);
    _unused(m);
    _unused(new_x);
    int i = 0, size = edges.size();
    for (; i < size; i++)
        g[i] = computeEdgeLength(i, x);
    return true;
}

bool Optimize::eval_jac_g(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                        Ipopt::Index m, Ipopt::Index nele_jac,
                        Ipopt::Index *iRow, Ipopt::Index *jCol, Ipopt::Number *values)
{
    _unused(n);
    _unused(m);
    _unused(new_x);
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
        for (; idx < nele_jac; idx++)
        {
            iRow[idx] = 0;
            jCol[idx] = 0;
        }
    }
    else
    {
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
        for (; idx < nele_jac; idx++)
        {
            values[idx] = 0;
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
    _unused(n);
    _unused(m);
    _unused(new_x);
    _unused(new_lambda);
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
        i = 0;
        size = EnergyVertices.size();
        for (; i < size; ++i)
        {
            idx_pv = idxPrevVertex[i];
            idx_nv = idxNextVertex[i];
            idx_cv = idxTheVertex[i];
            if (idx_pv >= 0)
            {
                if (idx_cv > idx_pv)
                {
                    setHessianPos(idx_cv, idx_pv, iRow, jCol, idx);
                }
                if (idx_nv > idx_pv)
                {
                    setHessianPos(idx_nv, idx_pv, iRow, jCol, idx);
                }
            }
            if (idx_nv >= 0)
            {
                if (idx_pv > idx_nv)
                {
                    setHessianPos(idx_pv, idx_nv, iRow, jCol, idx);
                }
                if (idx_cv > idx_nv)
                {
                    setHessianPos(idx_cv, idx_nv, iRow, jCol, idx);
                }
            }
            if (idx_cv >= 0)
            {
                setHessianPos(idx_cv, idx_cv, iRow, jCol, idx);
                if (idx_pv > idx_cv)
                {
                    setHessianPos(idx_pv, idx_cv, iRow, jCol, idx);
                }
                if (idx_nv > idx_cv)
                {
                    setHessianPos(idx_nv, idx_cv, iRow, jCol, idx);
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

        Ipopt::Number tmp;

        i = 0;
        size = PositionConstraints.size();
        tmp = 2 * pOp->PositionConstraintsWeight * obj_factor;
        for (; i < size; i++)
        {
            setHessianValues(idx, values, tmp);
        }
        i = 0;
        size = EnergyVertices.size();
        for (; i < size; i++)
        {
            tmp = obj_factor * pOp->BendingEnergyCoef / EdgesLengthProduct[i] * VerticesWeight[i];
            idx_pv = idxPrevVertex[i];
            idx_nv = idxNextVertex[i];
            idx_cv = idxTheVertex[i];
            if (idx_pv >= 0)
            {
                if (idx_cv > idx_pv)
                {
                    setHessianValues(idx, values, -tmp);
                }
                if (idx_nv > idx_pv)
                {
                    setHessianValues(idx, values, tmp);
                }
            }
            if (idx_nv >= 0)
            {
                if (idx_pv > idx_nv)
                {
                    setHessianValues(idx, values, tmp);
                }
                if (idx_cv > idx_nv)
                {
                    setHessianValues(idx, values, -tmp);
                }
            }
            if (idx_cv >= 0)
            {
                setHessianValues(idx, values, 2 * tmp);
                if (idx_pv > idx_cv)
                {
                    setHessianValues(idx, values, -tmp);
                }
                if (idx_nv > idx_cv)
                {
                    setHessianValues(idx, values, -tmp);
                }
            }
        }
        for (; idx < nele_hess; idx++)
        {
            values[idx] = 0;
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
    assert(n == int(vertices.size() * 3));
    int idx = 0, size = vertices.size();
    for (; idx < size; idx++)
    {
        curve->set_point(vertices[idx],OpenMesh::vector_cast<PetCurve::Point>(OpenMesh::Vec3d(x + 3 * idx)));
    }
}

void Optimize::computeEF(const int i, const Ipopt::Number *x, Ipopt::Number *e, Ipopt::Number *f)
{
    const Ipopt::Number *thevertex;
    thevertex = theVertices(i, x);
    sub(thevertex, prevVertices(i, x), e);
    sub(nextVertices(i, x), thevertex, f);
}

Ipopt::Number Optimize::computeSqK(int i, const Ipopt::Number *x)
{
    Ipopt::Number e[3], f[3];
    computeEF(i, x, e, f);
    return (1 - dot(e,f) / EdgesLengthProduct[i]);
}

void Optimize::computeKb(int i, const Ipopt::Number *x, Ipopt::Number *r)
{
    Ipopt::Number e[3], f[3];
    computeEF(i, x, e, f);
    cross(e, f, r);
    Ipopt::Number dn = EdgesLengthProduct[i] + dot(e,f);
    if (dn < Mep)
    {
        setZeros(r);
        return;
    }
    multiplyByScale(r, 2/dn);
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
    sub(pv, nv, e);
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

inline void Optimize::setZeros(Ipopt::Number *r)
{
    r[0] = r[1] = r[2] = 0;
}

inline void Optimize::setZeros(Ipopt::Number *r, int n)
{
    for (int i = 0; i < n; ++i)
        r[i] = 0;
}

inline void Optimize::setValues(Ipopt::Number *r, Ipopt::Number v)
{
    r[0] = r[1] = r[2] = v;
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

inline void Optimize::multiplyByScaleTo(const Ipopt::Number *x, Ipopt::Number dn, Ipopt::Number *r)
{
    r[0] += x[0] * dn;
    r[1] += x[1] * dn;
    r[2] += x[2] * dn;
}

inline void Optimize::divideByScale(const Ipopt::Number *x, Ipopt::Number dn, Ipopt::Number *r)
{
    r[0] = x[0] / dn;
    r[1] = x[1] / dn;
    r[2] = x[2] / dn;
}
