#ifndef OPTIMIZE_ELASITC_H
#define OPTIMIZE_ELASITC_H


#include <set>
#include <deque>
#include <coin/IpTNLP.hpp>
#include <Eigen/Core>

#include "elastic.h"

class OptimizeElastic : public Ipopt::TNLP
{
public:

    //naive implementation
    template <typename T>
    class PointArray_t
    {
    public:
        PointArray_t(const T* _p) : p(_p) {}
        inline const T& operator()(const int i, const int j) const {return p[i * 3 + j];}
        inline const T* operator()(const int i) const { return p + 3 * i;}
        PointArray_t& operator=(const T* _p) {p = _p;}
    private:
        const T* p;
    };
    typedef PointArray_t<Ipopt::Number> PointArray;

    template <typename T>
    class PointArrayEdit_t
    {
    public:
        PointArrayEdit_t(T* _p) : p(_p) {}
        inline T& operator()(const int i, const int j) const {return p[i * 3 + j];}
        inline T* operator()(const int i) const { return p + 3 * i;}
        PointArrayEdit_t& operator=(T* _p) {p = _p;}
    private:
        T* p;
    };
    typedef PointArrayEdit_t<Ipopt::Number> PointArrayEdit;
    typedef OpenMesh::Vec3d Point;
    typedef Eigen::Matrix<double, 3, 3, Eigen::RowMajor> Matrix3;
    typedef typename Eigen::Vector3d Vector3;

    OptimizeElastic(Elastic* p);
    ~OptimizeElastic();
    virtual bool get_nlp_info(Ipopt::Index &n, Ipopt::Index &m,
                              Ipopt::Index &nnz_jac_g, Ipopt::Index &nnz_h_lag, Ipopt::TNLP::IndexStyleEnum &index_style);
    virtual bool get_bounds_info(Ipopt::Index n, Ipopt::Number *x_l, Ipopt::Number *x_u,
                                 Ipopt::Index m, Ipopt::Number *g_l, Ipopt::Number *g_u);
    virtual bool get_starting_point(Ipopt::Index n,
                                    bool init_x, Ipopt::Number *x,
                                    bool init_z, Ipopt::Number *z_L, Ipopt::Number *z_U,
                                    Ipopt::Index m, bool init_lambda, Ipopt::Number *lambda);

    virtual bool eval_f(Ipopt::Index n, const Ipopt::Number *x, bool new_x, Ipopt::Number &obj_value);
    virtual bool eval_grad_f(Ipopt::Index n, const Ipopt::Number *x, bool new_x, Ipopt::Number *grad_f);
    virtual bool eval_g(Ipopt::Index n, const Ipopt::Number *x, bool new_x, Ipopt::Index m, Ipopt::Number *g);
    virtual bool eval_jac_g(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                            Ipopt::Index m, Ipopt::Index nele_jac,
                            Ipopt::Index *iRow, Ipopt::Index *jCol, Ipopt::Number *values);
    virtual bool eval_h(Ipopt::Index n, const Ipopt::Number *x,
                        bool new_x, Ipopt::Number obj_factor,
                        Ipopt::Index m, const Ipopt::Number *lambda, bool new_lambda,
                        Ipopt::Index nele_hess, Ipopt::Index *iRow, Ipopt::Index *jCol, Ipopt::Number *values);

    virtual void finalize_solution(Ipopt::SolverReturn status, Ipopt::Index n, const Ipopt::Number *x,
                                   const Ipopt::Number *z_L, const Ipopt::Number *z_U,
                                   Ipopt::Index m, const Ipopt::Number *g, const Ipopt::Number *lambda,
                                   Ipopt::Number obj_value,
                                   const Ipopt::IpoptData *ip_data, Ipopt::IpoptCalculatedQuantities *ip_cq);


    double computeBeta(int i, const Ipopt::Number *x);
    Ipopt::Number computeEdgeLength(int i, const Ipopt::Number *x);
    const Ipopt::Number* EdgePrevVertex(int i, const Ipopt::Number *x);
    const Ipopt::Number* EdgeNextVertex(int i, const Ipopt::Number *x);
    const Ipopt::Number* TangentPrevVertex(int i, const Ipopt::Number *x);
    const Ipopt::Number* TangentNextVertex(int i, const Ipopt::Number *x);
    void computeTangentEdge(int i, const Ipopt::Number *x, Ipopt::Number* r);
    void computeEdge(int i, const Ipopt::Number *x, Ipopt::Number *r);
    Point ComputeEdge(const PetCurve::HalfedgeHandle &h_hnd, const Ipopt::Number *x);


    PetCurve::Point getPoint(const int i, const double* x);

    inline void cross(const Ipopt::Number* p, const Ipopt::Number* q, Ipopt::Number* r);
    inline void add(const Ipopt::Number* p, const Ipopt::Number* q, Ipopt::Number* r);
    inline void sub(const Ipopt::Number* p, const Ipopt::Number* q, Ipopt::Number* r);
    inline void addTo(const Ipopt::Number *p, Ipopt::Number* dst);
    inline void copy(const Ipopt::Number* src, Ipopt::Number* dst);
    inline void copy(const Ipopt::Number* src, Ipopt::Number* dst, const Ipopt::Index n);
    inline void multiplyByScale(Ipopt::Number *x, Ipopt::Number dn);
    inline void multiplyByScale(Ipopt::Number *x, Ipopt::Number dn, int n);
    inline void divideByScale(Ipopt::Number *x, Ipopt::Number dn);
    inline void multiplyByScale(const Ipopt::Number *x, Ipopt::Number dn, Ipopt::Number *r);
    inline void multiplyByScaleTo(const Ipopt::Number *x, Ipopt::Number dn, Ipopt::Number *r);
    inline void multiplyByScaleTo(const Ipopt::Number *x, Ipopt::Number dn, Ipopt::Number *r, int n);
    inline void divideByScale(const Ipopt::Number *x, Ipopt::Number dn, Ipopt::Number *r);
    inline void setZeros(Ipopt::Number *r);
    inline void setZeros(Ipopt::Number *r, int n);
    inline void setValues(Ipopt::Number *r, Ipopt::Number v);
    void crossNormal(const double* p0,
                     const double* p1,
                     const double* q0,
                     const double* q1,
                     double* r);
    void crossDiag(const double* p0,
                   const double* p1,
                   const double* q0,
                   const double* q1,
                   double* r);
    void setHessianPos(const int i, const int j,
                                    Ipopt::Index* iRow,Ipopt::Index* jCol,
                                    int& idx);
    void setHessianPos_bending(const int i, const int j,
                                    Ipopt::Index* iRow,Ipopt::Index* jCol,
                                    int& idx);
    inline void setHessianValues(int& idx, Ipopt::Number* x, const Ipopt::Number v);
    void setHessianValues_bending(const int i, const int row, const int col,
                                            int& idx, double *values, const double* x, const double v);
    void setHessianVal_bending_helper(const Point& p, const Point& q, double *values, const int s = 1);
    void setHessianVal_bending_diag(const double v, double *values, const int s = 1);
    void setHessianPos_SkewSym(const int i, const int j, Ipopt::Index* iRow,Ipopt::Index* jCol, int& idx);
    void setHessianValues_SkewSym(const int i, const int row, const int col,
                                  Ipopt::Number *values, int& idx, const double *x, double weight);
    void setHessianValues_SkewSym_helper(const Point& p, Ipopt::Number *values, int& idx);

    void computeEF(const int i, const Ipopt::Number *x, Point& e, Point& f);
    void computeEF(const int i, const Ipopt::Number *x, double *e, double *f);
    Point ComputeKb(const int i, const double *x);

    inline Ipopt::Number dot(const Ipopt::Number* p, const Ipopt::Number* q);
    inline Ipopt::Number sqrnorm(const Ipopt::Number* p);
    inline Ipopt::Number norm(const Ipopt::Number* p);

    inline const Ipopt::Number* prevVertices(const int i, const Ipopt::Number *x);
    inline const Ipopt::Number* theVertices(const int i, const Ipopt::Number *x);
    inline const Ipopt::Number* nextVertices(const int i, const Ipopt::Number *x);


    //edges intersection
    //fast collision detection to filter out line segments far away.
    bool LineSegmentsCollide(const Point& p0,
                                       const Point& p1,
                                       const Point& q0,
                                       const Point& q1);

    bool LineSegmentsCollide(const PetCurve::HalfedgeHandle& i, const PetCurve::HalfedgeHandle& j);

    //Line segments distance
    double LineSegmentsSqDistance(const Point& p0,
                                       const Point& p1,
                                       const Point& q0,
                                       const Point& q1);

    double LineSegmentsSqDistance(const int p0,
                                       const int p1,
                                       const int q0,
                                       const int q1,
                                  const double* x);

    double LineSegmentsSqDistance(const PetCurve::HalfedgeHandle& he_i, const PetCurve::HalfedgeHandle& he_j);

    double LineSegmentsSqDistance(const PetCurve::HalfedgeHandle& he_i, const PetCurve::HalfedgeHandle& he_j, const double *x);

    double LineSegmentsSqDistance(const int idx, const int ref_i, const int ref_j, const double* x);

    double LineSegmentsDistance(const Point& p0,
                                       const Point& p1,
                                       const Point& q0,
                                       const Point& q1);

    double LineSegmentsDistance(const PetCurve::EdgeHandle, const PetCurve::EdgeHandle);

    double CrossLineSegmentsDistance(const Point& p0,
                                     const Point& p1,
                                     const Point& q0,
                                     const Point& q1);

    void EdgesIntersections();

    bool updateCross(const double* x);

    double computeCrossDiagDistance(const int i, const double* x);

    void setJacGPos_cross(const int ref, const int base, const int i, int& idx, int* iRow, int* jCol);

    void ComputeGradientCross(const int i, int& idx, double *values, const double *x);

    void AddHessianCrossing(const int row, const int col, const Point &p, double *values);

    // material frames
    Point ParallelTransportation(Point t1, Point t2, const Point &x);

    double ComputeDifferenceAngle(const Point &axis, const Point &up, const Point &x);

    Point ComputeTwistGradient(const Point &E, const Point &F, const int i);

    Point ComputeKb(const Point &E, const Point &F);

    // Hessian setting
    inline void AddHessianAtEntry(int i, int j, const int c_i, const int c_j, double *x, const double value);

    inline void AddHessianAtDiagogal(const int i, const int j, double *x, double value);

    inline void AddHessianAtEntries(const int i, const int j, double *x, const double *values);

    void AddHessianAtEntries(const int i, const int j, double *x, const Matrix3 &values);

    void AddHessianAtEntriesBendingHelper(const Point &p, const Point &q, Matrix3 &m);

    void AddHessianAtEntriesBendingHelper(const int i, const int j, const Point &p, const Point &q, double *values);

    void AddHessianAtEntriesBendingHelper(const Point &p, const Point &q, double m[9]);

    void AddHessianBending(const int i, const int row, const int col,
                           double *values, const double *x, const double coef);

    void AddHessianTwistingStep1(double *values, const double *x, const double coef);

    Matrix3 ComputePQT(const Point &p, const Point &q);
    Matrix3 ComputeGradientKb(const Point &e, const Point &f, const int i);
    Matrix3 ComputeCrossMatrix(const Point &p);
    void AddHessianTwistingStep2(double *values, const double *x, const double coef);
    void AddHessianAtDiagonal(const int i, const int j, double *x, double value);
    double ComputeTotalLength();
    double ComputeWritheFraction(const double *x);
    Point ComputeParallelTransportation(Point v, const double *x);
    Point FindPerpendicularVec(const PetCurve::HalfedgeHandle &h_hnd, const double *x, Point p=Point(0,0,1));
    bool ComputeWritheNumber(const double *x);
    double ComputeWritheFractionByArea(const double *x);
    double SphericalArea(const Point &t_N, const Point &t_1, Point t_2) const;
    inline double Sign(const double p) const
    {
        if (p > 0) return 1;
        if (p < 0) return -1;
        return 0;
    }

    inline double SphericalAreaHelper(const Point &t_k, const Point &t_l, const Point &t_m) const
    {
        return acos(((t_k|t_m) - (t_k | t_l)*(t_l | t_m)) / ((t_k % t_l).norm() * (t_l % t_m).norm()));
    }

private:
    Elastic* pElastic;
    PetCurve* curve;
    std::vector<PetCurve::VertexHandle>& vertices;
    std::vector<PetCurve::EdgeHandle> edges;
    std::vector<pair<int, int> > idxVertexEdges;
    std::vector<int> idxPrevVertex;
    std::vector<int> idxTheVertex;
    std::vector<int> idxNextVertex;
    Ipopt::Index n_variables, n_constraints;
    std::vector<Ipopt::Number> EdgesLengthProduct,VerticesWeight,edgesSqLength;
    std::vector<int> PositionConstraints;
    std::vector<pair<PetCurve::Point, double> > PlaneConstraintsInfo;
    std::vector<int> PlaneConstraints;
    std::vector<pair<int,int> > TangentConstraints;
    std::vector<OpenMesh::Vec3d> tangents;
    std::map<PetCurve::VertexHandle,int> idx_mapping_;
    std::vector<PetCurve::VertexHandle> EnergyVertices;
    Elastic::pOptimize* pOp;
    std::vector<OpenMesh::Vec3d> additional_points_;
    int InsertAdditionalPoint(const PetCurve::VertexHandle& v_hnd);
    std::vector<OpenMesh::Vec3d> positions;

    // for crossing
    std::vector<std::pair<PetCurve::HalfedgeHandle, PetCurve::HalfedgeHandle> > crosses;
    std::vector<deque<int> > crossE;
    std::vector<deque<int> > crossF;
    std::vector<pair<int, int> > crossRef;
    int refSize;

    // material frame
    int twist_times_;
    double writhe_number_;
    PetCurve::HalfedgeHandle halfedge_at_end_;
    double total_length_;
};

#endif // OPTIMIZE_ELASITC_H
