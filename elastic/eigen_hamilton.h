#ifndef EIGEN_HAMILTON_H
#define EIGEN_HAMILTON_H

#include <set>
#include <deque>
#include <Eigen/Core>
#include <Eigen/Sparse>
#include <fstream>
#include <ctime>

#include "elastic.h"

class HamiltonProjection
{
public:
    typedef Eigen::Matrix3d Mat3d;
    typedef Eigen::Vector3d Vec3d;
    typedef Eigen::SparseMatrix<double, Eigen::ColMajor> SpMat;
    typedef Eigen::VectorXd VecXd;
    HamiltonProjection(Elastic* p);
    ~HamiltonProjection();
    Elastic* elastic_;
    std::vector<PetCurve::VertexHandle>& vertices_;
    int n_variables_;
    Eigen::VectorXd rhs_, lhs_, x_, v_, gradients_;
    Eigen::VectorXd g_;
    SpMat grad_g_;
    typedef Eigen::Triplet<double> Tri;
    std::vector<Eigen::Triplet<double> > triplet_;
    const Elastic::pOptimize* parameter_;
    ofstream fout;
    bool check_derivative_, tangent_gradient_, initialized_;
    void check_derivative(bool check);
    double dt_;
    int n_constraints_;
    std::map<PetCurve::VertexHandle,int> idx_mapping_;
    std::map<PetCurve::HalfedgeHandle,double> edge_length_mapping_;
    typedef std::map<PetCurve::CurveHandle, double> CurvePropertyDouble;
    CurvePropertyDouble curve_length_mapping_;
    CurvePropertyDouble curve_twisting_;
    CurvePropertyDouble curve_material_;
    typedef std::map<PetCurve::CurveHandle, int> CurvePropertyInt;
    CurvePropertyInt curve_twisting_times_;
    std::vector<double> edge_length_;
    PetCurve* curve_;
    std::vector<pair<PetCurve::HalfedgeHandle, PetCurve::HalfedgeHandle> > crossings_;
    std::vector<PetCurve::HalfedgeHandle> halfedges_;
    int Next();
    int point_index(const PetCurve::VertexHandle &v_hnd);
    void GetEdgeLengths();
    void GetCrossings();
    typedef PetCurve::Point Point;
    Eigen::Vector3d point(const PetCurve::VertexHandle &v_hnd, const Eigen::VectorXd& x);
    double LineSegmentsSqDistance(const Point &p0, const Point &p1, const Point &q0, const Point &q1);
    double LineSegmentsSqDistance(const PetCurve::HalfedgeHandle &h_hnd1, const PetCurve::HalfedgeHandle &h_hnd2);
    int ComputeGradients(const Eigen::VectorXd &x, Eigen::VectorXd& gradients);
    void GetHalfedges();
    double dt(const Eigen::VectorXd &x, const Eigen::VectorXd &grads);
    int SymplecticEuler();
    int Projection();
    int ComputeG(const Eigen::VectorXd &x, Eigen::VectorXd &g);
    Eigen::Vector3d ComputeEdge(const PetCurve::HalfedgeHandle &h_hnd, const Eigen::VectorXd x);
    double CrossingDistance(const unsigned int i, const Eigen::VectorXd &x);
    double LineSegmentsSqDistance(const Eigen::Vector3d &p0, const Eigen::Vector3d &p1, const Eigen::Vector3d &q0, const Eigen::Vector3d &q1);
    int ComputeGradientG(const Eigen::VectorXd &x, SpMat &g);
    Eigen::Vector3d ComputeKb(const Eigen::Vector3d &e, const Eigen::Vector3d &f);
    Eigen::Matrix3d ComputeJKb(const Eigen::Vector3d &e, const Eigen::Vector3d &f, int i);
    Eigen::Matrix3d CrossMatrix(const Eigen::Vector3d &p);
    int GetStartPoint();
    double CrossLineSegmentDistance(unsigned int i, const Eigen::VectorXd &x);
    void InsertTriplet(const int i, const int j, const Vec3d &v, std::vector<Tri> &triplet);
    void ComputeGradientCrossing(const int idx_cross, const int base, const Eigen::VectorXd &x, std::vector<Tri> &triplet);
    double LineSegmentsSqDistance(const PetCurve::HalfedgeHandle &h_hnd1, const PetCurve::HalfedgeHandle &h_hnd2, const Eigen::VectorXd &x);
    double UpdateTheta(const PetCurve::CurveHandle &curve, const Eigen::VectorXd &x);
    Eigen::Vector3d ParallelTransport(Vec3d t1, Vec3d t2, const Vec3d &x);
    double ComputeDifferenceAngle(const Vec3d &axis, const Vec3d &up, const Vec3d &x);
    int InitialTwisting();
    int UpdateMesh();
    int UpdateCrossings(const Eigen::VectorXd &x);
    double ComputeWritheFraction(const PetCurve::CurveHandle &curve, const Eigen::VectorXd &x);
    int ComputeEnergy(const Eigen::VectorXd &x, double &energy);
    double MaxNorm(const Eigen::VectorXd &x);
    struct Run
    {
        int step;
        double timing_tangent_optimal;
        double timing_dt;
        double timing_projection;
    }run_;
    HamiltonProjection::Vec3d ComputeGradientBending(const Vec3d &e, const Vec3d f, int i);
    double RelativeError(const double x, const double y);
    int CheckDerivative();
    double Rand();
    int OutputStatus();
    double OneNorm(const VecXd &v);
    double dt_full(const VecXd &x, const VecXd &grads);
};

#endif // EIGEN_HAMILTON_H
