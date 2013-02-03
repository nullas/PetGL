#ifndef PETCURVE_H
#define PETCURVE_H

#include <vector>
#include <QString>
#include "PetMesh.h"

using namespace std;

class PetCurve : public PetMesh
{

public:
    PetCurve();
    bool read_curve(QString File);
    virtual bool save(QString filename);
    OpenMesh::EPropHandleT<bool> isCurveEdge;
    OpenMesh::HPropHandleT<bool> isCurveHalfEdge;
    typedef PetCurve::Face Curve;
    typedef PetCurve::FaceHandle CurveHandle;
    typedef PetCurve::FaceIter CurveIter;
    typedef PetCurve::FaceVertexIter FaceVertexIter;
    typedef PetCurve::FaceEdgeIter CurveEdgeIter;
    typedef PetCurve::FaceHalfedgeIter CurveHalfedgeIter;

    virtual void init();
    virtual bool iscurve();
    virtual void updateVBO();
    virtual void createVBO();
    virtual void render();

    virtual void getSelectedEdges(std::vector<EdgeHandle> &hnd);
    virtual void getSelectedEdges(std::vector<unsigned int>&);

    void setCurveSelected(const unsigned int);
    void setCurvesSelected(const std::vector<unsigned int>&);

    void setCurveUnselected(const unsigned int);
    void setCurvesUnselected(const std::vector<unsigned int>&);

    void getSelectedCurves(std::vector<unsigned int>&);

    //handle based

    void getSelectedCurves(std::vector<PetCurve::CurveHandle>& idx);

    void setVerticesSelectedByCurves(const std::vector<PetCurve::CurveHandle> &hnd);
    void setVerticesSelectedByCurve(const PetCurve::CurveHandle& hnd);

    void clearSelectedCurves();

    inline unsigned int n_curve_edges(){return _n_curve_edges;}

private:
    unsigned int _n_curve_edges;

};

#endif // PETCURVE_H
