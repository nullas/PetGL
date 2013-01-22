#ifndef PETCURVE_H
#define PETCURVE_H

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

    virtual void init();
    virtual void updateVBO();
    virtual void createVBO();
    virtual void render();

private:
    inline unsigned int n_curve_edges(){return _n_curve_edges;}
    unsigned int _n_curve_edges;

};

#endif // PETCURVE_H
