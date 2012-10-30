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

};

#endif // PETCURVE_H
