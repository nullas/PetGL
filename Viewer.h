#ifndef VIEWER_H
#define VIEWER_H

#include <limits.h>
#include <vector>

#include <GL/glew.h>

#include <QGLViewer/qglviewer.h>

#include "PetGL.h"
#include "Viewer.h"
#include "PetMesh.h"
#include "PetCurve.h"


#include "PetMesh.h"

class Viewer : public QGLViewer {

  Q_OBJECT

public:
  Viewer(QWidget * parent);

  // overload several QGLViewer virtual functions
  void draw();
  void init();

  void drawMesh(PetMesh& petMesh);

}; // end class Viewer

#endif // VIEWER_H
