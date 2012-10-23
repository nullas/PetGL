#ifndef VIEWER_H
#define VIEWER_H

#include <QGLViewer/qglviewer.h>

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
