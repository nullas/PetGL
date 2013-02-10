#ifndef VIEWER_H
#define VIEWER_H

#include <limits.h>
#include <vector>

#include <GL/glew.h>

#include <QMouseEvent>

#include <QGLViewer/qglviewer.h>

#include "PetGL.h"
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
  void Select(QMouseEvent*);
  void Deselect(QMouseEvent*);
  void drawPickVertices();
  void pickVertices(QMouseEvent* e);
  void unpickVertices(QMouseEvent*);
  void drawPickEdges();
  void pickEdges(QMouseEvent* e);
  void unpickEdges(QMouseEvent* e);
protected:
  virtual void mousePressEvent(QMouseEvent* e);
  virtual void keyPressEvent(QKeyEvent *e);
private:
  static const int PickRadius;
  int selecttype;
}; // end class Viewer

#endif // VIEWER_H
