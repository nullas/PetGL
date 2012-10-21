#ifndef VIEWER_H
#define VIEWER_H

#include <QGLViewer/qglviewer.h>

class Viewer : public QGLViewer {

  Q_OBJECT

public:
  Viewer(QWidget * parent);

  // overload several QGLViewer virtual functions
  void draw();
  void init();
  QWidget m_parent;

}; // end class Viewer

#endif // VIEWER_H
