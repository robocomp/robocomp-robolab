//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    vector_display.h
\brief   C++ Implementation: VectorDisplay
\author  Joydeep Biswas, (C) 2010
*/
//========================================================================

#include <QWidget>
#include <QGLWidget>
#include <QMutex>
#include <QPainter>
#include <QPaintEvent>
#include <vector>

#ifndef VECTOR_LIDAR_DISPLAY_H
#define VECTOR_LIDAR_DISPLAY_H

#include "geometry.h"
//#include "gltext.h"

using namespace std;

#define BACKGROUND_COLOR 1.0,1.0,1.0,1.0

class VectorDisplay : public QGLWidget
{
  Q_OBJECT
  
public:
  class Color{
    public:
      float r,g,b;
      Color(){r=g=b=0.0;}
      Color(float _r, float _g, float _b);
      Color(uint32_t col);
      float red(){return r;};
      float green(){return g;};
      float blue(){return b;};
  };
  
private:
  static const double minZValue = -10;
  static const double maxZValue = 10;
  
  QMutex graphicsMutex; /// Mutex to arbitrate access to drawing primitives and display 
  vector<line2f> lines;
  vector<vector2f> points;
  vector<vector2f> circles;
  vector<Color> lineColors;
  vector<Color> pointColors;
  vector<Color> circleColors;
  
  float lineThickness;
  float pointsSize;
  
  vector2d robotLoc;
  double robotAngle;
  double displayWindow; /// The display window size (in world coords)
  
  double viewScale;
  double viewXOffset;
  double viewYOffset;
  
  bool followRobot;
  bool showRobot;
  
  bool leftButton;
  bool midButton;
  bool rightButton;
  
  int mouseStartX;
  int mouseStartY;
  //Callback for setting location, GoTo point, etc.
  void (*ptrCallback)(vector2d,vector2d,double,int);
  vector2d setLocation;
  
private slots:
  void redrawHandler(); /// Accept Signals to update display
signals:
  void redraw(); /// Thread-safe way of scheduling display updates
  
protected:
  void paintEvent ( QPaintEvent * event );
  void wheelEvent ( QWheelEvent * event );
  void mouseMoveEvent ( QMouseEvent * event );
  void keyPressEvent ( QKeyEvent * event );
  void mousePressEvent( QMouseEvent * event );
  void mouseReleaseEvent( QMouseEvent * event );
  void resizeEvent ( QResizeEvent * event );
  void initializeGL();
  void resizeGL(int width, int height);
  void setupViewport(int width, int height);
  QSize sizeHint () const {return QSize(580,1000);} 
  
  template <class num> void drawQuad(GVector::vector2d<num> loc1, GVector::vector2d<num> loc2, num z=0.0);
  void drawQuad(double x1, double y1, double x2, double y2, double z=0.0)
    {drawQuad(vector2d(x1,y1),vector2d(x2,y2),z);}
  template <class num> void drawArc(GVector::vector2d<num> loc, num r1, num r2, num theta1, num theta2, num z=0.0, num dTheta = -1);
  void drawArc(double x, double y, double r1, double r2, double theta1, double theta2, double z=0.0, double dTheta = -1)
    {drawArc(vector2d(x,y),r1,r2,theta1,theta2,z,dTheta);}
  void drawCircles(float lineThickness);
  void drawLines(float lineThickness);
  void drawPoints(float pointsSize);
  template <class num> void drawLine(Line2d<num> &line, num lineWidth);
  template <class num> void drawLine(GVector::vector2d<num> &p0, GVector::vector2d<num> &p1, num lineWidth);
  template <class num> void drawPoint(GVector::vector2d< num > loc, float pointSize);
  
public: 
  VectorDisplay(QWidget *parent = 0);
  void updateLines(vector<line2f> _lines, vector<Color> _lineColors = vector<Color>());
  void updatePoints(vector< vector2f > _points, vector<Color> _pointColors = vector<Color>());
  void updateCircles(vector< vector2f > _circles, vector<Color> _circleColors = vector<Color>());
  void updateDisplay(vector2d _robotLoc, double _robotAngle, double _displayWindow, 
                     vector<line2f> _lines, vector<vector2f> _points, vector<vector2f> _circles, 
                     vector<Color> _lineColors = vector<Color>(), 
                     vector<Color> _pointColors = vector<Color>(), 
                     vector<Color> _circleColors = vector<Color>());
  void resetView();
  /**
  Set callback function to call when the display is clicked
  @param _ptrCallback Function pointer of signature void (*)(vector2d,double,int). The first parameter of the callback is the click location (in world space), the second the orientation (in radians), the third the click type
  **/
  void setCallback(void (*_ptrCallback)(vector2d,vector2d,double,int) ){ptrCallback = _ptrCallback;}
};

#endif //VECTOR_LIDAR_DISPLAY_H
