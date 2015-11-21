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
\brief   A GUI for Vector Localization; C++ Interface: VectorDisplay
\author  Joydeep Biswas, (C) 2010
*/
//========================================================================

#include "vector_display.h"

VectorDisplay::Color::Color(uint32_t col)
{
  r=float((col & 0xFF0000ull)>>16)/255.0;
  g=float((col & 0xFF00ull)>>8)/255.0;
  b=float(col & 0xFFull)/255.0;
}

VectorDisplay::Color::Color(float _r, float _g, float _b)
{
  r=_r;
  g=_g;
  b=_b;
}

VectorDisplay::VectorDisplay(QWidget* parent) : QGLWidget(QGLFormat ( QGL::SampleBuffers ),parent)
{
  lines.clear();
  points.clear();
  robotAngle = 0.0;
  robotLoc.set(0.0,0.0);
  displayWindow = 100.0;
  
  lineThickness = 2.0;
  pointsSize = 1.5;
  
  viewScale = 1.0;
  viewXOffset = viewYOffset = 0.0;
  
  ptrCallback = NULL;
  followRobot = false;
  showRobot = true;
  
  connect(this, SIGNAL(redraw()), this, SLOT(redrawHandler()));
  
}

void VectorDisplay::redrawHandler()
{
  graphicsMutex.lock();
  this->update();
  graphicsMutex.unlock();
}

void VectorDisplay::mousePressEvent(QMouseEvent* event)
{
  leftButton = event->buttons().testFlag(Qt::LeftButton);
  midButton = event->buttons().testFlag(Qt::MidButton);
  rightButton = event->buttons().testFlag(Qt::RightButton);
  bool shiftKey = event->modifiers().testFlag(Qt::ShiftModifier);
  bool ctrlKey = event->modifiers().testFlag(Qt::ControlModifier);
  bool altKey = event->modifiers().testFlag(Qt::AltModifier);
  
  if(leftButton || midButton){
    // Start Pan
    mouseStartX = event->x();
    mouseStartY = event->y();
    redraw();
  }
  if( (ctrlKey || shiftKey || altKey) && leftButton ){
    double scale = 2.0*viewScale*min(width(),height())/displayWindow;
    double x = (double(event->x()) - 0.5*double(width()) - viewXOffset)/scale;
    double y = (0.5*double(height()) - double(event->y()) - viewYOffset)/scale;
    
    setLocation.set(x,y);
    mouseStartX = event->x();
    mouseStartY = event->y();
  }
  
}

void VectorDisplay::mouseReleaseEvent(QMouseEvent* event)
{
  bool shiftKey = event->modifiers().testFlag(Qt::ShiftModifier);
  bool ctrlKey = event->modifiers().testFlag(Qt::ControlModifier);
  bool altKey = event->modifiers().testFlag(Qt::AltModifier);
  
  double scale = 2.0*viewScale*min(width(),height())/displayWindow;
  double x = (double(event->x()) - 0.5*double(width()) - viewXOffset)/scale;
  double y = (0.5*double(height()) - double(event->y()) - viewYOffset)/scale;
  vector2d setLocation2(x,y);
  
  if( shiftKey && leftButton ){
    double setOrientation = atan2(-(event->y()-mouseStartY),event->x()-mouseStartX);
    if(ptrCallback)
      ptrCallback(setLocation,setLocation2,setOrientation, 3);
  }else if( ctrlKey && leftButton ){
    double setOrientation = atan2(-(event->y()-mouseStartY),event->x()-mouseStartX);
    if(ptrCallback)
      ptrCallback(setLocation,setLocation2,setOrientation, 2);
  }else if( altKey && leftButton ){
    double setOrientation = atan2(-(event->y()-mouseStartY),event->x()-mouseStartX);
    if(ptrCallback)
      ptrCallback(setLocation,setLocation2,setOrientation, 1);
  }
}

void VectorDisplay::mouseMoveEvent(QMouseEvent* event)
{
  static const bool debug = false;
  bool leftButton = event->buttons().testFlag(Qt::LeftButton);
  bool midButton = event->buttons().testFlag(Qt::MidButton);
  bool rightButton = event->buttons().testFlag(Qt::RightButton);
  bool shiftKey = event->modifiers().testFlag(Qt::ShiftModifier);
  bool ctrlKey = event->modifiers().testFlag(Qt::ControlModifier);
  bool altKey = event->modifiers().testFlag(Qt::AltModifier);
  
  if(debug) printf("MouseMove Event, Left:%d Mid:%d Right:%d\n", leftButton?1:0, midButton?1:0, rightButton?1:0);
  if(shiftKey || ctrlKey || altKey){
    //No need to do anything - location or target will be set
    return;
  }
  if(leftButton){
    //Pan
    viewXOffset += double(event->x() - mouseStartX);
    viewYOffset -= double(event->y() - mouseStartY);
    mouseStartX = event->x();
    mouseStartY = event->y();
    redraw();
  }else if(midButton){
    //Zoom
    double zoomRatio = -double(event->y() - mouseStartY)/500.0;
    double oldScale = viewScale;
    viewScale = viewScale*(1.0+zoomRatio);
    viewXOffset *= viewScale/oldScale;
    viewYOffset *= viewScale/oldScale;
    mouseStartX = event->x();
    mouseStartY = event->y();
    redraw();
  }
}

void VectorDisplay::wheelEvent(QWheelEvent* event)
{
  static const bool debug = false;
  double zoomRatio = double(event->delta())/1000.0;
  double oldScale = viewScale;
  viewScale = viewScale*(1.0+zoomRatio);
  viewXOffset *= viewScale/oldScale;
  viewYOffset *= viewScale/oldScale;
  if(debug) printf("Zoom: %5.3f\n",viewScale);
  redraw();
}

void VectorDisplay::keyPressEvent(QKeyEvent* event)
{
  static const bool debug = false;
  // bool shiftKey = event->modifiers().testFlag(Qt::ShiftModifier);
  bool ctrlKey = event->modifiers().testFlag(Qt::ControlModifier);
  if(debug) printf("KeyPress: 0x%08X\n",event->key());
  if(event->key() == Qt::Key_Space)
    resetView();
  if(event->key() == Qt::Key_Escape)
    close();
  if(event->key() == Qt::Key_F){
    followRobot = !followRobot;
    if(followRobot)
      viewXOffset = viewYOffset = 0.0;
    redraw();
  }
  if(event->key() == Qt::Key_R){
    showRobot = !showRobot;
    redraw();
  }
  if(event->key() == Qt::Key_BracketLeft){
    if(ctrlKey)
      pointsSize = max(0.1,pointsSize-0.1);
    else
      lineThickness = max(0.1,lineThickness-0.1);
    redraw();
  }
  if(event->key() == Qt::Key_BracketRight){
    if(ctrlKey)
      pointsSize += 0.1;
    else
      lineThickness += 0.1;
    redraw();
  }
}

void VectorDisplay::resetView()
{
  viewScale = 1.0;
  viewXOffset = viewYOffset = 0.0;
  redraw();
}

void VectorDisplay::initializeGL()
{
  glEnable(GL_MULTISAMPLE);
}

void VectorDisplay::setupViewport(int width, int height)
{
  glViewport(0, 0, width, height);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho(-0.5*viewScale*width+viewXOffset, 0.5*viewScale*width+viewXOffset, -0.5*viewScale*height+viewYOffset, 0.5*viewScale*height+viewYOffset, minZValue, maxZValue);
  glMatrixMode(GL_MODELVIEW);
}

void VectorDisplay::resizeGL(int width, int height)
{
  setupViewport(width, height);
}

void VectorDisplay::drawCircles(float lineThickness)
{
  static const float circleSize = 0.3;
  
  bool coloredCircles = (circleColors.size()==circles.size());
  if(coloredCircles){
    for(unsigned int i=0; i<circles.size(); i++){
      glColor4f(circleColors[i].red(), circleColors[i].green(), circleColors[i].blue(), 1.0);
      drawArc(circles[i].x, circles[i].y, circleSize,circleSize+lineThickness,0,M_2PI);
    }
  }else{
    glColor4f(0.35,0.35,0.35,1.0);
    for(unsigned int i=0; i<circles.size(); i++){
      drawArc(circles[i].x, circles[i].y, circleSize,circleSize+lineThickness,0,M_2PI);
    }
  }
}

template <class num> void VectorDisplay::drawLine(GVector::vector2d< num >& p0, GVector::vector2d< num >& p1, num lineWidth)
{
  Line2d<num> l(p0,p1);
  drawLine<num>(l,lineWidth);
}

template <class num> void VectorDisplay::drawLine(Line2d< num >& line, num lineWidth)
{
  GVector::vector2d<num> v0=line.P0(), v1=line.P1();
  GVector::vector2d<num> d = line.Dir();
  GVector::vector2d<num> n = line.Perp()*lineWidth;
  d *= lineWidth;
  v0 = v0-d;
  v1 = v1+d;
  GVector::vector2d<num> p0=v0-n, p1=v1-n, p2=v1+n, p3=v0+n;
  
  //glColor4f(0.35,0.35,0.35,1.0);
  glBegin(GL_QUADS);
  glVertex3f(V2COMP(p0),0.0);
  glVertex3f(V2COMP(p1),0.0);
  glVertex3f(V2COMP(p2),0.0);
  glVertex3f(V2COMP(p3),0.0);
  glEnd();
}

template <class num> void VectorDisplay::drawPoint(GVector::vector2d<num> loc, float pointSize)
{
  drawQuad(loc.x-pointSize, loc.y-pointSize, loc.x+pointSize, loc.y+pointSize, 0.2);
}


void VectorDisplay::drawLines(float lineThickness)
{
  bool coloredLines = (lineColors.size()==lines.size());
  if(coloredLines){
    for(unsigned int i=0; i<lines.size(); i++){
      glColor4f(lineColors[i].red(), lineColors[i].green(), lineColors[i].blue(), 1.0);
      drawLine(lines[i],lineThickness);
    }
  }else{
    glColor4f(0.35,0.35,0.35,1.0);
    for(unsigned int i=0; i<lines.size(); i++){
      drawLine(lines[i],lineThickness);
    }
  }
}

void VectorDisplay::drawPoints(float pointsSize)
{
  bool coloredPoints = (pointColors.size()==points.size());
  if(coloredPoints){
    for(unsigned int i=0; i<points.size(); i++){
      glColor4f(pointColors[i].red(), pointColors[i].green(), pointColors[i].blue(), 1.0);
      drawPoint(points[i],pointsSize);
    }
  }else{
    glColor4f(0.94,0.46,0.12,1.0);
    
    for(unsigned int i=0; i<points.size(); i++){
      drawPoint(points[i],pointsSize);
    }
  }
}

void VectorDisplay::paintEvent(QPaintEvent* event)
{  
  
  graphicsMutex.lock();
  makeCurrent();
  glClearColor(BACKGROUND_COLOR);
  glShadeModel(GL_SMOOTH);
  glDisable(GL_LIGHTING);
  glDisable(GL_CULL_FACE);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_MULTISAMPLE);
  
  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glLoadIdentity();
  
  double scale = 2.0*viewScale*min(width(),height())/displayWindow;
  glTranslatef(viewXOffset, viewYOffset, 0.0);
  glScalef(scale,scale,scale);
  
  if(followRobot)
    glTranslatef(-(robotLoc.x),-(robotLoc.y),0.0);
  
  drawLines(lineThickness/scale);
  drawCircles(lineThickness/scale);
  drawPoints(pointsSize/scale);
  
  //drawArc(0,0,1.0, 1.5,0.0,M_2PI);
  if(showRobot){
    const double robotSize = 0.25;
    vector2d h;
    h.heading(robotAngle);
    vector2d p = robotLoc + 2.0*robotSize*h;
    glColor4f(1.0, 0.416, 0.0, 1.0);
    drawArc(robotLoc,robotSize, robotSize+1.5*lineThickness/scale,0.0,M_2PI,0.0,RAD(4.0));
    drawLine(robotLoc,p, 0.75*lineThickness/scale);
  }
  
  //vectorTextTest();
  glPopMatrix();
  swapBuffers();
  graphicsMutex.unlock();
}

void VectorDisplay::updateLines(vector<line2f> _lines, vector<Color> _lineColors)
{
  graphicsMutex.lock();
  lines = _lines;
  if(lines.size()==_lineColors.size())
    lineColors = _lineColors;
  else
    lineColors.clear();
  graphicsMutex.unlock();
  redraw();
}

void VectorDisplay::updatePoints(vector< vector2f > _points, vector< Color > _pointColors)
{
  graphicsMutex.lock();
  points = _points;
  if(points.size()==_pointColors.size())
    pointColors = _pointColors;
  else
    pointColors.clear();
  graphicsMutex.unlock();
  redraw();
}

void VectorDisplay::updateCircles(vector< vector2f > _circles, vector< Color > _circleColors)
{
  graphicsMutex.lock();
  circles = _circles;
  if(circles.size()==_circleColors.size())
    circleColors = _circleColors;
  else
    circleColors.clear();
  graphicsMutex.unlock();
  redraw();
}

void VectorDisplay::updateDisplay(vector2d _robotLoc, double _robotAngle, double _displayWindow, 
                                  vector<line2f> _lines, vector<vector2f> _points, vector<vector2f> _circles, 
                                  vector<Color> _lineColors, vector<Color> _pointColors, vector<Color> _circleColors)
{
  updateLines(_lines,_lineColors);
  updatePoints(_points,_pointColors);
  updateCircles(_circles,_circleColors);
  graphicsMutex.lock();
  robotAngle = _robotAngle;
  robotLoc = _robotLoc;
  displayWindow = _displayWindow;
  graphicsMutex.unlock();
  redraw();
}

void VectorDisplay::resizeEvent(QResizeEvent* event)
{
  QGLWidget::resizeEvent(event);
  redraw();
}

template <class num> void VectorDisplay::drawQuad(GVector::vector2d< num > loc1, GVector::vector2d< num > loc2, num z)
{
  glBegin(GL_QUADS);
  glVertex3d(loc1.x,loc1.y,z);
  glVertex3d(loc2.x,loc1.y,z);
  glVertex3d(loc2.x,loc2.y,z);
  glVertex3d(loc1.x,loc2.y,z);
  glEnd();
}

template <class num> void VectorDisplay::drawArc(GVector::vector2d< num > loc, num r1, num r2, num theta1, num theta2, num z, num dTheta)
{
  static const double tesselation = 0.1;
  if(dTheta<0){
    dTheta = tesselation/r2;
  }
  glBegin(GL_QUAD_STRIP);
  for(double theta=theta1; theta<theta2; theta+=dTheta){
    double c1 = cos(theta), s1 = sin(theta);
    glVertex3d(r2*c1+loc.x,r2*s1+loc.y,z);
    glVertex3d(r1*c1+loc.x,r1*s1+loc.y,z);
  }
  double c1 = cos(theta2), s1 = sin(theta2);
  glVertex3d(r2*c1+loc.x,r2*s1+loc.y,z);
  glVertex3d(r1*c1+loc.x,r1*s1+loc.y,z);
  glEnd();
}
