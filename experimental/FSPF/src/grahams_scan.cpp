#include "grahams_scan.h"

static const double Epsilon = 0.000001;

GrahamsScan::GrahamsScan()
{
  firstPoint = NULL;
}

GrahamsScan::~GrahamsScan()
{
  deleteAllPoints();
}

std::vector< vector2f, std::allocator< vector2f > > GrahamsScan::run(vector< vector2f > points)
{
  static const bool debug = false;
  vector<vector2f> convexPolygon;
  convexPolygon.clear();
  int minPoint=0;
  point *tempPtr;
  NumPoints = int(points.size());
  firstPoint=NULL; //INIT FIRSTPOINT POINTER
  
  if(NumPoints<4)
    return points;
  
  if(debug) printf("\nGraham scan:\n");
  for (int k=1;k<NumPoints;k++){  //FIND MIN POINT
    if(debug)
      printf("%d. %7.4f , %7.4f\n",k,V2COMP(points[k]));
    if( (points[k].y==points[minPoint].y && points[k].x<points[minPoint].x) ||
      (points[k].y<points[minPoint].y) )
      minPoint=k;
  }
  
  if(debug) printf("%d points, minPoint:%d\n",NumPoints, minPoint);
  /*
  if(printDebugStream)
    debugStream<<NumPoints<<" points, minPoint:"<<minPoint<<"\n";
  */
  
  GrahamsScan::point p;
  for (int i=0;i<NumPoints;i++){ //SORT RANDOM POINTS
    p.loc = points[i];
    p.angle = findAngle(points[minPoint],points[i]);
    addPoint(p);
  }
  
  tempPtr=firstPoint;
  do{ //FIND LAST NODE IN LINKED LIST
    tempPtr=tempPtr->next;
  } while (tempPtr->next!=NULL);
  
  tempPtr->next=firstPoint; //COMPLETE CIRCULAR LINKED LIST
  firstPoint->prev=tempPtr; //COMPLETE CIRCULAR LINKED LIST
  if(debug){
    tempPtr=firstPoint;
    int k=0;
    printf("Sorted list:\n");
    do{ //FIND LAST NODE IN LINKED LIST
      printf("%d. %7.4f , %7.4f (%7.4f , %7.4f @ %.7f\u00b0)\n",k,V2COMP(tempPtr->loc),V2COMP(tempPtr->loc-firstPoint->loc), DEG((tempPtr->loc-firstPoint->loc).angle()));
      tempPtr=tempPtr->next;
      k++;
    } while (tempPtr!=firstPoint);
    printf("Checking Convexity:\n");
  }
  grahamScan(firstPoint);
  
  tempPtr=firstPoint;
  for (int i=0;i<NumPoints;i++) 
  {
    convexPolygon.push_back(tempPtr->loc);
    tempPtr=tempPtr->next;
    if(tempPtr==firstPoint)
      break;
  }
  deleteAllPoints();
  return convexPolygon;
}

void GrahamsScan::grahamScan(GrahamsScan::point* P)
{
  static const bool debug = false;
  point *tempPrev, *tempNext;
  P=P->next;
  do{
    while(P!=firstPoint && !isConvexPoint(P)){
      if(P->prev==firstPoint){
        tempPrev=P->prev; 
        tempNext=P->next;
        tempPrev->next=tempNext;
        tempNext->prev=tempPrev;
        if(debug) printf("deleting colinear point %f,%f\n",V2COMP(P->loc));
        delete P; //FREE MEMORY
        P = tempNext;
      }else{
        tempPrev=P->prev; 
        tempNext=P->next;
        tempPrev->next=tempNext;
        tempNext->prev=tempPrev;
        if(debug) printf("deleting non-convex point %f,%f\n",V2COMP(P->loc));
        delete P; //FREE MEMORY
        P = tempPrev;
      }
    }
    if(debug) printf("accepting convex point %f,%f\n",V2COMP(P->loc));
    P = P->next;
  }while(P != firstPoint);
  
}

bool GrahamsScan::isConvexPoint(GrahamsScan::point* P)
{
  return (P->loc - P->prev->loc).cross(P->next->loc - P->loc) > 0.0;
}

double GrahamsScan::findAngle(vector2f& loc1, vector2f& loc2)
{
  //This function doesn't really return the true angle, it just returns a value x between -2 to 2 such that 
  // there is a one to one mapping between true angle and x, with x(0)=0, x(pi/2)=1, x(pi-eps)=2, 
  // x(-pi/2)=-1, and x(-pi+eps)=-2
  // This works for Graham's algorithm since the true angle isn't important, only the ordering of angles is.
  
  vector2f delta = loc2-loc1;
  
  double d = delta.length();
  if(fabs(delta.x)<DBL_MIN){
    if(delta.y>0.0)
      return 1.0;
    else
      return -1.0;
  }else if(delta.x>0.0){
    return delta.y/d;
  }else{ //delta.x<0.0
    if(delta.y>0.0)
      return 1.0 - delta.x/d;
    else
      return -1.0 + delta.x/d;
  }
}

void GrahamsScan::addPoint(point& Point)
{
  static const bool debug = false;
  point *tempPoint,*tempPointA,*tempPointB, *curPoint;
  
  //ALLOCATE A NEW POINT STRUCTURE AND INITIALIZE INTERNAL VARIABLES
  tempPoint = new point;
  tempPoint->loc = Point.loc;
  tempPoint->angle=Point.angle;  
  tempPoint->next=NULL;
  tempPoint->prev=NULL;
  
  //TEST IF LIST IS EMPTY
  if (firstPoint==NULL){
    firstPoint=tempPoint;
    return;
  }
  
  //TEST IF ONLY ONE NODE IN LIST AND CURRENT NODE HAS GREATER ANGLE
  if (firstPoint->next==NULL && tempPoint->angle >= firstPoint->angle){
    firstPoint->next=tempPoint;
    tempPoint->prev=firstPoint;
    return;
  }
  
  curPoint=firstPoint;
  
  //CONTINUE THROUGH LIST UNTIL A NODE IS FOUND WITH A GREATER ANGLE THAN CURRENT NODE
  while ((tempPoint->angle > curPoint->angle || (tempPoint->angle + Epsilon > curPoint->angle && tempPoint->loc.y > curPoint->loc.y) )&& curPoint->next!=NULL)
    curPoint=curPoint->next;
  
  //Check if the point is a duplicate
  if(curPoint->prev != NULL){
    if(curPoint->prev->loc==tempPoint->loc)
      return;
  }
  
  //TEST IF NODE IS FIRSTPOINT.  IF SO, ADD AT FRONT OF LIST.
  if (curPoint==firstPoint){
    if(debug) printf("Adding %f,%f at the front\n",V2COMP(tempPoint->loc));
    firstPoint->prev=tempPoint;
    tempPoint->next=firstPoint;
    firstPoint=tempPoint;
    return;
  }else if(curPoint->next==NULL && tempPoint->angle >= curPoint->angle){
    //TEST IF WHILE LOOP REACHED FINAL NODE IN LIST.  IF SO, ADD AT END OF THE LIST.
    if(debug) printf("Adding %f,%f at the end\n",V2COMP(tempPoint->loc));
    curPoint->next=tempPoint;
    tempPoint->prev=curPoint;
    return;
  }else{ //OTHERWISE, INTERMEDIATE NODE HAS BEEN FOUND.  INSERT INTO LIST.  
    tempPointA=curPoint->prev;
    tempPointB=curPoint->prev->next;
    if(debug) printf("Adding %f,%f after %f,%f\n",V2COMP(tempPoint->loc),V2COMP(tempPointA->loc));
    tempPoint->next=tempPointB;
    tempPoint->prev=tempPointA;
    tempPoint->prev->next=tempPoint;
    tempPoint->next->prev=tempPoint;
  }
  
  return;   
}


void GrahamsScan::deleteAllPoints()
{
  point *curPtr = firstPoint;
  point *nextPtr = curPtr->next;
  
  while(curPtr != NULL){
    nextPtr = curPtr->next;
    delete curPtr;
    curPtr = nextPtr;
    if(curPtr==firstPoint)
      break;
  }
  firstPoint = NULL;
  mzero<GrahamsScan>(*this);
}