#include <vector>
#include <math.h>
#include "geometry.h"
#include <sstream>
#include "util.h"
#include "timer.h"

#ifndef GRAHAMS_SCAN_H
#define GRAHAMS_SCAN_H

using namespace std;

class GrahamsScan{
  
public:
  struct point
  {
    vector2f loc;
    double angle;
    point *next; //POINTER TO NEXT NODE IN THE LIST
    point *prev; //POINTER TO PREVIOUS NODE IN THE LIST
  };
  
private:
  int NumPoints;
  point* firstPoint; //POINTER TO MIN POINT IN DOUBLELY LINKED LIST
  
public:
  GrahamsScan();  
  ~GrahamsScan(); 
  vector<vector2f> run(vector< vector2f > points); 
  void grahamScan(point* P);//ACTUAL GRAHAM'S SCAN PROCEDURE
  
private:
  bool isConvexPoint(point *P); //TEST POINT FOR CONVEXITY
  void addPoint(GrahamsScan::point &Point); //ADDS POINT TO DOUBLELY LINKED LIST (USED DURING SORTING)
  double findAngle(vector2f &loc1, vector2f &loc2); //FIND ANGLE GIVEN TWO POINTS
  void deleteAllPoints();
};

#endif //GRAHAMS_SCAN_H