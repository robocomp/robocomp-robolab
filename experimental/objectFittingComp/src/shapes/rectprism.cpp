#include "rectprism.h"

RectPrism::RectPrism()
{
  
  center.push_back(0.0);
  center.push_back(0.0);
  center.push_back(0.0);
  rotation.push_back(0.0);
  rotation.push_back(0.0);
  rotation.push_back(0.0);
  Wx=1.0;
  Wy=1.0;
  Wz=1.0;
  
}

RectPrism::RectPrism(const QVec &center, const QVec &rotation, double Wx, double Wy, double Wz)
{
  this->center=center;
  this->rotation=rotation;
  this->Wx=Wx;
  this->Wy=Wy;
  this->Wz=Wz;
}


void printCode(uint8_t code)
{
  for (uint8_t i=0; i<=7; ++i)
  {
    uint8_t teta = (code<<(7-i));
    printf("%d", teta>>7 );
  }
  printf("\n");
}

double RectPrism::distance_p2p (double x1, double y1, double z1, double x2, double y2, double z2)
{
  //cout<<"distance: "<<x1<<" "<<y1<<" "<<z1<<" "<<x2<<" "<<y2<<" "<<z2<<endl;
  return sqrt(pow(x1-x2,2.0)+pow(y1-y2,2.0)+pow(z1-z2,2.0));
}

double RectPrism::distance(const QVec &point)
{
  QVec point2 = placePoint(point);
  //point.print("point");
  //point2.print("point2");
  
  uint8_t code = collisionVector(point2);
  
  //printCode(code);
  
  switch(code)
  {
    case LAF: return fabs(distance_p2p(point2(0),point2(1),point2(2),-(Wx/2),(Wy/2),-(Wz/2)));
    case LAM: return fabs(sqrt(pow(point2(0)-(-Wx/2),2)+pow(point2(1)-(Wy/2),2)));
    case LAB: return fabs(distance_p2p(point2(0),point2(1),point2(2),-(Wx/2),(Wy/2),(Wz/2)));
    case LMF: return fabs(sqrt(pow(point2(0)-(-Wx/2),2)+pow(point2(2)-(-Wz/2),2)));
    case LMM: return fabs(point2(0)-(-Wx/2));
    case LMB: return fabs(sqrt(pow(point2(0)-(-Wx/2),2)+pow(point2(2)-(Wz/2),2)));
    case LBF: return fabs(distance_p2p(point2(0),point2(1),point2(2),-(Wx/2),-(Wy/2),-(Wz/2)));
    case LBM: return fabs(sqrt(pow(point2(0)-(-Wx/2),2)+pow(point2(1)-(-Wy/2),2)));
    case LBB: return fabs(distance_p2p(point2(0),point2(1),point2(2),-(Wx/2),-(Wy/2),(Wz/2)));
    
    case MAF: return fabs(sqrt(pow(point2(1)-(Wy/2),2)+pow(point2(2)-(-Wz/2),2)));
    case MAM: return fabs(point2(1)-(Wy/2));
    case MAB: return fabs(sqrt(pow(point2(1)-(Wy/2),2)+pow(point2(2)-(Wz/2),2)));
    case MMF: return fabs(point2(2)-(-Wz/2));
    
    
    case MMM: 
      double distances[6];
      double min_distance;
      //-x
      distances[0]=fabs((-Wx/2)-point2(0));
      //y
      distances[1]=fabs((Wy/2)-point2(1));
      //-z
      distances[2]=fabs((-Wz/2)-point2(2));
      //-y
      distances[3]=fabs((-Wy/2)-point2(1));
      //z
      distances[4]=fabs((Wz/2)-point2(2));
      //x
      distances[5]=fabs((Wx/2)-point2(0));
      
      //get the minimum distance
      min_distance=distances[0];
      for(int i=1;i<6;i++)
      {
        if (min_distance>distances[i])
        {
          min_distance=distances[i];
        }
      }
      return min_distance;
    
    
    case MMB: return fabs(point2(2)-(Wz/2));
    case MBF: return fabs(sqrt(pow(point2(1)-(-Wy/2),2)+pow(point2(2)-(-Wz/2),2)));
    case MBM: return fabs(point2(1)-(-Wy/2));
    case MBB: return fabs(sqrt(pow(point2(1)-(-Wy/2),2)+pow(point2(2)-(Wz/2),2)));
    
    case RAF: return fabs(distance_p2p(point2(0),point2(1),point2(2),(Wx/2),(Wy/2),-(Wz/2)));
    case RAM: return fabs(sqrt(pow(point2(0)-(Wx/2),2)+pow(point2(1)-(Wy/2),2)));
    case RAB: return fabs(distance_p2p(point2(0),point2(1),point2(2),(Wx/2),(Wy/2),(Wz/2)));
    case RMF: return fabs(sqrt(pow(point2(0)-(Wx/2),2)+pow(point2(2)-(-Wz/2),2)));
    case RMM: //printf("Point: %f, Wx/2: %f\n", point2(0), Wx/2 );
      return fabs(point2(0)-(Wx/2));
    case RMB: return fabs(sqrt(pow(point2(0)-(Wx/2),2)+pow(point2(2)-(Wz/2),2)));
    case RBF: return fabs(distance_p2p(point2(0),point2(1),point2(2),(Wx/2),-(Wy/2),-(Wz/2)));
    case RBM: return fabs(sqrt(pow(point2(0)-(Wx/2),2)+pow(point2(1)-(-Wy/2),2)));
    case RBB: return fabs(distance_p2p(point2(0),point2(1),point2(2),(Wx/2),-(Wy/2),(Wz/2)));
  }
  
  return -1;
}


QVec RectPrism::placePoint(const QVec &point)
{
  return RTMat(-rotation(0), -rotation(1), -rotation(2), QVec::vec3(-center(0), -center(1), -center(2)))*QVec::vec4(point(0), point(1), point(2), 1); 
}

uint8_t RectPrism::collisionVector(const QVec &point)
{
  uint8_t code = 0;
  
  //check X axis
  if (point(0)>(Wx/2))
    code=code|1<<XP;
  else if (point(0)<(-(Wx/2)))
    code=code|1<<XN;
  
  //check Y axis
  
  if (point(1)>(Wy/2))
    code=code|1<<YP;
  else if(point(1)<(-Wy/2))
    code=code|1<<YN;
  
  //check Z axis
  if (point(2)>(Wz/2))
    code=code|1<<ZP;
  else if(point(2)<((-Wz/2)))
    code=code|1<<ZN;
  
  //printCode(code);
  
  return code;
}

