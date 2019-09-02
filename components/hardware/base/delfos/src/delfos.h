#ifndef DELFOS_H
#define DELFOS_H

#include <QMutex>

#include <iostream>
#include <stdio.h>
#include <string.h>

#include "RoboteqDevice.h"
#include "ErrorCodes.h"
#include "Constants.h"
#include <unistd.h>
#include <math.h>

#define deg2rad 0.01745
#define rad2deg 57.32

using namespace std;



class Delfos
{
public:
    Delfos();
    ~Delfos();
    void setVelocity(float x, float z, float angle);
    void setVelocity(float V1, float V2, float V3, float V4);

private:
    QMutex *mutex;
    int status;
    RoboteqDevice device;
    int PrevVD, VD, PrevVTheta, VTheta, V1, V2, V3, V4;
    float ThetaD45, AUX_VD, AUX_Th,FR,LR,ThetaD, PrevThetaD;

};

#endif // DELFOS_H
