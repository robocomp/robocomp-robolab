#include <delfos.h>

#include <QMutexLocker>

Delfos::Delfos()
{
    mutex = new QMutex(QMutex::Recursive);
    string response = "";
    status = device.Connect("/dev/ttyUSB0");

    if (status != RQ_SUCCESS)
    {
        cout<<"Error connecting to device: "<<status<<"."<<endl;
        throw 1;
    }
}


Delfos::~Delfos()
{
    device.Disconnect();
}


void Delfos::setVelocity(float LR, float FR, float VTheta)
{
	float wheelPermimeter = 2.*M_PI_2*76.2; // 76.2 it's the wheels' radius
	
	LR /= wheelPermimeter;
	FR /= wheelPermimeter;

	QMutexLocker l(mutex);

	AUX_VD = sqrt(LR * LR + FR * FR);// ' sqrt returns result * 1000
	if (FR > 0.0001)
	{
		AUX_Th = atan(((LR)/(FR))); //' atan takes input * 1000 and returns angle in degrees * 10
		if (FR<0) AUX_Th += 3.1416;
		if ((FR>0) && (LR<0)) AUX_Th += 6.28;
	}
	else
	{
		if (LR >0)
			AUX_Th = 3.1416/2.;
		if (LR <0)
			AUX_Th = 3.1416 + (3.1416/2.);
	}

	VD = AUX_VD;
	ThetaD = AUX_Th;


	ThetaD45  = (ThetaD + (3.1416/4));//' compute once angle + 45 for use in the 4 equations
	V1 = (VD * sin(ThetaD45)) + VTheta;  //' sin takes degrees and returns result * 1000
	V2 = (VD * cos(ThetaD45)) - VTheta;
	V3 = (VD * cos(ThetaD45)) + VTheta;
	V4 = (VD * sin(ThetaD45)) - VTheta;
	printf("VD: %d,ThetaD: %f,VTheta: %f, ThetaD45: %f, %d, %d, %d, %d \n", VD,ThetaD,VTheta,ThetaD45,V1,V2,V3,V4);


	PrevVD = VD;
	PrevThetaD = ThetaD;
	PrevVTheta = VTheta;

	device.SetCommand(false, _GO, 1, V1);
	device.SetCommand(false, _GO, 2, -V2);
	device.SetCommand(true, _GO, 1, V3);
	device.SetCommand(true, _GO, 2, -V4);
}


void Delfos::setVelocity(float V1, float V2, float V3, float V4)
{
	device.SetCommand(false, _GO, 1, V1);
	device.SetCommand(false, _GO, 2, V2);
	device.SetCommand(true,  _GO, 1, V3);
	device.SetCommand(true,  _GO, 2, V4);
}





