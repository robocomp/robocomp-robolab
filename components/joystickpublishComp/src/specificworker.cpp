/*
 *    Copyright (C) 2006-2010 by RoboLab - University of Extremadura
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
 
 #include "specificworker.h"

/**
* \brief Default constructor
*/

SpecificWorker::SpecificWorker(MapPrx& mprx, QObject *parent) : GenericWorker(mprx, parent)	
{
	sendEvent = false;
	jtimer = new QTimer( );
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{

}
void SpecificWorker::compute( )
{
	
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	active = false;

	qDebug() <<"joystickUniversalComp::Worker::setParams(): "+QString::fromStdString(params["joystickUniversal.Device"].value)+" - "+QString::fromStdString(joystickParams.device);

	if( params["joystickUniversal.Device"].value != joystickParams.device)
	{
		joystickParams.device = params["joystickUniversal.Device"].value;
		joystickParams.numAxes = QString::fromStdString(params["joystickUniversal.NumAxes"].value).toInt();
		joystickParams.numButtons = QString::fromStdString(params["joystickUniversal.NumButtons"].value).toInt();
		joystickParams.basicPeriod = QString::fromStdString(params["joystickUniversal.BasicPeriod"].value).toInt();
	
		for (int i=0; i<joystickParams.numAxes; i++)
		{
			std::string s= QString::number(i).toStdString();
			RoboCompJoystickAdapter::AxisParams apar;
			apar.name = params["joystickUniversal.Axis_" + s +".Name"].value;
			if(params["joystickUniversal.VelocityAxis"].value == apar.name)
			{
				data.velAxisIndex = i;
				qDebug() << "Setting vel index to:"+QString::number(data.velAxisIndex);
			}
			if(params["joystickUniversal.DirectionAxis"].value == apar.name)
			{
				data.dirAxisIndex = i;
					qDebug() << "Setting dir index to:"+QString::number(data.dirAxisIndex);
			}			
			apar.value = 0;
			data.axes.push_back(apar);
				
			axesParams aux;
			aux.name = apar.name;
			aux.minRange = QString::fromStdString(params["joystickUniversal.Axis_" + s +".MinRange"].value).toInt();
			aux.maxRange = QString::fromStdString(params["joystickUniversal.Axis_" + s +".MaxRange"].value).toInt();
			aux.inverted = QString::fromStdString(params["joystickUniversal.Axis_" + s +".Inverted"].value).contains("true");
			qDebug()<<"axes"<<QString::fromStdString(aux.name)<<aux.minRange<<aux.maxRange<<aux.inverted;
			joystickParams.axes[i] = aux;
		}
		for (int i=0; i<joystickParams.numButtons; i++)
		{
			RoboCompJoystickAdapter::ButtonParams bpar;
			bpar.clicked = false;
			data.buttons.push_back(bpar);
		}
			//TODO: INITIALIZE JOYSTICK
			
			//~ // Set the base joystick axes initial data
			//~ joy_axes.actualX = 0.;
			//~ joy_axes.actualY = 0.;

		joystick = new QJoyStick( QString::fromStdString(joystickParams.device) );
		if ( joystick->openQJoy() )
		{
			joystick->start();
			qDebug() << "JOYSTICK STARTED";
			if (joystickParams.basicPeriod < 1)
				joystickParams.basicPeriod = 1;
			timer.start(Period);
		}
		else
		{
			qDebug() << "FAILED TO STAR JOYSTICK";
		}

		// Connect signals
		connect( joystick, SIGNAL( inputEvent(int, int, int) ), this, SLOT( receivedJoystickEvent(int, int, int) ) );
		connect( &timer, SIGNAL( timeout() ), this, SLOT( sendJoystickEvent() ) );
		//~ qWarning("[joystickUniversalComp]: New Joystick Handler settings: XMotionAxis [%2d], YMotionAxis [%2d]", config.XMotionAxis, config.YMotionAxis);
		//~ qWarning("[joystickUniversalComp]: Max advance speed: [%i], Max steering speed: [%f]",config.maxAdv,config.maxRot);
	}
	else
	{
		qDebug("Device has not change. No reconfiguration needed.");
	}
	active = true;
	
	return true;
};

void SpecificWorker::sendJoystickEvent()
{
	try
	{
		if(sendEvent)
		{
			rDebug("trying to send event. NumAxes="+QString::number(joystickParams.numAxes));
			for(int x=0; x < joystickParams.numAxes; x++)
			{
					qDebug() << "axes "+QString::fromStdString(data.axes[x].name)+" = "+QString::number(data.axes[x].value);
					//~ sendEvent &= (fabs(joystickParams.data.axes[x].value)<0.1);
			}
			if(sendEvent)
				sendEvent = false;
			joystickadapter->sendData(data);
		}
	}
	catch(const Ice::Exception& ex)
	{
		cout << "[joystickUniversalComp ]: Fallo la comunicacion a traves del proxy (base). Waiting" << endl;
		cout << "[joystickUniversalComp]: Motivo: " << endl << ex << endl;
	}
}


void SpecificWorker::receivedJoystickEvent(int value, int type, int number)
{
	switch( type)
	{
		case  JOYSTICK_EVENT_TYPE_AXIS:
		{
			if(number >= joystickParams.numAxes)
			{
				qDebug() << "ERROR: Event received for not configured axes ( received "+QString::number(number)+", configured max index "+QString::number(joystickParams.numAxes-1)+")";
				break;
			}

			//~ rDebug(QString::number(fabs(normalized_value - joystickParams.data.axes[number].value) > JOYSTICK_PRECISION ));
			//~ rDebug("New value: "+QString::number(normalized_value)+", Old value: "+QString::number(joystickParams.data.axes[number].value)+", Diff: "+QString::number(fabs(normalized_value - joystickParams.data.axes[number].value)));
			float normalized_value;
			if(fabs(value) < JOYSTICK_CENTER)
			{
				normalized_value = normalize(0, joystickParams.axes[number].minRange, joystickParams.axes[number].maxRange, -1.f , 1.f );
				qDebug() << "Invert: "+QString::number(joystickParams.axes[number].inverted);
				data.axes[number].value = normalized_value;
				sendEvent = true;
				break;
			}
			else
			{
				if(joystickParams.axes[number].inverted)
				{
					value *= -1;
				}
				
				normalized_value = normalize(value, joystickParams.axes[number].minRange, joystickParams.axes[number].maxRange, -1.f , 1.f );
				
				if( fabs(normalized_value - data.axes[number].value) > JOYSTICK_PRECISION )
				{
					//~ rDebug(QString::number(fabs(normalized_value - joystickParams.data.axes[number].value) > JOYSTICK_PRECISION ));
					data.axes[number].value = normalized_value;
					sendEvent = true;
				}
			}
		}
		break;
		case JOYSTICK_EVENT_TYPE_BUTTON:
		{
			//rDebug("Received Button Event: "+QString::number(value)+", "+QString::number(type)+", "+QString::number(number));
			if(number >= joystickParams.numButtons)
			{
				qDebug() << "ERROR: Event received for not configured button (received "+QString::number(number)+", configured max index "+QString::number(joystickParams.numButtons-1)+")";
				break;
			}
			data.buttons[number].clicked = (bool) value;
			qDebug() << "Button "+QString::number(number)+": "+QString::number(data.buttons[number].clicked);
			sendEvent = true;
		}
		break;
		default:
			qDebug() << "Unknown joystick Event: "+QString::number(value)+", "+QString::number(type)+", "+QString::number(number);
	}
}

// float SpecificWorker::normalize(float X, float A, float B, float C, float D)
// {
// 	return ((D-C)*(X-A)/(B-A))+C;
// }

float SpecificWorker::normalize(float X, float A, float B, float C, float D)
{
	QList<QPair<QPointF,QPointF> > intervals;
	intervals.append(QPair<QPointF,QPointF>(QPointF(A,B),QPointF(C,D))); 
	//qDebug() << __FUNCTION__ << intervals << "X" << X;

	QMat m = QMat::afinTransformFromIntervals( intervals );
	return (m * QVec::vec2(X,1))[0];
	
}
