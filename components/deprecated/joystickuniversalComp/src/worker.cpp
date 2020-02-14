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
#include "worker.h"
#include "const.h"


/**
* \brief Default constructor
*/
Worker::Worker(RoboCompJoystickAdapter::JoystickAdapterPrx joystickadapterprx, QObject *parent): QThread(parent)
{
	this->joyAdapterPrx = joystickadapterprx;

	monitor_mutex = new QMutex(QMutex::Recursive);
	//setupUi(this);
	//show();
	sendEvent = false;
	Period = BASIC_PERIOD;
	jtimer = new QTimer( );
}

/**
* \brief Default destructor
*/
Worker::~Worker()
{
	jtimer->stop();
	joystick->stop();
	
	disconnect ( joystick );
	disconnect ( jtimer );
	
	delete jtimer;
	delete joystick;
}

void Worker::run()
{
	int cont = 0;
	forever
	{
		if (active)
		{
			cont++;
			if(cont > 40)
			{
				rDebug("running");
				cont = 0;
			}
			//~ this->sendJoystickEvent();
			this->usleep(Period*1000);
		}
	}
}

void Worker::killYourSelf()
{
	rDebug("Killing myself");
	
	emit kill();
	exit(1);
}
/**
* \brief Change compute period
* @param per Period in ms
*/
void Worker::setPeriod(int p)
{
	rDebug("Period changed"+QString::number(p));
	Period = p;
}

void Worker::setParams(RoboCompCommonBehavior::ParameterList _params)
{
	QMutexLocker lock(monitor_mutex);
	active = false;

	rDebug("joystickUniversalComp::Worker::setParams(): "+QString::fromStdString(_params["joystickUniversal.Device"].value)+" - "+QString::fromStdString(joystickParams.device));

	if(_params["joystickUniversal.Device"].value != joystickParams.device)
	{
		
		//TODO: STOP JOYSTICK
		//~ if(handler!= NULL) 
			//~ delete handler;
		joystickParams.device = _params["joystickUniversal.Device"].value;
		joystickParams.numAxes = QString::fromStdString(_params["joystickUniversal.NumAxes"].value).toInt();
		joystickParams.numButtons = QString::fromStdString(_params["joystickUniversal.NumButtons"].value).toInt();
		joystickParams.basicPeriod = QString::fromStdString(_params["joystickUniversal.BasicPeriod"].value).toInt();

		
		for (int i=0; i<joystickParams.numAxes; i++)
		{
			std::string s= QString::number(i).toStdString();
			RoboCompJoystickAdapter::AxisParams apar;
			apar.name = _params["joystickUniversal.Axis_" + s +".Name"].value;
			rDebug("Axis = "+QString::number(i)+" name "+QString::fromStdString(apar.name)+" vs "+QString::fromStdString(_params["joystickUniversal.VelocityAxis"].value));
			if(_params["joystickUniversal.VelocityAxis"].value == apar.name)
			{
				data.velAxisIndex = i;
				rDebug("Setting vel index to:"+QString::number(data.velAxisIndex));
			}
			if(_params["joystickUniversal.DirectionAxis"].value == apar.name)
			{
				data.dirAxisIndex = i;
				rDebug("Setting dir index to:"+QString::number(data.dirAxisIndex));
			}			
			apar.value = 0;
			data.axes.push_back(apar);
			
			axesParams aux;
			aux.name = apar.name;
			aux.minRange = QString::fromStdString(_params["joystickUniversal.Axis_" + s +".MinRange"].value).toInt();
			aux.maxRange = QString::fromStdString(_params["joystickUniversal.Axis_" + s +".MaxRange"].value).toInt();
			aux.inverted = QString::fromStdString(_params["joystickUniversal.Axis_" + s +".Inverted"].value).contains("true");
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
			rDebug("JOYSTICK STARTED");
			if (joystickParams.basicPeriod < 1)
				joystickParams.basicPeriod = 1;
			jtimer->start( 1000 /  joystickParams.basicPeriod );
		}
		else
		{
			rDebug("FAILED TO STAR JOYSTICK");
		}

		// Connect signals
		connect( joystick, SIGNAL( inputEvent(int, int, int) ), this, SLOT( receivedJoystickEvent(int, int, int) ) );
		connect( jtimer, SIGNAL( timeout() ), this, SLOT( sendJoystickEvent() ) );
		//~ qWarning("[joystickUniversalComp]: New Joystick Handler settings: XMotionAxis [%2d], YMotionAxis [%2d]", config.XMotionAxis, config.YMotionAxis);
		//~ qWarning("[joystickUniversalComp]: Max advance speed: [%i], Max steering speed: [%f]",config.maxAdv,config.maxRot);
		this->start();
	}
	else
	{
		rDebug("Device has not change. No reconfiguration needed.");
	}
	active = true;
}

void Worker::sendJoystickEvent()
{
	try
	{
		if(sendEvent)
		{
			rDebug("trying to send event. NumAxes="+QString::number(joystickParams.numAxes));
			for(int x=0; x < joystickParams.numAxes; x++)
			{
					rDebug("axes "+QString::fromStdString(data.axes[x].name)+" = "+QString::number(data.axes[x].value));
					//~ sendEvent &= (fabs(joystickParams.data.axes[x].value)<0.1);
			}
			if(sendEvent)
				sendEvent = false;
			joyAdapterPrx->sendData(data);
		}
	}
	catch(const Ice::Exception& ex)
	{
		cout << "[joystickUniversalComp ]: Fallo la comunicacion a traves del proxy (base). Waiting" << endl;
		cout << "[joystickUniversalComp]: Motivo: " << endl << ex << endl;
	}
}


void Worker::receivedJoystickEvent(int value, int type, int number)
{
	switch( type)
	{
		case  JOYSTICK_EVENT_TYPE_AXIS:
		{
			if(number >= joystickParams.numAxes)
			{
				rDebug("ERROR: Event received for not configured axes ( received "+QString::number(number)+", configured max index "+QString::number(joystickParams.numAxes-1)+")");
				break;
			}

			//~ rDebug(QString::number(fabs(normalized_value - joystickParams.data.axes[number].value) > JOYSTICK_PRECISION ));
			//~ rDebug("New value: "+QString::number(normalized_value)+", Old value: "+QString::number(joystickParams.data.axes[number].value)+", Diff: "+QString::number(fabs(normalized_value - joystickParams.data.axes[number].value)));
			float normalized_value;
			if(fabs(value) < JOYSTICK_CENTER)
			{
				normalized_value = normalize(0, joystickParams.axes[number].minRange, joystickParams.axes[number].maxRange, -1.f , 1.f );
				rDebug("Invert: "+QString::number(joystickParams.axes[number].inverted));
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
				rDebug("ERROR: Event received for not configured button (received "+QString::number(number)+", configured max index "+QString::number(joystickParams.numButtons-1)+")");
				break;
			}
			data.buttons[number].clicked = (bool) value;
			rDebug("Button "+QString::number(number)+": "+QString::number(data.buttons[number].clicked));
			sendEvent = true;
		}
		break;
		default:
			rDebug("Unknown joystick Event: "+QString::number(value)+", "+QString::number(type)+", "+QString::number(number));
	}
}

float Worker::normalize(float X, float A, float B, float C, float D)
{
	return ((D-C)*(X-A)/(B-A))+C;
}

  //~ struct AxisParams    //Configuration Params  
  //~ {
    //~ string name;         // name of motor
    //~ int value;
    //~ int negRange;
    //~ int posRange;
  //~ };
  //~ 
  //~ struct ButtonParams
  //~ {
  //~ 
	//~ bool clicked;
	//~ 
  //~ };
  //~ 
  //~ 
  //~ sequence<AxisParams> AxisList;
  //~ sequence<ButtonParams> ButtonsList;
  //~ struct JoystickData
  //~ {
    //~ AxisList axes;
	//~ ButtonsList buttons;
    //~ int velAxisIndex;
    //~ int dirAxisIndex;
  //~ };
   //~ 
  //~ struct JoystickParams
  //~ {
    //~ string device;
    //~ int numAxes;
    //~ int numButtons;
    //~ int basicPeriod; //milliseconds
    //~ JoystickData data;
    //~ int normalizationValue;
  //~ };

