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
SpecificWorker::SpecificWorker(MapPrx& mprx, bool startup_check) : GenericWorker(mprx)
{
	this->startup_check_flag = startup_check;
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{ }

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
	}
	else
	{
		qDebug("Device has not change. No reconfiguration needed.");
	}
	for (int i=0; i<joystickParams.numAxes; i++)
	{
		std::string s= QString::number(i).toStdString();
		RoboCompJoystickAdapter::AxisParams apar;
        apar.name = params["joystickUniversal.Axis_" + s +".Name"].value;
		apar.value = 0;
		data.axes.push_back(apar);
			
		axesParams aux;
		aux.name = apar.name;
        aux.axis = QString::fromStdString(params["joystickUniversal.Axis_" + s +".Axis"].value).toInt();
		aux.minRange = QString::fromStdString(params["joystickUniversal.Axis_" + s +".MinRange"].value).toInt();
		aux.maxRange = QString::fromStdString(params["joystickUniversal.Axis_" + s +".MaxRange"].value).toInt();
		aux.inverted = QString::fromStdString(params["joystickUniversal.Axis_" + s +".Inverted"].value).contains("true");
		aux.dead_zone = QString::fromStdString(params["joystickUniversal.Axis_" + s +".DeadZone"].value).toFloat();
		qDebug() << __FUNCTION__ << "axes" << QString::fromStdString(aux.name) << aux.minRange << aux.maxRange << aux.inverted;
		joystickParams.axes.push_back(aux);
	}
    for (int i=0; i<joystickParams.numButtons; i++)
    {
        std::string s= QString::number(i).toStdString();
        RoboCompJoystickAdapter::ButtonParams apar;
        apar.name = params["joystickUniversal.Button_" + s +".Name"].value;
        apar.step = 0;
        data.buttons.push_back(apar);

        buttonsParams aux;
        aux.name = apar.name;
        aux.number = QString::fromStdString(params["joystickUniversal.Button_" + s +".Number"].value).toInt();
        aux.step = QString::fromStdString(params["joystickUniversal.Button_" + s +".Step"].value).toInt();
        qDebug() << __FUNCTION__ << "buttons" << QString::fromStdString(aux.name) << aux.number << aux.step;
        joystickParams.buttons.push_back(aux);
    }
	active = true;
	return true;
};


void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;
	this->Period = period;
	if(this->startup_check_flag)
	{
		this->startup_check();
	}
	else
	{
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
			qDebug() << "FAILED TO START JOYSTICK";
		}

		// Connect signals
		connect( joystick, SIGNAL( inputEvent(int, int, int) ), this, SLOT( receivedJoystickEvent(int, int, int) ) );
		connect( &timer, SIGNAL( timeout() ), this, SLOT( sendJoystickEvent() ) );
		
		//~ qWarning("[joystickUniversalComp]: New Joystick Handler settings: XMotionAxis [%2d], YMotionAxis [%2d]", config.XMotionAxis, config.YMotionAxis);
		//~ qWarning("[joystickUniversalComp]: Max advance speed: [%i], Max steering speed: [%f]",config.maxAdv,config.maxRot);

		this->Period = 100;
		timer.start(Period);
	}
}

void SpecificWorker::compute( )
{

}

////////////////////////////////////////////////////////////////////////777

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
			joystickadapter_pubproxy->sendData(data);
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
    //for(auto &ax : data.axes)
    //    ax.value = 0;
    //for(auto &bu : data.buttons)
    //    bu.step = 0;

    switch( type)
	{
		case  JOYSTICK_EVENT_TYPE_AXIS:
		{
		    if(auto r=std::find_if(joystickParams.axes.begin(), joystickParams.axes.end(), [number](axesParams &a)
		                    { return (a.axis == number);}) ; r==joystickParams.axes.end())
			{
				qDebug() << "ERROR: Event received for not configured axis:" << QString::number(number);
				break;
			}
		    else
            {
                auto axis = *r;
                float normalized_value;
                if (fabs(value) > JOYSTICK_CENTER)
                {
                    normalized_value = normalize(value, -32000, 32000, axis.minRange, axis.maxRange, axis.dead_zone);
                    if (axis.inverted) normalized_value *= -1;
                    if(auto dr=std::find_if(data.axes.begin(), data.axes.end(),[axis](auto &a){ return a.name == axis.name;}); dr!=data.axes.end())
                    {
                        dr->value = normalized_value;
                        qDebug() << "Axis:" << number << "Value:" << normalized_value;
                        sendEvent = true;
                    }
                    break;
                }
            }
		}
		case JOYSTICK_EVENT_TYPE_BUTTON:
        {
            if (auto r = std::find_if(joystickParams.buttons.begin(), joystickParams.buttons.end(), [number](auto &a) { return (a.number == number); }); r ==
                                                                                                                                                         joystickParams.buttons.end())
            {
                qDebug() << "ERROR: Event received for not configured button: " << QString::number(number);
                break;
            } else
            {
                auto button = *r;
                if (auto dr = std::find_if(data.buttons.begin(), data.buttons.end(), [button](auto &a) { return a.name == button.name; }); dr !=
                                                                                                                                           data.buttons.end())
                {
                    dr->step = button.step;
                    qDebug() << "Button " + QString::number(number) + ": " << value << "name: " + QString::fromStdString(button.name);
                    sendEvent = true;
                }
            }
            break;
        }
		default:
			qDebug() << "Unknown joystick Event: "+QString::number(value)+", "+QString::number(type)+", "+QString::number(number);
	}
}

// X value, min X value, max X value, min Y value, max y value, dead_zone in Y domain
float SpecificWorker::normalize(float X, float A, float B, float C, float D, float dead_zone)
{
	QList<QPair<QPointF,QPointF> > intervals;
	intervals.append(QPair<QPointF,QPointF>(QPointF(A,B),QPointF(C,D)));
	QMat m = QMat::afinTransformFromIntervals( intervals );
	auto res = (m * QVec::vec2(X,1))[0];
	if(res < dead_zone and res > -dead_zone) res = 0;
	return res;
}

////////////////////////////////////////////////////////777
int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}


