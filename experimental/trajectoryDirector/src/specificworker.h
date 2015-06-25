/*
 *    Copyright (C) 2015 by YOUR NAME HERE
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

/**
       \brief
       @author authorname
*/


#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>

class SpecificWorker : public GenericWorker
{
	Q_OBJECT
	public:
		SpecificWorker(MapPrx& mprx);	
		~SpecificWorker();
		bool setParams(RoboCompCommonBehavior::ParameterList params);
		void newAprilTag(const tagsList &tags);

	public slots:
		void compute(); 	

	private:
		struct Tag
		{
			QElapsedTimer tagClock;
			QVec coords;
			int id;

			//Constructor por defecto.
			Tag(){}	 
			//Constructor parametrizado.
			Tag(int id_, float x, float y, float z)
			{ 
			tagClock.start();
			coords = QVec::vec3(x,y,z);
			id = id_;
			}
			//Retorna el id.
			int getId() {return id;}	  
			//Retorna las coordenadas.
			QVec getCoords() {return coords;}
			//Retorna la coordenada x.
			float x() {return coords.x();}	  
			//Retorna la coordenada y.
			float y() {return coords.y();}
			//Retorna la coordenada z.
			float z() {return coords.z();}
			//Retorna true si su Time To Live es mayor al tiempo de su reloj, false en caso contrario.
			bool isValid(int TTL) {return (tagClock.elapsed() < TTL);}
		};
		
		struct TagList
		{
			QVector<Tag> list;
			QMutex mutex;

			//Añade un Tag al final de la lista y borra su copia antigua. Precondición: list no puede tener 2 tags con el mismo id.
			void addTag( const Tag &tag)
			{
				int i=0;
				QMutexLocker ml(&mutex);
				for(auto t : list)
				{
					if(tag.id == t.id) {list.remove(i);}
					else {i++;}
				}
				list.append(tag);
			}
			//Comprueba si el Tag está en la lista.
			bool checkTag(const Tag &tag)
			{
				QMutexLocker ml(&mutex);
				for(auto t : list)
				{
					if(tag.id == t.id) return true;
				}
				return false;
			}
			//Busca un Tag por su id y lo elimina de la lista. Retorna true si lo encuentra, false en caso contrario.
			bool getTagR(const int &id, Tag &tag)
			{
				int i=0;
				QMutexLocker ml(&mutex);
				for(auto t : list)
				{
					if(t.id == id)
					{
						list.remove(i);
						tag = t;
						return true;
					}
					i++;
				}
				return false;
			}
			bool getTag(const int &id, Tag &tag)
			{
				QMutexLocker ml(&mutex);
				for(auto t : list)
				{
					if(t.id == id)
					{
						tag = t;
						return true;
					}
				}
				return false;
			}

			//Devuelve el número de Tags de la lista.
			int size() {return list.size();}

		};

		TagList tagList;
		InnerModel *innerModel;
		QElapsedTimer clock;
		enum class State {GOTO , LOST, SEARCHING, STOP}; 
		State state;
		QMutex mutex_state;
		TargetPose target;

		void updateState(State st, QMutex& mutex_state);
		void stop();
		bool updateInnerModel(InnerModel* innerModel);
};
#endif