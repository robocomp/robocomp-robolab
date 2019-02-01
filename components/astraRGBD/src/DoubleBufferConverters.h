//
// Created by robolab on 4/09/18.
//

#ifndef PROJECT_DOUBLEBUFFERCONVERTERS_H
#define PROJECT_DOUBLEBUFFERCONVERTERS_H

#include <qdebug.h>


//TODO: This implementantions should go it's own file
class ByteSeqConverter : public Converter<astra::ColorFrame, RoboCompRGBD::imgType>
{
public:
    bool ItoO(const astra::ColorFrame &iTypeData, RoboCompRGBD::imgType &oTypeData)
    {
        if (iTypeData.is_valid())
        {

            //            this->resize(d.width() * d.height()*data_size);
            memcpy(&oTypeData[0], iTypeData.data(), iTypeData.width()*iTypeData.height()*3);
            //            std::copy(std::begin(d.data()), std::end(d.data()), std::begin(writeBuffer));
            return true;
        }
        return false;
    }

    bool OtoI(const RoboCompRGBD::imgType &oTypeData, astra::ColorFrame &iTypeData)
    {
        return false;
    }
};

class PointSeqConverter : public Converter<astra::PointFrame, RoboCompRGBD::PointSeq>
{
public:
    bool ItoO(const astra::PointFrame &iTypeData, RoboCompRGBD::PointSeq &oTypeData)
    {
        if (iTypeData.is_valid())
        {

            //            this->resize(d.width() * d.height()*data_size);
//            std::cout<<"iTypeData length "<<iTypeData.length()<<endl;
//            memcpy(&oTypeData[0], iTypeData.data(), iTypeData.length());
            int non0 = 0;
			for(uint i = 0; i< iTypeData.length(); i++)
			{
				oTypeData[i].x = iTypeData.data()[i].x;
				oTypeData[i].y = iTypeData.data()[i].y;
				oTypeData[i].z = iTypeData.data()[i].z;
				if(iTypeData.data()[i].x != 0 or iTypeData.data()[i].y != 0 or iTypeData.data()[i].z != 0)
				{
					non0++;
				}
				std::cout<<non0<<" non 0 values in Point Stream";
			}
            //            std::copy(std::begin(d.data()), std::end(d.data()), std::begin(writeBuffer));
            return true;
        }
        return false;
    }

    bool OtoI(const RoboCompRGBD::PointSeq &oTypeData, astra::PointFrame &iTypeData)
    {
        return false;
    }
};


class PointStreamConverter : public Converter<astra::PointFrame, RoboCompRGBD::imgType>
{
public:
    bool ItoO(const astra::PointFrame &iTypeData, RoboCompRGBD::imgType &oTypeData)
    {
        if (iTypeData.is_valid())
        {

            //            this->resize(d.width() * d.height()*data_size);
//            std::cout<<"iTypeData length "<<iTypeData.length()<<endl;
            memcpy(&oTypeData[0], iTypeData.data(), iTypeData.length()*3*4);
            auto point = iTypeData.data()[10];
//			std::cout<<"Point ("<<point.x<<", "<<point.y<<", "<<point.z<<")"<<std::endl;
//            int non0 = 0;
//			for(uint i = 0; i< iTypeData.length(); i++)
//			{
//				oTypeData[i].x = iTypeData.data()[i].x;
//				oTypeData[i].y = iTypeData.data()[i].y;
//				oTypeData[i].z = iTypeData.data()[i].z;
////				if(iTypeData.data()[i].x != 0 or iTypeData.data()[i].y != 0 or iTypeData.data()[i].z != 0)
////				{
////					non0++;
////				}
////				std::cout<<non0<<" non 0 values in Point Stream"<<endl;
//			}
            //            std::copy(std::begin(d.data()), std::end(d.data()), std::begin(writeBuffer));
            return true;
        }
        return false;
    }

    bool OtoI(const RoboCompRGBD::imgType &oTypeData, astra::PointFrame &iTypeData)
    {
        return false;
    }
};

class ColorSeqConverter : public Converter<astra::ColorFrame, RoboCompRGBD::ColorSeq>
{
public:
    bool ItoO(const astra::ColorFrame &iTypeData, RoboCompRGBD::ColorSeq &oTypeData)
    {
        if (iTypeData.is_valid())
        {

            //            this->resize(d.width() * d.height()*data_size);
            memcpy(&oTypeData[0], iTypeData.data(), iTypeData.width()*iTypeData.height()*3);
            //            std::copy(std::begin(d.data()), std::end(d.data()), std::begin(writeBuffer));
            return true;
        }
        return false;
    }

    bool OtoI(const RoboCompRGBD::ColorSeq &oTypeData, astra::ColorFrame &iTypeData)
    {
        return false;
    }
};
//class ByteSeqConverter : public Converter<astra::ColorFrame, RoboCompRGBD::imgType>
//{
//public:
//    bool ItoO(const astra::ColorFrame &iTypeData, RoboCompRGBD::imgType &oTypeData)
//    {
//        if (iTypeData.is_valid())
//        {
//
//            //            this->resize(d.width() * d.height()*data_size);
//            memcpy(&oTypeData[0], iTypeData.data(), iTypeData.width()*iTypeData.height()*3);
//            //            std::copy(std::begin(d.data()), std::end(d.data()), std::begin(writeBuffer));
//            return true;
//        }
//        return false;
//    }
//
//    bool OtoI(const RoboCompRGBD::imgType &oTypeData, astra::ColorFrame &iTypeData)
//    {
//        return false;
//    }
//};

class FloatSeqConverter : public Converter<astra::DepthFrame, RoboCompRGBD::DepthSeq>
{
public:
    bool ItoO(const astra::DepthFrame &iTypeData, RoboCompRGBD::DepthSeq &oTypeData)
    {
        if (iTypeData.is_valid())
        {
            std::copy(&iTypeData.data()[0], &iTypeData.data()[0]+(iTypeData.width()*iTypeData.height()), std::begin(oTypeData));
            return true;
        }
        return false;
    }

    bool OtoI(const RoboCompRGBD::DepthSeq &oTypeData, astra::DepthFrame &iTypeData)
    {
        return false;
    }
};


#endif //PROJECT_DOUBLEBUFFERCONVERTERS_H
