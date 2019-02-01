
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
			std::cout<<"Point ("<<point.x<<", "<<point.y<<", "<<point.z<<")"<<std::endl;
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

    }
};


#endif //PROJECT_DOUBLEBUFFERCONVERTERS_H
