//
// Created by robolab on 4/09/18.
//

#ifndef PROJECT_DOUBLEBUFFERCONVERTERS_H
#define PROJECT_DOUBLEBUFFERCONVERTERS_H

#include <qdebug.h>


//TODO: This implementantions should go it's own file
class ByteSeqConverter : public Converter<astra::ColorFrame, RoboCompRGBD::imgType>
{
private:
	std::size_t size=0;
public:
	ByteSeqConverter(std::size_t _size)
	{
		size=_size;
	}
	bool clear(RoboCompRGBD::imgType &oTypeData)
	{
		oTypeData.clear();
		return true;
	}
    bool ItoO(const astra::ColorFrame &iTypeData, RoboCompRGBD::imgType &oTypeData)
    {
		if(oTypeData.size()<this->size)
		{
			oTypeData.resize(size);
		}
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
private:
	std::size_t size=0;
public:
	PointSeqConverter(std::size_t _size)
	{
		size=_size;
	}
	bool clear(RoboCompRGBD::PointSeq &oTypeData)
	{
		oTypeData.clear();
		return true;
	}
    bool ItoO(const astra::PointFrame &iTypeData, RoboCompRGBD::PointSeq &oTypeData)
    {
		if(oTypeData.size()<this->size)
		{
			oTypeData.resize(size);
		}
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
private:
	std::size_t size=0;
public:
	PointStreamConverter(std::size_t _size)
	{
		size=_size;
	}
	bool clear(RoboCompRGBD::imgType &oTypeData)
	{
		oTypeData.clear();
		return true;
	}
    bool ItoO(const astra::PointFrame &iTypeData, RoboCompRGBD::imgType &oTypeData)
    {
		if(oTypeData.size()<this->size)
		{
			oTypeData.resize(size);
		}
        if (iTypeData.is_valid())
        {

            //            this->resize(d.width() * d.height()*data_size);
//            std::cout<<"iTypeData length "<<iTypeData.length()<<endl;
            memcpy(&oTypeData[0], iTypeData.data(), iTypeData.length()*3*4);
//            auto point = iTypeData.data()[10];
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
private:
	std::size_t size=0;
public:
	ColorSeqConverter(std::size_t _size)
	{
		size=_size;
	}
	bool clear(RoboCompRGBD::ColorSeq &oTypeData)
	{
		oTypeData.clear();
		return true;
	}
    bool ItoO(const astra::ColorFrame &iTypeData, RoboCompRGBD::ColorSeq &oTypeData)
    {
		if(oTypeData.size()<this->size)
		{
			oTypeData.resize(size);
		}
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
//private:
//	std::size_t size=0;
//public:
//	ByteSeqConverter(std::size_t _size)
//	{
//		size=_size;
//	}
//bool clear(, RoboCompRGBD::imgType &)
//{
//	oTypeData.clear();
//return true;
//}
//    bool ItoO(const astra::ColorFrame &iTypeData, RoboCompRGBD::imgType &oTypeData)
//    {
//		  if(oTypeData.size()<this->size)
// {
// oTypeData.resize(size);
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
private:
	std::size_t size=0;
public:
	FloatSeqConverter(std::size_t _size)
	{
		size=_size;
	}
	bool clear(RoboCompRGBD::DepthSeq &oTypeData)
	{
		oTypeData.clear();
		return true;
	}
    bool ItoO(const astra::DepthFrame &iTypeData, RoboCompRGBD::DepthSeq &oTypeData)
    {
		if(oTypeData.size()<this->size)
		{
			oTypeData.resize(size);
		}
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


class BodiesPeopleConverter : public Converter<astra::BodyFrame, RoboCompHumanTracker::PersonList>
{
	static bool astraBodies2RobocompPersonList(const astra::BodyFrame &iTypeData, RoboCompHumanTracker::PersonList &oTypeData)
	{
		std::map<astra::JointType, ::std::string> JOINT2STRING_ = {
				std::make_pair(astra::JointType::Head,"Head"),
				std::make_pair(astra::JointType::Neck,"Neck"),
				std::make_pair(astra::JointType::ShoulderSpine,"ShoulderSpine"),
				std::make_pair(astra::JointType::LeftShoulder,"LeftShoulder"),
				std::make_pair(astra::JointType::LeftElbow,"LeftElbow"),
				std::make_pair(astra::JointType::LeftWrist,"LeftWrist"),
				std::make_pair(astra::JointType::LeftHand,"LeftHand"),
				std::make_pair(astra::JointType::RightShoulder,"RightShoulder"),
				std::make_pair(astra::JointType::RightElbow,"RightElbow"),
				std::make_pair(astra::JointType::RightWrist,"RightWrist"),
				std::make_pair(astra::JointType::RightHand,"RightHand"),
				std::make_pair(astra::JointType::MidSpine,"MidSpine"),
				std::make_pair(astra::JointType::BaseSpine,"BaseSpine"),
				std::make_pair(astra::JointType::LeftHip,"LeftHip"),
				std::make_pair(astra::JointType::LeftKnee,"LeftKnee"),
				std::make_pair(astra::JointType::LeftFoot,"LeftFoot"),
				std::make_pair(astra::JointType::RightHip,"RightHip"),
				std::make_pair(astra::JointType::RightKnee,"RightKnee"),
				std::make_pair(astra::JointType::RightFoot,"RightFoot")
		};
		if (iTypeData.is_valid()) {
			const auto& bodies = iTypeData.bodies();
			if (bodies.empty())
				return false;

			for (auto& body : bodies) {
//            qDebug()<<"------------------ Found person " << body.id()<<"--------------------";

				RoboCompHumanTracker::TPerson person;
				auto status = body.status();

				switch (status) {
				case astra::BodyStatus::NotTracking:person.state = RoboCompHumanTracker::TrackingState::NotTracking;
					break;
				case astra::BodyStatus::TrackingLost:person.state = RoboCompHumanTracker::TrackingState::TrackingLost;
					break;
				case astra::BodyStatus::TrackingStarted:person.state = RoboCompHumanTracker::TrackingState::TrackingStarted;
					break;
				case astra::BodyStatus::Tracking:person.state = RoboCompHumanTracker::TrackingState::Tracking;
					break;
				default:qDebug() << "Invalid body state";
				}

				RoboCompHumanTracker::jointListType joints_list;
				RoboCompHumanTracker::jointListType joints_depth;

				const auto& joints = body.joints();

				if (!joints.empty()) {

					for (const auto& j : joints) {
						if (j.status()==astra::JointStatus::Tracked or
								j.status()==astra::JointStatus::LowConfidence) {
							auto& jnt = j.world_position();
							auto& jntdepth = j.depth_position();

							RoboCompHumanTracker::joint pointindepth;
							pointindepth.push_back(jntdepth.x);
							pointindepth.push_back(jntdepth.y);

							RoboCompHumanTracker::joint JointP;
							JointP.push_back(jnt.x);
							JointP.push_back(jnt.y);
							JointP.push_back(jnt.z);

							astra::JointType type = j.type();
							std::string typejoint;

							typejoint = JOINT2STRING_.at(type);
							joints_list[typejoint] = JointP;
							joints_depth[typejoint] = pointindepth;

						}
					}
				}
				else
					qDebug() << "Joints is empty";

				person.joints = joints_list;
				oTypeData[body.id()] = person;

			}
			return true;
		}
		return false;
	}
public:

	bool clear(RoboCompHumanTracker::PersonList &oTypeData)
	{
		oTypeData.clear();
		return true;
	}
	bool ItoO(const astra::BodyFrame &iTypeData, RoboCompHumanTracker::PersonList &oTypeData)
	{

		return astraBodies2RobocompPersonList(iTypeData, oTypeData);
	}

	bool OtoI(const RoboCompHumanTracker::PersonList &oTypeData, astra::BodyFrame &iTypeData)
	{
		return false;
	}
};

class BodyRGBConverter : public Converter<std::tuple<astra::ColorFrame&,astra::BodyFrame&, long int>, RoboCompHumanTrackerJointsAndRGB::MixedJointsRGB>
{

	static bool astraBodies2RobocompPersonList(const astra::BodyFrame &iTypeData, RoboCompHumanTrackerJointsAndRGB::PersonList &oTypeData)
	{
		std::map<astra::JointType, ::std::string> JOINT2STRING_ = {
				std::make_pair(astra::JointType::Head,"Head"),
				std::make_pair(astra::JointType::Neck,"Neck"),
				std::make_pair(astra::JointType::ShoulderSpine,"ShoulderSpine"),
				std::make_pair(astra::JointType::LeftShoulder,"LeftShoulder"),
				std::make_pair(astra::JointType::LeftElbow,"LeftElbow"),
				std::make_pair(astra::JointType::LeftWrist,"LeftWrist"),
				std::make_pair(astra::JointType::LeftHand,"LeftHand"),
				std::make_pair(astra::JointType::RightShoulder,"RightShoulder"),
				std::make_pair(astra::JointType::RightElbow,"RightElbow"),
				std::make_pair(astra::JointType::RightWrist,"RightWrist"),
				std::make_pair(astra::JointType::RightHand,"RightHand"),
				std::make_pair(astra::JointType::MidSpine,"MidSpine"),
				std::make_pair(astra::JointType::BaseSpine,"BaseSpine"),
				std::make_pair(astra::JointType::LeftHip,"LeftHip"),
				std::make_pair(astra::JointType::LeftKnee,"LeftKnee"),
				std::make_pair(astra::JointType::LeftFoot,"LeftFoot"),
				std::make_pair(astra::JointType::RightHip,"RightHip"),
				std::make_pair(astra::JointType::RightKnee,"RightKnee"),
				std::make_pair(astra::JointType::RightFoot,"RightFoot")
		};
		if (iTypeData.is_valid()) {
			const auto& bodies = iTypeData.bodies();
			if (bodies.empty())
				return false;

			for (auto& body : bodies) {
//            qDebug()<<"------------------ Found person " << body.id()<<"--------------------";

				RoboCompHumanTrackerJointsAndRGB::TPerson person;
				auto status = body.status();

				switch (status) {
				case astra::BodyStatus::NotTracking:person.state = RoboCompHumanTrackerJointsAndRGB::TrackingState::NotTracking;
					break;
				case astra::BodyStatus::TrackingLost:person.state = RoboCompHumanTrackerJointsAndRGB::TrackingState::TrackingLost;
					break;
				case astra::BodyStatus::TrackingStarted:person.state = RoboCompHumanTrackerJointsAndRGB::TrackingState::TrackingStarted;
					break;
				case astra::BodyStatus::Tracking:person.state = RoboCompHumanTrackerJointsAndRGB::TrackingState::Tracking;
					break;
				default:qDebug() << "Invalid body state";
				}

				RoboCompHumanTrackerJointsAndRGB::jointListType joints_list;
				RoboCompHumanTrackerJointsAndRGB::jointListType joints_depth;

				const auto& joints = body.joints();

				if (!joints.empty()) {

					for (const auto& j : joints) {
						if (j.status()==astra::JointStatus::Tracked or
								j.status()==astra::JointStatus::LowConfidence) {
							auto& jnt = j.world_position();
							auto& jntdepth = j.depth_position();

							RoboCompHumanTrackerJointsAndRGB::joint pointindepth;
							pointindepth.push_back(jntdepth.x);
							pointindepth.push_back(jntdepth.y);

							RoboCompHumanTrackerJointsAndRGB::joint JointP;
							JointP.push_back(jnt.x);
							JointP.push_back(jnt.y);
							JointP.push_back(jnt.z);

							astra::JointType type = j.type();
							std::string typejoint;

							typejoint = JOINT2STRING_.at(type);
							joints_list[typejoint] = JointP;
							joints_depth[typejoint] = pointindepth;

						}
					}
				}
				else
					qDebug() << "Joints is empty";

				person.joints = joints_list;
				oTypeData[body.id()] = person;

			}
			return true;
		}
		return false;
	}
public:

	bool clear(RoboCompHumanTrackerJointsAndRGB::MixedJointsRGB &oTypeData)
	{
		oTypeData.persons.clear();
		oTypeData.rgbImage.image.clear();
		oTypeData.rgbImage.height =0;
		oTypeData.rgbImage.width =0;
		oTypeData.rgbImage.depth =0;
		oTypeData.timeStamp = 0;
		return true;
	}
	bool ItoO(const std::tuple<astra::ColorFrame&, astra::BodyFrame&, long int> &iTypeData, RoboCompHumanTrackerJointsAndRGB::MixedJointsRGB &oTypeData)
	{
		const auto& colorFrame = std::get<0>(iTypeData);
		const auto& bodyFrame = std::get<1>(iTypeData);
		const auto& timestamp = (long int)std::get<2>(iTypeData);
		oTypeData.timeStamp = timestamp;

		oTypeData.persons.clear();
		astraBodies2RobocompPersonList(bodyFrame, oTypeData.persons);
		uint astraColorFrameSize = colorFrame.width()*colorFrame.height()*3;
/*		if (colorFrame.is_valid())
		{
			if(oTypeData.rgbImage.image.size()!= astraColorFrameSize)
			{
				oTypeData.rgbImage.height = colorFrame.height();
				oTypeData.rgbImage.width = colorFrame.width();
				oTypeData.rgbImage.depth = 3;
				oTypeData.rgbImage.image.resize(astraColorFrameSize);
			}

			//            this->resize(d.width() * d.height()*data_size);
			memcpy(&oTypeData.rgbImage.image[0], colorFrame.data(), astraColorFrameSize);
			//            std::copy(std::begin(d.data()), std::end(d.data()), std::begin(writeBuffer));
		}
		else{*/
			oTypeData.rgbImage.image.clear();
			oTypeData.rgbImage.height =0;
			oTypeData.rgbImage.width =0;
			oTypeData.rgbImage.depth =0;
		//}
		return true;

	}
	bool OtoI(const RoboCompHumanTrackerJointsAndRGB::MixedJointsRGB &oTypeData, std::tuple<astra::ColorFrame&,astra::BodyFrame&, long int> &iTypeData)
	{
		return false;
	}
};


#endif //PROJECT_DOUBLEBUFFERCONVERTERS_H
