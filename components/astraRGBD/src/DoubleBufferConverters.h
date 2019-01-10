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

//class BodiesPeopleConverter : public Converter<astra::BodyFrame, RoboCompHumanTracker::PersonList>
//{
//public:
//    bool ItoO(const astra::BodyFrame &iTypeData, RoboCompHumanTracker::PersonList &oTypeData)
//    {
//        oTypeData.clear();
//
//        const auto& bodies = iTypeData.bodies();
//        qDebug()<<"SIZE OF BODIES "<<bodies.size();
//        if (bodies.empty())
//        {
//            qDebug()<<"SIZE DE LO QUE SALE "<<oTypeData.size() ;
//            return false;
//        }
//
//        for (auto& body : bodies)
//        {
//            TPerson person;
//            auto status = body.status();
//
//            switch (status){
//                case  astra::BodyStatus::NotTracking:
//                    qDebug()<<"NOT TRACKING";
//                    person.state =  RoboCompHumanTracker::TrackingState::NotTracking;
//                    break;
//                case astra::BodyStatus::TrackingLost:
//                    qDebug()<<"TRACKING LOST";
//                    person.state =  RoboCompHumanTracker::TrackingState::TrackingLost;
//                    break;
//                case astra::BodyStatus::TrackingStarted:
//                    qDebug()<<"TRACKING STARTED";
//                    person.state =  RoboCompHumanTracker::TrackingState::TrackingStarted;
//                    break;
//                case astra::BodyStatus::Tracking:
//                    qDebug()<<"TRACKING";
//                    person.state = RoboCompHumanTracker::TrackingState::Tracking;
//                    break;
//
//                default:
//                    qDebug()<<"CAGOENTO";
//            }
//
//
//            jointListType joints_list;
//            const auto& joints = body.joints();
//
//            if (!joints.empty())
//            {
//                qDebug()<<"JOINTS DE LA PERSONA " <<body.id();
//                for (const auto& j : joints)
//                {
//                    if (j.status() == astra::JointStatus::Tracked)
//                    {
//                        auto &jnt = j.world_position();
//                        joint JointP;
//                        JointP.push_back(jnt.x);
//                        JointP.push_back(jnt.y);
//                        JointP.push_back(jnt.z);
//
//                        qDebug()<< JointP[0] <<" "<< JointP[1]<<" "<< JointP[2];
//
//                        astra::JointType type = j.type();
//                        std::string typejoint;
//
//                        switch (type)
//                        {
//                            case astra::JointType::Head:
//                                typejoint = "Head";
//                                break;
//                            case astra::JointType::Neck:
//                                typejoint = "Neck";
//                                break;
//                            case astra::JointType::ShoulderSpine:
//                                typejoint = "ShoulderSpine";
//                                break;
//                            case astra::JointType::LeftShoulder:
//                                typejoint = "LeftShoulder";
//                                break;
//                            case astra::JointType::LeftElbow:
//                                typejoint = "LeftElbow";
//                                break;
//                            case astra::JointType::LeftWrist:
//                                typejoint = "LeftWrist";
//                                break;
//                            case astra::JointType::LeftHand:
//                                typejoint = "LeftHand";
//                                break;
//                            case astra::JointType::RightShoulder:
//                                typejoint = "RightShoulder";
//                                break;
//                            case astra::JointType::RightElbow:
//                                typejoint = "RightElbow";
//                                break;
//                            case astra::JointType::RightWrist:
//                                typejoint = "RightWrist";
//                                break;
//                            case astra::JointType::RightHand:
//                                typejoint = "RightHand";
//                                break;
//                            case astra::JointType::MidSpine:
//                                typejoint = "MidSpine";
//                                break;
//                            case astra::JointType::BaseSpine:
//                                typejoint = "BaseSpine";
//                                break;
//                            case astra::JointType::LeftHip:
//                                typejoint = "LeftHip";
//                                break;
//                            case astra::JointType::LeftKnee:
//                                typejoint = "LeftKnee";
//                                break;
//                            case astra::JointType::LeftFoot:
//                                typejoint = "LeftFoot";
//                                break;
//                            case astra::JointType::RightHip:
//                                typejoint = "RightHip";
//                                break;
//                            case astra::JointType::RightKnee:
//                                typejoint = "RightKnee";
//                                break;
//                            case astra::JointType::RightFoot:
//                                typejoint = "RightFoot";
//                                break;
//                            default:
//                                typejoint = " ";
//                        }
//
//                        joints_list[typejoint] = JointP;
//
//                    }
//                }
//
//            }
//
//            person.joints = joints_list;
//            oTypeData[body.id()] = person;
//        }
//        qDebug()<<"oTypeData on return "<<oTypeData.size() ;
//        return true;
//    }
//
//
//    bool OtoI(const RoboCompHumanTracker::PersonList &oTypeData, astra::BodyFrame &iTypeDat)
//    {
//        return false;
//    }
//};



//class FloatSeqConverter : public Converter<astra::DepthFrame, RoboCompRGBD::DepthSeq>
//{
//public:
//    bool ItoO(const astra::DepthFrame &iTypeData, RoboCompRGBD::DepthSeq &oTypeData)
//    {
//        if (iTypeData.is_valid())
//        {
//
//            //            this->resize(d.width() * d.height()*data_size);
//            std::copy(&iTypeData.data()[0], &iTypeData.data()[0]+(iTypeData.width()*iTypeData.height()), std::end(oTypeData));
//            //            std::copy(std::begin(d.data()), std::end(d.data()), std::begin(writeBuffer));
//            return true;
//        }
//        return false;
//    }
//
//    bool OtoI(const RoboCompRGBD::DepthSeq &oTypeData, astra::DepthFrame &iTypeData)
//    {
//        return false;
//    }
//};


#endif //PROJECT_DOUBLEBUFFERCONVERTERS_H
