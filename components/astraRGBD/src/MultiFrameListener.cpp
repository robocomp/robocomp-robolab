//
// Created by robolab on 23/07/18.
//

#include "MultiFrameListener.h"

#include <ctime>


MultiFrameListener::MultiFrameListener(astra::StreamReader& reader_)
{
    streamBools["depth"]=false;
    streamBools["color"]=false;
    streamBools["ir"]=false;
    streamBools["points"]=false;
    streamBools["hand"]=false;
    streamBools["body"]=false;
    reader = &reader_;

    colorStream = new astra::ColorStream(configure_color(*reader));
    depthStream = new astra::DepthStream(configure_depth(*reader));
    pointStream = new astra::PointStream(configure_point(*reader));
    bodyStream  = new astra::BodyStream(configure_body(*reader));


    colorBuff2.init(640*480*3, byteConverter);
    colorBuff.init(640*480, colorConverter);
    depthBuff.init(640*480, depthConverter);
//    bodyBuff.init(bodiesConverter);
    end = chrono::steady_clock::now();
}

void MultiFrameListener::on_frame_ready(astra::StreamReader& reader, astra::Frame& frame)
{
//    auto start = chrono::steady_clock::now();
//    cout << "lapse "<<chrono::duration <double, milli> (end-start).count() << " ms" << endl;
//    end =  chrono::steady_clock::now();

    if (streamBools["depth"])
    {
        const astra::DepthFrame depthFrame = frame.get<astra::DepthFrame>();
        if (depthFrame.is_valid())
        {
            depthBuff.put(depthFrame, sizeof(short));
        }
    }
    if (streamBools["color"])
    {
        astra::ColorFrame colorFrame = frame.get<astra::ColorFrame>();
        if(colorFrame.is_valid())
        {
            // Huge odd job because of the 2 image formats od the RGBD interface ColorSeq and ImgType
            colorBuff.put(colorFrame, sizeof(astra::RgbPixel));
            colorBuff2.put(colorFrame, sizeof(astra::RgbPixel));
        }
    }
	if (streamBools["point"])
	{
		astra::ColorFrame colorFrame = frame.get<astra::ColorFrame>();
		if(colorFrame.is_valid())
		{
			// Huge odd job because of the 2 image formats od the RGBD interface ColorSeq and ImgType
			colorBuff.put(colorFrame, sizeof(astra::RgbPixel));
			colorBuff2.put(colorFrame, sizeof(astra::RgbPixel));
		}
	}

    if (streamBools["body"])
    {
        astra::BodyFrame bodyFrame = frame.get<astra::BodyFrame>();
        if (!bodyFrame.is_valid() || bodyFrame.info().width() == 0 || bodyFrame.info().height() == 0)
        {
            return;
        }
        // NOT USING DOUBLE BUFFER BECAUSE A BUG ON RETAINING UNEXISTNG PEOPLE
//        bodyBuff.put(bodyFrame, sizeof(astra::BodyFrame));
        bodylist.clear();
        PersonDepth.clear();


        const auto& bodies = bodyFrame.bodies();
        if (bodies.empty())
            return;

        antonio = true;
        for (auto& body : bodies)
        {
//            qDebug()<<"------------------ Found person " << body.id()<<"--------------------";

            TPerson person;
            auto status = body.status();

            switch (status){
                case  astra::BodyStatus::NotTracking:
                    person.state =  RoboCompHumanTracker::TrackingState::NotTracking;
                    break;
                case astra::BodyStatus::TrackingLost:
                    person.state =  RoboCompHumanTracker::TrackingState::TrackingLost;
                    break;
                case astra::BodyStatus::TrackingStarted:
                    person.state =  RoboCompHumanTracker::TrackingState::TrackingStarted;
                    break;
                case astra::BodyStatus::Tracking:
                    person.state = RoboCompHumanTracker::TrackingState::Tracking;
                    break;
                default:
                    qDebug()<<"CAGOENTO";
            }


            jointListType joints_list;
            jointListType joints_depth;

            const auto& joints = body.joints();

            if (!joints.empty())
            {

                for (const auto& j : joints)
                {
                    if (j.status() == astra::JointStatus::Tracked or j.status() == astra::JointStatus::LowConfidence)
                    {
                        auto &jnt = j.world_position();
                        auto &jntdepth = j.depth_position();

                        joint pointindepth;
                        pointindepth.push_back(jntdepth.x);
                        pointindepth.push_back(jntdepth.y);


                        joint JointP;
                        JointP.push_back(jnt.x);
                        JointP.push_back(jnt.y);
                        JointP.push_back(jnt.z);

                        astra::JointType type = j.type();
                        std::string typejoint;

                        switch (type)
                        {
                            case astra::JointType::Head:
                                typejoint = "Head";

                                break;
                            case astra::JointType::Neck:
                                typejoint = "Neck";
                                break;
                            case astra::JointType::ShoulderSpine:
                                typejoint = "ShoulderSpine";
                                break;
                            case astra::JointType::LeftShoulder:
                                typejoint = "LeftShoulder";
                                break;
                            case astra::JointType::LeftElbow:
                                typejoint = "LeftElbow";
                                break;
                            case astra::JointType::LeftWrist:
                                typejoint = "LeftWrist";
                                break;
                            case astra::JointType::LeftHand:
                                typejoint = "LeftHand";
                                break;
                            case astra::JointType::RightShoulder:
                                typejoint = "RightShoulder";
                                break;
                            case astra::JointType::RightElbow:
                                typejoint = "RightElbow";
                                break;
                            case astra::JointType::RightWrist:
                                typejoint = "RightWrist";
                                break;
                            case astra::JointType::RightHand:
                                typejoint = "RightHand";
                                break;
                            case astra::JointType::MidSpine:
                                typejoint = "MidSpine";
                                break;
                            case astra::JointType::BaseSpine:
                                typejoint = "BaseSpine";
                                break;
                            case astra::JointType::LeftHip:
                                typejoint = "LeftHip";
                                break;
                            case astra::JointType::LeftKnee:
                                typejoint = "LeftKnee";
                                break;
                            case astra::JointType::LeftFoot:
                                typejoint = "LeftFoot";
                                break;
                            case astra::JointType::RightHip:
                                typejoint = "RightHip";
                                break;
                            case astra::JointType::RightKnee:
                                typejoint = "RightKnee";
                                break;
                            case astra::JointType::RightFoot:
                                typejoint = "RightFoot";
                                break;
                            default:
                                typejoint = " ";
                        }
                        joints_list[typejoint] = JointP;
                        joints_depth[typejoint]=pointindepth;

                    }
                }
            }
            else  qDebug()<<"Joints is empty";

            person.joints = joints_list;
            bodylist[body.id()] = person;

            PersonDepth[body.id()] = joints_depth;

        }
        qDebug()<<" PERSONAS = "<<bodylist.size() ;
        antonio = false;
        return;
    }

}

void MultiFrameListener::set_color_stream(bool color_bool)
{

    if (color_bool){
        colorStream->start();
    }
    else{
        colorStream->stop();
    }
    streamBools["color"] = color_bool;
}

void MultiFrameListener::set_depth_stream(bool depth_bool)
{
    if (depth_bool){
        depthStream->start();
    }
    else{
        depthStream->stop();
    }
    streamBools["depth"] = depth_bool;
}

void MultiFrameListener::set_point_stream(bool point_bool)
{
    if (point_bool){
        pointStream->start();
    }
    else{
        pointStream->stop();
    }
    streamBools["point"] = point_bool;
}

void MultiFrameListener::set_body_stream(bool body_bool)
{
    if (body_bool){
        bodyStream->start();
    }
    else{
        bodyStream->stop();
    }
    streamBools["body"] = body_bool;
}


void MultiFrameListener::get_depth(DepthSeq& depth)
{
//    cout<<"MultiFrameListener::get_depth"<<endl;
    depthBuff.get(depth);

}

void MultiFrameListener::get_points(PointSeq& points)
{
//    cout<<"MultiFrameListener::get_points"<<endl;
    pointBuff.get(points);
}
void MultiFrameListener::get_color(ColorSeq& colors)
{
//    cout<<"MultiFrameListener::get_color"<<endl;
    colorBuff.get(colors);
}

void MultiFrameListener::get_color(imgType& colors)
{
//    if (colorBuff2.size==0) return;
//    cout<<"MultiFrameListener::get_color2"<<endl;
    colorBuff2.get(colors);
    // Hack to fix a problem with the ColorSeq and imgType types of RGBD interface
//    colors.resize(colors.size()/3);
}


void MultiFrameListener::get_people(PersonList& people)
{
//    qDebug()<<"get_People_1 "<<people.size();
//    bodyBuff.get(people);
//    qDebug()<<"get_People_2 "<<people.size();

    if (antonio) //la bandera dice si se esta leyendo a la vez que escribiendo
    {
        qDebug()<<"--------------------------------------------------------";
        qDebug()<<"--------------------------------------------------------";
        qDebug()<<"--------------- SOY EL GATO CON BOTAS ------------------";
        qDebug()<<"--------------------------------------------------------";
        qDebug()<<"--------------------------------------------------------";
    }

    else
        people = bodylist;

}

joint MultiFrameListener::getJointDepth(int idperson, string idjoint)
{
    jointListType p = PersonDepth[idperson];
    auto pos = p[idjoint];
    return pos;

}


astra::DepthStream MultiFrameListener::configure_depth(astra::StreamReader& reader)
{
    auto depthStream = reader.stream<astra::DepthStream>();

    auto oldMode = depthStream.mode();

    //We don't have to set the mode to start the stream, but if you want to here is how:
    astra::ImageStreamMode depthMode;

    depthMode.set_width(640);
    depthMode.set_height(480);
    depthMode.set_pixel_format(astra_pixel_formats::ASTRA_PIXEL_FORMAT_DEPTH_MM);
    depthMode.set_fps(30);

    depthStream.set_mode(depthMode);

    auto newMode = depthStream.mode();
    printf("Changed depth mode: %dx%d @ %d -> %dx%d @ %d\n",
           oldMode.width(), oldMode.height(), oldMode.fps(),
           newMode.width(), newMode.height(), newMode.fps());

    return depthStream;
}

astra::InfraredStream MultiFrameListener::configure_ir(astra::StreamReader& reader, bool useRGB)
{
    auto irStream = reader.stream<astra::InfraredStream>();

    auto oldMode = irStream.mode();

    astra::ImageStreamMode irMode;
    irMode.set_width(640);
    irMode.set_height(480);

    if (useRGB)
    {
        irMode.set_pixel_format(astra_pixel_formats::ASTRA_PIXEL_FORMAT_RGB888);
    }
    else
    {
        irMode.set_pixel_format(astra_pixel_formats::ASTRA_PIXEL_FORMAT_GRAY16);
    }

    irMode.set_fps(30);
    irStream.set_mode(irMode);

    auto newMode = irStream.mode();
    printf("Changed IR mode: %dx%d @ %d -> %dx%d @ %d\n",
           oldMode.width(), oldMode.height(), oldMode.fps(),
           newMode.width(), newMode.height(), newMode.fps());

    return irStream;
}

astra::ColorStream MultiFrameListener::configure_color(astra::StreamReader& reader)
{
    auto colorStream = reader.stream<astra::ColorStream>();

    auto oldMode = colorStream.mode();

    astra::ImageStreamMode colorMode;
    colorMode.set_width(640);
    colorMode.set_height(480);
    colorMode.set_pixel_format(astra_pixel_formats::ASTRA_PIXEL_FORMAT_RGB888);
    colorMode.set_fps(30);

    colorStream.set_mode(colorMode);

    auto newMode = colorStream.mode();
    printf("Changed color mode: %dx%d @ %d -> %dx%d @ %d\n",
           oldMode.width(), oldMode.height(), oldMode.fps(),
           newMode.width(), newMode.height(), newMode.fps());

    return colorStream;
}


astra::PointStream MultiFrameListener::configure_point(astra::StreamReader& reader)
{
    auto pointStream = reader.stream<astra::PointStream>();

//    auto oldMode = pointStream.mode();
//
//    astra::ImageStreamMode pointMode;
//    pointMode.set_width(640);
//    pointMode.set_height(480);
//    pointMode.set_fps(30);
//
//    pointStream.set_mode(pointMode);
//
//    auto newMode = pointStream.mode();
//    printf("Changed point mode: %dx%d @ %d -> %dx%d @ %d\n",
//           oldMode.width(), oldMode.height(), oldMode.fps(),
//           newMode.width(), newMode.height(), newMode.fps());

    return pointStream;
}


astra::BodyStream MultiFrameListener::configure_body(astra::StreamReader& reader)
{
    auto bodyStream = reader.stream<astra::BodyStream>();

    return bodyStream;
}

