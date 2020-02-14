//
// Created by robolab on 23/07/18.
//

#include "MultiFrameListener.h"

#include <ctime>
#include "innermodel/innermodel.h"

MultiFrameListener::MultiFrameListener(RoboCompHumanTrackerJointsAndRGB::HumanTrackerJointsAndRGBPrx &_pubproxy, int _cameraID) : pubproxy (_pubproxy)
{
    cameraID = _cameraID;
    streamBools["depth"]=false;
    streamBools["color"]=false;
    streamBools["ir"]=false;
    streamBools["points"]=false;
    streamBools["hand"]=false;
    streamBools["body"]=false;
    reader = new astra::StreamReader(streamSet.create_reader());
    reader->add_listener(*this);

    colorStream = new astra::ColorStream(configure_color(*reader));
    depthStream = new astra::DepthStream(configure_depth(*reader));
    pointStream = new astra::PointStream(configure_point(*reader));
    bodyStream  = new astra::BodyStream(configure_body(*reader));


    byteConverter = new ByteSeqConverter(640*480*3);
    depthConverter = new FloatSeqConverter(640*480);
    colorConverter = new ColorSeqConverter(640*480);
    pointStreamConverter = new PointStreamConverter(640*480*3*4);
    pointConverter = new PointSeqConverter(640*480);
    bodiesConverter = new BodiesPeopleConverter();
    bodyRgbConverter = new BodyRGBConverter();


    colorBuff2.init(*byteConverter);
    colorBuff.init(*colorConverter);
    depthBuff.init(*depthConverter);
    pointStreamBuff.init(*pointStreamConverter);
    pointBuff.init(*pointConverter);
    bodyBuff.init(*bodiesConverter);
    bodyRGBMix.init(*bodyRgbConverter);

    end = chrono::steady_clock::now();
    std::map<astra::JointType, ::std::string> JOINT2STRING = {
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
}
void MultiFrameListener::on_frame_ready(astra::StreamReader& reader, astra::Frame& frame)
{
    auto start = chrono::steady_clock::now();
//	cout << "SAME FRAME READY " << endl;
    auto t0 = std::chrono::high_resolution_clock::now();
    unsigned long milliseconds_since_epoch = t0.time_since_epoch() / std::chrono::milliseconds(1);
//    qDebug()<<typeid(milliseconds_since_epoch).name()<<endl;

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
//        cout << "lapse COLORS "<<chrono::duration <double, milli> (end-start).count() << " ms" << endl;
        end =  chrono::steady_clock::now();
    }
	if (streamBools["point"])
	{

		astra::PointFrame pointFrame = frame.get<astra::PointFrame>();
		if(pointFrame.is_valid())
		{
//		pointBuff.put(pointFrame, sizeof(float)*4);
            pointStreamBuff.put(pointFrame, sizeof(float)*4);
		}
        end =  chrono::steady_clock::now();
	}


    if (streamBools["body"])
    {
        astra::BodyFrame bodyFrame = frame.get<astra::BodyFrame>();
        if (!bodyFrame.is_valid() || bodyFrame.bodies().size()==0)
        {

            bodyBuff.clear();
        }
//		cout << "lapse BODYs "<<chrono::duration <double, milli> (end-start).count() << " ms" << endl;

        // NOT USING DOUBLE BUFFER BECAUSE A BUG ON RETAINING UNEXISTNG PEOPLE
        bodyBuff.put(bodyFrame, sizeof(astra::BodyFrame));
//        bodylist.clear();
//        PersonDepth.clear();
//
//
//        const auto& bodies = bodyFrame.bodies();
//        if (bodies.empty())
//            return;
//
//        is_writting = true;
//        for (auto& body : bodies)
//        {
////            qDebug()<<"------------------ Found person " << body.id()<<"--------------------";
//
//			RoboCompHumanTracker::TPerson person;
//            auto status = body.status();
//
//            switch (status){
//                case  astra::BodyStatus::NotTracking:
//                    person.state =  RoboCompHumanTracker::TrackingState::NotTracking;
//                    break;
//                case astra::BodyStatus::TrackingLost:
//                    person.state =  RoboCompHumanTracker::TrackingState::TrackingLost;
//                    break;
//                case astra::BodyStatus::TrackingStarted:
//                    person.state =  RoboCompHumanTracker::TrackingState::TrackingStarted;
//                    break;
//                case astra::BodyStatus::Tracking:
//                    person.state = RoboCompHumanTracker::TrackingState::Tracking;
//                    break;
//                default:
//                    qDebug()<<"Invalid body state";
//            }
//
//
//            RoboCompHumanTrackerJointsAndRGB::jointListType joints_list;
//            RoboCompHumanTrackerJointsAndRGB::jointListType joints_depth;
//
//            const auto& joints = body.joints();
//
//            if (!joints.empty())
//            {
//
//                for (const auto& j : joints)
//                {
//                    if (j.status() == astra::JointStatus::Tracked or j.status() == astra::JointStatus::LowConfidence)
//                    {
//                        auto &jnt = j.world_position();
//                        auto &jntdepth = j.depth_position();
//
//                        RoboCompHumanTrackerJointsAndRGB::joint pointindepth;
//                        pointindepth.push_back(jntdepth.x);
//                        pointindepth.push_back(jntdepth.y);
//
//
//                        RoboCompHumanTrackerJointsAndRGB::joint JointP;
//                        JointP.push_back(jnt.x);
//                        JointP.push_back(jnt.y);
//                        JointP.push_back(jnt.z);
//
//                        astra::JointType type = j.type();
//                        std::string typejoint;
//
//                        typejoint = JOINT2STRING.at(type);
//                        joints_list[typejoint] = JointP;
//                        joints_depth[typejoint]=pointindepth;
//
//                    }
//                }
//            }
//            else  qDebug()<<"Joints is empty";
//
//            person.joints = joints_list;
//            bodylist[body.id()] = person;
//
//            PersonDepth[body.id()] = joints_depth;
//
//        }
//        qDebug()<<" PERSONAS = "<<bodylist.size() ;
//        is_writting = false;
//        return;
    }

    //Special case for syncronized color and joints
    //TODO: better coding and clean-up
    if (streamBools["color"] and streamBools["body"]) {
//        cout << "BODY AND COLOR"<< endl;
        astra::BodyFrame bodyFrame = frame.get<astra::BodyFrame>();
        astra::ColorFrame colorFrame = frame.get<astra::ColorFrame>();
        //Both frame vaalidation is done in the Corresponding Converter.
        auto thetuple = std::forward_as_tuple(colorFrame, bodyFrame, milliseconds_since_epoch);
                bodyRGBMix.put(thetuple, 0);
		RoboCompHumanTrackerJointsAndRGB::MixedJointsRGB output;
		bodyRGBMix.get(output);
		output.cameraID = cameraID;
                qDebug()<<"OUTPUT is Frame with timestamp: "<<output.timeStamp<<" and Person with "<< output.persons.size()<<" and color"<< output.rgbImage.width<<endl;
		pubproxy->newPersonListAndRGB(output);
if(output.persons.size()>0){ 
qDebug()<<"******************BODY******************";
        const float fx=535.4, fy=539.2, sx=1, sy=1, Ox=320, Oy=240;
        QMat K = QMat::zeros(3,3);
	K(0,0) = fx/sx; K(0,1) = 0.f; 		K(0,2) = Ox;
	K(1,0) = 0; 	 K(1,1) = -fy/sy; 	K(1,2) = Oy;
	K(2,0) = 0;		 K(2,1) = 0;		K(2,2) = 1;
RoboCompHumanTrackerJointsAndRGB::PersonList people = output.persons;
DepthSeq depth;
depthBuff.get(depth);
qDebug()<<"depth size"<<depth.size();
for (auto person: people){
try{
    QVec p3D = QVec::vec3(person.second.joints.at("Head")[0], person.second.joints.at("Head")[1], person.second.joints.at("Head")[2]);
    p3D.print("Head joint");
    //point in depth image
    QVec img = K * p3D;
    int x = img[0]/img[2];
    int y = img[1]/img[2];
    float depthP = depth[y*640+x];
    QVec pDepth= QVec::vec3( (x-320) *depthP/535.4  , -(y-240)*depthP/539.2, depthP);
    pDepth.print("Depth point");
}catch(...)
{qDebug()<<"no data for that joint";}
}
}
//points 
//PointsSeq points = pointBuff.get(points);

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
	typedef std::chrono::duration<float> fsec;
	std::chrono::system_clock::time_point initial_time = std::chrono::system_clock::now();
    pointBuff.get(points);
	std::cout<<"Multiframe: fps "<<fsec(1)/( std::chrono::system_clock::now() - initial_time)<<endl;
}

void MultiFrameListener::get_points_stream(imgType& pointsStream)
{
//    cout<<"MultiFrameListener::get_points"<<endl;
    typedef std::chrono::duration<float> fsec;
    std::chrono::system_clock::time_point initial_time = std::chrono::system_clock::now();
    pointStreamBuff.get(pointsStream);
    std::cout<<"Multiframe: fps "<<fsec(1)/( std::chrono::system_clock::now() - initial_time)<<endl;
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


void MultiFrameListener::get_people(RoboCompHumanTracker::PersonList& people)
{
    qDebug()<<"get_People_1 "<<people.size();
    bodyBuff.get(people);
    qDebug()<<"get_People_2 "<<people.size();

//    if (is_writting) //la bandera dice si se esta leyendo a la vez que escribiendo
//    {
//        qDebug()<<"--------------------------------------------------------";
//        qDebug()<<"--------------------------------------------------------";
//        qDebug()<<"- Possible sync problem while reading people structure -";
//        qDebug()<<"--------------------------------------------------------";
//        qDebug()<<"--------------------------------------------------------";
//    }
//
//    else
//        people = bodylist;

}

RoboCompHumanTracker::joint MultiFrameListener::getJointDepth(int idperson, string idjoint)
{
    RoboCompHumanTracker::jointListType p = PersonDepth[idperson];
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

