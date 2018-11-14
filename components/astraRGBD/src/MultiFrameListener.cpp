//
// Created by robolab on 23/07/18.
//

#include "MultiFrameListener.h"




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


//void MultiFrameListener::update_depth(astra::Frame& frame)
//{
//    const astra::DepthFrame depthFrame = frame.get<astra::DepthFrame>();
//
//    if (!depthFrame.is_valid())
//    {
//
//        return;
//    }
//
//    const int depthWidth = depthFrame.width();
//    const int depthHeight = depthFrame.height();
////    qDebug()<<"Depth frame readed";
//
//
////    const cv::Mat mImageDepth( depthHeight, depthWidth, CV_16UC1, (void*)depthFrame.data());
////
////    cv::Mat mScaledDepth;
////    mImageDepth.convertTo( mScaledDepth, CV_8U, 255.0 / 6000 );
////
////    cv::imshow( "Depth Image", mScaledDepth );
//    depthBuff.resize(depthWidth*depthHeight);
////    auto start = std::chrono::steady_clock::now();
//    // More optimized way to copy than the for loop. No memcpy can be used in this case becuase of the casting.
////    std::copy(&depthFrame.data()[0], &depthFrame.data()[0]+(depthWidth*depthHeight* sizeof(short)), begin(*depthBuff.getNextPtr()));
////    for(int i=0;i<depthWidth*depthHeight;i++) depthBuff.operator[](i)=(float)depthFrame.data()[i];
////    auto duration = std::chrono::duration_cast< std::chrono::nanoseconds> (std::chrono::steady_clock::now() - start);
////    printf("%d\n",duration);
//
//
//}


//void MultiFrameListener::update_color(astra::Frame& frame)
//{
//    const astra::ColorFrame colorFrame = frame.get<astra::ColorFrame>();
//
//    if (!colorFrame.is_valid())
//    {
//
//        return;
//    }
//
//    const int colorWidth = colorFrame.width();
//    const int colorHeight = colorFrame.height();
////    qDebug()<<"Color frame readed";
////    qDebug()<<colorWidth<<" "<<colorHeight;
//    colorBuff.resize(colorWidth*colorHeight*3);
////    const cv::Mat mImageRGB(colorHeight, colorWidth, CV_8UC3, (void*)colorFrame.data());
////    cv::Mat cImageBGR;
////    cv::cvtColor( mImageRGB, cImageBGR, CV_RGB2BGR );
////    cv::imshow( "Color Image", cImageBGR );
////    for(int i=0;i<colorWidth*colorHeight;i++) colorBuff.operator[](i)=(RoboCompRGBD::ColorRGB)colorFrame.data()[i];
//    memcpy(&colorBuff[0],colorFrame.data(),colorWidth*colorHeight* sizeof(astra::RgbPixel));
//
////    const astra::RgbPixel* color = colorFrame.data();
////    uint8_t* buffer = &colorView_.buffer_[0];
////
////    for(int i = 0; i < colorWidth * colorHeight; i++)
////    {
////        const int rgbaOffset = i * 4;
////        buffer[rgbaOffset] = color[i].r;
////        buffer[rgbaOffset + 1] = color[i].g;
////        buffer[rgbaOffset + 2] = color[i].b;
////        buffer[rgbaOffset + 3] = 255;
////    }
//
//}


//void MultiFrameListener::update_point(astra::Frame &frame)
//{
//    const astra::PointFrame pointFrame = frame.get<astra::PointFrame>();
//
//    if (!pointFrame.is_valid())
//    {
//        return;
//    }
//
//    int pointWidth = pointFrame.width();
//    int pointHeight = pointFrame.height();
////    qDebug()<<"Points frame readed";
//    pointBuff.resize(pointWidth*pointHeight);
//    for(int i=0;i<pointWidth*pointHeight;i++)
//    {
//        pointBuff[i].x = pointFrame.data()[i].x;
//        pointBuff[i].y = pointFrame.data()[i].y;
//        pointBuff[i].z = pointFrame.data()[i].z;
//    }
//}

//void MultiFrameListener::update_ir_16(astra::Frame& frame)
//{
//    const astra::InfraredFrame16 irFrame = frame.get<astra::InfraredFrame16>();
//
//    if (!irFrame.is_valid())
//    {
//        return;
//    }
//
//    const int irWidth = irFrame.width();
//    const int irHeight = irFrame.height();
////    qDebug()<<"IR16 frame readed";
//
////    const uint16_t* ir_values = irFrame.data();
////    uint8_t* buffer = &colorView_.buffer_[0];
////    for (int i = 0; i < irWidth * irHeight; i++)
////    {
////        const int rgbaOffset = i * 4;
////        const uint16_t value = ir_values[i];
////        const uint8_t red = static_cast<uint8_t>(value >> 2);
////        const uint8_t blue = 0x66 - red / 2;
////        buffer[rgbaOffset] = red;
////        buffer[rgbaOffset + 1] = 0;
////        buffer[rgbaOffset + 2] = blue;
////        buffer[rgbaOffset + 3] = 255;
////    }
//
//}

//void MultiFrameListener::update_ir_rgb(astra::Frame& frame)
//{
//    const astra::InfraredFrameRgb irFrame = frame.get<astra::InfraredFrameRgb>();
//
//    if (!irFrame.is_valid())
//    {
//        return;
//    }
//
//    int irWidth = irFrame.width();
//    int irHeight = irFrame.height();
////    qDebug()<<"Ir frame readed";
//
////    const astra::RgbPixel* irRGB = irFrame.data();
////    uint8_t* buffer = &colorView_.buffer_[0];
////    for (int i = 0; i < irWidth * irHeight; i++)
////    {
////        const int rgbaOffset = i * 4;
////        buffer[rgbaOffset] = irRGB[i].r;
////        buffer[rgbaOffset + 1] = irRGB[i].g;
////        buffer[rgbaOffset + 2] = irRGB[i].b;
////        buffer[rgbaOffset + 3] = 255;
////    }
//
//}

void MultiFrameListener::on_frame_ready(astra::StreamReader& reader, astra::Frame& frame)
{
    auto start = chrono::steady_clock::now();
//    cout << "lapse "<<chrono::duration <double, milli> (end-start).count() << " ms" << endl;
    end =  chrono::steady_clock::now();


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

        const auto& bodies = bodyFrame.bodies();
        if (bodies.empty())
            return;

        antonio = true;
        for (auto& body : bodies)
        {

            TPerson person;
            auto status = body.status();

            switch (status){
                case  astra::BodyStatus::NotTracking:
                    qDebug()<<"NOT TRACKING";
                    person.state =  RoboCompHumanTracker::TrackingState::NotTracking;
                    break;
                case astra::BodyStatus::TrackingLost:
                    qDebug()<<"TRACKING LOST";
                    person.state =  RoboCompHumanTracker::TrackingState::TrackingLost;
                    break;
                case astra::BodyStatus::TrackingStarted:
                    qDebug()<<"TRACKING STARTED";
                    person.state =  RoboCompHumanTracker::TrackingState::TrackingStarted;
                    break;
                case astra::BodyStatus::Tracking:
                    qDebug()<<"TRACKING";
                    person.state = RoboCompHumanTracker::TrackingState::Tracking;
                    break;
                default:
                    qDebug()<<"CAGOENTO";
            }


            jointListType joints_list;
            const auto& joints = body.joints();

            if (!joints.empty())
            {
//                qDebug()<<"JOINTS DE LA PERSONA " <<body.id();
                for (const auto& j : joints)
                {



                    if (j.status() == astra::JointStatus::Tracked)
                    {
                        auto &jnt = j.world_position();
                        joint JointP;
                        JointP.push_back(jnt.x);
                        JointP.push_back(jnt.y);
                        JointP.push_back(jnt.z);

//                        qDebug()<< JointP[0] <<" "<< JointP[1]<<" "<< JointP[2];

                        astra::JointType type = j.type();
                        std::string typejoint;
                        if (type == astra::JointType::Head)
                        {
//                            qDebug()<<"---------------------MATRIZ ROTACION CABEZA--------------------";
//                            auto o = j.orientation();
//                            qDebug()<< o.m00() << " " << o.m01() <<" " << o.m02();
//                            qDebug()<< o.m10() << " " << o.m11() <<" " << o.m12();
//                            qDebug()<< o.m20() << " " << o.m21() <<" " << o.m22();

                        }

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

                    }
                }

            }

            person.joints = joints_list;
            bodylist[body.id()] = person;
        }
        qDebug()<<"bodylist on return "<<bodylist.size() ;
        antonio = false;
        return;
    }

//    if (streamBools["ir"]) {
//        update_ir_rgb(frame);
//    }
//    if (streamBools["point"]) {
//        update_point(frame);
//        pointBuff.swap();
//    }
//    std::lock_guard<std::mutex> lock(my_mutex);
//    if (streamBools["depth"]) {
////        const astra::DepthFrame depthFrame = frame.get<astra::DepthFrame>();
////        depthBuff.put_stdcopy(depthFrame, sizeof(short));
////        depthBuff.swap();
//        if (update_depth(frame)) {
//            colorBuff.swap()
//        }
//    }
//    if (streamBools["color"]) {
////        astra::ColorFrame colorFrame = frame.get<astra::ColorFrame>();
////        colorBuff.put_memcpy(colorFrame, sizeof(astra::RgbPixel));
////        colorBuff.swap();
//        if (update_depth(frame))
//            depthBuff.swap();
//    }
//    if (streamBools["ir"]) {
//        update_ir_rgb(frame);
//    }
//    if (streamBools["point"]) {
//        if (update_point(frame))
//        {
//            pointBuff.swap();
//        }
//
//    }
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

    if (antonio)
    {
        qDebug()<<"--------------------------------------------------------";
        qDebug()<<"--------------------------------------------------------";
        qDebug()<<"--------------- SOY EL GATO CON BOTAS ------------------";
        qDebug()<<"--------------------------------------------------------";
        qDebug()<<"--------------------------------------------------------";
    }



     people = bodylist;
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