//
// Created by robolab on 23/07/18.
//

#include "MultiFrameListener.h"



MultiFrameListener::MultiFrameListener(astra::StreamReader& reader_)
{
    RGBMutex = new QMutex();
    depthMutex = new QMutex();
    pointMutex = new QMutex();
    streamBools["depth"]=false;
    streamBools["color"]=false;
    streamBools["ir"]=false;
    streamBools["points"]=false;
    streamBools["body"]=false;
    streamBools["hand"]=false;
    reader = &reader_;

    colorStream = new astra::ColorStream(configure_color(*reader));
    depthStream = new astra::DepthStream(configure_depth(*reader));
    pointStream = new astra::PointStream(configure_point(*reader));
}


void MultiFrameListener::update_depth(astra::Frame& frame)
{
    const astra::DepthFrame depthFrame = frame.get<astra::DepthFrame>();

    if (!depthFrame.is_valid())
    {

        return;
    }

    const int depthWidth = depthFrame.width();
    const int depthHeight = depthFrame.height();
//    qDebug()<<"Depth frame readed";


//    const cv::Mat mImageDepth( depthHeight, depthWidth, CV_16UC1, (void*)depthFrame.data());
//
//    cv::Mat mScaledDepth;
//    mImageDepth.convertTo( mScaledDepth, CV_8U, 255.0 / 6000 );
//
//    cv::imshow( "Depth Image", mScaledDepth );
    depthBuff.resize(depthWidth*depthHeight);
//    auto start = std::chrono::steady_clock::now();
    // More optimized way to copy than the for loop. No memcpy can be used in this case becuase of the casting.
    std::copy(&depthFrame.data()[0], &depthFrame.data()[0]+(depthWidth*depthHeight* sizeof(short)), begin(*depthBuff.getWriter()));
//    for(int i=0;i<depthWidth*depthHeight;i++) depthBuff.operator[](i)=(float)depthFrame.data()[i];
//    auto duration = std::chrono::duration_cast< std::chrono::nanoseconds> (std::chrono::steady_clock::now() - start);
//    printf("%d\n",duration);


}


void MultiFrameListener::update_color(astra::Frame& frame)
{
    const astra::ColorFrame colorFrame = frame.get<astra::ColorFrame>();

    if (!colorFrame.is_valid())
    {

        return;
    }

    const int colorWidth = colorFrame.width();
    const int colorHeight = colorFrame.height();
//    qDebug()<<"Color frame readed";
//    qDebug()<<colorWidth<<" "<<colorHeight;
    colorBuff.resize(colorWidth*colorHeight*3);
//    const cv::Mat mImageRGB(colorHeight, colorWidth, CV_8UC3, (void*)colorFrame.data());
//    cv::Mat cImageBGR;
//    cv::cvtColor( mImageRGB, cImageBGR, CV_RGB2BGR );
//    cv::imshow( "Color Image", cImageBGR );
//    for(int i=0;i<colorWidth*colorHeight;i++) colorBuff.operator[](i)=(RoboCompRGBD::ColorRGB)colorFrame.data()[i];
    memcpy(&colorBuff[0],colorFrame.data(),colorWidth*colorHeight* sizeof(astra::RgbPixel));

//    const astra::RgbPixel* color = colorFrame.data();
//    uint8_t* buffer = &colorView_.buffer_[0];
//
//    for(int i = 0; i < colorWidth * colorHeight; i++)
//    {
//        const int rgbaOffset = i * 4;
//        buffer[rgbaOffset] = color[i].r;
//        buffer[rgbaOffset + 1] = color[i].g;
//        buffer[rgbaOffset + 2] = color[i].b;
//        buffer[rgbaOffset + 3] = 255;
//    }

}


void MultiFrameListener::update_point(astra::Frame &frame)
{
    const astra::PointFrame pointFrame = frame.get<astra::PointFrame>();

    if (!pointFrame.is_valid())
    {
        return;
    }

    int pointWidth = pointFrame.width();
    int pointHeight = pointFrame.height();
//    qDebug()<<"Points frame readed";
    pointBuff.resize(pointWidth*pointHeight);
    for(int i=0;i<pointWidth*pointHeight;i++)
    {
        pointBuff[i].x = pointFrame.data()[i].x;
        pointBuff[i].y = pointFrame.data()[i].y;
        pointBuff[i].z = pointFrame.data()[i].z;
    }
}

void MultiFrameListener::update_ir_16(astra::Frame& frame)
{
    const astra::InfraredFrame16 irFrame = frame.get<astra::InfraredFrame16>();

    if (!irFrame.is_valid())
    {
        return;
    }

    const int irWidth = irFrame.width();
    const int irHeight = irFrame.height();
//    qDebug()<<"IR16 frame readed";

//    const uint16_t* ir_values = irFrame.data();
//    uint8_t* buffer = &colorView_.buffer_[0];
//    for (int i = 0; i < irWidth * irHeight; i++)
//    {
//        const int rgbaOffset = i * 4;
//        const uint16_t value = ir_values[i];
//        const uint8_t red = static_cast<uint8_t>(value >> 2);
//        const uint8_t blue = 0x66 - red / 2;
//        buffer[rgbaOffset] = red;
//        buffer[rgbaOffset + 1] = 0;
//        buffer[rgbaOffset + 2] = blue;
//        buffer[rgbaOffset + 3] = 255;
//    }

}

void MultiFrameListener::update_ir_rgb(astra::Frame& frame)
{
    const astra::InfraredFrameRgb irFrame = frame.get<astra::InfraredFrameRgb>();

    if (!irFrame.is_valid())
    {
        return;
    }

    int irWidth = irFrame.width();
    int irHeight = irFrame.height();
//    qDebug()<<"Ir frame readed";

//    const astra::RgbPixel* irRGB = irFrame.data();
//    uint8_t* buffer = &colorView_.buffer_[0];
//    for (int i = 0; i < irWidth * irHeight; i++)
//    {
//        const int rgbaOffset = i * 4;
//        buffer[rgbaOffset] = irRGB[i].r;
//        buffer[rgbaOffset + 1] = irRGB[i].g;
//        buffer[rgbaOffset + 2] = irRGB[i].b;
//        buffer[rgbaOffset + 3] = 255;
//    }

}

void MultiFrameListener::on_frame_ready(astra::StreamReader& reader,
                            astra::Frame& frame)
{
    if (streamBools["depth"]) {
        update_depth(frame);
        depthBuff.swap();
    }
    if (streamBools["color"]) {
        update_color(frame);
        colorBuff.swap();
    }
    if (streamBools["ir"]) {
        update_ir_rgb(frame);
    }
    if (streamBools["point"]) {
        update_point(frame);
        pointBuff.swap();
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

void MultiFrameListener::get_depth(DepthSeq& depth)
{
    if (depthBuff.size==0) return;
    depthBuff.copy(depth);
//    const cv::Mat mImageDepth( 480, 640, CV_32F,&depth);
//    cv::imshow( "Depth Image", mImageDepth);
////    cv::Mat mScaledDepth;
////    mImageDepth.convertTo( mScaledDepth, CV_8U, 255.0 / 6000 );
////
////    cv::imshow( "Depth Image", mScaledDepth );
}

void MultiFrameListener::get_points(PointSeq& points)
{
    pointBuff.copy(points);
}
void MultiFrameListener::get_color(ColorSeq& colors)
{
    colorBuff.copy(colors);
}

void MultiFrameListener::get_color(imgType& colors)
{
    if (colorBuff.size==0) return;
    colorBuff.copy((ColorSeq&)colors);
    // Hack to fix a problem with the ColorSeq and imgType types of RGBD interface
    colors.resize(colors.size()/3);
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