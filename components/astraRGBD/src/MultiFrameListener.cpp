//
// Created by robolab on 23/07/18.
//

#include "MultiFrameListener.h"



MultiFrameListener::MultiFrameListener(astra::StreamReader& reader_)
{
    RGBMutex = new QMutex();
    depthMutex = new QMutex();
    streamBools["depth"]=false;
    streamBools["color"]=false;
    streamBools["ir"]=false;
    streamBools["points"]=false;
    streamBools["body"]=false;
    streamBools["hand"]=false;
    reader = &reader_;

    colorStream = new astra::ColorStream(reader->stream<astra::ColorStream>());
    depthStream = new astra::DepthStream(reader->stream<astra::DepthStream>());
    pointStream = new astra::PointStream(reader->stream<astra::PointStream>());
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
    qDebug()<<"Depth frame readed";

//    astra::RgbPixel* vizBuffer = visualizer_.get_output();
//    uint8_t* buffer = &depthView_.buffer_[0];
//    for (int i = 0; i < depthWidth * depthHeight; i++)
//    {
//        const int rgbaOffset = i * 4;
//        buffer[rgbaOffset] = vizBuffer[i].r;
//        buffer[rgbaOffset + 1] = vizBuffer[i].g;
//        buffer[rgbaOffset + 2] = vizBuffer[i].b;
//        buffer[rgbaOffset + 3] = 255;
//    }

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
    qDebug()<<"Color frame readed";


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

void MultiFrameListener::update_ir_16(astra::Frame& frame)
{
    const astra::InfraredFrame16 irFrame = frame.get<astra::InfraredFrame16>();

    if (!irFrame.is_valid())
    {
        return;
    }

    const int irWidth = irFrame.width();
    const int irHeight = irFrame.height();
    qDebug()<<"IR16 frame readed";

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

void MultiFrameListener::update_point(astra::Frame &frame)
{
    const astra::PointFrame pointFrame = frame.get<astra::PointFrame>();

    if (!pointFrame.is_valid())
    {
        return;
    }

    int irWidth = pointFrame.width();
    int irHeight = pointFrame.height();
    qDebug()<<"Points frame readed";

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
    qDebug()<<"Ir frame readed";

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
    }
    if (streamBools["color"]) {
        update_color(frame);
    }
    if (streamBools["ir"]) {
        update_ir_rgb(frame);
    }
    if (streamBools["point"]) {
        update_point(frame);
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