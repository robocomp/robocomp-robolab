//
// Created by robolab on 23/07/18.
//

#ifndef PROJECT_MULTIFRAMELISTENER_H
#define PROJECT_MULTIFRAMELISTENER_H

#include <astra/astra.hpp>
#include <cstdio>
#include <iostream>
#include <QMutex>
#include <map>
#include <genericworker.h>
#include <doublebuffer/DoubleBuffer.h>
#include <DoubleBufferConverters.h>
#include <opencv2/opencv.hpp>
#include <mutex>
#include <chrono>

class MultiFrameListener : public astra::FrameListener
{
    ::std::map< ::std::string, bool> streamBools;
    astra::StreamReader *reader;
    astra::PointStream *pointStream;
    astra::DepthStream *depthStream;
    astra::ColorStream *colorStream;
    astra::InfraredStream *irStream;
    astra::BodyStream *bodyStream;
    astra::HandStream *handStream;
    DoubleBuffer<astra::PointFrame, RoboCompRGBD::PointSeq, ByteSeqConverter> pointBuff;
    DoubleBuffer<astra::DepthFrame, RoboCompRGBD::DepthSeq, FloatSeqConverter> depthBuff;
    DoubleBuffer<astra::ColorFrame, RoboCompRGBD::ColorSeq, ByteSeqConverter> colorBuff;
    DoubleBuffer<astra::ColorFrame, RoboCompRGBD::imgType, ByteSeqConverter> colorBuff2;
    ByteSeqConverter byteConverter;
    FloatSeqConverter depthConverter;
    std::chrono::steady_clock::time_point end;
//    DoubleBuffer<RoboCompRGBD::PointSeq> pointBuff;
//    DoubleBuffer<RoboCompRGBD::DepthSeq> depthBuff;
//    DoubleBuffer<RoboCompRGBD::ColorSeq> colorBuff;
//    DoubleBuffer<RoboCompRGBD::DepthSeq> irBuff;



public:


    MultiFrameListener(astra::StreamReader& reader_);

    void update_depth(astra::Frame& frame);

    void update_color(astra::Frame& frame);

    void update_ir_16(astra::Frame& frame);
    void update_ir_rgb(astra::Frame& frame);
    void update_point(astra::Frame& frame);
    virtual void on_frame_ready(astra::StreamReader& reader, astra::Frame& frame) override;
    void set_color_stream(bool color_bool);
    void set_depth_stream(bool depth_bool);
    void set_point_stream(bool point_bool);
    void get_depth(DepthSeq& depth);
    void get_points(PointSeq& points);
    void get_color(ColorSeq& colors);
    void get_color(imgType& colors);

private:
    mutable std::mutex my_mutex;
    astra::DepthStream configure_depth(astra::StreamReader& reader);

    astra::InfraredStream configure_ir(astra::StreamReader& reader, bool useRGB);

    astra::ColorStream configure_color(astra::StreamReader& reader);

    astra::PointStream configure_point(astra::StreamReader& reader);
};


#endif //PROJECT_MULTIFRAMELISTENER_H
