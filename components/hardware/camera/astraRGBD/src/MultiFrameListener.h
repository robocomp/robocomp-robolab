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
//#include <opencv2/opencv.hpp>
#include <mutex>
#include <chrono>
#include "RGBD.h"


class MultiFrameListener : public astra::FrameListener
{
	astra::StreamSet streamSet;
	astra::StreamReader *reader;


    ::std::map< ::std::string, bool> streamBools;
    astra::PointStream *pointStream;
    astra::DepthStream *depthStream;
    astra::ColorStream *colorStream;
    astra::BodyStream *bodyStream;

    astra::InfraredStream *irStream;
    astra::HandStream *handStream;

    DoubleBuffer<astra::PointFrame, RoboCompRGBD::imgType, PointStreamConverter> pointStreamBuff;
	DoubleBuffer<astra::PointFrame, RoboCompRGBD::PointSeq, PointSeqConverter> pointBuff;
    DoubleBuffer<astra::DepthFrame, RoboCompRGBD::DepthSeq, FloatSeqConverter> depthBuff;
    DoubleBuffer<astra::ColorFrame, RoboCompRGBD::ColorSeq, ColorSeqConverter> colorBuff;
    DoubleBuffer<astra::ColorFrame, RoboCompRGBD::imgType, ByteSeqConverter> colorBuff2;
    DoubleBuffer<astra::BodyFrame, RoboCompHumanTracker::PersonList, BodiesPeopleConverter> bodyBuff;
	DoubleBuffer<std::tuple<astra::ColorFrame&,astra::BodyFrame&, long int>, RoboCompHumanTrackerJointsAndRGB::MixedJointsRGB, BodyRGBConverter> bodyRGBMix;


    ByteSeqConverter *byteConverter;
    FloatSeqConverter *depthConverter;
    ColorSeqConverter *colorConverter;
    PointStreamConverter *pointStreamConverter;
	PointSeqConverter *pointConverter;
    BodiesPeopleConverter *bodiesConverter;
	BodyRGBConverter *bodyRgbConverter;

    RoboCompHumanTracker::PersonList bodylist;
    std::chrono::steady_clock::time_point end;
	RoboCompHumanTrackerJointsAndRGB::HumanTrackerJointsAndRGBPrx &pubproxy;
//    DoubleBuffer<RoboCompRGBD::PointSeq> pointBuff;
//    DoubleBuffer<RoboCompRGBD::DepthSeq> depthBuff;
//    DoubleBuffer<RoboCompRGBD::ColorSeq> colorBuff;
//    DoubleBuffer<RoboCompRGBD::DepthSeq> irBuff;


	std::map<int, RoboCompHumanTracker::jointListType> PersonDepth;
        int cameraID;
public:

    bool is_writting = false; //bandera

    MultiFrameListener(RoboCompHumanTrackerJointsAndRGB::HumanTrackerJointsAndRGBPrx &pubproxy, int cameraID);

    void update_depth(astra::Frame& frame);

    void update_color(astra::Frame& frame);

    void update_ir_16(astra::Frame& frame);
    void update_ir_rgb(astra::Frame& frame);
    void update_point(astra::Frame& frame);
    virtual void on_frame_ready(astra::StreamReader& reader, astra::Frame& frame) override;
    void set_color_stream(bool color_bool);
    void set_depth_stream(bool depth_bool);
    void set_point_stream(bool point_bool);
    void set_body_stream(bool body_bool);

    void get_depth(DepthSeq& depth);
    void get_points(PointSeq& points);
    void get_points_stream(imgType& pointStream);
    void get_color(ColorSeq& colors);
    void get_color(imgType& colors);
    void get_people(RoboCompHumanTracker::PersonList& people);
	RoboCompHumanTracker::joint getJointDepth(int idperson, string idjoint);


private:
	std::map<astra::JointType, ::std::string> JOINT2STRING ;

    mutable std::mutex my_mutex;
    astra::DepthStream configure_depth(astra::StreamReader& reader);

    astra::InfraredStream configure_ir(astra::StreamReader& reader, bool useRGB);

    astra::ColorStream configure_color(astra::StreamReader& reader);

    astra::PointStream configure_point(astra::StreamReader& reader);

    astra::BodyStream configure_body(astra::StreamReader& reader);
};


#endif //PROJECT_MULTIFRAMELISTENER_H
