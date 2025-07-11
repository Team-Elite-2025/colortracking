#include "opencv2/opencv.hpp"
#include "depthai/depthai.hpp"
#include <iostream>
#include "json/json.h"

using namespace cv;
using namespace std;


    
// const int max_value_H = 360/2;
const int max_value = 255;
const String window_capture_name = "Video Capture";
const String window_detection_name = "Object Detection";
int low_L = 0;
int low_A = 0;
int low_B = 0;
int high_L = max_value; 
int high_A = max_value;
int high_B = max_value;

static void on_low_L_thresh_trackbar(int, void *)
{
 low_L = min(high_L-1, low_L);
 setTrackbarPos("Low L", window_detection_name, low_L);
}
static void on_high_L_thresh_trackbar(int, void *)
{
high_L = max(high_L, low_L+1);
 setTrackbarPos("High L", window_detection_name, high_L);
}
static void on_low_A_thresh_trackbar(int, void *)
{
 low_A = min(high_A-1, low_A);
 setTrackbarPos("Low A", window_detection_name, low_A);
}
static void on_high_A_thresh_trackbar(int, void *)
{
 high_A = max(high_A, low_A+1);
 setTrackbarPos("High A", window_detection_name, high_A);
}
static void on_low_B_thresh_trackbar(int, void *)
{
 low_B = min(high_B-1, low_B);
 setTrackbarPos("Low B", window_detection_name, low_B);
}
static void on_high_B_thresh_trackbar(int, void *)
{
 high_B = max(high_B, low_B+1);
 setTrackbarPos("High B", window_detection_name, high_B);
}
int main(int argc, char* argv[])
{

    // File Reading
    ifstream thresholds("thresholds.json");
    if (!thresholds.is_open()) {
        // print error message and return
        cerr << "Failed to read tresholds: thresholds.json" << endl;

        return 1;
    }
    Json::Value readThresholds;
    thresholds >> readThresholds;
    thresholds.close();

    
    // Create pipeline
    dai::Pipeline pipeline;
    dai::ImageManipConfig cfg;
    pipeline.setXLinkChunkSize(0);

    // Define source and output
    auto camRgb = pipeline.create<dai::node::ColorCamera>();
    auto xoutVideo = pipeline.create<dai::node::XLinkOut>();
    auto configIn = pipeline.create<dai::node::XLinkIn>();
    
   cfg.setCropRect(readThresholds["crop"]["x1"].asFloat(), readThresholds["crop"]["y1"].asFloat(), 0, 0);
    camRgb->initialControl.setManualFocus(readThresholds["focus"].asInt());
    camRgb->initialControl.setManualWhiteBalance(readThresholds["wb"].asInt());
    camRgb->initialControl.setManualExposure(readThresholds["exposure"]["time"].asInt(), readThresholds["exposure"]["sens"].asInt());
    camRgb.get()->initialControl.setContrast(readThresholds["contrast"].asInt());
    camRgb.get()->initialControl.setSaturation(readThresholds["saturation"].asInt());
    camRgb.get()->initialControl.setSharpness(readThresholds["sharpness"].asInt());

    xoutVideo->setStreamName("video");
    configIn->setStreamName("config");

    // Properties
    camRgb->setBoardSocket(dai::CameraBoardSocket::CAM_A);
    camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
        //camRgb->setVideoSize(1920,1080);
    camRgb->setVideoSize(readThresholds["crop"]["width"].asInt(), readThresholds["crop"]["height"].asInt());
    camRgb->setFps(35);



    xoutVideo->input.setBlocking(false);
    xoutVideo->input.setQueueSize(1);

    // Linking
    camRgb->video.link(xoutVideo->input);
    configIn->out.link(camRgb->inputConfig);




    
    
    // Connect to device and start pipeline
    dai::Device device(pipeline, dai::UsbSpeed::SUPER_PLUS);

    auto video = device.getOutputQueue("video", 1, false);
    auto configQueue = device.getInputQueue("config");

    configQueue->send(cfg);
 VideoCapture cap(argc > 1 ? atoi(argv[1]) : 0);
 namedWindow(window_capture_name, WINDOW_NORMAL);
 namedWindow(window_detection_name, WINDOW_NORMAL);
 resizeWindow(window_capture_name, Size(1280,600));
  resizeWindow(window_detection_name, Size(1280,600));

  Mat mask = Mat::zeros(Size(655, 600), CV_8UC1);
    ellipse(mask,Point(readThresholds["mask1"]["x"].asInt(), readThresholds["mask1"]["y"].asInt()), Size(readThresholds["mask1"]["size1"].asInt(), readThresholds["mask1"]["size2"].asInt()), 0, 0, 360, Scalar(255), -1);

 // Trackbars to set thresholds for HSV values
 createTrackbar("Low L", window_detection_name, &low_L, max_value, on_low_L_thresh_trackbar);
 createTrackbar("High L", window_detection_name, &high_L, max_value, on_high_L_thresh_trackbar);
 createTrackbar("Low A", window_detection_name, &low_A, max_value, on_low_A_thresh_trackbar);
 createTrackbar("High A", window_detection_name, &high_A, max_value, on_high_A_thresh_trackbar);
 createTrackbar("Low B", window_detection_name, &low_B, max_value, on_low_B_thresh_trackbar);
 createTrackbar("High B", window_detection_name, &high_B, max_value, on_high_B_thresh_trackbar);
 Mat frame, frame_HSV, frame_threshold;
 while (true) {
    Mat masked;
    auto videoIn = video->get<dai::ImgFrame>();
    frame = videoIn->getCvFrame();
    
    if(frame.empty())
    {
    break;
    }
    resize(frame, frame, Size(655, 600), cv::INTER_AREA);    
    copyTo(frame, masked, mask);
    cvtColor(masked, frame_HSV, COLOR_BGR2Lab);
    // Detect the object based on HSV Range Values
    inRange(masked, Scalar(low_B, low_A, low_L), Scalar(high_B, high_A, high_L), frame_threshold);
    // Show the frames

    imshow(window_capture_name, masked);
    imshow(window_detection_name, frame_threshold);
    char key = (char) waitKey(30);
    if (key == 'q' || key == 27)
    {
    break;
    }
 }
 return 0;
}
