#include "srRecorder.h"
#include <GL/glew.h>
#include <opencv2/opencv.hpp>
#include <cvaux.h>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <Configuration.h>

srRecorder* srRecorder::GetRecorder(){
  static srRecorder recorder_;
  return &recorder_;
}
srRecorder::srRecorder():
  is_first_call_(true),
  writer_(NULL),
  img_width_(1280), img_height_(720)
{}
srRecorder::~srRecorder(){
  cvReleaseVideoWriter(&writer_);
}

void srRecorder::CaptureCurrentScreen(int fps){
  if(is_first_call_){
    _InitiateSetup(fps);
    is_first_call_ = false;
  }
  unsigned char *raw_image = (unsigned char*) calloc(img_width_ * img_height_ * 4, sizeof(char));
  glReadBuffer(GL_FRONT);
  glReadPixels(0, 0, img_width_, img_height_, GL_RGBA, GL_UNSIGNED_BYTE, raw_image);
  IplImage* img = cvCreateImage(cvSize(img_width_, img_height_), IPL_DEPTH_8U, 4);

  img->imageData = (char *)raw_image;
  cvFlip(img,img,0);

  cvWriteFrame(writer_, img);      // add the frame to the file
  cvReleaseImage(&img);
}

void srRecorder::_InitiateSetup(int fps){
  remove(THIS_COM"experiment_data/video_output.avi");
  writer_ = cvCreateVideoWriter(THIS_COM"experiment_data/video_output.avi", CV_FOURCC('X', '2', '6', '4'), fps, cvSize(img_width_, img_height_), 1);

}

