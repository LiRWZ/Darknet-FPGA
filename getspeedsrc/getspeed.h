#pragma once
#include <opencv2\highgui.hpp>
#include <opencv2\imgproc.hpp>
#include <opencv2\calib3d.hpp>
#include "track.h"
#include <vector>

typedef struct userparam
{
	cv::Mat* img;
	cv::String* windowname;
	cv::Point* pt;
}userparam;


cv::Mat getHomography(cv::Mat& image);
void onMouse(int event, int x, int y, int flags, void *ustc);
float getSpeed(std::vector<tracker>& trackers, cv::Mat& H, const cv::Mat& image);