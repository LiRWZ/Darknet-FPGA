#include "getspeed.h"
#include <iostream>
#include <math.h>

using namespace std;
using namespace cv;

static float getX(const Point& p1, const Point& p2, int& y);
static void pointsCorrection(const Point* pts, const Mat& image, vector<Point2f>& newpts);

static float getX(const Point& p1, const Point& p2, int& y)
{
	return float((float(y) - float(p1.y)) * (float(p2.x) - float(p1.x)) / (float(p2.y) - float(p1.y)) + float(p1.x));
}

static void pointsCorrection(const Point* pts, const Mat& image, vector<Point2f>& newpts)
{
	int y1_ = (pts->y < (pts + 3)->y) ? pts->y : (pts + 3)->y;
	int y2_ = image.rows;
	cout << "y1_:" << y1_ << "  y2_:" << y2_ << endl;
	Point2f temp;
	temp.x = getX(*(pts), *(pts + 1), y1_);
	temp.y = float(y1_);
	newpts.push_back(temp);
	temp.x = getX(*(pts), *(pts + 1), y2_);
	temp.y = float(y2_);
	newpts.push_back(temp);
	temp.x = getX(*(pts + 2), *(pts + 3), y2_);
	temp.y = float(y2_);
	newpts.push_back(temp);
	temp.x = getX(*(pts + 2), *(pts + 3), y1_);
	temp.y = float(y1_);
	newpts.push_back(temp);
}


Mat getHomography(Mat& image)
{
	userparam usprm;
	usprm.img = &image;
	String windowname("mark the points");
	usprm.windowname = &windowname;
	usprm.pt = (Point*)malloc(sizeof(Point) * 4);
	namedWindow(windowname, WINDOW_AUTOSIZE);
	imshow(windowname, image);
	setMouseCallback(windowname, onMouse, &usprm);
	waitKey(0);
	destroyWindow(windowname);
	cout << "piont1: (" << (usprm.pt)->x << "," << (usprm.pt)->y << ")" << endl;
	cout << "piont2: (" << (usprm.pt + 1)->x << "," << (usprm.pt + 1)->y << ")" << endl;
	cout << "piont3: (" << (usprm.pt + 2)->x << "," << (usprm.pt + 2)->y << ")" << endl;
	cout << "piont4: (" << (usprm.pt + 3)->x << "," << (usprm.pt + 3)->y << ")" << endl;

	//根据两点式计算新的四个点
	vector<Point2f> corners1, corners2;
	pointsCorrection(usprm.pt, *usprm.img, corners1);
	Point2f temp;
	temp.x = 0; temp.y = 0;
	corners2.push_back(temp);
	temp.y = usprm.img->rows * 2;
	corners2.push_back(temp);
	temp.x = usprm.img->cols;
	corners2.push_back(temp);
	temp.y = 0;
	corners2.push_back(temp);

	Mat H = findHomography(corners1, corners2);
	Mat newPerspect;
	Size dsize;
	dsize.height = image.rows*2;
	dsize.width = image.cols;
	warpPerspective(*usprm.img, newPerspect, H, dsize);
	dsize.height = 0;
	dsize.width = 0;
	resize(newPerspect, newPerspect, dsize, 1, 0.9, INTER_CUBIC);
	imshow("warped", newPerspect);
	waitKey(0);
	destroyWindow("warped");
	return H;
}

//鼠标事件响应回调函数
void onMouse(int event, int x, int y, int flags, void *ustc)
{
	userparam * data = (userparam*)ustc;
	static int count = 0;
	Point str_pos, rect_pos1, rect_pos2;
	str_pos.x = 0; str_pos.y = 20;
	rect_pos1.x = 0; rect_pos1.y = 0;
	rect_pos2.x = 120; rect_pos2.y = 30;
	if (event == CV_EVENT_LBUTTONDOWN)
	{
		rectangle(*(data->img), rect_pos1, rect_pos2, Scalar(0, 0, 0), CV_FILLED, 8, 0);
		String str("point " + to_string(count + 1));
		putText(*(data->img), str, str_pos, FONT_HERSHEY_COMPLEX_SMALL, 1, Scalar(0, 0, 255), 1, 8, 0);
		imshow(*(data->windowname), *(data->img));
		((data->pt) + count)->x = x;
		((data->pt) + count)->y = y;
		count++;
		if (count == 4)
		{
			rectangle(*(data->img), rect_pos1, rect_pos2, Scalar(0, 0, 0), CV_FILLED, 8, 0);
			String str("ROGER!");
			putText(*(data->img), str, str_pos, FONT_HERSHEY_COMPLEX_SMALL, 1, Scalar(0, 0, 255), 1, 8, 0);
			line(*(data->img), *((data->pt) + 0), *((data->pt) + 1), Scalar(0, 255, 0), 1, LINE_AA);
			line(*(data->img), *((data->pt) + 1), *((data->pt) + 2), Scalar(0, 0, 255), 1, LINE_AA);
			line(*(data->img), *((data->pt) + 2), *((data->pt) + 3), Scalar(255, 0, 0), 1, LINE_AA);
			line(*(data->img), *((data->pt) + 3), *((data->pt) + 0), Scalar(255, 255, 0), 1, LINE_AA);
			imshow(*(data->windowname), *(data->img));
		}
	}
}

float getSpeed(vector<tracker>& trackers, Mat& H, const Mat& image)
{
	float averageSpeed = 0.0;
	float count = 0;
	int rows = image.rows;
	int cols = image.cols;
	Mat HH;
	HH.create(3, 3, CV_32FC1);
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			HH.at<float>(i, j) = H.at<double>(i, j);
	for (auto p = trackers.begin(); p != trackers.end(); p++)
	{
		int points_num = p->trace.size();
		if (points_num >= 12)
		{
			count += 1;
			Mat pointbegin,pointbegin_;
			pointbegin.create(3, 1, CV_32FC1);
			pointbegin.at<float>(0, 0) = p->trace[points_num - 12].x * cols;
			pointbegin.at<float>(1, 0) = p->trace[points_num - 12].y * rows;
			pointbegin.at<float>(2, 0) = 1.0;

			Mat pointend,pointend_;
			pointend.create(3, 1, CV_32FC1);
			pointend.at<float>(0, 0) = p->trace.back().x * cols;
			pointend.at<float>(1, 0) = p->trace.back().y * rows;
			pointend.at<float>(2, 0) = 1.0;
			pointbegin_ = HH * pointbegin;
			pointbegin_.at<float>(0, 0) = pointbegin_.at<float>(0, 0) / pointbegin_.at<float>(2, 0);
			pointbegin_.at<float>(1, 0) = pointbegin_.at<float>(1, 0) / pointbegin_.at<float>(2, 0);
			pointend_ = HH * pointend;
			pointend_.at<float>(0, 0) = pointend_.at<float>(0, 0) / pointend_.at<float>(2, 0);
			pointend_.at<float>(1, 0) = pointend_.at<float>(1, 0) / pointend_.at<float>(2, 0);

			//cout << "Pointbegin:" << pointbegin_ << endl;
			//cout << "Pointend" << pointend_ << endl;
			

			float pathlenth = sqrt(pow((pointend_.at<float>(0, 0) - pointbegin_.at<float>(0, 0)), 2) + pow((pointend_.at<float>(1, 0) - pointbegin_.at<float>(1, 0)), 2));
			p->speed = pathlenth * 2;
			averageSpeed = p->speed + averageSpeed;
		}
	}
	if (count == 0)
		return .0;
	else
	{
		averageSpeed = averageSpeed / count;
		return averageSpeed;
	}
	
}
