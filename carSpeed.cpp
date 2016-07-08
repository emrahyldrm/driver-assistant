#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "hardware.h"
#include <iostream>

using namespace cv;
using namespace std;

static void help()
{
	cout <<
		"\nThis program demonstrates dense optical flow algorithm by Gunnar Farneback\n"
		"Mainly the function: calcOpticalFlowFarneback()\n"
		"Call:\n"
		"./fback\n"
		"This reads from video camera 0\n" << endl;
}
static double drawOptFlowMap(const Mat& flow, Mat& cflowmap, int step,
	double, const Scalar& color)
{
	double totalMagnitude = 0;
	for (int y = 0; y < cflowmap.rows; y += step)
		for (int x = 0; x < cflowmap.cols; x += step)
		{
		const Point2f& fxy = flow.at<Point2f>(y, x);
		line(cflowmap, Point(x, y), Point(cvRound(x + fxy.x), cvRound(y + fxy.y)),
			color);
		totalMagnitude += sqrt(pow((x - cvRound(x + fxy.x)), 2) + pow((y - cvRound(y + fxy.y)), 2));
		circle(cflowmap, Point(x, y), 2, color, -1);
		}
	return totalMagnitude;
}
/*
int main(int, char**)
{

	VideoCapture cap("yol1.avi");
	help();
	if (!cap.isOpened()){
		cout << "Capture cannot be opened" << endl;		
		return -1;
	}

	hardwareSetup();	

	cap.set(CV_CAP_PROP_POS_MSEC, 6*60 * 1000);

	Mat prevgray, gray, flow, cflow, frame;
	//namedWindow("flow", 1);

	int allFrameTotalMagnitude = 0;
	int magnitudeArray[30];
	string speedText = "";

	Mat roi;
	for (int index = 0;; ++index)
	{
		cap >> frame;
		resize(frame, frame, Size(frame.size().width / 2, frame.size().height / 2));
		roi = frame.rowRange(0, 144).colRange(100, 360).clone();
		cvtColor(roi, gray, CV_BGR2GRAY);
		//imshow("roi", roi);

		if (prevgray.data)
		{
			calcOpticalFlowFarneback(prevgray, gray, flow, 0.5, 3, 15, 3, 5, 1.2, 0);
			cvtColor(prevgray, cflow, CV_GRAY2BGR);


			//cout << "magnitude" << drawOptFlowMap(flow, cflow, 16, 1.5, CV_RGB(0, 255, 0)) << endl;
			//cout << "index " << index << endl;

			allFrameTotalMagnitude += drawOptFlowMap(flow, cflow, 16, 1.5, CV_RGB(0, 255, 0));
			//cout << "allframeMagnitude" << allFrameTotalMagnitude << endl;
			//cout << allFrameTotalMagnitude / index << endl;



			if (index == 30){

				if (allFrameTotalMagnitude / index < 20)
					speedText = "Duruyor";
				else if (allFrameTotalMagnitude / index < 50)
					speedText = "Yavas";
				else if (allFrameTotalMagnitude / index < 100){
					reset();
					speedText = "Orta";
					ledStateChange(1);					
					}
				else if (allFrameTotalMagnitude / index < 200){
					reset();
					speedText = "Hizli";
					vibrationStateChange(1);					
				}
				else
					speedText = "Cok Hizli";
				if (index % 30 == 0)
				{
					index = 0;
					allFrameTotalMagnitude = 0;
				}
			}
			putText(frame, "speed: " + speedText, cv::Point(10, frame.rows - 20), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, Scalar(255, 255, 255), 1);

			imshow("frame", frame);
			//imshow("flow", cflow);

			//waitKey(0); 

		}
		if (waitKey(1) == 27)
			break;
		std::swap(prevgray, gray);
	}
	return 0;
}

*/
