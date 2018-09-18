#pragma once
#include <string>
#include <iostream>
#include <map>
#include <opencv2\core.hpp>
#include <opencv2\imgproc.hpp>
#include <opencv2\highgui.hpp>

//using namespace cv;
using namespace std;

class Object
{
public:
	Object();
	~Object(void);

	Object(string name);

	int getXPos();
	void setXPos(int x);

	int getYPos();
	void setYPos(int y);

	cv::Scalar getHSVmin();
	cv::Scalar getHSVmax();

	void setHSVmin(cv::Scalar min);
	void setHSVmax(cv::Scalar max);

	//void on_trackbar(int, void*);
	string intToString(int number);
	void createTrackbars();
	void position(int x, int y);
	void LRposition(int MaxX, int MaxY, int MinX, int MinY);
	void drawObject(vector<Object> theObjects, cv::Mat &frame);
	void morphOps(cv::Mat &thresh);
	void trackFilteredObject(cv::Mat threshold, cv::Mat HSV, cv::Mat &cameraFeed);
	void mainProgram(cv::Mat cameraFeed);


	string getType() { return type; }
	void setType(string t) { type = t; }

	cv::Scalar getColor() {
		return Color;
	}

	std::map<string, cv::Point> GetPointTable();

	void setColor(cv::Scalar c) {

		Color = c;
	}

	//initial min and max HSV filter values.
	//these will be changed using trackbars
	int H_MIN = 0;
	int H_MAX = 256;
	int S_MIN = 0;
	int S_MAX = 256;
	int V_MIN = 0;
	int V_MAX = 256;

	int H_MIN_init = 0;
	int H_MAX_init = 16;
	int S_MIN_init = 160;
	int S_MAX_init = 216;
	int V_MIN_init = 135;
	int V_MAX_init = 210;

	//default capture width and height
	const int FRAME_WIDTH = 640;
	const int FRAME_HEIGHT = 480;

private:
	
	int xPos, yPos;
	string type;
	cv::Scalar HSVmin, HSVmax;
	cv::Scalar Color;

	std::map<string, cv::Point> PointTable;

	//max number of objects to be detected in frame
	const int MAX_NUM_OBJECTS = 50;
	//minimum and maximum object area
	const int MIN_OBJECT_AREA = 20 * 20;
	const int MAX_OBJECT_AREA = FRAME_HEIGHT * FRAME_WIDTH / 1.5;
	//names that will appear at the top of each window
	const string windowName = "Original Image";
	const string windowName1 = "HSV Image";
	const string windowName2 = "Thresholded Image";
	const string windowName3 = "After Morphological Operations";
	const string trackbarWindowName = "Trackbars";

	//The following for canny edge detec
	cv::Mat detected_edges;
	cv::Mat src, src_gray;
	int edgeThresh = 1;
	int lowThreshold;
	int const max_lowThreshold = 100;
	int ratio = 3;
	int kernel_size = 3;
	const char* window_name = "Edge Map";
};
