#include "FindColor.h"


Object::Object()
{
	//set values for default constructor
	setType("Object");
	setColor(cv::Scalar(0, 0, 0));

}

Object::Object(string name) {

	setType(name);

	if (name == "blue") {

		//TODO: use "calibration mode" to find HSV min
		//and HSV max values

		setHSVmin(cv::Scalar(92, 0, 0));
		setHSVmax(cv::Scalar(124, 256, 256));

		//BGR value for Blue:
		setColor(cv::Scalar(255, 0, 0));

	}
	if (name == "green") {

		//TODO: use "calibration mode" to find HSV min
		//and HSV max values

		setHSVmin(cv::Scalar(34, 50, 50));
		setHSVmax(cv::Scalar(80, 220, 200));

		//BGR value for Green:
		setColor(cv::Scalar(0, 255, 0));

	}
	if (name == "yellow") {

		//TODO: use "calibration mode" to find HSV min
		//and HSV max values

		setHSVmin(cv::Scalar(20, 124, 123));
		setHSVmax(cv::Scalar(30, 256, 256));

		//BGR value for Yellow:
		setColor(cv::Scalar(0, 255, 255));

	}
	if (name == "red") {

		//TODO: use "calibration mode" to find HSV min
		//and HSV max values

		setHSVmin(cv::Scalar(0, 200, 0));
		setHSVmax(cv::Scalar(19, 255, 255));

		//BGR value for Red:
		setColor(cv::Scalar(0, 0, 255));

	}
}

Object::~Object(void)
{
}

int Object::getXPos() {

	return Object::xPos;

}

void Object::setXPos(int x) {

	Object::xPos = x;

}

int Object::getYPos() {

	return Object::yPos;

}

void Object::setYPos(int y) {

	Object::yPos = y;

}

cv::Scalar Object::getHSVmin() {

	return Object::HSVmin;

}
cv::Scalar Object::getHSVmax() {

	return Object::HSVmax;
}

void Object::setHSVmin(cv::Scalar min) {

	Object::HSVmin = min;
}


void Object::setHSVmax(cv::Scalar max) {

	Object::HSVmax = max;
}

void on_trackbar(int, void*)
{//This function gets called whenever a
 // trackbar position is changed

}

string Object::intToString(int number) {

	std::stringstream ss;
	ss << number;
	return ss.str();
}

void Object::createTrackbars() {

	//create window for trackbars
	cv::namedWindow(trackbarWindowName, 0);
	//create memory to store trackbar name on window
	char TrackbarName[50];
	sprintf_s(TrackbarName, "H_MIN", H_MIN);
	sprintf_s(TrackbarName, "H_MAX", H_MAX);
	sprintf_s(TrackbarName, "S_MIN", S_MIN);
	sprintf_s(TrackbarName, "S_MAX", S_MAX);
	sprintf_s(TrackbarName, "V_MIN", V_MIN);
	sprintf_s(TrackbarName, "V_MAX", V_MAX);
	//create trackbars and insert them into window
	//3 parameters are: the address of the variable that is changing when the trackbar is moved(eg.H_LOW),
	//the max value the trackbar can move (eg. H_HIGH),
	//and the function that is called whenever the trackbar is moved(eg. on_trackbar)
	//                                  ---->    ---->     ---->
	cv::createTrackbar("H_MIN", trackbarWindowName, &H_MIN_init, H_MAX, on_trackbar);
	cv::createTrackbar("H_MAX", trackbarWindowName, &H_MAX_init, H_MAX, on_trackbar);
	cv::createTrackbar("S_MIN", trackbarWindowName, &S_MIN_init, S_MAX, on_trackbar);
	cv::createTrackbar("S_MAX", trackbarWindowName, &S_MAX_init, S_MAX, on_trackbar);
	cv::createTrackbar("V_MIN", trackbarWindowName, &V_MIN_init, V_MAX, on_trackbar);
	cv::createTrackbar("V_MAX", trackbarWindowName, &V_MAX_init, V_MAX, on_trackbar);
}

void Object::position(int x, int y) {
	point.x = x;
	point.y = y;
}


void Object::drawObject(vector<Object> theObjects, cv::Mat &frame) {
	int array[2] = { 0, 0 };
	for (int i = 0; i<theObjects.size(); i++) {

		cv::circle(frame, cv::Point(theObjects.at(i).getXPos(), theObjects.at(i).getYPos()), 10, cv::Scalar(0, 0, 255));
		cv::putText(frame, intToString(theObjects.at(i).getXPos()) + " , " + intToString(theObjects.at(i).getYPos()), cv::Point(theObjects.at(i).getXPos(), theObjects.at(i).getYPos() + 20), 1, 1, cv::Scalar(0, 255, 0));
		cv::putText(frame, theObjects.at(i).getType(), cv::Point(theObjects.at(i).getXPos(), theObjects.at(i).getYPos() - 30), 1, 2, theObjects.at(i).getColor());
		position(theObjects.at(i).getXPos(), theObjects.at(i).getYPos());
	}
}

void Object::morphOps(cv::Mat &thresh) {

	//create structuring element that will be used to "dilate" and "erode" image.
	//the element chosen here is a 3px by 3px rectangle
	cv::Mat erodeElement = getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
	//dilate with larger element so make sure object is nicely visible
	cv::Mat dilateElement = getStructuringElement(cv::MORPH_RECT, cv::Size(8, 8));
	//«I»k
	erode(thresh, thresh, erodeElement);
	erode(thresh, thresh, erodeElement);
	//¿±µÈ
	dilate(thresh, thresh, dilateElement);
	dilate(thresh, thresh, dilateElement);
}
void Object::trackFilteredObject(cv::Mat threshold, cv::Mat HSV, cv::Mat &cameraFeed)
{
	vector <Object> objects;
	cv::Mat temp;
	threshold.copyTo(temp);
	//these two vectors needed for output of findContours
	vector< vector<cv::Point> > contours;
	vector<cv::Vec4i> hierarchy;
	//find contours of filtered image using openCV findContours function §ä½ü¹ø
	findContours(temp, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
	//use moments method to find our filtered object
	double refArea = 0;
	bool objectFound = false;
	if (hierarchy.size() > 0) {
		int numObjects = hierarchy.size();
		//if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
		if (numObjects<MAX_NUM_OBJECTS)
		{
			for (int index = 0; index >= 0; index = hierarchy[index][0])
			{
				cv::Moments moment = moments((cv::Mat)contours[index]);
				double area = moment.m00;
				//if the area is less than 20 px by 20px then it is probably just noise
				//if the area is the same as the 3/2 of the image size, probably just a bad filter
				//we only want the object with the largest area so we safe a reference area each
				//iteration and compare it to the area in the next iteration.
				if (area>MIN_OBJECT_AREA)
				{
					Object object;

					object.setXPos(moment.m10 / area);
					object.setYPos(moment.m01 / area);

					objects.push_back(object);

					objectFound = true;

				}
				else objectFound = false;
			}
			//let user know you found an object
			if (objectFound == true)
			{
				//draw object location on screen
				drawObject(objects, cameraFeed);
			}
		}
		else putText(cameraFeed, "TOO MUCH NOISE! ADJUST FILTER", cv::Point(0, 50), 1, 2, cv::Scalar(0, 0, 255), 2);
	}
}



void Object::mainProgram(cv::Mat cameraFeed) {

	//Matrix to store each frame of the webcam feed
	cv::Mat threshold;
	cv::Mat HSV;

	//convert frame from BGR to HSV colorspace
	cvtColor(cameraFeed, HSV, cv::COLOR_BGR2HSV);

	//need to find the appropriate color range values
	// calibrationMode must be false

	//if in calibration mode, we track objects based on the HSV slider values.
	cvtColor(cameraFeed, HSV, cv::COLOR_BGR2HSV);
	inRange(HSV, cv::Scalar(H_MIN_init, S_MIN_init, V_MIN_init), cv::Scalar(H_MAX_init, S_MAX_init, V_MAX_init), threshold);
	morphOps(threshold);
	imshow(windowName2, threshold);

	//the folowing for canny edge detec
	
	/// Convert the image to grayscale
	cvtColor(cameraFeed, src_gray, CV_BGR2GRAY);
	/// Create a window
	cv::namedWindow(window_name, CV_WINDOW_AUTOSIZE);
	/// Create a Trackbar for user to enter threshold
	cv::createTrackbar("Min Threshold:", window_name, &lowThreshold, max_lowThreshold);
	/// Show the image
	trackFilteredObject(threshold, HSV, cameraFeed);

	//show frames
	//imshow(windowName2,threshold);

	imshow(windowName, cameraFeed);
	//imshow(windowName1,HSV);

	//delay 30ms so that screen can refresh.
	//image will not appear without this waitKey() command
}