#include "KinectV2.h"
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

#define PI 3.14159

KinectV2::KinectV2(){
	m_DepthHeight = 0;
	m_DepthWidth = 0;
	m_ColorHeight = 0;
	m_ColorWidth = 0;
	m_IRWidth = 0;
	m_IRHeight = 0;

	m_DepthBuffer = nullptr;
	m_KinectSensor = nullptr;
	m_DepthReader = nullptr;
	m_ColorReader = nullptr;
	m_IRReader = nullptr;
	m_Mapper = nullptr;

	m_ToggleThreadDepth = true;
}


KinectV2::~KinectV2()
{

	m_DepthReader->Release();
	m_ColorReader->Release();

	m_Mapper->Release();
	// Close Sensor
	cout << "close sensor" << endl;
	m_KinectSensor->Close();

	// Release Sensor
	cout << "Release sensor" << endl;
	m_KinectSensor->Release();

	m_ToggleThreadDepth = false;
}


//Kinect 初始化
bool KinectV2::Init() {

	cout << "Try to get default sensor" << endl;

	HRESULT hr;

	hr = GetDefaultKinectSensor(&m_KinectSensor);
	if (FAILED(hr)) {
		cerr << "Get Kinect sensor failed!" << endl;
		return false;
	}

	cout << "Try to open sensor" << endl;

	hr = m_KinectSensor->Open();
	if (FAILED(hr)) {
		cerr << "Can't open sensor" << endl;
		return false;
	}
		
	if (SUCCEEDED(hr)) {

		
		IDepthFrameSource* m_DepthSource;
		hr = m_KinectSensor->get_DepthFrameSource(&m_DepthSource);
		if (SUCCEEDED(hr)) {

			hr = m_DepthSource->OpenReader(&m_DepthReader);
			if (FAILED(hr)) {
				std::cout << "Open Depth Reader Failed" << std::endl;
				return false;
			}
		}
		else {
			std::cout << "Get Depth Source Failed" << std::endl;
			return false;
		}

		IFrameDescription* m_DepthDescription = nullptr;
		hr = m_DepthSource->get_FrameDescription(&m_DepthDescription);
		if (SUCCEEDED(hr)) {
			m_DepthDescription->get_Width(&m_DepthWidth);
			m_DepthDescription->get_Height(&m_DepthHeight);
			m_DepthSource->get_DepthMinReliableDistance(&mDepthMin);
			m_DepthSource->get_DepthMaxReliableDistance(&mDepthMax);
			m_DepthDescription->Release();
			m_DepthDescription = nullptr;
			m_DepthSource->Release();
			
			m_DepthBufferSize = m_DepthWidth * m_DepthHeight;
			mDepthImg.create(m_DepthHeight, m_DepthWidth, CV_16UC1);
		}
		else {
			std::cout << "Get Depth Frame Description Failed" << std::endl;
			return false;
		}

		IColorFrameSource* m_ColorSource;
		hr = m_KinectSensor->get_ColorFrameSource(&m_ColorSource);
		if (SUCCEEDED(hr)) {
			hr = m_ColorSource->OpenReader(&m_ColorReader);
			if (FAILED(hr)) {
				std::cout << "Open Color Reader Failed" << std::endl;
				return false;
			}
		}
		else{
			std::cout << "Get Color Source Failed" << std::endl;
			return false;
		}

		IFrameDescription* m_ColorDescription;
		hr = m_ColorSource->get_FrameDescription(&m_ColorDescription);
		if (SUCCEEDED(hr)) {
			m_ColorDescription->get_Width(&m_ColorWidth);
			m_ColorDescription->get_Height(&m_ColorHeight);
			m_ColorBufferSize = m_ColorWidth * m_ColorHeight*4;
			m_ColorDescription->Release();
			m_ColorSource->Release();

			mColorImg.create(m_ColorHeight, m_ColorWidth, CV_8UC4);
		}
		else{
			std::cout << "Get Color Frame Description Failed" << std::endl;
			return false;
		}

		IInfraredFrameSource* m_IRSource = nullptr;
		hr = m_KinectSensor->get_InfraredFrameSource(&m_IRSource);
		if (SUCCEEDED(hr)) {
			hr = m_IRSource->OpenReader(&m_IRReader);
			if (FAILED(hr)) {
				std::cout << "Open Infrared Reader Failed" << std::endl;
				return false;
			}
		}
		else {
			std::cout << "Get Infrared Source Failed" << std::endl;
			return false;
		}

		IFrameDescription* m_IRDescription;
		hr = m_IRSource->get_FrameDescription(&m_IRDescription);
		if (SUCCEEDED(hr)) {
			m_IRDescription->get_Width(&m_IRWidth);
			m_IRDescription->get_Height(&m_IRHeight);
			m_IRDescription->Release();
			m_IRSource->Release();

			//mInfraredImg.create(m_InfraredHeight, m_InfraredWidth, CV_16UC1);
		}
		else {
			std::cout << "Get Infrared Frame Description Failed" << std::endl;
			return false;
		}
		hr = m_KinectSensor->get_CoordinateMapper(&m_Mapper);
		if (FAILED(hr)) {
			std::cout << "Open Mapper Failed" << std::endl;
			return false;
		}

	}

	//m_ThreadDepth = std::thread(&KinectV2::DepthUpdate, this);
	
	//m_ThreadDepth.join();
	return true;
}

//Get DepthData
void KinectV2::DepthUpdate() {
	
	IDepthFrame* m_DepthFrame;
		
	if (m_DepthReader->AcquireLatestFrame(&m_DepthFrame) == S_OK){

		m_DepthFrame->AccessUnderlyingBuffer(&m_DepthBufferSize, &m_DepthBuffer);

		DepthImg8UC4();
		//copy the depth map to image
		m_DepthFrame->CopyFrameDataToArray(m_DepthWidth * m_DepthHeight,
			reinterpret_cast<UINT16*>(mDepthImg.data));

		//release frame
		m_DepthFrame->Release();
	}
	
}

//Get ColorData
void KinectV2::ColorUpdate() {

	IColorFrame* m_ColorFrame = nullptr;	

	if (m_ColorReader->AcquireLatestFrame(&m_ColorFrame) == S_OK) {

		m_ColorFrame->CopyConvertedFrameDataToArray(
			m_ColorBufferSize, mColorImg.data, ColorImageFormat_Bgra);

		m_ColorFrame->Release();		
	}	
}


//Get IRData
void KinectV2::IRUpdate() {
	IInfraredFrame* m_IRFrame = nullptr;
	UINT IRBufferSize = 0;
	UINT16* IRBuffer = nullptr;
	mIRImg.create(m_IRHeight, m_IRWidth, CV_16UC1);
	if (m_IRReader->AcquireLatestFrame(&m_IRFrame) == S_OK) {
		m_IRFrame->AccessUnderlyingBuffer(&IRBufferSize, &IRBuffer);

		m_IRFrame->CopyFrameDataToArray(IRBufferSize,
			reinterpret_cast<UINT16*>(mIRImg.data));
		
		m_IRFrame->Release();
	}
}

//get Depth  8UC4 Image
void KinectV2::DepthImg8UC4() {
	int width = m_DepthWidth;
	int height = m_DepthHeight;

	//cv::Mat image(height, width, CV_8UC4);
	depth8UC4.create(m_DepthHeight, m_DepthWidth, CV_8UC4);

	for (int y = 0; y < height; y++) {
		int heightOffset = y * width;
		for (int x = 0; x < width; x++) {
			int index = (x + heightOffset);

			float distance = m_DepthBuffer[index];

			float dep = distance / 10;
			depth8UC4.at<Vec4b>(y, x) = Vec4b(dep, dep, dep, 255);
			/*depth8UC4.at<Vec4b>(y, x)[0] = dep;
			depth8UC4.at<Vec4b>(y, x)[1] = dep;
			depth8UC4.at<Vec4b>(y, x)[2] = dep;
			depth8UC4.at<Vec4b>(y, x)[3] = 255;*/
		}
	}
}


void KinectV2::ColorFrameToDepthSpace() {
	DepthSpacePoint * output = new DepthSpacePoint[m_ColorBufferSize/4];
	mMapImg.create(m_ColorHeight, m_ColorWidth, CV_8UC4);
	if (m_Mapper->MapColorFrameToDepthSpace(m_DepthBufferSize, reinterpret_cast<UINT16*>(mDepthImg.data), 
		m_ColorBufferSize/4, output) == S_OK)
	{

		for (int y = 0; y < m_ColorHeight; ++y)
			for (int x = 0; x < m_ColorWidth; ++x)
			{
				DepthSpacePoint tPoint = output[y * m_ColorWidth + x];    
				if (tPoint.X >= 0 && tPoint.X < m_DepthWidth && 
					tPoint.Y >= 0 && tPoint.Y < m_DepthHeight)  {

					mMapImg.at<Vec4b>(y, x) = mColorImg.at<Vec4b>(y, x);
				}
			}
	}
	delete[] output;
}

void KinectV2::DepthFrameToColorSpace() {
	ColorSpacePoint* output = new ColorSpacePoint[m_DepthBufferSize];
	HRESULT hr = m_Mapper->MapDepthFrameToColorSpace(m_DepthBufferSize, reinterpret_cast<UINT16*>(mDepthImg.data), m_DepthBufferSize, output);
	std::vector<BYTE> buffer(m_DepthBufferSize * 4);
	mMapImg.create(m_DepthHeight, m_DepthWidth, CV_8UC4);
	if (SUCCEEDED(hr)) {
		for (int y = 0; y < m_DepthHeight; ++y) {
			unsigned int depthOffset = y * m_DepthWidth;

			for (int x = 0; x < m_DepthWidth; ++x)
			{
				unsigned int depthIndex = (depthOffset + x)*4;
				ColorSpacePoint tPoint = output[y * m_DepthWidth + x];
				if (tPoint.X >= 0 && tPoint.X < m_ColorWidth &&
					tPoint.Y >= 0 && tPoint.Y < m_ColorHeight) {
					const unsigned int colorIndex = (tPoint.Y * m_ColorWidth + tPoint.X) * 4;
					mMapImg.data[depthIndex + 0] = mColorImg.data[colorIndex + 0];
					mMapImg.data[depthIndex + 1] = mColorImg.data[colorIndex + 1];
					mMapImg.data[depthIndex + 2] = mColorImg.data[colorIndex + 2];
					mMapImg.data[depthIndex + 3] = 255;
					//mMapImg.at<Vec4b>(y, x) = mColorImg.at<Vec4b>(y, x);
				}
			}

		}
		//mMapImg = cv::Mat(m_ColorHeight, m_ColorWidth, CV_8UC4, &buffer[0]);
		imshow("123", mMapImg);
	}

	delete[] output;
}

void KinectV2::DepthFrameToCameraSpace() {
	int size = mDepthImg.rows * mDepthImg.cols;
	Size nSize = mDepthImg.size();
	CameraSpacePoint* cameraPoints = new CameraSpacePoint[size];

	vector<Mat> output(3);
		
	output[0] = Mat(nSize, CV_32F);
	output[1] = Mat(nSize, CV_32F);
	output[2] = Mat(nSize, CV_32F);

	//mPointCloud.create(mDepthImg.size(), CV_32FC3);

	UINT pixelNum = mDepthImg.rows * mDepthImg.cols;
	if (m_Mapper->MapDepthFrameToCameraSpace(m_DepthBufferSize, reinterpret_cast<UINT16*>(mDepthImg.data), pixelNum, cameraPoints) == S_OK) {
		for (int y = 0; y < mDepthImg.rows; y++) {
			for (int x = 0; x < mDepthImg.cols; x++) {
				//if 
				if (isinf(cameraPoints[y * mDepthImg.rows + x].X)) {
					output[0].at<float>(y, x) = 0;
				}
				else {
					output[0].at<float>(y, x) = cameraPoints[y * mDepthImg.cols + x].X * 100;
				}
				
				if (isinf(cameraPoints[y * mDepthImg.rows + x].Y)) {
					output[1].at<float>(y, x) = 0;
				}
				else {
					output[1].at<float>(y, x) = cameraPoints[y * mDepthImg.cols + x].Y * 100;
				}

				if (isinf(cameraPoints[y * mDepthImg.rows + x].Z)) {
					output[2].at<float>(y, x) = 0;
				}
				else {
					output[2].at<float>(y, x) = cameraPoints[y * mDepthImg.cols + x].Z * 100;
				}
			}
		}
	}

	merge(output, mPointCloud);
}

void KinectV2::ColorFrameToCameraSpace() {
	int size = mColorImg.rows * mColorImg.cols;
	Size nSize = mColorImg.size();
	CameraSpacePoint* cameraPoint = new CameraSpacePoint[size];

	vector<Mat> output(3);
	output[0] = Mat(nSize, CV_32F);
	output[1] = Mat(nSize, CV_32F);
	output[2] = Mat(nSize, CV_32F);

	UINT pixelNum = mColorImg.rows * mColorImg.cols;
	if (m_Mapper->MapColorFrameToCameraSpace(m_DepthBufferSize, reinterpret_cast<UINT16*>(mDepthImg.data), pixelNum, cameraPoint) == S_OK) {
		for (int y = 0; y < mColorImg.rows; y++) {
			for (int x = 0; x < mColorImg.cols; x++) {
				int idx = x + y * m_ColorWidth;
				CameraSpacePoint &cPoints = cameraPoint[idx];

				if (isinf(cameraPoint[y * mColorImg.rows + x].X)) {
					output[0].at<float>(y, x) = 0;
				}
				else {
					output[0].at<float>(y, x) = cameraPoint[y * mColorImg.cols + x].X * 100;
				}

				if (isinf(cameraPoint[y * mColorImg.rows + x].Y)) {
					output[1].at<float>(y, x) = 0;
				}
				else {
					output[1].at<float>(y, x) = cameraPoint[y * mColorImg.cols + x].Y * 100;
				}

				if (isinf(cameraPoint[y * mColorImg.rows + x].Z)) {
					output[2].at<float>(y, x) = 0;
				}
				else {
					output[2].at<float>(y, x) = cameraPoint[y * mColorImg.cols + x].Z * 100;
				}
			}
		}
	}
	merge(output, mPointCloud);
}


void KinectV2::GetArmAngle(cv::Point3d Apos, cv::Point3d Bpos, cv::Point3d Cpos) {

	float ax = Apos.x, ay = Apos.y, az = Apos.z;
	float bx = Bpos.x, by = Bpos.y, bz = Bpos.z;
	float cx = Cpos.x, cy = Cpos.y, cz = Cpos.z;
	std::cout << "A: " << ax << "," << ay << "," << az << std::endl;
	std::cout << "A: " << bx << "," << by << "," << bz << std::endl;
	std::cout << "A: " << cx << "," << cy << "," << cz << std::endl;
	float abx = bx - ax, aby = by - ay, abz = bz - az;
	float acx = cx - ax, acy = cy - ay, acz = cz - az;

	int normalx = aby * acz - abz * acy;
	int normaly = abz * acx - abx * acz;
	int normalz = abx * acy - aby * acx;
	std::cout << normalx << "," << normaly << "," << normalz << std::endl;
	float R = atan2(normalz, sqrt(normalx * normalx + normaly * normaly)) * 180 / PI;
	float phi = atan2(normaly, normalx) * 180 / PI;

	std::cout << "R角: " << R << " , " << "phi角: " << phi << std::endl;

	ArmCode[0] = R;
	ArmCode[1] = phi;
	ArmCode[2] = Apos.x;
	ArmCode[3] = Apos.y;
	ArmCode[4] = Apos.z;
}
//get World position
void KinectV2::WorldPosition(int x, int y) {
	int size = mColorImg.rows * mColorImg.cols;
	CameraSpacePoint* cameraPoint = new CameraSpacePoint[size];

	cv::Point3f CamearaPos;

	UINT pixelNum = mColorImg.rows * mColorImg.cols;
	if (m_Mapper->MapColorFrameToCameraSpace(m_DepthBufferSize, reinterpret_cast<UINT16*>(mDepthImg.data), pixelNum, cameraPoint) == S_OK) {
		
		if (isinf(cameraPoint[y * mColorImg.rows + x].X)) {
			if (isinf(cameraPoint[(y - 1) * mColorImg.cols + x].X)) {
				CamearaPos.x = cameraPoint[(y - 1) * mColorImg.cols + x].X * 100;
				CamearaPos.y = cameraPoint[(y - 1) * mColorImg.cols + x].Y * 100;
				CamearaPos.z = cameraPoint[(y - 1) * mColorImg.cols + x].Z * 100;
			}
			else {
				CamearaPos.x = cameraPoint[(y + 1) * mColorImg.cols + x].X * 100;
				CamearaPos.y = cameraPoint[(y + 1) * mColorImg.cols + x].Y * 100;
				CamearaPos.z = cameraPoint[(y + 1) * mColorImg.cols + x].Z * 100;
			}
		}
		else {
			CamearaPos.x = cameraPoint[y * mColorImg.cols + x].X * 100;
			CamearaPos.y = cameraPoint[y * mColorImg.cols + x].Y * 100;
			CamearaPos.z = cameraPoint[y * mColorImg.cols + x].Z * 100;
		}
	}


	float capx = 50, capy = 50, capz = 100;
	float armx = 0, army = 0, armz = 0;

	float ThetaX = PI;
	float ThetaY = 0;
	float ThetaZ = 0;

	int intputpoint[4][1] = { 0,0,0,1 };

	for (int i = 0; i < 3; i++) {
		intputpoint[0][0] = CamearaPos.x;
		intputpoint[1][0] = CamearaPos.y;
		intputpoint[2][0] = CamearaPos.z;

		//std::cout << intputpoint[i][0] << std::endl;
	}

	float TMatrix[4][4] = { { 1, 0, 0, capx - armx },{ 0, 1, 0, capy - army },{ 0, 0, 1, capz - armz },{ 0, 0, 0, 1 } };
	float RMatrix[4][4] = { { cos(ThetaY)* cos(ThetaZ) - sin(ThetaX)*sin(ThetaY)*sin(ThetaZ), -cos(ThetaX)*sin(ThetaZ), sin(ThetaY)*cos(ThetaZ) + sin(ThetaX)*cos(ThetaY)*sin(ThetaZ), 0 },
						  { cos(ThetaY)*sin(ThetaZ) + sin(ThetaX)*sin(ThetaY)*cos(ThetaZ), cos(ThetaX)*cos(ThetaZ), sin(ThetaY)*sin(ThetaZ) - sin(ThetaX)*cos(ThetaY)*cos(ThetaZ), 0 },
						  { -cos(ThetaX)*sin(ThetaY), sin(ThetaX), cos(ThetaX)*cos(ThetaY), 0 },{ 0, 0, 0, 1 } };

	float product[4][4] = { { 0, 0, 0, 0 },{ 0, 0, 0, 0 },{ 0, 0, 0, 0 },{ 0, 0, 0, 0 } };
	float outputpoint[4][1] = { 0,0,0,0 };

	for (int row = 0; row < 4; row++) {
		for (int col = 0; col < 4; col++) {
			for (int inner = 0; inner < 4; inner++) {
				product[row][col] += TMatrix[row][inner] * RMatrix[inner][col];
			}
		}
	}

	for (int row = 0; row < 4; row++) {
		for (int col = 0; col < 1; col++) {
			for (int inner = 0; inner < 4; inner++) {
				outputpoint[row][0] += product[row][inner] * intputpoint[inner][col];
			}
		}
	}

	float w = outputpoint[0][4];
	for (int i = 0; i < 4; i++) {
		outputpoint[i][0] / w;
	}

	WorldPos.x = outputpoint[0][0];
	WorldPos.y = outputpoint[1][0];
	WorldPos.z = outputpoint[2][0];
}

void KinectV2::Finalangle() {
	/*cv::Point Center;
	map<string, cv::Point>::iterator iter;
	PointTable.value_comp();
	iter = PointTable.find("Center");
	Center = iter->swap;
	std::cout << Center << std::endl;
	PointTable.find("RightUpper");
	PointTable.find("RightDown");
	PointTable.find("LeftUpper");
	PointTable.find("LeftDown");*/
}

void KinectV2::GetFPS(cv::Mat img, double t) {
	/*主程式要加
	//t = (double)cv::getTickCount();
	//t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
	*/
	
	double fps;
	char string[10];
	if (!img.empty()) {
		fps = 1.0 / t;
		sprintf_s(string, "%.2f", fps);
		std::string fpsString("FPS:");
		fpsString += string;

		cv::putText(img, fpsString, cv::Point(5, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255));
	}
}

float* KinectV2::GetArmCode() {
	return ArmCode;
}

int KinectV2::GetDepthMax() {
	return mDepthMax;
}

Mat KinectV2::GetDepthImg() {
	return mDepthImg;
}

cv::Mat KinectV2::GetColorImg() {
	return mColorImg;
}

cv::Mat KinectV2::GetIRImg() {
	return mIRImg;
}

cv::Mat KinectV2::GetDepthImg8UC4() {
	return depth8UC4;
}

cv::Mat KinectV2::GetPointCloudImg() {
	return mPointCloud;
}

cv::Mat KinectV2::GetMapperImg() {
	return mMapImg;
}

cv::Point3f KinectV2::GetWorldPos() {
	return WorldPos;
}



//void KinectV2::DepthUpdate() {
//
//	while (m_ToggleThreadDepth) {
//		IDepthFrame* m_DepthFrame;
//
//		if (m_DepthReader->AcquireLatestFrame(&m_DepthFrame) == S_OK) {
//
//			m_DepthFrame->AccessUnderlyingBuffer(&m_DepthBufferSize, &m_DepthBuffer);
//
//			DepthImg8UC4();
//			//copy the depth map to image
//			m_DepthFrame->CopyFrameDataToArray(m_DepthWidth * m_DepthHeight,
//				reinterpret_cast<UINT16*>(mDepthImg.data));
//
//			//release frame
//			m_DepthFrame->Release();
//		}
//	}
//}
void KinectV2::close() {
	
	m_ToggleThreadDepth = false;
	m_ThreadDepth.join();
}