#pragma once
#include <Kinect.h>
#include <opencv2/core.hpp>
#include <thread>
#include <map>
class KinectV2{
public:

	//std::map<string, cv::Point> PointTable;

	KinectV2();
	~KinectV2();


	bool Init();
	void DepthUpdate();
	void ColorUpdate();
	void IRUpdate();
	void DepthImg8UC4();

	void ColorFrameToDepthSpace();
	void ColorFrameToCameraSpace();
	void DepthFrameToColorSpace();
	void DepthFrameToCameraSpace();

	void GetArmAngle(cv::Point3d Apos, cv::Point3d Bpos, cv::Point3d Cpos);
	void Finalangle();
	void WorldPosition(int x, int y);
	void GetFPS(cv::Mat img, double t);
	
	float* GetArmCode();
	int GetDepthMax();

	cv::Mat GetDepthImg();
	cv::Mat GetColorImg();
	cv::Mat GetIRImg();
	cv::Mat GetDepthImg8UC4();
	cv::Mat GetPointCloudImg();
	cv::Mat GetMapperImg();
	cv::Point3f GetWorldPos();

	void close();

private:

	IKinectSensor* m_KinectSensor;
	IDepthFrameReader* m_DepthReader;
	IColorFrameReader* m_ColorReader;
	IInfraredFrameReader* m_IRReader;
	ICoordinateMapper* m_Mapper;

	UINT m_DepthBufferSize,m_ColorBufferSize;
	UINT16 * m_DepthBuffer;
	int m_DepthWidth, m_DepthHeight , m_ColorWidth , m_ColorHeight;
	int m_IRWidth, m_IRHeight;
	USHORT mDepthMin = 0;
	USHORT mDepthMax = USHRT_MAX;

	float ArmCode[5];
	float RR;
	float LR;
	float Rphi;
	float Lphi;

	cv::Mat mDepthImg;
	cv::Mat mColorImg;
	cv::Mat mIRImg;
	cv::Mat depth8UC4;
	cv::Mat mMapImg;
	cv::Mat mPointCloud;

	cv::Point3f WorldPos;
	

	std::thread m_ThreadDepth;
	std::thread m_ThreadColor;

	bool m_ToggleThreadDepth;
};

