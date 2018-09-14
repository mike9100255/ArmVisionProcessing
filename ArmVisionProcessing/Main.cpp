#include "FindColor.h"
#include "KinectV2.h"

#define PI 3.1415926
void GetArmAngle(cv::Point3d Apos, cv::Point3d Bpos, cv::Point3d Cpos);

float ArmCode[5];

int main() {
	KinectV2 cap;
	Object RednWood;

	bool flag = true;
	RednWood.createTrackbars();
	flag = cap.Init();
	if (flag == false) {
		return 0;
	}

	while (flag) {

		cv::Mat ColorImg;
		cv::Mat MapImg;
		cv::Point2d goal;
		cv::Point3f GoalWorld;
		cv::Point3f GoalWorld_B;
		cv::Point3f GoalWorld_C;

		cap.ColorUpdate();
		cap.DepthUpdate();
		//cap.ColorFrameToCameraSpace();
		if (!cap.GetColorImg().empty()) {
			ColorImg = cap.GetColorImg();
			RednWood.mainProgram(ColorImg);
			/*MapImg = cap.GetPointCloudImg();
			imshow("123", MapImg);*/
		}

		int key = cv::waitKey(1);
		switch (key)
		{
		case'r':
			goal = RednWood.GetPoint();
			cap.WorldPosition(goal.x, goal.y);
			GoalWorld = cap.GetWorldPos();
			cap.WorldPosition(goal.x + 1, goal.y - 1);
			GoalWorld_B = cap.GetWorldPos();
			cap.WorldPosition(goal.x + 1, goal.y + 1);
			std::cout << GoalWorld.x << "," << GoalWorld.y << "," << GoalWorld.z << std::endl;
			GoalWorld_C = cap.GetWorldPos();
			GetArmAngle(GoalWorld, GoalWorld_B, GoalWorld_C);
			
			std::cout << "R¨¤: " << ArmCode[0] << " , " << "phi¨¤: " << ArmCode[1] << "®y¼Ð¡G ("
				<< ArmCode[2] << ", " << ArmCode[3] << ", " << ArmCode[4] << ")" << std::endl;
			break;
		default:
			break;
		}
		if (key == VK_ESCAPE) {
			break;
		}
	}

	return 0;
}


void GetArmAngle(cv::Point3d Apos, cv::Point3d Bpos, cv::Point3d Cpos) {

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

	std::cout << "R¨¤: " << R << " , " << "phi¨¤: " << phi << std::endl;

	ArmCode[0] = R;
	ArmCode[1] = phi;
	ArmCode[2] = Apos.x;
	ArmCode[3] = Apos.y;
	ArmCode[4] = Apos.z;
}