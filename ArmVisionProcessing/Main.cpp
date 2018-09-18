#include "FindColor.h"
#include "KinectV2.h"
#include "TcpClient.h"

#define PI 3.1415926
void GetArmAngle(cv::Point3d Apos, cv::Point3d Bpos, cv::Point3d Cpos);

float ArmCode[5];

int main() {
	KinectV2 cap;
	Object RednWood;
	TcpClient Client;

	cv::Mat ColorImg;
	cv::Mat MapImg;
	cv::Point2d goal;
	cv::Point2d goal_LU;
	cv::Point2d goal_LD;
	cv::Point2d goal_RU;
	cv::Point2d goal_RD;

	cv::Point3f GoalWorld;
	cv::Point3f GoalWorld_LU;
	cv::Point3f GoalWorld_LD;
	cv::Point3f GoalWorld_RU;
	cv::Point3f GoalWorld_RD;

	Client.InitSocket();

	Client.ConnectSocket("127.0.0.1", 4568);

	bool flag = true;
	RednWood.createTrackbars();
	flag = cap.Init();
	if (flag == false) {
		return 0;
	}

	while (flag) {

		cap.ColorUpdate();
		cap.DepthUpdate();
		
		if (!cap.GetColorImg().empty()) {
			ColorImg = cap.GetColorImg();
			RednWood.mainProgram(ColorImg);
		}

		if (Client.ReceiveData()) {
			//Client.GetData();

			//Client.SendData();
		}

		int key = cv::waitKey(1);
		switch (key)
		{
		case'r':
			/*goal = RednWood.GetPoint();*/
			//cap.PointTable = RednWood.GetPointTable();
			/*cap.WorldPosition(goal.x, goal.y);
			GoalWorld = cap.GetWorldPos();*/
			//cap.Finalangle();
			/*goal_LU = RednWood.GetLUPoint();
			goal_LD = RednWood.GetLDPoint();
			goal_RU = RednWood.GetRUPoint();
			goal_RD = RednWood.GetRDPoint();*/
			/*cap.WorldPosition(goal_LU.x, goal_LU.y);
			GoalWorld_LU = cap.GetWorldPos();
			cap.WorldPosition(goal_LD.x, goal_LD.y);
			GoalWorld_LD = cap.GetWorldPos();
			cap.WorldPosition(goal_RU.x, goal_RU.y);
			GoalWorld_RU = cap.GetWorldPos();
			cap.WorldPosition(goal_RD.x, goal_RD.y);
			GoalWorld_RD = cap.GetWorldPos();*/

			/*std::cout << GoalWorld.x << "," << GoalWorld.y << "," << GoalWorld.z << std::endl;
			
			cap.GetArmAngle(GoalWorld, GoalWorld_LU, GoalWorld_LD);*/
			//ArmCode = cap.GetArmCode();
			/*std::cout << "R¨¤: " << ArmCode[0] << " , " << "phi¨¤: " << ArmCode[1] << "®y¼Ð¡G ("
				<< ArmCode[2] << ", " << ArmCode[3] << ", " << ArmCode[4] << ")" << std::endl;*/
			break;

		case'w':
			
			if (!cap.GetColorImg().empty()) {
				ColorImg = cap.GetColorImg();
				cv::imwrite("testimag.png", ColorImg);
				
			}
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