#include "Camera.h"
#define PI 3.14159265359

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <stdlib.h>

//using namespace cv;

//triangulation 함수 생성
vector <Vector3d> est_all_point;

Vector3d tri(vector<Camera*> cameras, int index)  //tri(temps, 0)
{
	int count = 0;
	vector<int> indexes;
	vector<pair<double, double>> xy;

	for (int i = 0; i < cameras.size(); i++)
	{
		if (cameras[i]->pixl_result[index].first > 0 && cameras[i]->pixl_result[index].first < 320 && cameras[i]->pixl_result[index].second > 0 && cameras[i]->pixl_result[index].second < 240)
		{
			count++;
			indexes.push_back(i);

		}
	}

	cv::Mat AA;
	AA.create(4, 4, CV_64F);

	cv::Mat P_right_1 = cv::Mat::zeros(3, 1, CV_64F);
	P_right_1.at<double>(0, 0) = 146.58;
	P_right_1.at<double>(0, 1) = 120.12;
	P_right_1.at<double>(0, 2) = 62.215;
	P_right_1.at<double>(0, 3) = -30.957;

	P_right_1.at<double>(1, 0) = 69.049;
	P_right_1.at<double>(1, 1) = -1.2052;
	P_right_1.at<double>(1, 2) = 163.53;
	P_right_1.at<double>(1, 3) = -24.463;

	P_right_1.at<double>(2, 0) = 0.9183;
	P_right_1.at<double>(2, 1) = -0.01603;
	P_right_1.at<double>(2, 2) = 0.3955;
	P_right_1.at<double>(2, 3) = -0.2097;

	cv::Mat P_right_2 = cv::Mat::zeros(3, 1, CV_64F);
	P_right_2.at<double>(0, 0) = 138.31;
	P_right_2.at<double>(0, 1) = 202.55;
	P_right_2.at<double>(0, 2) = 67.311;
	P_right_2.at<double>(0, 3) = -33.516;

	P_right_2.at<double>(1, 0) = 98.788;
	P_right_2.at<double>(1, 1) = 1.3104;
	P_right_2.at<double>(1, 2) = 219.52;
	P_right_2.at<double>(1, 3) = -34.935;

	P_right_2.at<double>(2, 0) = 0.9925;
	P_right_2.at<double>(2, 1) = 0.01317;
	P_right_2.at<double>(2, 2) = 0.1219;
	P_right_2.at<double>(2, 3) = -0.2072;


	pair<int, int> pt1;
	pair<int, int> pt2;

	pt1.first = 160;
	pt1.second = 75;
	pt2.first = 139;
	pt2.second = 100;

	pair<cv::Mat,pair<int, int>> tmp1;
	pair<cv::Mat, pair<int, int>> tmp2;

	tmp1.first = P_right_1;
	tmp1.second = pt1;

	tmp2.first = P_right_2;
	tmp2.second = pt2;

	int width = 640;
	int height = 480;

	vector<pair<cv::Mat,pair<int, int>>> tri_pt;

	for (int i = 0; i < tri_pt.size(); i++)
	{
		int x = width - tri_pt.at(i).second.first;
		int y = height - tri_pt.at(i).second.second;

		for (int j = 0; j < 4; j++)
		{
			AA.at<double>(2 * i, j) = x*tri_pt.at(i).first.at<double>(2, j) - tri_pt.at(i).first.at<double>(0, j);
			AA.at<double>(2 * i + 1, j) = y*tri_pt.at(i).first.at<double>(2, j) - tri_pt.at(i).first.at<double>(1, j);
		}
	}

	//AA.create(indexes.size() * 2, 4 , CV_64F);

	//for (int i = 0; i < indexes.size(); i++)
	//{
	//	//cameras[indexes[i]]; //해당 카메라

	//	for (int j = 0; j < 4; j++)
	//	{
	//		//cout <<"i번째" << i << endl;

	//		AA.at<double>(2 * i, j) = (cameras[indexes[i]]->pixl_result[index].first * cameras[indexes[i]]->InEx(2, j)) - cameras[indexes[i]]->InEx(0, j);
	//		AA.at<double>(2 * i + 1, j) = (cameras[indexes[i]]->pixl_result[index].second * cameras[indexes[i]]->InEx(2, j)) - cameras[indexes[i]]->InEx(1, j);

	//		//cout << cameras[indexes[i]]->InEx(0, j) << endl;
	//		//cout << cameras[indexes[i]]->InEx(1, j) << endl;
	//		//cout<< cameras[indexes[i]]->InEx(2, j) << endl;
	//		//cout << "----------------------------" << endl;

	//	}
	//	//cout << cameras[indexes[i]]->pixl_result[index].first << "," << cameras[indexes[i]]->pixl_result[index].second  << endl;
	//}

	////cout << AA << endl;

	


	cv::Mat w, u, vt;
	cv::SVD::compute(AA, w, u, vt, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);
	//cout << vt.row(3) << endl;
	cv::Mat ans = vt.row(3);
	ans = ans.t(); //ans=해, AX=0을 만족하는 X가 됨
	ans = ans / ans.at<double>(3, 0);
	Vector3d answer;
	answer[0] = ans.at<double>(0, 0);
	answer[1] = ans.at<double>(1, 0);
	answer[2] = ans.at<double>(2, 0);

	//cout << ans << endl;
	// cout << answer << endl;
	est_all_point.push_back(answer);
	
	return answer;
}


vector<double> get_error(vector<Vector3d> real_all_point, vector <Vector3d> est_all_point)
{
	vector<double> error;
	//double error_compute;
	for (int i = 0; i < real_all_point.size(); i++)
	{
		
		//cout << "est" << est_all_point.at(i) << endl;
		Vector3d error_compute_1 = (real_all_point.at(i) - est_all_point.at(i)); 

		double error_x = error_compute_1(0)*error_compute_1(0);
		double error_y = error_compute_1(1)*error_compute_1(1);
		double error_z = error_compute_1(2)*error_compute_1(2);

		double error_com = sqrt(error_x + error_y + error_z);
		error.push_back(error_com);

		FILE *pFile;
		fopen_s(&pFile, "error_out.txt", "a");
		fprintf_s(pFile, "%lf\n", error_com);
		fclose(pFile);


		//cout << error_com << endl;
	}

	return error;
}

int main(void)
{
	
	//Camera Camera_O;
	//Camera Camera_F;
	//Camera Camera_R;
	//Camera Camera_L;
	//
	//double main_fx = 196.1717;
	//double main_fy = 198.037449;
	//double main_cx = 146.045479;
	//double main_cy = 115.695610;

	//Camera_O.find_Intrinsic(main_fx, main_fy, main_cx, main_cy);
	//Camera_F.find_Intrinsic(main_fx, main_fy, main_cx, main_cy);
	//Camera_R.find_Intrinsic(main_fx, main_fy, main_cx, main_cy);
	//Camera_L.find_Intrinsic(main_fx, main_fy, main_cx, main_cy);

	//double xyx_th1 = 90 * PI / 180;
	//double xyx_th2 = 90 * PI / 180;
	//double xyx_th3 = -10 * PI / 180;

	//double dist = 1;

	//Camera_O.find_Extrinsic(xyx_th1, xyx_th2, xyx_th3, 0, 0, 0);
	//Camera_F.find_Extrinsic(xyx_th1, xyx_th2, xyx_th3, dist, 0, 0);
	//Camera_R.find_Extrinsic(xyx_th1, xyx_th2, xyx_th3, 0, -dist, 0);
	//Camera_L.find_Extrinsic(xyx_th1, xyx_th2, xyx_th3, 0, dist, 0);
	//
	//vector<Vector3d> real_all_point;

	//Vector4d point1;
	//Vector3d point1_mod;
	//vector<int> real_index;
	//int rreal_index = 0;
	//for (int i = 0; i < 9; i++)
	//{
	//	for (int j = 0; j < 5; j++)
	//	{
	//		point1(0) = 4;
	//		point1(1) = -2 + (0.5*i);
	//		point1(2) = 0 + (0.5*j);
	//		point1(3) = 1;
	//		
	//		rreal_index++;

	//		point1_mod(0) = point1(0);
	//		point1_mod(1) = point1(1);
	//		point1_mod(2) = point1(2);

	//		real_all_point.push_back(point1_mod);

	//	    Camera_O.InExpt1(point1);
	//		Camera_F.InExpt1(point1);
	//		Camera_R.InExpt1(point1);
	//		Camera_L.InExpt1(point1);

	//		Camera_O.InEx;
	//		Camera_F.InEx;
	//		Camera_R.InEx;
	//		Camera_L.InEx;

	//		//cout << Camera_O.InEx << endl;
	//		//cout << "----------------------------------------------------" << endl;
	//		//cout << Camera_F.InEx << endl;
	//		//cout << "----------------------------------------------------" << endl;
	//		//cout << Camera_R.InEx << endl;
	//		//cout << "----------------------------------------------------" << endl;
	//		//cout << Camera_L.InEx << endl;
	//		//cout << "----------------------------------------------------" << endl;

	//		//Camera_O.show_pixl();
	//		//Camera_F.show_pixl();
	//		//Camera_R.show_pixl();
	//		//Camera_L.show_pixl();

	//		real_index.push_back(rreal_index);
	//		
	//	}

	//}

	//vector<Camera*> temps;
	//temps.push_back(&Camera_O);
	//temps.push_back(&Camera_F);
	//temps.push_back(&Camera_R);
	//temps.push_back(&Camera_L);
	//
 //   //int main_count = (Camera_O.count / 45) + (Camera_F.count / 45) + (Camera_R.count / 45) + (Camera_L.count / 45);
	////
	////int main_count = Camera_O.count + Camera_F.count + Camera_R.count + Camera_L.count;
	//for (int i = 0; i < 45; i++)
	//{
	//	tri(temps, i);
	//}
	//
	//vector<double> error_f;

	//error_f = get_error(real_all_point, est_all_point);


	cv::Mat AA;
	AA.create(4, 4, CV_64F);

	cv::Mat P_right_1 = cv::Mat::zeros(3, 4, CV_64F);
	P_right_1.at<double>(0, 0) = 146.58;
	P_right_1.at<double>(0, 1) = 120.12;
	P_right_1.at<double>(0, 2) = 62.215;
	P_right_1.at<double>(0, 3) = -30.957;

	P_right_1.at<double>(1, 0) = 69.049;
	P_right_1.at<double>(1, 1) = -1.2052;
	P_right_1.at<double>(1, 2) = 163.53;
	P_right_1.at<double>(1, 3) = -24.463;

	P_right_1.at<double>(2, 0) = 0.9183;
	P_right_1.at<double>(2, 1) = -0.01603;
	P_right_1.at<double>(2, 2) = 0.3955;
	P_right_1.at<double>(2, 3) = -0.2097;

	cv::Mat P_right_2 = cv::Mat::zeros(3, 4, CV_64F);
	P_right_2.at<double>(0, 0) = 138.31;
	P_right_2.at<double>(0, 1) = 202.55;
	P_right_2.at<double>(0, 2) = 67.311;
	P_right_2.at<double>(0, 3) = -33.516;

	P_right_2.at<double>(1, 0) = 98.788;
	P_right_2.at<double>(1, 1) = 1.3104;
	P_right_2.at<double>(1, 2) = 219.52;
	P_right_2.at<double>(1, 3) = -34.935;

	P_right_2.at<double>(2, 0) = 0.9925;
	P_right_2.at<double>(2, 1) = 0.01317;
	P_right_2.at<double>(2, 2) = 0.1219;
	P_right_2.at<double>(2, 3) = -0.2072;


	pair<int, int> pt1;
	pair<int, int> pt2;

	pt1.first = 160;
	pt1.second = 75;
	pt2.first = 139;
	pt2.second = 100;

	pair<cv::Mat, pair<int, int>> tmp1;
	pair<cv::Mat, pair<int, int>> tmp2;

	tmp1.first = P_right_1;
	tmp1.second = pt1;

	tmp2.first = P_right_2;
	tmp2.second = pt2;

	int width = 640;
	int height = 480;

	vector<pair<cv::Mat, pair<int, int>>> tri_pt;
	tri_pt.push_back(tmp1);
	tri_pt.push_back(tmp2);

	for (int i = 0; i < tri_pt.size(); i++)
	{
		int x = width - tri_pt.at(i).second.first;
		int y = height - tri_pt.at(i).second.second;

		for (int j = 0; j < 4; j++)
		{
			AA.at<double>(2 * i, j) = x*tri_pt.at(i).first.at<double>(2, j) - tri_pt.at(i).first.at<double>(0, j);
			AA.at<double>(2 * i + 1, j) = y*tri_pt.at(i).first.at<double>(2, j) - tri_pt.at(i).first.at<double>(1, j);
		}
	}

	cv::Mat w, u, vt;
	cv::SVD::compute(AA, w, u, vt, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);
	//cout << vt.row(3) << endl;
	cv::Mat ans = vt.row(3);
	ans = ans.t(); //ans=해, AX=0을 만족하는 X가 됨

	ans = ans / ans.at<double>(3, 0);

	//cout << ans.at<double>(3, 0) << endl;

	Vector3d answer;
	answer[0] = ans.at<double>(0, 0);
	answer[1] = ans.at<double>(1, 0);
	answer[2] = ans.at<double>(2, 0);

	//cout << answer[0] << "  " << answer[1] << "  " << answer[2] << endl;

	//cout << sin(90 * M_PI / 180) << endl;
	//cout << sin(90) << endl;

	srand(time(NULL));
	int a = rand()%100;
	//cout << a << endl;


	return 0;

}




/*참고 - Euler Angle 
Eigen::Vector3f v3fCameraPosition;
v3fCameraPosition[0] = mv3fCurrentPose[0] + d3*cos(mv3fCurrentPose[2]) - d1*sin(mv3fCurrentPose[2]);
v3fCameraPosition[1] = mv3fCurrentPose[1] + d3*sin(mv3fCurrentPose[2]) + d1*cos(mv3fCurrentPose[2]);
v3fCameraPosition[2] = mv3fCurrentPose[2] + th2*M_PI / 180;

cv::Mat tc2w = cv::Mat::zeros(3, 1, CV_64F); // tc2w 는 world 기준 카메라 좌표
tc2w.at<double>(0, 0) = v3fCameraPosition[0];
tc2w.at<double>(1, 0) = v3fCameraPosition[1];
tc2w.at<double>(2, 0) = d2;

float fCtemp = cos(v3fCameraPosition[2] + M_PI / 2.0);
float fStemp = sin(v3fCameraPosition[2] + M_PI / 2.0);

float fCTilting = cos(th1 * M_PI / 180);
float fSTilting = sin(th1 * M_PI / 180);

cv::Mat Rw2c = cv::Mat::zeros(cv::Size(3, 3), CV_64F);

Rw2c.at<double>(0, 0) = fCtemp; Rw2c.at<double>(0, 1) = fStemp; Rw2c.at<double>(0, 2) = 0;
Rw2c.at<double>(1, 0) = fSTilting * fStemp; Rw2c.at<double>(1, 1) = -fSTilting * fCtemp; Rw2c.at<double>(1, 2) = fCTilting;
Rw2c.at<double>(2, 0) = fCTilting * fStemp; Rw2c.at<double>(2, 1) = -fCTilting * fCtemp; Rw2c.at<double>(2, 2) = -fSTilting;

cv::Mat tw2c = -Rw2c * tc2w; // tw2c 는 카메라 기준 world 원점 좌표

cv::Mat Tcw = cv::Mat::zeros(cv::Size(4, 4), CV_64F);
Rw2c.copyTo(Tcw.rowRange(0, 3).colRange(0, 3));    // Tcw 는 Tw2c 랑 같음
tw2c.copyTo(Tcw.rowRange(0, 3).col(3));
Tcw.row(3).col(3) = 1;

*/