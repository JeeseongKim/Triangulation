#include "Camera.h"

#include <iostream>
#include <Eigen/Dense>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/StdVector>
#include <cstdlib>
#include <ctime>

using namespace std;
using namespace Eigen;

#define PI 3.14159265359

Camera::Camera()
{

}

Camera::~Camera()
{

}

void Camera::find_Extrinsic(double xyx_th1, double xyx_th2, double xyx_th3, double transx, double transy, double transz)
{
	Extrinsic(0, 0) = cos(xyx_th2);
	Extrinsic(1, 0) = sin(xyx_th2)*sin(xyx_th3);
	Extrinsic(2, 0) = cos(xyx_th3)*sin(xyx_th2);

	Extrinsic(0, 1) = sin(xyx_th1)*sin(xyx_th2);
	Extrinsic(1, 1) = (cos(xyx_th1)*cos(xyx_th3)) - (cos(xyx_th2)*sin(xyx_th1)*sin(xyx_th3));
	Extrinsic(2, 1) = (-cos(xyx_th1)*sin(xyx_th3)) - (cos(xyx_th2)*cos(xyx_th3)*sin(xyx_th1));

	Extrinsic(0, 2) = -cos(xyx_th1)*sin(xyx_th2);
	Extrinsic(1, 2) = (cos(xyx_th3)*sin(xyx_th1)) + (cos(xyx_th1)*cos(xyx_th2)*sin(xyx_th3));
	Extrinsic(2, 2) = (cos(xyx_th1)*cos(xyx_th2)*cos(xyx_th3)) - (sin(xyx_th1)*sin(xyx_th3));

	Extrinsic(0, 3) = transx;
	Extrinsic(1, 3) = transy;
	Extrinsic(2, 3) = transz;
}

void Camera::find_Intrinsic(double fx, double fy, double cx, double cy)
{
	Intrinsic(0, 0) = fx;
	Intrinsic(1, 0) = 0;
	Intrinsic(2, 0) = 0;

	Intrinsic(0, 1) = 0;
	Intrinsic(1, 1) = fy;
	Intrinsic(2, 1) = 0;

	Intrinsic(0, 2) = cx;
	Intrinsic(1, 2) = cy;
	Intrinsic(2, 2) = 1;

}

void Camera::InExpt1(Vector4d &pt1)
{
	InEx = Intrinsic*Extrinsic;
	exptres = Extrinsic*pt1;
	inexptres = Intrinsic*Extrinsic*pt1 / exptres(2);

	pair<int, int> a;
	a.first = inexptres(0);
	a.second = inexptres(1);

	pixl_result.push_back(a);
}


void Camera::show_pixl()
{
	for (int i = 0; i < pixl_result.size(); i++)
	{
		cout << pixl_result[i].first << ", " << pixl_result[i].second << endl;
	}

}
