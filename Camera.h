#ifndef CAMERA_
#define CAMERA_

#include <iostream>
#include <Eigen/Dense>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/StdVector>
#include <cstdlib>
#include <ctime>

using namespace std;
using namespace Eigen;

class Camera
{
private:

	double xyx_th1, xyx_th2, xyx_th3;
	double transx, transy, transz;
	double fx, fy, cx, cy;

	Matrix<double, 3, 4> Ex;
	Vector4d pt1;
	Matrix<double, 3, 3> In;

public:

	Camera();
	~Camera();

	Matrix<double, 3, 3> Intrinsic;
	Matrix<double, 3, 4> Extrinsic;

	double pixel_x, pixel_y;
	vector<pair<double, double>> pixl_result;

	void find_Extrinsic(double xyx_th1, double xyx_th2, double xyx_th3, double transx, double transy, double transz);
	void find_Intrinsic(double fx, double fy, double cx, double cy);

	void Expt1(Vector4d &pt1);
	void InExpt1(Matrix<double, 3, 3> &In, Matrix<double, 3, 4> &Ex, Vector4d &pt1);

	void InExpt1(Vector4d &pt1);

	Vector3d exptres;
	Vector3d inexptres;
	Matrix<double, 3, 4> InEx;

	void save_pixel(double x1, double y1);
	void show_pixl();
};




#endif