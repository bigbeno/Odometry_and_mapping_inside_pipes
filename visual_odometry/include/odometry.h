/*
 * odometry.h
 *
 *  Created on: May 24, 2016
 *      Author: biorobotics
 */

#ifndef ODOMETRY_H_
#define ODOMETRY_H_

#include "opencv2/opencv.hpp"
#include <cmath>

#ifndef PI
const double PI = 3.14159265358979323846;
#endif
const double TWOPI = 2.0*PI;

class odometry
{
public:

	odometry()
		:dx(-1), dy(-1), dz(-1),
		 droll(-1), dpitch(-1), dyaw(-1),
		 dR(cv::Mat::eye(3,3,CV_64F)*-1),
		 sigma_dx(-1), sigma_dy(-1), sigma_dz(-1),
		 sigma_droll(-1), sigma_dpitch(-1), sigma_dyaw(-1),
		 joint_x(-1), joint_y(-1), joint_z(-1),
		 available(false), time(-1)
	{};

	double dx;
	double dy;
	double dz;

	double droll;
	double dpitch;
	double dyaw;

	cv::Mat dR;

	double sigma_dx;
	double sigma_dy;
	double sigma_dz;

	double sigma_droll;
	double sigma_dpitch;
	double sigma_dyaw;

	double joint_x;
	double joint_y;
	double joint_z;


	bool available;

	double time; //[ms]

	bool build_from_H_and_Sigma(cv::Mat H, cv::Mat Sigma)
	{
		// Given the Homogeneous transformation and the matrix covariance
		// Extract translation and rotation values and uncertainties.

		build_from_H(H);

		sigma_dx = Sigma.at<double>(0,0);
		sigma_dy = Sigma.at<double>(1,0);
		sigma_dz = Sigma.at<double>(2,0);

		sigma_droll = Sigma.at<double>(3,0);
		sigma_dpitch = Sigma.at<double>(4,0);
		sigma_dyaw = Sigma.at<double>(5,0);


		return true;
	}

	bool build_from_H(cv::Mat H)
	{
		// Given the Homogeneous transformation
		// Extract translation and rotation values

		dx = H.at<double>(0,3);
		dy = H.at<double>(1,3);
		dz = H.at<double>(2,3);

		std::cout << "t" << std::endl << dx <<" "<< dy <<" "<< dz << std::endl;

		H(cv::Rect(0,0,3,3)).copyTo(dR);

		std::cout << "R" << std::endl << dR << std::endl << std::endl;

		std::vector<double> RPY = RPYfromR(H(cv::Rect(0,0,3,3)));
		droll = RPY[0];
		dpitch = RPY[1];
		dyaw = RPY[2];

		return true;
	}

private:


	inline double standardRad(double t)
	{
	    if (t >= 0.)
	        t = fmod(t+PI, TWOPI) - PI;
	    else
	        t = fmod(t-PI, -TWOPI) + PI;
	    return t;
	}


	std::vector<double> RPYfromR(cv::Mat R)
	{

	    dyaw = standardRad(atan2(R.at<double>(1,0), R.at<double>(0,0)));
	    double c = cos(dyaw);
	    double s = sin(dyaw);
	    dpitch = standardRad(atan2(-R.at<double>(2,0), R.at<double>(0,0)*c + R.at<double>(1,0)*s));
	    droll  = standardRad(atan2(R.at<double>(0,2)*s - R.at<double>(1,2)*c, -R.at<double>(0,1)*s + R.at<double>(1,1)*c));

	    std::vector<double>  RPY = {droll, dpitch, dyaw};

	    return RPY;
	}


};


#endif /* ODOMETRY_H_ */
