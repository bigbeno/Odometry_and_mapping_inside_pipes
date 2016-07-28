/*
 * circleOdometryEstimator.cpp
 *
 *  Created on: May 27, 2016
 *      Author: biorobotics
 */

#include "../include/circleOdometryEstimator.h"

using namespace std;
using namespace cv;

static cv::Scalar red = cv::Scalar(0, 0, 255);
static cv::Scalar green = cv::Scalar(0, 255, 0);
static cv::Scalar blue = cv::Scalar(255, 0, 0);
static cv::Scalar white = cv::Scalar(255, 255, 255);
static cv::Scalar black = cv::Scalar(0, 0, 0);

// Preprocessing variables
float weight = 0.01f * 0;
float scale = 0.01f * 1;
float gaussianRadius = 21;

// Hough detection varibles
float houghResolution = 0.1f * 140;
float cannyThreshold = 20;
float accumulatorThreshold = 100;
int maxRadius = 400;
int minRadius = 80;


// Camera matrix
const float focal =830;
const float pp_x =1080;
const float pp_y =540;
const cv::Mat_<float>  K = (cv::Mat_<float>(3, 3) <<
		focal, 0, 1080,
		0, focal, 540,
		0, 0, 1);

// Frame size
const int width = 1920;
const int height = 1080;


// Radius of the real pipe
float real_radius = 0.05; // [m]


odometry circleOdometryEstimator::compute(Mat originalFrame, double current_time)
{
	cout << endl <<"-------------- CIRCLE-TRACKING ODOMETRY --------------" << endl;
	time_t tstart;
	tstart = time(0);

	originalFrame.copyTo(current_frame);

	odometry featureOdometry;

	// Image processing
	Mat frame = process_image(originalFrame);

	// Run hough circle detector
	vector<Vec3f> detected_circles = detect_circles(frame);

	if (detected_circles.size() >0)
	{
		// Run circle tracker
		circleTracker::circle filtered_circle = filter_circle(detected_circles[0], current_time);

		// If circle tracker estimated that a pipe-joint is visible
		if (filtered_circle.available)
		{
			// Use pinhole camera approximation to compute position in metric units
			// of the joint with respect to the camera
			Vec3f current_joint_position = pinhole_camera_model(filtered_circle.position);

			// Use uncertainty propagation rule to convert uncertainty
			// on the circle position to compute uncertainty on joint position
			Mat current_joint_uncertainty = pinhole_camera_model_uncertainty(filtered_circle.position, filtered_circle.uncertainty);

			joint current_joint(current_joint_position, current_joint_uncertainty);

			// If we known also the previous position of the circle, comput translation
			if (filtered_circle.new_track==false)
				featureOdometry = motion_estimate(current_joint, previous_joint);

			previous_joint = current_joint;
		}
	}

	cout << endl << "--> It took "<< difftime(time(0), tstart) <<" second(s) to do one odometry iteration"<< endl;

	return featureOdometry;
}

Mat circleOdometryEstimator::process_image(Mat&  originalFrame)
{
	cout << endl << "* PROCESS IMAGE:" << endl;
	// Transforma the image with Gaussian and laplacian filters
	// to improve the performance of the Hough detector
	// (Not my idea, reference: )

	time_t tstart; 	tstart = time(0);

	Mat grayImg;
	Mat grayImgf;
	Mat_<float> blurredf;
	Mat_<float> laplaccian;
	Mat sharpened;

	std::vector<Vec3f> circles;

	/* TO GRAY COLOR*/
	cvtColor(originalFrame, grayImg, COLOR_BGR2GRAY);
	grayImg.convertTo(grayImgf, CV_32F);

	/*GAUSSIAN BLURRING*/
	GaussianBlur(grayImgf, blurredf, Size(gaussianRadius, gaussianRadius), 0);
	Mat blurred;
	blurredf.convertTo(blurred, CV_8U);

	/* LAPLACIAN*/
	Laplacian(blurredf, laplaccian, CV_32F);

	/* GRAY * (SCALE * LAPLACIAN)*/
	Mat_<float> grayTimesLapf = grayImgf.mul(scale * laplaccian);//(scale * laplaccian); //modifiable
	Mat grayTimesLap;
	grayTimesLapf.convertTo(grayTimesLap, CV_8U);

	/* GRAY - BLURRED*/
	Mat_<float> grayMinusBlurred_f = 1.5f * grayImgf - 0.5f * blurredf;
	Mat grayMinusBlurred;
	grayMinusBlurred_f.convertTo(grayMinusBlurred, CV_8U);

	Mat_<float> sharpenedf =
		grayMinusBlurred_f
		- weight * grayTimesLapf;

	sharpenedf.convertTo(sharpened, CV_8U);

	cout << "It took "<< difftime(time(0), tstart) <<" second(s)."<< endl;


	return sharpened;

}

vector<Vec3f> circleOdometryEstimator::detect_circles(Mat frame)
{
	cout << endl << "* DETECT CIRCLES:" << endl;

	time_t tstart;
	tstart = time(0);

	vector<Vec3f> detected_circles;

	int minDistance = frame.size().width / 30;

	HoughCircles(frame,
			detected_circles,
			HOUGH_GRADIENT,
			houghResolution,
			minDistance,
			cannyThreshold,
			accumulatorThreshold,
			minRadius,
			maxRadius);

	cout << "It took "<< difftime(time(0), tstart) <<" second(s)."<< endl;

	if (detected_circles.size()>0)
	{
		cvNamedWindow("detected circle", WINDOW_NORMAL);
		imshow("detected circle", display_one_circle(current_frame, detected_circles[0], blue));
	}

	return detected_circles;
}

circleTracker::circle circleOdometryEstimator::filter_circle(Vec3f detected_circle, double current_time)
{
	// Use detected circle from Hough detector
	// to update circle tracler
	cout << endl << "* FILTER CIRCLE:" << endl;
	time_t tstart;
	tstart = time(0);

	// Feed the current frmae (just for plotting purposes)
	circle_tracker.give_frame_for_plotting(current_frame);

	// Update circle tracker with detected circle
	circleTracker::circle filtered_circle = circle_tracker.update( detected_circle, current_time);
	cvNamedWindow("filtered circle");


	if (filtered_circle.available)
	{
		cout << endl << " -- Tracked circle! -- " << endl;
		imshow("filtered circle", display_one_circle(current_frame, filtered_circle.position, red));
	}
	else
	{
		cout << endl << " -- No tracked circle -- " << endl;
		imshow("filtered circle", current_frame);
	}

	cout << "It took "<< difftime(time(0), tstart) <<" second(s)."<< endl;

	return filtered_circle;
}

Vec3f circleOdometryEstimator::pinhole_camera_model(Vec3f filtered_circle)
{
	// Compute the position of the joint given the position of its visual circle
	cout << endl << "* JOINT FROM CIRCLE:" << endl;

	time_t tstart;
	tstart = time(0);

	Vec3f joint_position;

	float x_center = filtered_circle[0];
	float y_center = filtered_circle[1];
	float r = filtered_circle[2];

	float z =  focal*real_radius / r; // [?]
	float x = (x_center - pp_x) /  focal * z;
	float y = (y_center - pp_y) / focal * z;

	joint_position = {x, y, z};

	cout << "circle, xcc:" << x_center <<", ycc:" << y_center << ", r:" << r << " [pxl]" << endl;
	cout << "joint, x:" << x <<", y:" << y << ", z:" << z << " [m]" <<  endl;

	cout << "It took "<< difftime(time(0), tstart) <<" second(s)."<< endl;


	return joint_position;
}

Mat circleOdometryEstimator::pinhole_camera_model_uncertainty(Vec3f filtered_circle, Mat circle_uncertainty)
{
	cout << endl << "* JOINT UNCERTAINTY FROM CIRCLE UNCERTAINTY:" << endl;

	// We have input = [xcc, ycc, r]
	// with uncorrelated covariances sigma = [sigma_xcc sigma_ycc sigma_r]
	// (since circle uncertainty is diagonal matrix)
	// and output = [x, y, z]

	float x_cc = filtered_circle[0];
	float y_cc = filtered_circle[1];
	float r = filtered_circle[2];

	float sigma_xcc = circle_uncertainty.at<float>(0,0);
	float sigma_ycc = circle_uncertainty.at<float>(1,1);
	float sigma_r = circle_uncertainty.at<float>(2,2);


	/** SIGMA X **/
	// x = (x_cc - width / 2) /  focal * z
	// = (x_cc - width / 2) * real_radius / r = fx(x_cc, r)
	// hence
	// sigma_x = sqrt(sigma_x_xcc.^2 + sigma_x_r.^2)

	// sigma_x_xcc = d(fx)/dx_cc * sigma_xcc
	float sigma_x_xcc = real_radius/r * sigma_xcc;
	// sigma_x_r = d(fx)/dr* sigma_r
	float sigma_x_r = real_radius*(pp_x- x_cc)/(r*r) * sigma_r;
	float sigma_x = sqrt(sigma_x_xcc*sigma_x_xcc + sigma_x_r*sigma_x_r);

	/** SIGMA Y **/
	// y = (y_cc - height / 2) /  focal * z
	// = (y_cc - height / 2) * real_radius / r = fy(y_cc, r)
	// hence
	// sigma_y = sqrt(sigma_y_ycc.^2 + sigma_y_r.^2)

	// sigma_y_ycc = d(fy)/dy_cc * sigma_ycc
	float sigma_y_ycc = real_radius/r * sigma_ycc;
	// sigma_y_r = d(fy)/dr* sigma_r
	float sigma_y_r = real_radius*(pp_y - y_cc)/(r*r) * sigma_r;
	float sigma_y = sqrt(sigma_y_ycc*sigma_y_ycc + sigma_y_r*sigma_y_r);

	/** SIGMA Z **/
	// z = focal*real_radius / r = fz(r)
	// hence
	// sigma_z = |d(fr)/dr * sigma_r|
	float sigma_z = abs(-focal*real_radius/(r*r) * sigma_r);


	Mat joint_uncertainty = Mat::zeros(3,3, CV_32F);
	joint_uncertainty.at<float>(0,0) = sigma_x;
	joint_uncertainty.at<float>(1,1) = sigma_y;
	joint_uncertainty.at<float>(2,2) = sigma_z;

	cout << "circle, sigma xcc:" << sigma_xcc <<", sigma ycc:" << sigma_ycc << ", sigma r:" << sigma_r << " [pxl]" << endl;
	cout << "joint, sigma x:" << sigma_x <<", sigma y:" << sigma_y << ", sigma z:" << sigma_z << " [m]" <<  endl;

	return joint_uncertainty;
}

odometry circleOdometryEstimator::motion_estimate(joint current_joint, joint previous_joint)
{
	cout << endl << "* MOTION FROM JOINTS:" << endl;

	odometry featureOdometry;

	// Differntial motion of the camera
	// delta camera = - delta joint
	// hence camera-current - camera_previous = joint_previous - joint_current
	featureOdometry.dx = previous_joint.position[0] - current_joint.position[0];
	featureOdometry.dy = previous_joint.position[1] - current_joint.position[1];
	featureOdometry.dz = previous_joint.position[2] - current_joint.position[2];

	cout << "joint previous, x:" << previous_joint.position[0] <<
							", y:" << previous_joint.position[1] <<
							", z:" << previous_joint.position[2] << " [m]" <<  endl;
	cout << "joint current, x:" << current_joint.position[0] <<
							", y:" << current_joint.position[1] <<
							", z:" << current_joint.position[2] << " [m]" <<  endl;
	cout << "camera motion , dx:" << featureOdometry.dx <<
							", dy:" << featureOdometry.dy <<
							", dz:" << featureOdometry.dz << " [m]" <<  endl;


	// Uncertainty on the differntial motion of the camera
	// dx = x1 - x2
	// hence
	// sigma dx = sqrt(sigma_x1^2 + sigma_x2^2)
	featureOdometry.sigma_dx = sqrt(previous_joint.uncertainty.at<float>(0,0)*previous_joint.uncertainty.at<float>(0,0) +
								current_joint.uncertainty.at<float>(0,0)*current_joint.uncertainty.at<float>(0,0));
	featureOdometry.sigma_dy = sqrt(previous_joint.uncertainty.at<float>(1,1)*previous_joint.uncertainty.at<float>(1,1) +
								current_joint.uncertainty.at<float>(1,1)*current_joint.uncertainty.at<float>(1,1));
	featureOdometry.sigma_dz = sqrt(previous_joint.uncertainty.at<float>(2,2)*previous_joint.uncertainty.at<float>(2,2) +
								current_joint.uncertainty.at<float>(2,2)*current_joint.uncertainty.at<float>(2,2));

	cout << endl;
	cout << "joint previous sigma, x:" << previous_joint.uncertainty.at<float>(0,0) <<
							", sigma y:" << previous_joint.uncertainty.at<float>(1,1) <<
							", sigma z:" << previous_joint.uncertainty.at<float>(2,2) << " [m]" <<  endl;
	cout << "joint current, sigma x:" << current_joint.uncertainty.at<float>(0,0) <<
							", y:" << current_joint.uncertainty.at<float>(1,1) <<
							", z:" << current_joint.uncertainty.at<float>(2,2) << " [m]" <<  endl;
	cout << "camera motion , sigma dx:" << featureOdometry.sigma_dx <<
							", sigma dy:" << featureOdometry.sigma_dy <<
							", sigma dz:" << featureOdometry.sigma_dz << " [m]" <<  endl;



	// Output the joint position
	featureOdometry.joint_x = current_joint.position[0];
	featureOdometry.joint_y = current_joint.position[1];
	featureOdometry.joint_z = current_joint.position[2];

	featureOdometry.available = true;

	return featureOdometry;
}


Mat circleOdometryEstimator::display_one_circle(const Mat& colorImg, Vec3f circle, Scalar color)
{
	Mat display = colorImg.clone();
	Point center(cvRound(circle[0]), cvRound(circle[1]));
	int radius = cvRound(circle[2]);

	cv::circle(display, center, 3, Scalar(0, 255, 0), 3, 8, 0);
	cv::circle(display, center, radius, color, 3, 8, 0);

	return display;
}
