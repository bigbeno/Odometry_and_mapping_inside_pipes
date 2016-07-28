/*
 * circleOdometryEstimator.h
 *
 *  Created on: May 27, 2016
 *      Author: biorobotics
 */

#ifndef CIRCLEODOMETRYESTIMATOR_H_
#define CIRCLEODOMETRYESTIMATOR_H_

#include "odometry.h"
#include "opencv2/opencv.hpp"
#include <iostream>
#include "circleTracker.h"

class circleOdometryEstimator
{
public:

	odometry compute(cv::Mat frame, double time); //[sec]

private:

	struct joint
	{
		joint()
			: position({-1,-1,-1}),
			  uncertainty()
		{}

		joint(cv::Vec3f mPosition, cv::Mat mUncertainty)
		{
			position = mPosition;
			mUncertainty.copyTo(uncertainty);
		}

		joint& operator=( const joint& other )
		{
			position = other.position;
			other.uncertainty.copyTo(uncertainty);
		    return *this;
		}

		cv::Vec3f position; //{x, y, z} [?]
		cv::Mat uncertainty;
	};

	circleTracker circle_tracker;
	joint previous_joint;

	cv::Mat current_frame;

	cv::Mat process_image(cv::Mat&  originalFrame);

	std::vector<cv::Vec3f> detect_circles(cv::Mat frame);

	circleTracker::circle filter_circle(cv::Vec3f detected_circle, double current_time);

	circleTracker::circle track_circles(cv::Vec3f detected_circles, double time);

	cv::Vec3f pinhole_camera_model(cv::Vec3f circle_position);

	cv::Mat pinhole_camera_model_uncertainty(cv::Vec3f circle_position, cv::Mat circle_uncertainty);

	odometry motion_estimate(joint current_joint, joint previous_joint);

	/** Old functions **/

	cv::Mat display_one_circle(const cv::Mat& colorImg, cv::Vec3f circle, cv::Scalar color);

};



#endif /* CIRCLEODOMETRYESTIMATOR_H_ */
