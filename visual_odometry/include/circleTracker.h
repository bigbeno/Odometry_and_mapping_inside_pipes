/*
 * circleTracker.h
 *
 *  Created on: May 28, 2016
 *      Author: biorobotics
 */

#ifndef CIRCLETRACKER_H_
#define CIRCLETRACKER_H_

#include "opencv2/opencv.hpp"
#include <string>
#include <iostream>
#include <sstream>

class circleTracker
{
public:

	struct circle
	{
		circle()
			:position{-1, -1, -1},
			uncertainty(cv::Mat::eye(3,3,CV_64F)*-1),
			available(false),
			new_track(true)
		{};

		cv::Vec3f position;
		cv::Mat uncertainty;
		bool available;
		bool new_track;
	};


	circle update(cv::Vec3f detected_circle, double time); //time is in s

	void give_frame_for_plotting(cv::Mat m_current_frame)
	{
		m_current_frame.copyTo(current_frame);
	}


private:

	double previous_time; //[s]

	struct candidate_track
	{

		candidate_track& operator=(candidate_track other)
	    {
			other.x_a_priori.copyTo(x_a_priori);
			other.x_a_posteriori.copyTo(x_a_posteriori);
			other.P_a_priori.copyTo(P_a_priori);
			other.P_a_posteriori.copyTo(P_a_posteriori);
			other.A.copyTo(A);
			other.C.copyTo(C);
			other.Q.copyTo(Q);
			other.R.copyTo(R);
			current_status = other.current_status;
			missed_validations = other.missed_validations;
	        return *this;
	    }

		//state: x = [x_center, y_center, r, x_center_dot, y_center_dot, r_dot]

		cv::Mat x_a_priori;
		cv::Mat x_a_posteriori;

		cv::Mat P_a_priori;
		cv::Mat P_a_posteriori;

		cv::Mat A; // process function
		cv::Mat C; // measurement function
		cv::Mat Q; // process noise covariance
		cv::Mat R; // measurement noise covariance

		enum status
		{
			never_supported, // did past measurements agree enough to consider it supported?
			supported, // did past measurements agree enough to consider it supported?
			current, // supported and current active track
			to_kill, // did past measurements agree enough to consider it supported?
		};

		status current_status;
		int missed_validations;

	};

	std::vector<candidate_track> list_candidate_tracks;

	cv::Mat current_frame;

	void init_track(cv::Vec3f init_circle);

	void update_candidate_track(candidate_track& track, cv::Vec3f circle, double dt);

	circle choose_track();

	float compute_PD(cv::Mat z_predicted);

	bool remove_duplicate_track(unsigned int track);

	float compute_track_distance(unsigned int trackOne_idx, unsigned int trackTwo_idx);

	cv::Mat display_one_circle_with_std(const cv::Mat& colorImg, candidate_track track);

	cv::Mat display_one_circle(const cv::Mat& colorImg, cv::Vec3f circle);

	void plot_tracks(cv::Vec3f detected_circle);

	unsigned int choose_worse_track(unsigned int trackOne_idx, unsigned int trackTwo_idx);

};


#endif /* CIRCLETRACKER_H_ */
