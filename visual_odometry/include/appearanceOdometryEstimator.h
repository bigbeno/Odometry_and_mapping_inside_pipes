/*
 * appearanceOdometryEstimator.h
 *
 *  Created on: May 24, 2016
 *      Author: biorobotics
 */

#ifndef APPEARANCEODOMETRYESTIMATOR_H_
#define APPEARANCEODOMETRYESTIMATOR_H_

#include "../include/odometry.h"
#include "opencv2/opencv.hpp"
#include <iostream>
#include <cassert>


class appearanceOdometryEstimator
{
public:

	odometry compute(cv::Mat frame);


private:

	struct matchingFeatures
	{
		matchingFeatures()
		{
			old_features.clear();
			new_features.clear();
			old_threeD.clear();
			new_threeD.clear();
			history.clear();

		}

		std::vector<cv::Point2f> old_features;
		std::vector<cv::Point2f> new_features;
		std::vector<cv::Point3f> old_threeD;
		std::vector<cv::Point3f> new_threeD;
		std::vector<std::vector<cv::Point2f>> history;
	};

	matchingFeatures carried_matches;

	cv::Mat previous_frame;
	cv::Mat current_frame;


	cv::Mat process_image (cv::Mat frame);

	std::vector<cv::Point2f> extract_features(cv::Mat frame);

	matchingFeatures match_features(cv::Mat frame, cv::Mat previous_frame,
			matchingFeatures carried_matches);

	cv::Mat compute_E(matchingFeatures& feature_matches, cv::Mat K);

	cv::Mat estimate_R_t(cv::Mat E, appearanceOdometryEstimator::matchingFeatures& feature_matches);

	matchingFeatures triangulate_points(matchingFeatures matches, cv::Mat H);

	cv::Mat estimate_R_t_uncertainty(int a, int b, int c, int d, int e, double scale);

	double scale_translation(matchingFeatures feature_matches, cv::Mat& H);

	void  draw_trace(cv::Mat dst, appearanceOdometryEstimator::matchingFeatures features,
			std::vector<uchar> status, std::string transf_type, std::vector<float> error = {});


	std::vector<cv::Point2f> extract_points_evenly(cv::Mat previous_frame);

	matchingFeatures match_points(cv::Mat previous_frame, cv::Mat frame,
			matchingFeatures& previous_features);

	void matching(cv::Mat previous_frame, cv::Mat frame,
			std::vector<cv::Point2f>& matchingFeatures, std::vector<cv::Point2f>& points_current, std::vector<uchar>& status, cv::Mat mask);

	void draw_points(cv::Mat src, std::vector<cv::Point2f> src_corners, std::string title, int block_size = 0);

	void skim_points(matchingFeatures& features, std::vector<uchar> status);

	int remove_bad_points(std::vector<cv::Point2f>& vectorOne, std::vector<uchar> status);

	int remove_bad_points(std::vector<cv::Point3f>& vectorOne, std::vector<uchar> status);

	int remove_bad_points(std::vector<std::vector<cv::Point2f>>& vectorOne, std::vector<uchar> status);

	void  draw_flow(cv::Mat src, std::vector<cv::Point2f> src_corners, std::vector<cv::Point2f> dst_corners,
			std::vector<uchar> status, std::string transf_type, std::vector<float> error = {});

	double computeScale(std::vector<cv::Point3f> points_previous, std::vector<cv::Point3f> points_current);

	void write(matchingFeatures matches, std::string name);


};



#endif /* ODOMETRYESTIMATOR_H_ */
