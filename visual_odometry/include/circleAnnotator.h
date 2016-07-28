/*
 * circleAnnotator.h
 *
 *  Created on: Jul 1, 2016
 *      Author: biorobotics
 */

#ifndef CIRCLEANNOTATOR_H_
#define CIRCLEANNOTATOR_H_

#include "opencv2/opencv.hpp"
#include <string>
#include <iostream>
#include <sstream>
#include <fstream>

class circleAnnotator
{
public:

	bool annotate_frame_with_reference(cv::Mat frame, double time);

	void write_reference_file(std::string reference_file_name);

private:

	struct reference
	{
		reference(double mtime, cv::Vec3f mcircle)
		{
			time = mtime;
			circle = mcircle;
		}

		double time;
		cv::Vec3f circle;
	};

	std::vector<reference> saved_references;

	void add_reference_circle(double time, cv::Point point1, double radius);

};


#endif /* CIRCLEANNOTATOR_H_ */
