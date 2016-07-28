/*
 * circleAnnotator.cpp
 *
 *  Created on: Jul 1, 2016
 *      Author: biorobotics
 */


#include "../include/circleAnnotator.h"

using namespace std;
using namespace cv;

void mouse_click_for_reference(int event, int x, int y, int flags, void *param);
cv::Point pt1, pt2;
bool circle_annotation_mode;
bool second_click;

static cv::Scalar red = cv::Scalar(0, 0, 255);
static cv::Scalar green = cv::Scalar(0, 255, 0);
static cv::Scalar blue = cv::Scalar(255, 0, 0);
static cv::Scalar white = cv::Scalar(255, 255, 255);
static cv::Scalar black = cv::Scalar(0, 0, 0);

bool circleAnnotator::annotate_frame_with_reference(cv::Mat frame, double time)
{
	cout << "annotate_frame_with_reference" << endl;

		using namespace std;

		char key = ' ';
		bool go_to_next_frame = false;
		pt1 = cv::Point(0, 0); pt2 = cv::Point(0, 0);

		cvNamedWindow("Annotated frame", CV_WINDOW_NORMAL);

		while (go_to_next_frame == false)
		{
			cv::Mat frame_for_reference;
			frame.copyTo(frame_for_reference);

			putText(frame_for_reference, "1) click on the circonference of the circle", cvPoint(30, 30),
					FONT_HERSHEY_SIMPLEX, 1.5, red, 3);
			putText(frame_for_reference, "2) click on the diametrically opposite point", cvPoint(30, 80),
					FONT_HERSHEY_SIMPLEX, 1.5, red, 3);
			putText(frame_for_reference, "Press 'q' to skip this frame, press X to stop annotating this video", cvPoint(30, 130),
					FONT_HERSHEY_SIMPLEX, 1.5, red, 3);
			imshow("Annotated frame", frame_for_reference);

			circle_annotation_mode = true;
			second_click = false;
			while (pt2 == cv::Point(0, 0) && key != 'q' && key !='X')
			{
				cvSetMouseCallback("Annotated frame", mouse_click_for_reference, 0);
				key = cvWaitKey(100);
			}
			circle_annotation_mode = false;
			second_click = false;

			if (pt2 == cv::Point(0, 0))
			{
				cout << "No reference circle for this frame" << endl;
				go_to_next_frame = true;
			}
			else
			{
				frame.copyTo(frame_for_reference);
				cv::Point center = cv::Point((pt1.x+pt2.x)/2, (pt1.y+pt2.y)/2);
				double radius = norm(pt2 - pt1)/2
						;
				cv::circle(frame_for_reference, center, 10, cv::Scalar(0, 255, 0), -1, 8, 0);
				cv::circle(frame_for_reference, center, radius, cv::Scalar(255, 0, 0), 5, 8, 0);

				putText(frame_for_reference, "Does the reference look ok? (Y/N)", cvPoint(30, 30),
						FONT_HERSHEY_SIMPLEX, 1.5, red, 3);
				imshow("Annotated frame", frame_for_reference);
				cvWaitKey(1);

				cout << "Does the reference look ok? (Y/N)" << endl;
				key = cvWaitKey(0);
				if (key == 'y' || key == 'Y')
				{
					add_reference_circle(time, center, radius);
					cout << "Reference saved" << endl;
					go_to_next_frame = true;
				}
				else
				{
					cout << "Ok, retry" << endl;
					pt1 = cv::Point(0, 0); pt2 = cv::Point(0, 0);
				}
			}
		}

		if (key != 'X')
			return false;
		else
			return true; // Had enough!

		cout << "End of this frame" << endl;
}
void circleAnnotator::write_reference_file(std::string reference_file_name)
{
	cout << "write_reference_file" << endl;

	std::ofstream reference_file;
	reference_file.open(reference_file_name.c_str());
	if (reference_file.is_open())
	{
		for (unsigned int i = 0; i < saved_references.size(); i++)
		{
			reference_file << saved_references[i].time << ", " <<
				saved_references[i].circle[0] << ", " << saved_references[i].circle[1] << ", " <<
				saved_references[i].circle[2] << endl;
		}
		reference_file.close();
	}
	else
		std::cerr << "Unable to open reference file to write" << std::endl;

}

void circleAnnotator::add_reference_circle(double time, cv::Point point1, double radius)
{
	cout << "add_reference_circle" << endl;

	cv::Vec3f reference_circle = cv::Vec3f(point1.x, point1.y, radius);
	saved_references.push_back(reference(time, reference_circle));
}

void mouse_click_for_reference(int event, int x, int y, int flags, void *param)
{
	using namespace cv;

	switch (event)
	{
	case CV_EVENT_LBUTTONDOWN:
	{
		if (circle_annotation_mode)
		{
			std::cout << "Mouse Pressed" << std::endl;
			if (second_click == false)
			{
				pt1.x = x;
				pt1.y = y;
				second_click = true;
			}
			else
			{
				pt2.x = x;
				pt2.y = y;
			}
		}
		break;
	}

	}

}


