#include <iostream>
#include <fstream>
#include "opencv2/opencv.hpp"
#include "../include/odometry.h"
#include "../include/appearanceOdometryEstimator.h"
#include "../include/circleOdometryEstimator.h"
#include "../include/circleAnnotator.h"

using namespace std;

void write_odometry_on_file(odometry model, ofstream& myfile);

int main()
{

	/* Options */
	string video_file = "./lab_pipe_videos/run_1_GOPRO_final.mp4";
	int frame_step = 5;
	int starting_time = 2; //[sec]


	/************************* INITIALIZE VIDEO *************************/

	/* Structures */
	cv::VideoCapture video;
	cv::Mat frame;
	int frame_idx; // [-]
	double current_time; //[ms]
	double fps;

	/* Load video */
	video.open(video_file);
	if (!video.isOpened())
	{
		cerr << "Could not open video" << endl;
		return 1;
	}

	video >> frame;

	fps = video.get(CV_CAP_PROP_FPS);
	frame_idx = video.get(CV_CAP_PROP_POS_FRAMES);
    current_time = frame_idx / fps * 1000; //[ms]


    /************************** VISUAL ODOMETRY **************************/

	/* Structures */
    odometry appearanceOdometry, circleOdometry;
    appearanceOdometryEstimator myAppearanceOdometryEstimator;
    circleOdometryEstimator myCircleOdometryEstimator;

	std::ofstream appearance_odometry_file, feature_odometry_file ;
	appearance_odometry_file.open("appearance_odometry.csv");
	feature_odometry_file.open("circle_odometry.csv");

	//For each frame
	while (true)
	{

		//Skip unwanted frames
		for (int y = 0; y < frame_step; y++)
			video >> frame;
		//Exit if end of the video
		if (frame.empty())
			break;

		current_time = video.get(CV_CAP_PROP_POS_FRAMES) / fps; //[sec]
		//Skip till starting time
		if (current_time<starting_time)
			continue;

		cout << endl << "===================================================" << endl;
		cout << "time: "<< current_time << " [sec]" << endl;


		appearanceOdometry = myAppearanceOdometryEstimator.compute(frame);
		appearanceOdometry.time = current_time *1000; // [ms]


		circleOdometry = myCircleOdometryEstimator.compute(frame, current_time);
		circleOdometry.time = current_time *1000; // [ms]


		write_odometry_on_file(appearanceOdometry, appearance_odometry_file);
		write_odometry_on_file(circleOdometry, feature_odometry_file);

		cout << endl<< "===================================================" << endl << endl;
        char key = (char)cvWaitKey(50);

        if( key  == 27  ) // 27 is ESC
        {
        	cout << "ESC pressed. Exiting" << endl;
            break;
        }

        if( key == 'p')
        {
        	cout << "PAUSED. Press any key to continue" << endl;
        	cvWaitKey(0);
    		cout << endl<< "===================================================" << endl << endl;
        }

	}

	cout << "End of video "<< endl;
	cvWaitKey();

	return 0;
}

void write_odometry_on_file(odometry model, std::ofstream& myfile)
{
	myfile << model.time << ", "
		<< model.dx << ", " << model.dy << ", " << model.dz << ", "
		<< model.sigma_dx << ", " << model.sigma_dy << ", " << model.sigma_dz << ", "
		<< model.droll << ", " << model.dpitch << ", " << model.dyaw << ", "
		<< model.sigma_droll << ", " << model.sigma_dpitch << ", " << model.sigma_dyaw << ", "
		<< model.dR.at<double>(0,0) << ", " << model.dR.at<double>(0,1) << ", " << model.dR.at<double>(0,2) << ", "
		<< model.dR.at<double>(1,0) << ", " << model.dR.at<double>(1,1) << ", " << model.dR.at<double>(1,2) << ", "
		<< model.dR.at<double>(2,0) << ", " << model.dR.at<double>(2,1) << ", " << model.dR.at<double>(2,2) << ", "
		<< model.available << ", "
		<< model.joint_x << ", " << model.joint_y << ", " << model.joint_z << endl;

}

int add_reference_circles()
{
	cout << "This is ADD REFERENCE CIRCLES procedure" << endl;
	cvWaitKey();

	/* Options */
	string video_file = "./lab_pipe_videos/run_1_GOPRO_final.mp4";
//	string video_file = "./lab_pipe_videos/GOPR0857_ud_WMP.mp4";
	int frame_step = 5;


	/************************* INITIALIZE VIDEO *************************/

	/* Structures */
	cv::VideoCapture video;
	cv::Mat frame;
	circleAnnotator annotator;
	int frame_idx; // [-]
	double current_time; //[ms]
	double fps;


	/* Load video */
	video.open(video_file);
	if (!video.isOpened())
	{
		cerr << "Could not open video" << endl;
		return 1;
	}

	video >> frame;

	fps = video.get(CV_CAP_PROP_FPS);
	current_time = frame_idx / fps * 1000; //[ms]

	cvWaitKey(1);

	cv::namedWindow("INPUT VIDEO", CV_WINDOW_NORMAL);

	while (1>0)
	{
		//Skip unwanted frames
		for (int y = 0; y < frame_step; y++)
			video >> frame;
		//Exit if end of the video
		if (frame.empty())
			break;

		current_time = video.get(CV_CAP_PROP_POS_FRAMES) / fps; //[sec]

		cout << current_time << ":" << endl;
		imshow("INPUT VIDEO", frame);
		cvWaitKey(1);

		// Exit if user pressed X
		bool enough = annotator.annotate_frame_with_reference(frame, current_time);

		if (enough)
			break;
	}
	cout << "Writing reference circles in a file" << endl;

	annotator.write_reference_file("reference_circles.csv");
	video.release();

	cout << "End of ADD REFERENCE CIRCLES procedure" << endl;
	cvWaitKey(1);
}

