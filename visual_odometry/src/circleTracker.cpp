/*
 * circleTracker.cpp
 *
 *  Created on: May 28, 2016
 *      Author: biorobotics
 */

#include "../include/circleTracker.h"

using namespace std;
using namespace cv;

// Q: uncertainty of motion model
const int Q_radius = 10;
const int Q_center = 10000;
// R; uncertainty of measurements
const int R_all = 20;
// Initial state covariance
const int P0_position = R_all;
const int P0_center_dot = Q_center;
const int P0_radius_dot = Q_radius;

// PDAF parameters:
// PG:
const float PG = 0.99f;
// gamma: coefficient dependent on PG
const float gamma_PDAF = 11.34f;

// Number of validations needed to consider a track supported
const int needed_validations = 4;

//Frame size
const int width = 1920;
const int height = 1080;

// Probability of detection
// If the pipe joint circle is completely contained inside the frame and is big enough
float PD_inside_frame = 0.99;
// If its circumferences goes beyond the frame size or is too small
float PD_outside_frame = 0.0;

// Maximum allowed uncertainty on the radius size over radius size
float max_allowed_sigma_r_percentage = 30; //[%]

// Minimum size of the circle to be considered detectable
// (should be coordinsted with Hough detector min circle)
int min_visible_r = 150; //[pxl]

// Battachayara distance among tracks to consider them too similar
float distance_threshold = 1000.0f;

static cv::Scalar red = cv::Scalar(0, 0, 255);
static cv::Scalar green = cv::Scalar(0, 255, 0);
static cv::Scalar blue = cv::Scalar(255, 0, 0);
static cv::Scalar white = cv::Scalar(255, 255, 255);
static cv::Scalar black = cv::Scalar(0, 0, 0);



circleTracker::circle circleTracker::update(Vec3f detected_circle, double time)
{
	circle tracked_circle;

	// Initalize a new track with the detected circle and zero speed
	init_track(detected_circle);

	// Going backwards through the list of existing
	// candidate tracks, so I can delete tracks
	for (unsigned int track_idx = list_candidate_tracks.size()-1; track_idx --> 0; )
	{
		candidate_track track = list_candidate_tracks[track_idx];

		// If too tracks are too similar, delete the worst one
		bool current_track_was_erased = remove_duplicate_track(track_idx);
		if (current_track_was_erased)
			continue;

		// If the track survived the similarity-based deletion, update it
		// with the detected circle
		update_candidate_track(track, detected_circle, time - previous_time);

		if (track.current_status == candidate_track::to_kill)
			list_candidate_tracks.erase(list_candidate_tracks.begin()+track_idx);

		list_candidate_tracks[track_idx] = track;
	}

	tracked_circle = choose_track();

	plot_tracks(detected_circle);

	previous_time = time;

	return tracked_circle;
}

/* OLD FUNCTIONS */

void circleTracker::init_track(Vec3f init_circle)
{
	candidate_track new_candidate_track;

	float init_circle_x = init_circle[0];
	float init_circle_y = init_circle[1];
	float init_circle_radius = init_circle[2];

	Mat A = (Mat_<float>(6, 6) <<
		1, 0, 0, 1, 0, 0,
		0, 1, 0, 0, 1, 0,
		0, 0, 1, 0, 0, 1,
		0, 0, 0, 1, 0, 0,
		0, 0, 0, 0, 1, 0,
		0, 0, 0, 0, 0, 1);
	A.copyTo(new_candidate_track.A);

	Mat C = (Mat_<float>(3, 6) << 1, 0, 0, 0, 0, 0,
		0, 1, 0, 0, 0, 0,
		0, 0, 1, 0, 0, 0);
	C.copyTo(new_candidate_track.C);

	Mat Q = (Mat_<float>(6, 6) <<
		(float)Q_center, 0, 0, 0, 0, 0,
		0, (float)Q_center, 0, 0, 0, 0,
		0, 0, (float)Q_radius, 0, 0, 0,
		0, 0, 0, (float)Q_center, 0, 0,
		0, 0, 0, 0, (float)Q_center, 0,
		0, 0, 0, 0, 0, (float)Q_radius);
	Q.copyTo(new_candidate_track.Q);

	new_candidate_track.R.create(3, 3, CV_32F);
	setIdentity(new_candidate_track.R, R_all);


	Mat x0 = (Mat_<float>(6, 1) <<
		init_circle_x,
		init_circle_y,
		init_circle_radius,
		0,
		0,
		0);


	Mat P0 = (Mat_<float>(6, 6) <<
		(float)P0_position, 0, 0, 0, 0, 0,
		0, (float)P0_position, 0, 0, 0, 0,
		0, 0, (float)P0_position, 0, 0, 0,
		0, 0, 0, (float)P0_center_dot, 0, 0,
		0, 0, 0, 0, (float)P0_center_dot, 0,
		0, 0, 0, 0, 0, (float)P0_radius_dot);

	x0.copyTo(new_candidate_track.x_a_posteriori);
	P0.copyTo(new_candidate_track.P_a_posteriori);


	new_candidate_track.current_status = new_candidate_track.never_supported;
	new_candidate_track.missed_validations = needed_validations;

	list_candidate_tracks.push_back(new_candidate_track);
}

circleTracker::circle circleTracker::choose_track()
{
	circle current_circle;

	int current_track_idx = -1;;
	bool current_is_previous_current = false;
	double smallest_P_norm = numeric_limits<double>::max();

	for (unsigned int idx = 0; idx < list_candidate_tracks.size(); idx++)
	{
		// If the previously chosen track is still around keep it as the current
		if (list_candidate_tracks[idx].current_status == candidate_track::current)
		{
			current_track_idx = idx;
			current_is_previous_current = true;
			break;
		}

		// Else consider the ones which are supported currently and choose the one with smaller uncertainty
		if (list_candidate_tracks[idx].current_status == candidate_track::supported)
			if (norm(list_candidate_tracks[idx].P_a_posteriori) < smallest_P_norm)
			{
				current_track_idx = idx;
				smallest_P_norm = norm(list_candidate_tracks[idx].P_a_posteriori);
			}
	}

	if (current_track_idx < 0)
		current_circle.available = false;
	else
	{
		list_candidate_tracks[current_track_idx].current_status = candidate_track::current;
		current_circle.available = true;
		list_candidate_tracks[current_track_idx].x_a_posteriori(cvRect(0, 0, 1, 3)).copyTo(current_circle.position);
		list_candidate_tracks[current_track_idx].P_a_posteriori.copyTo(current_circle.uncertainty);
		if (current_is_previous_current)
			current_circle.new_track = false;
		else
			current_circle.new_track = true;
	}

	return current_circle;
}

void circleTracker::update_candidate_track(candidate_track& track, Vec3f circle, double dt)
{
	/* PDAF */

	const float PI = 3.14159265f;
	Mat A, C, Q, R, x_a_priori, P_a_priori,
		z, v, S, z_a_priori,
		x_a_posteriori_km1, P_a_posteriori_km1,
		x_a_posteriori_k, P_a_posteriori_k;

	track.A.copyTo(A);
	Mat A_DT = (Mat_<float>(3, 3) <<
						(float)dt, 0, 0,
						0, (float)dt, 0,
						0, 0, (float)dt);
	A_DT.copyTo(A(Rect(3, 0, 3, 3)));

	track.C.copyTo(C);
	track.x_a_priori.copyTo(x_a_priori);
	track.P_a_priori.copyTo(P_a_priori);
	track.x_a_posteriori.copyTo(x_a_posteriori_km1);
	track.P_a_posteriori.copyTo(P_a_posteriori_km1);
	track.Q.copyTo(Q);
	track.R.copyTo(R);

	int missed_measurements = track.missed_validations;

	z.create(3, 1, CV_32F);
	z = (Mat_<float>(3, 1) << circle[0], circle[1], circle[2]);

	/* PREDICTION */
	Mat Atransp;
	transpose(A, Atransp);

	P_a_priori = A*P_a_posteriori_km1*Atransp + Q; //covariance of predicted state
	x_a_priori = A*x_a_posteriori_km1; // predicted state
	z_a_priori = C*x_a_priori;

	float PD = compute_PD(z_a_priori);

	/* INNOVATION */
	Mat Ctransp;
	transpose(C, Ctransp);
	v = z - z_a_priori; //innovation
	S = C*P_a_priori*Ctransp + R; //innovation covariaance
	float detS = (float)determinant(S);


	/* GAIN */
	Mat Sinv;
	invert(S, Sinv);
	Mat K; //gain
	K = P_a_priori*Ctransp*Sinv;


	/* MEASUREMENT VALIDATION */
	float nZ = 3.0f; // dimension of the measurement vector
	float c_3 = 4 * PI / 3; //unit hypersphere of dimension 3
	float V_volume = (float)(c_3 * pow(gamma_PDAF, nZ / 2) * sqrt(detS)); // volume of the validating region

	/* ASSOCIATION PROBABILITIES */
	/*gaussian distance of the measurement from expected one*/
	Mat vTransp; //innovation transpose
	transpose(v, vTransp);
	float f = (float)(1 / (sqrt(pow(2 * PI, 3)*detS)));
	Mat vSvMat = vTransp*Sinv*v;
	float vSv = vSvMat.at<float>(0, 0);
	float e = (float)exp(-(0.5)*vSv);
	float N = (float)(f*e); // pdf of a multivariate normal distribution
	/*likelihood*/
	float L = N*PD*V_volume; // likelihood of the measurement to originate from the target
	/*association probabilties*/
	float betaZero = (1 - PD*PG) / (1 - PD*PG + L);
	float betaOne = L / (1 - PD*PG + L);

	/*A POSTERIORI */
	x_a_posteriori_k = x_a_priori + K*betaOne*v;

	Mat P_corrected, P_spread;
	Mat KTransp;
	transpose(K, KTransp);
	P_corrected = P_a_priori - K*S*KTransp;
	P_spread = K*(betaOne*v*vTransp - betaOne*betaOne*v*vTransp)*KTransp;
	P_a_posteriori_k = betaZero*P_a_priori + (1 - betaZero)*P_corrected; //+ P_spread;


	R.copyTo(track.R);
	x_a_priori.copyTo(track.x_a_priori);
	P_a_priori.copyTo(track.P_a_priori);
	x_a_posteriori_k.copyTo(track.x_a_posteriori);
	P_a_posteriori_k.copyTo(track.P_a_posteriori);

	/* STATUS CHANGE */

	// A track is supported if has 0 missed measurements in the last 'needed_validations' detections

	if (betaZero >= 0.5) // Measurement did not agree
		missed_measurements = min(missed_measurements + 1, needed_validations);
	if (betaZero < 0.5) // Measurement agreed
		missed_measurements = max(missed_measurements - 1, 0);


	if (track.current_status == candidate_track::never_supported && missed_measurements==0)
		track.current_status = candidate_track::supported ;

	// A track looses its support is the uncertainty of the radius grows too large

	float sigma_r = P_a_posteriori_k.at<float>(2, 2);
	float r = x_a_posteriori_k.at<float>(2);
	float percentage_of_r_error = sigma_r / r * 100;

	if (percentage_of_r_error >  max_allowed_sigma_r_percentage)
		track.current_status = candidate_track::to_kill;

	track.missed_validations = missed_measurements;

	if (false)
	{
		cout << "----------- track ---------------" << endl;
		cout << "missed_measurements: " << missed_measurements << endl;
		cout << "current_status: ";
		switch(track .current_status)
		{
		    case candidate_track::never_supported  : cout << "never_supported";   break;
		    case candidate_track::supported: cout << "supported"; break;
		    case candidate_track::current : cout << "current";  break;
		    case candidate_track::to_kill : cout << "to_kill (SOMETHING IS WRONG)";  break;
		}
		cout << endl;


		cout << "x_a_posteriori_km1" << endl << x_a_posteriori_km1 << endl;
		cout << "P_a_posteriori_km1" << endl << P_a_posteriori_km1 << endl;
		cout << "Q" << endl << Q << endl;
		cout << "R" << endl << R << endl;

		cout << "dt = " << dt << " [ms]" << endl;
		cout << "A" << endl << A << endl;
		cout << "C" << endl << C << endl;

		cout << "x_a_priori" << endl << x_a_priori << endl;
		cout << "P_a_priori" << endl << P_a_priori << endl;
		cout << "P_a_priori = A*P_a_posteriori_km1*Atransp + Q" << endl;
		cout << "P_a_posteriori_km1 " << endl << P_a_posteriori_km1 << endl;
		cout << "(A*P_a_posteriori_km1)*Atransp = " << endl << A*P_a_posteriori_km1 << endl << "*" << endl << Atransp << endl;

		cout << "z_a_priori" << endl << z_a_priori << endl;
		cout << "z" << endl << z << endl;

		cout << "- PD = " << PD << endl;

		cout << "v" << endl << v << endl;
		cout << "S" << endl << S << endl;
		cout << "S = C*P_a_priori*Ctransp + R = " << endl << C*P_a_priori*Ctransp <<"+" << endl<< R << endl;
		cout << "Sinv" << endl << Sinv << endl;
		cout << "A*P_a_posteriori_km1*Atransp" << endl << A*P_a_posteriori_km1*Atransp << endl;

		cout << "det(S): " << detS << endl;

		cout << "K" << endl << K << endl << endl;
		cout << "v*S-1*v = vx*1/Sx*vx + vy*1/Sy*vy + vr*1/Sr*vr = "
			<< v.at<float>(0)*v.at<float>(0)*1/S.at<float>(0, 0) << " + "
			<< v.at<float>(1)*v.at<float>(1)*1/S.at<float>(1, 1) << " + "
			<< v.at<float>(2)*v.at<float>(2) * 1 / S.at<float>(2, 2) << " = " << vSv << endl;
		cout << "v*S-1*v = " << vSv << endl;
		cout << "e = exp(-0.5*vSv) " << e << endl;
		cout << "f = fun(detS) = " << f << endl;
		cout << "N = f*e = " << N << endl;
		cout << "V_volume = " << V_volume << endl;
		cout << "L = N*PD*V_volume = " << L << endl;
		cout << "L = everythingelse*e =" << L/e << "*"<< e << endl;
		cout << "(1 - PD*PG) = "<< (1 - PD*PG)  << endl;
		cout << "betaZero = (1 - PD*PG) / (1 - PD*PG + L) " << betaZero << endl;
		cout << "(if betaZero < 0.5 measurement agreed)" <<endl;

		cout << "k*v " << endl << K*v << endl << endl;
		cout << "x_a_posteriori_k " << endl << x_a_posteriori_k << endl;

		cout << "K*S*KTransp " << endl << K*S*KTransp << endl;
		cout << "P_corrected " << endl << P_corrected << endl;
		cout << "P_spread " << endl << P_spread << endl;
		cout << "P_a_posteriori_k " << endl << P_a_posteriori_k << endl;

		cout << "percentage_of_r_error = " << percentage_of_r_error << endl;
		cout << "max_allowed_sigma_r_percentage = " << max_allowed_sigma_r_percentage << endl;

//		cvWaitKey();
	}
}

float circleTracker::compute_PD(Mat z_predicted)
{
	// TODO add uncertainty on the predicted circle
	int x = (int)z_predicted.at<float>(0, 0);
	int y = (int)z_predicted.at<float>(1, 0);
	int r = (int)z_predicted.at<float>(2, 0);

	//First implementation
	int border_width = 0; //frame.size().width / 6;
	int upper_limit = 0 + border_width;
	int left_limit = 0 + border_width;
	int right_limit = width - border_width;
	int lower_limit = height - border_width;

	float PD;

	//If the predicted circle is even partly outside of the frame
	//then it's impossible that it's predicted
	//otherwise it's predicted with high probability
	if ((x - left_limit) < r || (right_limit - x) < r || (y - upper_limit) < r || (lower_limit - y) < r)
		PD = PD_outside_frame;
	else
		PD = PD_inside_frame;

	//If the radius is below the usually smallest detectable, same
	if (r < min_visible_r)
		PD = PD_outside_frame;

	return PD;
}

Mat circleTracker::display_one_circle_with_std(const Mat& colorImg, candidate_track track)
{
	cv::Scalar color;

	if (track.current_status == candidate_track::current)
		color = green;
	else if (track.current_status == candidate_track::supported)
		color = red;
	else if (track.current_status == candidate_track::never_supported)
		color = white;

	// Plot a posteriori
	Vec3f circle;
	track.x_a_posteriori(cvRect(0, 0, 1, 3)).copyTo(circle);

	float std = track.P_a_posteriori.at<float>(2, 2); // pick covariance o

	Mat display = colorImg.clone();
	Point center(cvRound(circle[0]), cvRound(circle[1]));
	int radius = cvRound(circle[2]);
	cv::circle(display, center, 3, Scalar(0, 255, 0), -1, 8, 0);
	cv::circle(display, center, radius, color, 3, 8, 0);
	cv::circle(display, center, radius + std , color, 2, 8, 0);
	if ((radius - std) > 0)
		cv::circle(display, center, radius - std, color, 2, 8, 0);

	// Plot a priori
	if (track.x_a_priori.cols > 0)
	{
		color = red;
		track.x_a_priori(cvRect(0, 0, 1, 3)).copyTo(circle);

		std = track.P_a_priori.at<float>(2, 2); // pick covariance o

		center = cv::Point(cvRound(circle[0]), cvRound(circle[1]));
		radius = cvRound(circle[2]);
		cv::circle(display, center, 3, Scalar(0, 255, 0), -1, 8, 0);
		cv::circle(display, center, radius, color, 3, 8, 0);
		cv::circle(display, center, radius + std , color, 2, 8, 0);
		if ((radius - std) > 0)
			cv::circle(display, center, radius - std, color, 2, 8, 0);
	}

	return display;
}

void circleTracker::plot_tracks(Vec3f detected_circle)
{
	//Plotting
	for (unsigned int idx = 0; idx < list_candidate_tracks.size(); idx++)
	{
		cout << "- Track #"<< idx << " is ";
		switch(list_candidate_tracks[idx].current_status)
		{
		    case candidate_track::never_supported  : cout << "never_supported";   break;
		    case candidate_track::supported: cout << "supported"; break;
		    case candidate_track::current : cout << "current";  break;
		    case candidate_track::to_kill : cout << "to_kill (SOMETHING IS WRONG)";  break;
		}
		cout << ", with " << list_candidate_tracks[idx].missed_validations << " missed validations" << endl;

		Mat frame_with_track;

		frame_with_track = display_one_circle_with_std(current_frame, list_candidate_tracks[idx]);
		frame_with_track = display_one_circle(frame_with_track, detected_circle);


		namedWindow("Track #"+to_string(idx), WINDOW_NORMAL);
		imshow("Track #"+to_string(idx), frame_with_track);

	}
}

cv::Mat circleTracker::display_one_circle(const cv::Mat& colorImg, cv::Vec3f circle)
{
	cv::Scalar color = blue;

	Mat display = colorImg.clone();
	Point center(cvRound(circle[0]), cvRound(circle[1]));
	int radius = cvRound(circle[2]);
	cv::circle(display, center, 3, Scalar(0, 255, 0), -1, 8, 0);
	cv::circle(display, center, radius, color, 3, 8, 0);

	return display;
}

bool circleTracker::remove_duplicate_track(unsigned int track_idx)
{
	//Check if the given track is too similar to other tracks

	bool given_track_was_erased = false;
	for (unsigned int other_track = list_candidate_tracks.size(); other_track --> track_idx+1; )
	{
		// Compute track similarity according to Battachayara distance
		float distance = compute_track_distance(track_idx, other_track);
		if (distance < distance_threshold)
		{
			// If they are too close, delete the worst track
			unsigned int idx_to_erase = choose_worse_track(track_idx, other_track);
			list_candidate_tracks.erase(list_candidate_tracks.begin()+idx_to_erase);
			cout << "Tracks "<<track_idx<< " and "<< other_track << " are very similar" << endl;
			cout << "Deleting track  "<<idx_to_erase<< endl;

			if (idx_to_erase == track_idx)
				given_track_was_erased = true;
		}
		else
		{
			cout << "Tracks "<<track_idx<< " and "<< other_track << " are not very similar, keeping both" << endl;
			cout << "Their distance is "<<distance << endl;
		}
		if (given_track_was_erased)
			continue;
	}

	return given_track_was_erased;
}

float circleTracker::compute_track_distance(unsigned int trackOne_idx, unsigned int trackTwo_idx)
{
	cv::Mat xOne, xTwo, POne, PTwo;

	candidate_track trackOne = list_candidate_tracks[trackOne_idx];
	candidate_track trackTwo = list_candidate_tracks[trackTwo_idx];

	trackOne.x_a_posteriori.copyTo(xOne);
	trackOne.P_a_posteriori.copyTo(POne);
	trackTwo.x_a_posteriori.copyTo(xTwo);
	trackTwo.P_a_posteriori.copyTo(PTwo);

	//Compute Bhattacharyya distance
	cv::Mat P = (POne + PTwo)*0.5;
	cv::Mat PInv;
	cv::invert(P, PInv);
	cv::Mat meanDiff = xOne - xTwo;
	cv::Mat meanDiffTransp;
	cv::transpose(meanDiff, meanDiffTransp);
	float detP = (float)cv::determinant(P);
	float detPone = (float)cv::determinant(POne);
	float detPTwo = (float)cv::determinant(PTwo);

	// Bhattacharyya distance
	cv::Mat DMat = 0.125f * meanDiffTransp *PInv *meanDiff + 0.5f * log(detP / (sqrt(detPone*detPTwo)));
	float D = DMat.at<float>(0, 0);
	//cout << "Computed distance: " << D << endl;

	return D;
}

unsigned int circleTracker::choose_worse_track(unsigned int trackOne_idx, unsigned int trackTwo_idx)
{
	// Chose which track to erase among two given ones

	cv::Mat xOne, xTwo, POne, PTwo;

	candidate_track trackOne = list_candidate_tracks[trackOne_idx];
	candidate_track trackTwo = list_candidate_tracks[trackTwo_idx];

	// If one of them is current, erase the other one
	if (trackOne.current_status == candidate_track::current)
		return trackTwo_idx;
	if (trackTwo.current_status == candidate_track::current)
		return trackOne_idx;

	// If only one of them is supported, erase the other one
	if (trackOne.current_status== candidate_track::supported && !
			(trackTwo.current_status== candidate_track::supported))
		return trackTwo_idx;
	if (trackTwo.current_status== candidate_track::supported && !
			(trackOne.current_status== candidate_track::supported))
		return trackOne_idx;

	//If both of them are not supported, erase the one with the bigger number of missed validations
	if (trackTwo.missed_validations >= trackTwo.missed_validations)
		return trackTwo_idx;
	else
		return trackOne_idx;

	cout << "IF you are here something went wrong!" << endl;
	cvWaitKey();

	return trackTwo_idx;

}



