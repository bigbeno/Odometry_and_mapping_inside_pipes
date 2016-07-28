/*
 * appearanceOdometryEstimator.cpp
 *
 *  Created on: May 24, 2016
 *      Author: biorobotics
 */

#include "../include/appearanceOdometryEstimator.h"

using namespace std;
using namespace cv;

// Focal length
const float focal =830;
// Frame size
const int width = 1920;
const int height = 1080;
// Camera matrix
const cv::Mat_<float>  K = (cv::Mat_<float>(3, 3) <<
		focal, 0, 1080, //-50
		0, focal, 540, //+200
		0, 0, 1);

const cv::Scalar red = cv::Scalar(0, 0, 255);
const cv::Scalar green = cv::Scalar(0, 255, 0);
const cv::Scalar blue = cv::Scalar(255, 0, 0);
const cv::Scalar white = cv::Scalar(255, 255, 255);
const cv::Scalar black = cv::Scalar(0, 0, 0);

/** PUBLIC **/

odometry appearanceOdometryEstimator::compute(Mat originalFrame)
{
	cout << endl <<"-------------- FEATURE-BASED ODOMETRY --------------" << endl;
	time_t tstart;
	tstart = time(0);
	odometry appearanceOdometry;

	// Writes on terminal the information about the feature matches from previous iteration
	write(carried_matches, "beginning of loop");

	// Save current frame
	originalFrame.copyTo(current_frame);

	// Process the image (just convert to grey scale)
	Mat frame = process_image(originalFrame);

	// Extract features
	vector<cv::Point2f> extracted_features = extract_features(frame);
	int features_extracted = extracted_features.size();

	draw_points(frame, extracted_features, "Extracted features");

	// If no features from previous iteration no estimate is possible
	if (carried_matches.old_features.empty())
		appearanceOdometry.available = false;
	else
	{

		// Match features from previous frame
		matchingFeatures feature_matches = match_features(frame, previous_frame, carried_matches);
		int features_matched_from_before = feature_matches.new_features.size(); //Number of surviving features

		// Compute Essential matrix
		Mat E = compute_E(feature_matches, K);
		int features_surviving_E = feature_matches.new_features.size();

		// Compute rotation and translation
		Mat H = estimate_R_t(E, feature_matches);
		int features_surviving_Rt = feature_matches.new_features.size();

		if (features_surviving_Rt>0)
		{

			// Estimate 3D points from 2D feature matches
			feature_matches = triangulate_points(feature_matches, H);
			int features_surviving_triangulation = feature_matches.old_threeD.size();

			// Fix the unitary-norm translation by estimating the
			// scaling between current and previous translation
			double scale = scale_translation(feature_matches, H);

			// Write down motion estimate (no knowledge about uncertainty!)
			cout << endl << "* ODOMETRY:" << endl;
			appearanceOdometry.build_from_H(H);
			appearanceOdometry.available = true;

			// Keep track of history of a feature (for the sake of it)
			for (unsigned int i=0; i< feature_matches.new_features.size(); i++)
			{
				// Add new match to the history of a feature
				if (i<feature_matches.history.size())
					feature_matches.history[i].push_back(feature_matches.new_features[i]);
				else
				{
					// Create a history for a new feature
					vector<Point2f> new_history;
					new_history.push_back(feature_matches.old_features[i]);
					new_history.push_back(feature_matches.new_features[i]);
					feature_matches.history.push_back(new_history);
				}
			}

			carried_matches.old_features = feature_matches.new_features;
			carried_matches.old_threeD = feature_matches.new_threeD;
			carried_matches.history = feature_matches.history;

			write(carried_matches, "to propagate");
		}

	}

	// Add extracted features to the saved features
	carried_matches.old_features.insert( carried_matches.old_features.end(),
			extracted_features.begin(), extracted_features.end() );

	previous_frame = frame;

	cout << endl << "--> It took "<< difftime(time(0), tstart) <<" second(s) to do one odometry iteration"<< endl;

	return appearanceOdometry;
}


/** PRIVATE **/

Mat appearanceOdometryEstimator::process_image(Mat originalFrame)
{
	Mat frame;

	cout << endl << "* PROCESS IMAGE" << endl;
	cvtColor(originalFrame, frame, CV_BGR2GRAY);

	return frame;
}

vector<Point2f> appearanceOdometryEstimator::extract_features(Mat frame)
{
	vector<Point2f> features;

	cout << endl<< "* EXTRACT FEATURES" << endl;

	time_t tstart; tstart = time(0);

	features = extract_points_evenly(frame);

	cout << "It took "<< difftime(time(0), tstart) <<" second(s)."<< endl;

	return features;
}

appearanceOdometryEstimator::matchingFeatures appearanceOdometryEstimator::match_features
(Mat frame, Mat previous_frame, matchingFeatures previous_matches)
{
	matchingFeatures feature_matches;

	cout << endl<< "* MATCH FEATURES" << endl;

	time_t tstart; tstart = time(0);

	feature_matches = match_points(previous_frame, frame,
			previous_matches);

	cout << "It took "<< difftime(time(0), tstart) <<" second(s)."<< endl;

	return feature_matches;
}

Mat appearanceOdometryEstimator::compute_E(appearanceOdometryEstimator::matchingFeatures& feature_matches, Mat K)
{
	Mat E;
	cout << endl<< "* COMPUTE ESSENTIAL MATRIX" << endl;

	time_t tstart;
	tstart = time(0);

	int method = RANSAC;
	double prob = 0.99;
	double threshold = 3;

	double focal = K.at<float>(0,0);
	Point2f pp = Point2f(K.at<float>(0,2), K.at<float>(1,2));

	vector<uchar> status;

	vector<Point2f> points_previous = feature_matches.old_features;
	vector<Point2f> points_current = feature_matches.new_features;

	E =  findEssentialMat(points_previous, points_current, focal, pp, method, prob, threshold, status);
	int good_points_E = countNonZero(Mat(status));
	cout << "E likes "<<good_points_E<<" out of " <<points_previous.size()
		<<" (" << (float)(good_points_E)/(float)(points_previous.size()) *100<<"%)"<<endl;

	feature_matches.old_features = points_previous;
	feature_matches.new_features = points_current;

	draw_trace(current_frame, feature_matches, status, " Essential matrix computation");

	skim_points(feature_matches, status);

	cout << "It took "<< difftime(time(0), tstart) <<" second(s)."<< endl;

	return E;
}

Mat appearanceOdometryEstimator::estimate_R_t(Mat E, appearanceOdometryEstimator::matchingFeatures& feature_matches)
{

	cout << endl<< "* ESTIMATE CAMERA MOTION" << endl;

	time_t tstart;
	tstart = time(0);

	Mat H = Mat::zeros(4,4, CV_64F);

	vector<uchar> status;

	vector<Point2f> points_previous = feature_matches.old_features;
	vector<Point2f> points_current = feature_matches.new_features;

	double focal = K.at<float>(0,0);
	Point2f pp = Point2f(K.at<float>(0,2), K.at<float>(1,2));

	Mat R, t;
	int good_points_R = recoverPose(E, points_previous, points_current, R, t, focal, pp, status);

	assert(good_points_R == countNonZero(Mat(status)));
	cout << "R likes "<<good_points_R<<" out of " <<points_previous.size()
		<<" (" << (float)(good_points_R)/(float)(points_previous.size()) *100<<"%)"<<endl;


	// Warning, setting to 1 the status which is not 0
	// (for some reason recoverPose returns statuses with non-binary values
	for (unsigned int i=0; i<status.size(); i++)
	{
		if (status[i]>0)
			status[i] = 1;
	}

	feature_matches.old_features = points_previous;
	feature_matches.new_features = points_current;

	draw_trace(current_frame, feature_matches, status, " Rotation and translation extraction");

	skim_points(feature_matches, status);

	// Build homogenous transformation [R, t; 0 0 0 1]
	R.copyTo(H(Rect(0,0,3,3)));
	t.copyTo(H(Rect(3,0,1,3)));

	cout << "It took "<< difftime(time(0), tstart) <<" second(s)."<< endl;

	return H;
}

Mat appearanceOdometryEstimator::estimate_R_t_uncertainty(int a, int b, int c, int d, int e, double scale)
{
	Mat Sigma = Mat::zeros(6,1, CV_64F);

	cout << endl<< "* COMPUTE UNCERTAINTY ON CAMERA MOTION (todo)" << endl;
	Sigma.at<double>(0,0) = (double)a;
	Sigma.at<double>(1,0) = (double)b;
	Sigma.at<double>(2,0) = (double)c;
	Sigma.at<double>(3,0) = (double)d;
	Sigma.at<double>(4,0) = (double)e;

	//
	Sigma.at<double>(5,0) = (double)scale;

	return Sigma;
}

appearanceOdometryEstimator::matchingFeatures appearanceOdometryEstimator::triangulate_points
	( matchingFeatures matches, cv::Mat H)
{

	cout << endl<< "* TRIANGULATE POINTS" << endl;

	int matched_couples = matches.new_features.size();
	if (matched_couples==0)
	{
		cout << "NO SURVIVING MATCH TO COMPUTE ODOMETRY!" << endl;
		return matches;
	}


	// Use only features matches
	vector<Point2f> previous_features;
	vector<Point2f> current_features;
	previous_features.insert( previous_features.end(),
				matches.old_features.begin(), matches.old_features.begin()+ matched_couples);
	current_features.insert( current_features.end(),
				matches.new_features.begin(), matches.new_features.begin()+ matched_couples);

	cv::Mat camera_pose_old;
	cv::Mat camera_pose_new;

	cv::Mat K_64F;
	K.convertTo(K_64F, 6);

	// Consider previous camera pose to be identity
	camera_pose_old = K_64F*Mat::eye(3,4,6);
	// Consider current camera pose to be the estimated pose from previo
	camera_pose_new = K_64F * H(Rect(0,0,4,3));

	Mat three_D_points_homogeneous;

	triangulatePoints(camera_pose_old, camera_pose_new,
			previous_features, current_features, three_D_points_homogeneous);

	// Output of triangulatePoints is homogeneous points (4D), convert to 3D points
	Mat three_D_points_homogeneous_t = three_D_points_homogeneous.t();
	Mat three_D_points_nonhomogeneous;
	convertPointsFromHomogeneous(Mat(three_D_points_homogeneous.t()).reshape(4, 1), three_D_points_nonhomogeneous);
	vector<Point3f> three_D_points;
	three_D_points_nonhomogeneous.copyTo(three_D_points);

	// Chierality check
	vector<uchar> status;
	for (unsigned int i = 0; i < three_D_points.size(); i++)
		status.push_back((three_D_points[i].z > 0) ? 1 : 0);

	cout << "triangulation likes "<<countNonZero(Mat(status))
			<<" out of " <<matched_couples
			<<" (" << (float)(countNonZero(Mat(status)))/(float)(matched_couples) *100<<"%)"<<endl;


	// Compute mean reprojection error, for the sake of it
	Mat rvec; Rodrigues(H(Rect(0,0,3,3)), rvec);
	Mat tvec; H(Rect(3,0,1,3)).copyTo(tvec);
	vector<Point2f> reprojected_features;
	projectPoints(three_D_points, rvec, tvec, K_64F, Mat(), reprojected_features);
	double mean_reprojection_error = 0;
	vector<double> reprojection_error;
	for (unsigned int i = 0; i < reprojected_features.size(); i++)
	{
		if (status[i])
			reprojection_error.push_back(norm(current_features[i] - reprojected_features[i]));
		mean_reprojection_error = mean_reprojection_error + reprojection_error[i];
	}
	mean_reprojection_error = mean_reprojection_error / reprojection_error.size();
	cout << "mean reprojection error: " << mean_reprojection_error << endl;


	draw_flow(current_frame, current_features, reprojected_features, status, " triangulation (REPROJECTION ERROR) ");

	matches.new_threeD = three_D_points;
	skim_points(matches, status);
	three_D_points = matches.new_threeD;

	cout << endl;

	return matches;
}

double appearanceOdometryEstimator::scale_translation(matchingFeatures matches, cv::Mat& H)
{
	cout << endl<< "* SCALE TRANSLATION" << endl;

	int threeD_matched_couples = matches.old_threeD.size();

	// We need at least 2 3D points from previous iteration to compute relative scale
	// of the translation
	if (threeD_matched_couples<2)
	{
		cout << "NO at least 2 OLD THREE D TO COMPUTE SCALE!" <<endl;
		return 1
				;
	}

	// Use matched 3D points
	vector<Point3f> previous_threeD;
	vector<Point3f> current_threeD;
	previous_threeD.insert( previous_threeD.end(),
				matches.old_threeD.begin(), matches.old_threeD.begin()+ threeD_matched_couples);
	current_threeD.insert( current_threeD.end(),
				matches.new_threeD.begin(), matches.new_threeD.begin()+ threeD_matched_couples);


	//compute Scale
	double scale =  computeScale(previous_threeD, current_threeD);
	cout << "Computed scale is " << scale << endl;

	// Sclae the translation
	cv::Mat scaled_t_H;
	H.copyTo(scaled_t_H);
	Mat t;
	scaled_t_H(Rect(3,0,1,3)).copyTo(t);
	t = t*scale;
	t.copyTo(scaled_t_H(Rect(3,0,1,3)));

	cout << endl;

	scaled_t_H.copyTo(H);

	return scale;
}



vector<Point2f> appearanceOdometryEstimator::extract_points_evenly(Mat previous_frame)
{
	// We want to extract feature from the current frame but
	// we want them to be evenly distibuted across the frame

	namedWindow("extract_points_evenly previous frame", WINDOW_NORMAL);
	imshow("extract_points_evenly previous frame", previous_frame);

	vector<cv::Point2f> new_points_previous;

	// Mask out the darker part of the frame,
	// because you don't wanna extract featured in to heavy shadows
	Mat mask;
	double thr_otsu = threshold(previous_frame, mask, 0, 0, THRESH_TOZERO | THRESH_OTSU);
	threshold(previous_frame, mask, thr_otsu/2, 255, THRESH_BINARY );
	blur(mask, mask, cv::Size(50,50));
	threshold(mask, mask, 254, 255, THRESH_BINARY );
	namedWindow("Mask inflated", WINDOW_NORMAL); imshow("Mask inflated", mask);


	//Let's split the frame into ROIs and let's search for many features in all of them
	int width_of_roi = 640; int height_of_roi = 480;

	for (int i = 0; i<previous_frame.rows; i=i+height_of_roi)
	{
		for (int j = 0; j<previous_frame.cols; j=j+width_of_roi)
		{
			//Extract ROI
			int width = min(width_of_roi, previous_frame.cols - j);
			int height = min(height_of_roi, previous_frame.rows - i);
			Mat roi = previous_frame( Rect(j,i,width,height));
			Mat roi_mask = mask( Rect(j,i,width,height));

			//Goal 600 features for 640x480 (reducing to 400)
			int area = countNonZero(Mat(roi_mask));
			unsigned int max_features = area * 600 / (640 * 480);

			if (max_features>10)
			{

				//Inital params
				float MIN_FEATURE_QUALITY = 0.1;
				float MIN_FEATURE_DISTANCE = 8; //20
				int blockSize = 18;

				//Sub pixel search params
				Size SUB_PXL_FEATURE_SIZE = Size(9,9);
				TermCriteria SUB_PXL_TERM_CRITERIA = TermCriteria(TermCriteria::COUNT | TermCriteria::EPS, 100, 0.001);

				vector<cv::Point2f> new_points_previous_roi;
				bool terminate = false;

				while (!terminate)
				{
					new_points_previous_roi.clear();

					//Extract features
					goodFeaturesToTrack(roi, new_points_previous_roi, max_features, MIN_FEATURE_QUALITY, MIN_FEATURE_DISTANCE, roi_mask, blockSize);
					// Refine their subpixel position
					if (new_points_previous_roi.empty()==false)
						cornerSubPix(roi, new_points_previous_roi, SUB_PXL_FEATURE_SIZE, Size(-1, -1), SUB_PXL_TERM_CRITERIA);


					if (new_points_previous_roi.size() == max_features)
					{
						// Terminate the search in this ROI because enough features found
						terminate = true;
						for (unsigned int k=0; k< new_points_previous_roi.size(); k++)
						{
							new_points_previous_roi[k].x += j;
							new_points_previous_roi[k].y += i;
						}
						new_points_previous.insert(new_points_previous.end(), new_points_previous_roi.begin(), new_points_previous_roi.end());
					}
					else
					{
						MIN_FEATURE_QUALITY = MIN_FEATURE_QUALITY/1.1;

						if (MIN_FEATURE_QUALITY < 0.0005)
						{
							// Terminate the search in this ROI because the quality dropped too low
							terminate = true;
						}

					}
				}
			}
		}
	}


	cout << "Extracted "<<new_points_previous.size() << " points" << endl;

	return new_points_previous;
}

void appearanceOdometryEstimator::draw_points(Mat src, vector<Point2f> src_corners, string title, int block_size)
{
	// Draws the (extracted) points and eventually the area in which they were searched for

	Mat copy;
	src.copyTo(copy);
	for (unsigned int i=0; i< src_corners.size(); i++)
	{
		if (block_size==0)
			circle(copy, src_corners[i], 3, Scalar(0, 255, 0));
		else
		{
			Point pt1(src_corners[i].x - block_size/2, src_corners[i].y - block_size/2);
			Point pt2(src_corners[i].x + block_size/2, src_corners[i].y + block_size/2);

			rectangle(copy, pt1, pt2, Scalar(255, 255, 255));
		}
	}

	namedWindow(title, WINDOW_NORMAL);
	imshow(title, copy);

}

void appearanceOdometryEstimator::skim_points(matchingFeatures& features, vector<uchar> status)
{
	// Remove all data corresponding to a feature which must be deleted

	vector<Point2f> points_previous = features.old_features;
	vector<Point2f> points_current = features.new_features;
	vector<Point3f> points_threeD_previous = features.old_threeD;
	vector<Point3f> points_threeD_current = features.new_threeD;
	vector<vector<Point2f>> points_history = features.history;


	int removed_pts = remove_bad_points(points_previous, status);
	assert(points_previous.size() == points_current.size()-removed_pts );
	assert(points_previous.size() == (unsigned int)countNonZero(Mat(status)));
	remove_bad_points(points_current, status);
	remove_bad_points(points_threeD_previous, status);
	remove_bad_points(points_threeD_current, status);
	remove_bad_points(points_history, status);

	features.old_features = points_previous;
	features.new_features = points_current;
	features.old_threeD = points_threeD_previous;
	features.new_threeD = points_threeD_current;
	features.history = points_history;
}

int appearanceOdometryEstimator::remove_bad_points(vector<Point2f>& vectorOne, vector<uchar> status)
{
	size_t i, k = 0;
	for (i = k = 0; i < vectorOne.size(); i++)
	{
		if (!status[i])
			continue;
		vectorOne[k] = vectorOne[i];
		k = k + 1;
	}

	int removed_points = vectorOne.size() - k;

	vectorOne.resize(k);

	return removed_points;
}

int appearanceOdometryEstimator::remove_bad_points(vector<Point3f>& vectorOne, vector<uchar> status)
{
	size_t i, k = 0;
	for (i = k = 0; i < vectorOne.size(); i++)
	{
		if (!status[i])
			continue;
		vectorOne[k] = vectorOne[i];
		k = k + 1;
	}

	int removed_points = vectorOne.size() - k;

	vectorOne.resize(k);

	return removed_points;
}

int appearanceOdometryEstimator::remove_bad_points(vector<vector<Point2f>>& vectorOne, vector<uchar> status)
{
	size_t i, k = 0;
	for (i = k = 0; i < vectorOne.size(); i++)
	{
		if (!status[i])
			continue;
		vectorOne[k] = vectorOne[i];
		k = k + 1;
	}

	int removed_points = vectorOne.size() - k;

	vectorOne.resize(k);

	return removed_points;
}

appearanceOdometryEstimator::matchingFeatures appearanceOdometryEstimator::match_points(Mat previous_frame, Mat frame,
		matchingFeatures& previous_matches)
{

	// Mask out darkest regions of the frame
	Mat mask;
	double thr_otsu = threshold(frame, mask, 0, 0, THRESH_TOZERO | THRESH_OTSU);
	threshold(frame, mask, thr_otsu/2, 255, THRESH_BINARY );
	blur(mask, mask, cv::Size(50,50));
	threshold(mask, mask, 254, 255, THRESH_BINARY );

	// Matching of previous features in current frame
	vector<uchar> status;
	vector<Point2f> points_previous = previous_matches.old_features;
	vector<Point2f> points_current;
	matching(previous_frame, frame, points_previous, points_current, status, mask);

	int good_points_Lk = countNonZero(Mat(status));
	cout << "LK OPTICAL FLOW likes "<<good_points_Lk<<" out of " <<points_current.size()
		<<" (" << (float)(good_points_Lk)/(float)(points_current.size()) *100<<"%)"<<endl;

	matchingFeatures new_matches;

	new_matches.old_features = points_previous;
	new_matches.new_features = points_current;
	new_matches.old_threeD = previous_matches.old_threeD;
	new_matches.new_threeD = previous_matches.new_threeD;
	new_matches.history = previous_matches.history;

	draw_trace(current_frame, new_matches, status, " feature matching");

	// Remove the previous features which were not matched in this frame
	skim_points(new_matches , status);

	return new_matches;

}

void appearanceOdometryEstimator::matching(Mat previous_frame, Mat frame,
		vector<Point2f>& points_previous, vector<Point2f>& points_current, vector<uchar>& status, cv::Mat mask)
{
	vector<float> err;
	Size winSize=Size(19,19); // was (21,21) was (11,11)
	int maxLevel=3;
	//TermCriteria criteria=TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 150, 0.01);
	TermCriteria criteria=TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01);

	int flags = 0;//OPTFLOW_LK_GET_MIN_EIGENVALS;
	double minEigThreshold=1e-4;

	calcOpticalFlowPyrLK(previous_frame, frame, points_previous, points_current, status, err, winSize, maxLevel, criteria, flags, minEigThreshold) ;

	//Setting as bad correspondences the ones with an extreme outside of the mask
	vector<float> good_erros;
	for (unsigned int k=0; k<status.size(); k++)
	{
		if (mask.at<uchar>(points_current[k])==0)
			status[k] = 0;
		if (status[k])
			good_erros.push_back(err[k]);
	}

	//Setting as bad correspondences the ones with an error bigger than the median*1.5
	sort(good_erros.begin(), good_erros.end());
	float median;
	if (good_erros.size() % 2 == 0)
		median = (good_erros[good_erros.size() / 2 - 1] + good_erros[good_erros.size() / 2]) / 2;
	else
		median = good_erros[good_erros.size() / 2];
	float sum = std::accumulate(good_erros.begin(), good_erros.end(), 0.0);
	float errors_mean = sum / good_erros.size();
	float sq_sum = std::inner_product(good_erros.begin(), good_erros.end(), good_erros.begin(), 0.0);
	float errors_stdev = std::sqrt(sq_sum / good_erros.size() - errors_mean * errors_mean);
	cout << "Out of "<< good_erros.size() << "  matched points, those with error < median*1.5 will survive" << endl;
	cout << "Average error is " << errors_mean << " +- " << errors_stdev << " (median: " <<median << ")"<< endl;
	for (unsigned int k=0; k<status.size(); k++)
	{
		if (status[k] && err[k]>median*1.5)
			status[k] = 0;
	}


}


void  appearanceOdometryEstimator::draw_trace(cv::Mat dst, appearanceOdometryEstimator::matchingFeatures features,
		std::vector<uchar> status, std::string transf_type, std::vector<float> error)
{
	vector<Point2f> src_corners = features.old_features;
	vector<Point2f> dst_corners = features.new_features;
	vector<uchar> status_extended = status;

	for (unsigned int i=0; i< features.history.size(); i++)
	{
		if (status[i])
		{
			for (unsigned int j=1; j< features.history[i].size(); j++)
			{
				src_corners.push_back(features.history[i][j-1]);
				dst_corners.push_back(features.history[i][j]);
				status_extended.push_back(2);
			}
		}
	}

	draw_flow(dst, src_corners, dst_corners, status_extended, transf_type, error);
}

void  appearanceOdometryEstimator::draw_flow(cv::Mat dst, std::vector<cv::Point2f> src_corners, std::vector<cv::Point2f> dst_corners,
		std::vector<uchar> status, std::string transf_type, std::vector<float> error)
{
	// Draw feature evolution
	// In green is the trnsition from previous frame to current frame
	// In case the history of the feature is known, it is shown in blue

	Mat dst_copy;
	dst.copyTo(dst_copy);
	if (dst_copy.channels()<3)
		cvtColor(dst_copy, dst_copy, CV_GRAY2BGR);

	assert(src_corners.size() == dst_corners.size());

	for (size_t i=0; i<src_corners.size(); i++)
	{
		if (status[i]==1)
		{
			if (error.empty())
				circle(dst_copy, dst_corners[i], 2, white, 3);
			else
				circle(dst_copy, dst_corners[i], (int)(error[i]), black);
			line(dst_copy, src_corners[i], dst_corners[i], green,3);

		}
		else if (status[i]==2) // showing trace
		{
			if (error.empty())
				circle(dst_copy, dst_corners[i], 2, white, 3);
			else
				circle(dst_copy, dst_corners[i], (int)(error[i]), black);
			line(dst_copy, src_corners[i], dst_corners[i], blue,3);
		}
		else
		{
			circle(dst_copy, dst_corners[i], 2, red, 3);
		}
	}

	std::string title = "Optical flow after " + transf_type;
	namedWindow(title, WINDOW_NORMAL);
	imshow(title, dst_copy);
}

double appearanceOdometryEstimator::computeScale(vector<Point3f> points_previous, vector<Point3f> points_current)
{

	// Scale is the ratio among the relative distance of 3D points in previous iteration
	// and current iteration
	// Since there are outliers, we compute the scale according to every couple of points
	// And use the median of the results

	std::vector<double> scale;
	int count = 0;
	for (unsigned int i = 0; i < points_previous.size(); i++)
	{
		Point3f previous_i = points_previous[i];
		Point3f current_i = points_current[i];
		for (unsigned int j = 0; j < points_previous.size(); j++)
		{
			if (i != j)
			{
				Point3f previous_j = points_previous[j];
				Point3f current_j = points_current[j];

				double abs_previous = norm(previous_j-previous_i);
				double abs_current = norm(current_j-current_i);
				if (abs_current != 0)
				{
					double current_scale = abs_previous / abs_current;
					scale.push_back(current_scale);
					count++;
				}
			}
		}
	}
	sort(scale.begin(), scale.end());
	double median;
	if (scale.size() % 2 == 0)
		median = (scale[scale.size() / 2 - 1] + scale[scale.size() / 2]) / 2;
	else
		median = scale[scale.size() / 2];

	double sum = std::accumulate(scale.begin(), scale.end(), 0.0);
	double scale_mean = sum / scale.size();
	double sq_sum = std::inner_product(scale.begin(), scale.end(), scale.begin(), 0.0);
	double scale_stdev = std::sqrt(sq_sum / scale.size() - scale_mean * scale_mean);
	cout << "SCALE (out of "<< scale.size() << " points)" << endl <<
			"mean: " << scale_mean << ", median: "<<  median << ", stdev : "<< scale_stdev << endl;
	return median;
}

void appearanceOdometryEstimator::write(appearanceOdometryEstimator::matchingFeatures matches, string name)
{
	// show on terminal data about the features

	cout << name << endl;
	cout << "Old features: " << matches.old_features.size() <<   endl;
	cout << "New features: " << matches.new_features.size() <<   endl;
	cout << "Old three D: " << matches.old_threeD.size() <<   endl;
	cout << "New three D: " << matches.new_threeD.size() <<   endl;
	cout << "Features history: " << matches.history.size() << " ";
	for (unsigned int i=0; i<matches.history.size(); i++)
		cout << matches.history[i].size() << ", ";
	cout << endl;

}
