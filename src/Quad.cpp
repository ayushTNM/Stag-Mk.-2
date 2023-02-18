#include "Quad.h"
#include "utility.h"

using cv::Point2d;
using cv::Point3d;
using cv::Mat;


Quad::Quad(vector<Point2d> inCorners, bool indark_inside)
{
	corners = inCorners;
	dark_inside = indark_inside;
	calculateLineAtInfinity();
	calculateProjectiveDistortion();
}

void Quad::check_color(Mat image, vector<Mat> innerLocs, vector<Mat> outerLocs) {
	// Mat image_thresh;
	// cv::threshold(image, image_thresh, 10, 255, cv::THRESH_BINARY);


	// for (int i = 0; i < 12; i++)
	// {
	// 	Mat projectedPoint = H * innerLocs[i];
	// 	Point2d pointloc = Point2d(projectedPoint.at<double>(0) / projectedPoint.at<double>(2), projectedPoint.at<double>(1) / projectedPoint.at<double>(2));
	// 	cv::circle(image_thresh,pointloc,2,cv::Scalar(0,0,255));
	// 	std::cout << image_thresh.at<double>(pointloc) << std::endl;
	// 	// samples[i + 48] = readPixelSafeBilinear(image, Point2d(projectedPoint.at<double>(0) / projectedPoint.at<double>(2), projectedPoint.at<double>(1) / projectedPoint.at<double>(2)));
	// 	// if (q.dark_inside == false) {
	// 	// 	samples[i+48] = 0;
	// 	// }
	// 	// std:: cout << samples[i+48] /255 << std::endl;
	// }
	// for (int i = 0; i < 12; i++)
	// {
	// 	Mat projectedPoint = H * outerLocs[i];
	// 	// samples[i + 60] = readPixelSafeBilinear(image, Point2d(projectedPoint.at<double>(0) / projectedPoint.at<double>(2), projectedPoint.at<double>(1) / projectedPoint.at<double>(2)));
	// 	// if (q.dark_inside == false) {
	// 	// samples[i+60] = 255;
	// 	// }
	// 	// std:: cout << samples[i+60] /255 << std::endl;
	// }
	// cv::cvtColor(image_thresh,image_thresh,cv::COLOR_GRAY2RGB);

	// cv::threshold(image, image_thresh, 0, 255, cv::THRESH_OTSU + cv::THRESH_BINARY);
	cv::Point2f orig_corners[4];
	orig_corners[0] = corners[0];
	orig_corners[1] = corners[1];
	orig_corners[2] = corners[2];
	orig_corners[3] = corners[3];

	// for (int i = 0; i < corners.size(); i++) {
	// 	cv::circle(image,corners[i],2,cv::Scalar(255,255,0));
	// 	cv::putText(image,std::to_string(i),corners[i],cv::FONT_HERSHEY_SIMPLEX,2,cv::Scalar(255,255,255));
	// }
	// cv::imshow("haha",image);

	cv::Point2f warped_corners[4];
	cv::Mat warped(200,200,image.type());
	warped_corners[0] = cv::Point2f(0,0);
	warped_corners[1] = cv::Point2f(warped.cols,0);
	warped_corners[2] = cv::Point2f(warped.cols,warped.rows);
	warped_corners[3] = cv::Point2f(0,warped.rows);
	// std::cout << orig_corners.size() << ' ' << warped_corners.size() << std::endl;

	Mat warpPerspectiveMatrix = cv::getPerspectiveTransform(orig_corners, warped_corners);
	cv::warpPerspective(image, warped,warpPerspectiveMatrix, warped.size());
	Mat mask = warped>0;
    double minc[1], maxc[1];

    minMaxLoc(warped, minc, maxc,NULL,NULL,mask);
	cv::threshold(warped, warped, minc[0], 255, cv::THRESH_OTSU + cv::THRESH_BINARY);
	float blackLocs=0;
	vector<cv::Mat> neededLocs = vector<cv::Mat>(8);

	for (int i = 1; i < innerLocs.size(); i+=3) {
		
		neededLocs[i/3] = innerLocs[i];
	}
	float borderDist = innerLocs[0].at<double>(0)/2;
	neededLocs[4] = (cv::Mat_ <double>(1,3) << 0.5,borderDist,1);
	neededLocs[5] = (cv::Mat_ <double>(1,3) << borderDist,0.5,1);
	neededLocs[6] = (cv::Mat_ <double>(1,3) << 0.5,1-borderDist,1);
	neededLocs[7] = (cv::Mat_ <double>(1,3) << 1-borderDist,0.5,1);

	for (int i = 0; i < neededLocs.size(); i++) {
		if ((int)warped.at<uchar>(cv::Point2d(neededLocs[i].at<double>(0)*200,neededLocs[i].at<double>(1)*200)) == 0) {
			cv::circle(warped,cv::Point2d(neededLocs[i].at<double>(0)*200,neededLocs[i].at<double>(1)*200),2,cv::Scalar(255,255,0));
			blackLocs++;
		}
	}
	// float percBlack = blackLocs/neededLocs.size();
	std::cout << blackLocs << std::endl;	
	

	if (blackLocs >= 4) {
		dark_inside= true;
		cv::putText(warped,"Black",Point2d(0,200),cv::FONT_HERSHEY_SIMPLEX,2,cv::Scalar(255,255,255));
	}
	else {
		dark_inside=false;
		cv::putText(warped,"White",Point2d(0,200),cv::FONT_HERSHEY_SIMPLEX,2,cv::Scalar(0,0,0));

	}
	// float percBlack = (float)cv::countNonZero(warped) / (float)warped.total();
	// std::cout << percBlack << ", " << warped.total() << std::endl;	
	// // cv::cvtColor(warped,warped,cv::COLOR_GRAY2RGB);
	// // for (int i = 0; i < 12; i++) {
	// // 	cv::circle(warped,cv::Point2d(innerLocs[i].at<double>(0)*200,innerLocs[i].at<double>(1)*200),2,cv::Scalar(255,255,0));
	// // }
	// if (percBlack > 0.3) {
	// 	dark_inside = false;
	// 	cv::putText(warped,"White",warped_corners[1],cv::FONT_HERSHEY_SIMPLEX,2,cv::Scalar(0,0,0));
	// }
	// else {
	// 	dark_inside = true;
	// 	cv::putText(warped,"Black",warped_corners[1],cv::FONT_HERSHEY_SIMPLEX,2,cv::Scalar(255,255,255));
	// }
	// cv::putText(warped,std::to_string(percBlack),warped_corners[1],cv::FONT_HERSHEY_SIMPLEX,2,cv::Scalar(255,255,255));
	cv::imshow("warp",warped);
	// // if (floor(percBlack*10) == 0.2)
}

void Quad::fix_white() {
	if (dark_inside == false) {
		Mat matcorn(3,1,CV_64F);
		matcorn.at<double>(0) = -(0.5 + 0.25);
		matcorn.at<double>(1) = 0.5;
		matcorn.at<double>(2) = 1;
		// std::cout << matcorn << std::endl;
		Mat projectedPoint = H * matcorn;
		corners[0] = Point2d(projectedPoint.at<double>(0) / projectedPoint.at<double>(2), projectedPoint.at<double>(1) / projectedPoint.at<double>(2));
		// cv::circle(image2,corner1,3,cv::Scalar(255,0,0));
		// matcorn(3,1,CV_64F);
		matcorn.at<double>(0) = 0.5;
		matcorn.at<double>(1) = -(0.5+0.25);
		matcorn.at<double>(2) = 1;
		// std::cout << matcorn << std::endl;
		projectedPoint = H * matcorn;
		corners[1] = Point2d(projectedPoint.at<double>(0) / projectedPoint.at<double>(2), projectedPoint.at<double>(1) / projectedPoint.at<double>(2));
		// cv::circle(image2,corner4,3,cv::Scalar(255,0,255));
		
		matcorn.at<double>(0) = 0.5;
		matcorn.at<double>(1) = 1.5+0.25;
		matcorn.at<double>(2) = 1;
		// std::cout << matcorn << std::endl;
		projectedPoint = H * matcorn;
		corners[3] = Point2d(projectedPoint.at<double>(0) / projectedPoint.at<double>(2), projectedPoint.at<double>(1) / projectedPoint.at<double>(2));
		// cv::circle(image2,corner2,3,cv::Scalar(0,255,0));

		matcorn.at<double>(0) = 1.5+0.25;
		matcorn.at<double>(1) = 0.5;
		matcorn.at<double>(2) = 1;
		// std::cout << matcorn << std::endl;
		projectedPoint = H * matcorn;
		corners[2] = Point2d(projectedPoint.at<double>(0) / projectedPoint.at<double>(2), projectedPoint.at<double>(1) / projectedPoint.at<double>(2));
		estimateHomography();
	}
}

void Quad::estimateHomography()
{
	// apply affine rectification to the corners
	vector<Point2d> affineCorners = vector<Point2d>(4);

	for (int i = 0; i < 4; i++)
		affineCorners[i] = Point2d(corners[i].x / (lineInf.x * corners[i].x + lineInf.y * corners[i].y + lineInf.z), corners[i].y / (lineInf.x * corners[i].x + lineInf.y * corners[i].y + lineInf.z));

	Mat HarInv = Mat::eye(3, 3, CV_64FC1);
	Mat Haffsim = Mat::eye(3, 3, CV_64FC1);

	// inverse of affine rectification
	HarInv.at<double>(2, 0) = -lineInf.x / lineInf.z;
	HarInv.at<double>(2, 1) = -lineInf.y / lineInf.z;
	HarInv.at<double>(2, 2) = 1 / lineInf.z;

	// find the affine transformation from square to affine rectified quad
	Haffsim.at<double>(0, 0) = affineCorners[1].x - affineCorners[0].x;
	Haffsim.at<double>(0, 1) = affineCorners[3].x - affineCorners[0].x;
	Haffsim.at<double>(0, 2) = affineCorners[0].x;
	Haffsim.at<double>(1, 0) = affineCorners[1].y - affineCorners[0].y;
	Haffsim.at<double>(1, 1) = affineCorners[3].y - affineCorners[0].y;
	Haffsim.at<double>(1, 2) = affineCorners[0].y;

	// product of these transformations is the homography
	H = HarInv * Haffsim;

	// locate the projection of the center of the marker
	Mat origCenter(3, 1, CV_64FC1);
	origCenter.at<double>(0) = 0.5;
	origCenter.at<double>(1) = 0.5;
	origCenter.at<double>(2) = 1;

	origCenter = H * origCenter;
	center.x = origCenter.at<double>(0) / origCenter.at<double>(2);
	center.y = origCenter.at<double>(1) / origCenter.at<double>(2);

	// if (dark_inside == false) {

	// 	Mat matcorn(3,1,CV_64F);
	// 	matcorn.at<double>(0) = -(0.5 + 0.25);
	// 	matcorn.at<double>(1) = 0.5;
	// 	matcorn.at<double>(2) = 1;
	// 	// std::cout << matcorn << std::endl;
	// 	Mat projectedPoint = H * matcorn;
	// 	corners[0] = Point2d(projectedPoint.at<double>(0) / projectedPoint.at<double>(2), projectedPoint.at<double>(1) / projectedPoint.at<double>(2));
	// 	// cv::circle(image2,corner1,3,cv::Scalar(255,0,0));
	// 	// matcorn(3,1,CV_64F);
	// 	matcorn.at<double>(0) = 0.5;
	// 	matcorn.at<double>(1) = -(0.5+0.25);
	// 	matcorn.at<double>(2) = 1;
	// 	// std::cout << matcorn << std::endl;
	// 	projectedPoint = H * matcorn;
	// 	corners[1] = Point2d(projectedPoint.at<double>(0) / projectedPoint.at<double>(2), projectedPoint.at<double>(1) / projectedPoint.at<double>(2));
	// 	// cv::circle(image2,corner4,3,cv::Scalar(255,0,255));
		
	// 	matcorn.at<double>(0) = 0.5;
	// 	matcorn.at<double>(1) = 1.5+0.25;
	// 	matcorn.at<double>(2) = 1;
	// 	// std::cout << matcorn << std::endl;
	// 	projectedPoint = H * matcorn;
	// 	corners[3] = Point2d(projectedPoint.at<double>(0) / projectedPoint.at<double>(2), projectedPoint.at<double>(1) / projectedPoint.at<double>(2));
	// 	// cv::circle(image2,corner2,3,cv::Scalar(0,255,0));

	// 	matcorn.at<double>(0) = 1.5+0.25;
	// 	matcorn.at<double>(1) = 0.5;
	// 	matcorn.at<double>(2) = 1;
	// 	// std::cout << matcorn << std::endl;
	// 	projectedPoint = H * matcorn;
	// 	corners[2] = Point2d(projectedPoint.at<double>(0) / projectedPoint.at<double>(2), projectedPoint.at<double>(1) / projectedPoint.at<double>(2));
	// 	// cv::circle(image2,corner3,3,cv::Scalar(0,255,255));
	// 	dark_inside = true;

	// 	affineCorners = vector<Point2d>(4);
	// 	for (int i = 0; i < 4; i++)
	// 		affineCorners[i] = Point2d(corners[i].x / (lineInf.x * corners[i].x + lineInf.y * corners[i].y + lineInf.z), corners[i].y / (lineInf.x * corners[i].x + lineInf.y * corners[i].y + lineInf.z));

	// 	HarInv = Mat::eye(3, 3, CV_64FC1);
	// 	Haffsim = Mat::eye(3, 3, CV_64FC1);

	// 	// inverse of affine rectification
	// 	HarInv.at<double>(2, 0) = -lineInf.x / lineInf.z;
	// 	HarInv.at<double>(2, 1) = -lineInf.y / lineInf.z;
	// 	HarInv.at<double>(2, 2) = 1 / lineInf.z;

	// 	// find the affine transformation from square to affine rectified quad
	// 	Haffsim.at<double>(0, 0) = affineCorners[1].x - affineCorners[0].x;
	// 	Haffsim.at<double>(0, 1) = affineCorners[3].x - affineCorners[0].x;
	// 	Haffsim.at<double>(0, 2) = affineCorners[0].x;
	// 	Haffsim.at<double>(1, 0) = affineCorners[1].y - affineCorners[0].y;
	// 	Haffsim.at<double>(1, 1) = affineCorners[3].y - affineCorners[0].y;
	// 	Haffsim.at<double>(1, 2) = affineCorners[0].y;

	// 	// product of these transformations is the homography
	// 	H = HarInv * Haffsim;

	// 	// locate the projection of the center of the marker
	// 	Mat origCenter(3, 1, CV_64FC1);
	// 	origCenter.at<double>(0) = 0.5;
	// 	origCenter.at<double>(1) = 0.5;
	// 	origCenter.at<double>(2) = 1;

	// 	origCenter = H * origCenter;
	// 	center.x = origCenter.at<double>(0) / origCenter.at<double>(2);
	// 	center.y = origCenter.at<double>(1) / origCenter.at<double>(2);
	// 	}
}


void Quad::calculateLineAtInfinity()
{
	// intersection points at the vanishing line
	Point2d inters1, inters2;

	// cross products of corners (i.e. lines representing the edges)
	double cross14 = crossProduct(corners[0], corners[3]);
	double cross23 = crossProduct(corners[1], corners[2]);
	double cross12 = crossProduct(corners[0], corners[1]);
	double cross34 = crossProduct(corners[2], corners[3]);

	// vectors going from one corner to another
	Point2d vec23(corners[1].x - corners[2].x, corners[1].y - corners[2].y);
	Point2d vec14(corners[0].x - corners[3].x, corners[0].y - corners[3].y);
	Point2d vec34(corners[2].x - corners[3].x, corners[2].y - corners[3].y);
	Point2d vec12(corners[0].x - corners[1].x, corners[0].y - corners[1].y);

	// if both edge pairs are parallel
	if ((crossProduct(vec14, vec23) == 0) && (crossProduct(vec12, vec34) == 0)) // lines are parallel
	{
		lineInf = Point3d(0, 0, 1);
		return;
	}
	// if one edge pair is parallel
	else if (crossProduct(vec14, vec23) == 0)
	{
		inters2.x = (cross12 * vec34.x - vec12.x * cross34) / (vec12.x * vec34.y - vec12.y * vec34.x);
		inters2.y = (cross12 * vec34.y - vec12.y * cross34) / (vec12.x * vec34.y - vec12.y * vec34.x);

		//this intersection is not real. doing this to find the equation of the line with only one point.
		inters1.x = inters2.x + vec14.x;
		inters1.y = inters2.y + vec14.y;
	}
	// if the other edge pair is parallel
	else if (crossProduct(vec12, vec34) == 0)
	{
		inters1.x = (cross14 * vec23.x - vec14.x * cross23) / (vec14.x * vec23.y - vec14.y * vec23.x);
		inters1.y = (cross14 * vec23.y - vec14.y * cross23) / (vec14.x * vec23.y - vec14.y * vec23.x);

		//this intersection is not real. doing this to find the equation of the line with only one point.
		inters2.x = inters1.x + vec12.x;
		inters2.y = inters1.y + vec12.y;
	}
	// if neither pairs are parallel
	else
	{
		inters1.x = (cross14 * vec23.x - vec14.x * cross23) / (vec14.x * vec23.y - vec14.y * vec23.x);
		inters1.y = (cross14 * vec23.y - vec14.y * cross23) / (vec14.x * vec23.y - vec14.y * vec23.x);

		inters2.x = (cross12 * vec34.x - vec12.x * cross34) / (vec12.x * vec34.y - vec12.y * vec34.x);
		inters2.y = (cross12 * vec34.y - vec12.y * cross34) / (vec12.x * vec34.y - vec12.y * vec34.x);
	}

	// find the vanishing line in homogeneous coordinates
	// l = P1 x P2
	double l1 = inters1.y - inters2.y;
	double l2 = inters2.x - inters1.x;
	double l3 = inters1.x * inters2.y - inters2.x * inters1.y;

	// normalize using http://homepages.inf.ed.ac.uk/rbf/CVonline/LOCAL_COPIES/BEARDSLEY/node2.html (13)
	double normalizer = sqrt(l1 * l1 + l2 * l2);
	l1 /= normalizer;
	l2 /= normalizer;
	l3 /= normalizer;

	lineInf = Point3d(l1, l2, l3);
}


void Quad::calculateProjectiveDistortion()
{
	// find the minimum and maximum distance from corners to the vanishing line
	// projective distortion = maxDist / minDist
	double curDist = abs(lineInf.x * corners[0].x + lineInf.y * corners[0].y + lineInf.z);

	double minDist = curDist;
	double maxDist = curDist;

	for (int i = 1; i < 4; i++)
	{
		curDist = abs(lineInf.x * corners[i].x + lineInf.y * corners[i].y + lineInf.z);
		if (curDist < minDist)
			minDist = curDist;
		if (curDist > maxDist)
			maxDist = curDist;
	}
	projectiveDistortion = maxDist / minDist;
}


Quad::Quad(const Quad &q)
{
	corners = q.corners;
	lineInf = q.lineInf;
	dark_inside = q.dark_inside;
	projectiveDistortion = q.projectiveDistortion;
	H = q.H.clone();
	center = q.center;
}