#include "Quad.h"
#include "utility.h"
#include "markerStats.h"

using cv::Mat;
using cv::Point2d;
using cv::Point3d;

Quad::Quad(vector<Point2d> inCorners)
{
	corners = inCorners;
	calculateLineAtInfinity();
	calculateProjectiveDistortion();
}

void Quad::check_color(Mat image, vector<Mat> innerLocs)
{
	// corner vector to corner array
	cv::Point2f orig_corners[4];
	std::copy(corners.begin(), corners.end(), orig_corners);

	// back project to check color
	cv::Point2f warped_corners[4];
	cv::Mat warped(200, 200, image.type());
	warped_corners[0] = cv::Point2f(0, 0);
	warped_corners[1] = cv::Point2f(warped.cols, 0);
	warped_corners[2] = cv::Point2f(warped.cols, warped.rows);
	warped_corners[3] = cv::Point2f(0, warped.rows);
	Mat warpPerspectiveMatrix = cv::getPerspectiveTransform(orig_corners, warped_corners);
	cv::warpPerspective(image, warped, warpPerspectiveMatrix, warped.size());

	// threshold using otsu and binary
	cv::threshold(warped, warped, 0, 255, cv::THRESH_OTSU + cv::THRESH_BINARY);

	float blackLocs = 0;
	vector<cv::Mat> neededLocs = vector<cv::Mat>(12);
	for (int i = 1; i < innerLocs.size(); i += 3)
		neededLocs[i / 3] = innerLocs[i];

	float borderDist = innerLocs[0].at<double>(0) / 2;
	neededLocs[4] = (cv::Mat_<double>(1, 3) << 0.3, borderDist, 1);
	neededLocs[5] = (cv::Mat_<double>(1, 3) << borderDist, 0.3, 1);
	neededLocs[6] = (cv::Mat_<double>(1, 3) << 0.3, 1 - borderDist, 1);
	neededLocs[7] = (cv::Mat_<double>(1, 3) << 1 - borderDist, 0.3, 1);
	neededLocs[8] = (cv::Mat_<double>(1, 3) << 0.7, borderDist, 1);
	neededLocs[9] = (cv::Mat_<double>(1, 3) << borderDist, 0.7, 1);
	neededLocs[10] = (cv::Mat_<double>(1, 3) << 0.7, 1 - borderDist, 1);
	neededLocs[11] = (cv::Mat_<double>(1, 3) << 1 - borderDist, 0.7, 1);

	for (int i = 0; i < innerLocs.size(); i++)
		if ((int)warped.at<uchar>(cv::Point2d(innerLocs[i].at<double>(0) * 200, innerLocs[i].at<double>(1) * 200)) == 0)
			blackLocs++;

	// check 4 points on every edge of square and decide if black or white based on half of the points
	if (blackLocs >= (neededLocs.size() / 2))
		black_square = true;
	else
		black_square = false;
}

void Quad::fix_white()
{
	if (black_square == false)
	{
		double offset = 1 - (markerStats::borderRatio + markerStats::diamondRatio);

		// corner matrix
		Mat matcorn(3, 1, CV_64F);

		matcorn.at<double>(0) = -(0.5 + 0.05 + offset);
		matcorn.at<double>(1) = 0.5;
		matcorn.at<double>(2) = 1;
		Mat projectedPoint = H * matcorn;
		corners[0] = Point2d(projectedPoint.at<double>(0) / projectedPoint.at<double>(2), projectedPoint.at<double>(1) / projectedPoint.at<double>(2));

		matcorn.at<double>(0) = 0.5;
		matcorn.at<double>(1) = -(0.5 + 0.05 + offset);
		matcorn.at<double>(2) = 1;
		projectedPoint = H * matcorn;
		corners[1] = Point2d(projectedPoint.at<double>(0) / projectedPoint.at<double>(2), projectedPoint.at<double>(1) / projectedPoint.at<double>(2));

		matcorn.at<double>(0) = 0.5;
		matcorn.at<double>(1) = 1.5 + 0.05 + offset;
		matcorn.at<double>(2) = 1;
		projectedPoint = H * matcorn;
		corners[3] = Point2d(projectedPoint.at<double>(0) / projectedPoint.at<double>(2), projectedPoint.at<double>(1) / projectedPoint.at<double>(2));

		matcorn.at<double>(0) = 1.5 + 0.05 + offset;
		matcorn.at<double>(1) = 0.5;
		matcorn.at<double>(2) = 1;
		projectedPoint = H * matcorn;
		corners[2] = Point2d(projectedPoint.at<double>(0) / projectedPoint.at<double>(2), projectedPoint.at<double>(1) / projectedPoint.at<double>(2));

		// recalculate stats
		calculateLineAtInfinity();
		calculateProjectiveDistortion();
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

		// this intersection is not real. doing this to find the equation of the line with only one point.
		inters1.x = inters2.x + vec14.x;
		inters1.y = inters2.y + vec14.y;
	}
	// if the other edge pair is parallel
	else if (crossProduct(vec12, vec34) == 0)
	{
		inters1.x = (cross14 * vec23.x - vec14.x * cross23) / (vec14.x * vec23.y - vec14.y * vec23.x);
		inters1.y = (cross14 * vec23.y - vec14.y * cross23) / (vec14.x * vec23.y - vec14.y * vec23.x);

		// this intersection is not real. doing this to find the equation of the line with only one point.
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
	black_square = q.black_square;
	projectiveDistortion = q.projectiveDistortion;
	H = q.H.clone();
	center = q.center;
}