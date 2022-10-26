#ifndef QUAD_H
#define QUAD_H

#include <vector>
#include "opencv2/opencv.hpp"

using std::vector;


class Quad
{
public:
	vector<cv::Point2d> corners;
	cv::Point3d lineInf;
	double projectiveDistortion = 0;
	bool dark_inside;
	cv::Mat H;
	cv::Point2d center;

	void calculateLineAtInfinity();
	void calculateProjectiveDistortion();
	void check_color(cv::Mat image);
	void fix_white();

	Quad(){}
	Quad(vector<cv::Point2d> inCorners, bool dark_inside);
	Quad(const Quad &q);
	void estimateHomography();
};

#endif